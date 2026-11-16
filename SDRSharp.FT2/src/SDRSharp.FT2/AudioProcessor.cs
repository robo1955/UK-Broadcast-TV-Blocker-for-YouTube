using System.Buffers;
using System.Collections.Concurrent;
using System.Runtime.InteropServices;
using SDRSharp.FT2.Dsp;
using SDRSharp.Radio;

namespace SDRSharp.FT2;

/// <summary>
/// Copies demodulated audio off the SDR# callback and decodes it on a worker thread.
/// The callback only enqueues. A backed-up worker drops blocks instead of stalling the radio.
/// </summary>
public sealed unsafe class AudioProcessor : IRealProcessor, IDisposable
{
    readonly record struct Block(float[] Buffer, int Length);

    readonly object _cmd = new();
    readonly BlockingCollection<Block> _queue = new(16);
    readonly Thread _worker;
    readonly RateConverter _rates = new();
    readonly SampleRing _ring = new(72000);
    readonly List<float> _base = new(8192);
    readonly List<Ft2Modem.Hit> _hits = new();
    readonly Dictionary<string, long> _seen = new();

    double _sampleRate = 48000;
    bool _rateDirty = true;
    bool _reset;
    long _holdUntil;
    long _dropped;
    int _closed;
    int _decodes;
    int _pending;
    volatile bool _enabled;
    string _fault = "";
    Status _published = new(0, 0, 0, 0, "", false);

    public event Action<Ft2Modem.Hit>? Message;

    public AudioProcessor()
    {
        _worker = new Thread(Loop) { IsBackground = true, Name = "FT2" };
        _worker.Start();
    }

    public bool Enabled
    {
        get => _enabled;
        set => _enabled = value;
    }

    public double SampleRate
    {
        set
        {
            if (value < 1000)
                return;
            lock (_cmd)
            {
                if (Math.Abs(value - _sampleRate) < 1)
                    return;
                _sampleRate = value;
                _rateDirty = true;
            }
        }
    }

    public Status Read() => _published;

    public void RequestReset()
    {
        lock (_cmd)
        {
            _reset = true;
            _holdUntil = Environment.TickCount64 + 250;
        }
    }

    public void Process(float* buffer, int length)
    {
        if (!_enabled || length <= 0 || Volatile.Read(ref _closed) != 0)
            return;
        var copy = ArrayPool<float>.Shared.Rent(length);
        fixed (float* dst = copy)
            Buffer.MemoryCopy(buffer, dst, length * sizeof(float), length * sizeof(float));
        try
        {
            if (!_queue.TryAdd(new Block(copy, length)))
            {
                ArrayPool<float>.Shared.Return(copy);
                Interlocked.Increment(ref _dropped);
            }
        }
        catch (InvalidOperationException)
        {
            ArrayPool<float>.Shared.Return(copy);
        }
    }

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _closed, 1) != 0)
            return;
        _enabled = false;
        _queue.CompleteAdding();
        _worker.Join(1500);
    }

    void Loop()
    {
        try
        {
            foreach (var block in _queue.GetConsumingEnumerable())
            {
                try
                {
                    Consume(block);
                }
                catch (Exception ex)
                {
                    _fault = ex.Message;
                    Publish();
                }
                finally
                {
                    ArrayPool<float>.Shared.Return(block.Buffer);
                }
            }
        }
        catch (Exception ex)
        {
            _fault = ex.Message;
            Publish();
        }
    }

    void Consume(Block block)
    {
        double sampleRate;
        var doRate = false;
        var doReset = false;
        lock (_cmd)
        {
            if (Environment.TickCount64 < _holdUntil)
                return;
            doRate = _rateDirty;
            _rateDirty = false;
            sampleRate = _sampleRate;
            doReset = _reset;
            _reset = false;
        }

        if (doReset || doRate)
        {
            _rates.Reset();
            _ring.Clear();
            _pending = 0;
            _base.Clear();
        }

        _base.Clear();
        _rates.Convert(block.Buffer.AsSpan(0, block.Length), sampleRate, _base);
        if (_base.Count > 0)
        {
            _ring.Add(CollectionsMarshal.AsSpan(_base));
            _pending += _base.Count;
        }

        if (_pending >= 8000 && _ring.Count >= Ft2Modem.FrameSamples + 6000)
        {
            var take = Math.Min(_ring.Count, 54000);
            var snap = ArrayPool<float>.Shared.Rent(take);
            try
            {
                _ring.CopyLast(take, snap);
                _pending = 0;
                Ft2Modem.SearchBase(snap.AsSpan(0, take), _hits);
                var now = Environment.TickCount64;
                foreach (var hit in _hits)
                {
                    var key = Convert.ToHexString(hit.Payload) + ":" + Math.Round(hit.AudioHz / 10.0).ToString("0");
                    if (_seen.TryGetValue(key, out var when) && now - when < 2200)
                        continue;
                    _seen[key] = now;
                    _decodes++;
                    Message?.Invoke(hit);
                }
                if (_seen.Count > 64)
                {
                    var stale = new List<string>();
                    foreach (var pair in _seen)
                    {
                        if (now - pair.Value > 8000)
                            stale.Add(pair.Key);
                    }
                    foreach (var key in stale)
                        _seen.Remove(key);
                }
            }
            finally
            {
                ArrayPool<float>.Shared.Return(snap);
            }
        }

        Publish();
    }

    void Publish()
    {
        _published = new Status(_sampleRate, Interlocked.Read(ref _dropped), _decodes, _hits.Count, _fault, _enabled);
    }

    public readonly record struct Status(double SampleRate, long Dropped, int Decodes, int LastHits, string Fault, bool Running);
}

sealed class SampleRing
{
    readonly float[] _data;
    int _write;
    int _count;

    public SampleRing(int capacity) => _data = new float[capacity];

    public int Count => _count;

    public void Clear()
    {
        _write = 0;
        _count = 0;
    }

    public void Add(ReadOnlySpan<float> samples)
    {
        foreach (var sample in samples)
        {
            _data[_write] = sample;
            _write++;
            if (_write == _data.Length)
                _write = 0;
            if (_count < _data.Length)
                _count++;
        }
    }

    public void CopyLast(int count, float[] destination)
    {
        if (count > _count)
            count = _count;
        var start = _write - count;
        if (start < 0)
            start += _data.Length;
        for (var i = 0; i < count; i++)
        {
            destination[i] = _data[start];
            start++;
            if (start == _data.Length)
                start = 0;
        }
    }
}
