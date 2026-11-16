using System;
using System.Buffers;
using System.Collections.Concurrent;
using System.Threading;
using SDRSharp.LBandAcars.Dsp;
using SDRSharp.Radio;

namespace SDRSharp.LBandAcars;

/// <summary>
/// Copies VFO IQ off the SDR# callback and demodulates it on a worker thread.
/// The callback only enqueues; a backed-up worker drops blocks instead of stalling the radio.
/// </summary>
public sealed unsafe class IqProcessor : IIQProcessor, IDisposable
{
    readonly record struct Block(float[] Buffer, int Length);

    readonly LBandEngine _engine;
    readonly object _cmd = new();
    readonly BlockingCollection<Block> _queue = new(4);
    readonly Thread _worker;

    int _bitRate = 1200;
    double _offset;
    double _sampleRate = 48000;
    bool _bitDirty = true;
    bool _offsetDirty;
    bool _rateDirty;
    bool _reset;
    long _holdUntil;
    long _dropped;
    int _closed;
    double _unlockedSeconds;
    volatile bool _enabled;
    string _fault = "";

    public event Action<DecodedMessage>? Message;
    Snapshot _published = new(new LinkStatus { State = "idle", BitRate = 1200 }, 0, 0, "");

    public IqProcessor(LBandEngine engine)
    {
        _engine = engine;
        _engine.Message += message => Message?.Invoke(message);
        _worker = new Thread(Loop) { IsBackground = true, Name = "L-Band ACARS" };
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

    public Snapshot Read() => _published;

    public void SetBitRate(int rate)
    {
        lock (_cmd)
        {
            _bitRate = rate;
            _bitDirty = true;
        }
    }

    public void SetManualOffset(double hz)
    {
        lock (_cmd)
        {
            _offset = hz;
            _offsetDirty = true;
        }
    }

    /// <summary>Drop IQ briefly, then restart acquisition. Used after the VFO moves.</summary>
    public void RequestReset()
    {
        lock (_cmd)
        {
            _reset = true;
            _holdUntil = Environment.TickCount64 + 250;
        }
    }

    public void Process(Complex* buffer, int length)
    {
        if (!_enabled || length <= 0 || Volatile.Read(ref _closed) != 0)
            return;
        var iq = ArrayPool<float>.Shared.Rent(length * 2);
        for (var i = 0; i < length; i++)
        {
            iq[i * 2] = buffer[i].Real;
            iq[i * 2 + 1] = buffer[i].Imag;
        }
        try
        {
            if (!_queue.TryAdd(new Block(iq, length * 2)))
            {
                ArrayPool<float>.Shared.Return(iq);
                Interlocked.Increment(ref _dropped);
            }
        }
        catch (InvalidOperationException)
        {
            ArrayPool<float>.Shared.Return(iq);
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
        int bit = 0;
        double offset = 0;
        double sampleRate = 0;
        var doBit = false;
        var doOffset = false;
        var doRate = false;
        var doReset = false;
        lock (_cmd)
        {
            if (Environment.TickCount64 < _holdUntil)
                return;
            doBit = _bitDirty;
            _bitDirty = false;
            bit = _bitRate;
            doOffset = _offsetDirty;
            _offsetDirty = false;
            offset = _offset;
            doRate = _rateDirty;
            _rateDirty = false;
            sampleRate = _sampleRate;
            doReset = _reset;
            _reset = false;
        }

        if (doReset)
            _engine.Reset();
        if (doBit)
            _engine.SetRate(bit);
        if (doOffset)
            _engine.ManualOffsetHz = offset;
        if (doRate)
        {
            var previous = _engine.SampleRate;
            _engine.SampleRate = sampleRate;
            if (previous > 1000 && Math.Abs(sampleRate - previous) / previous > 0.02)
                _engine.Reset();
        }

        _engine.ProcessIq(block.Buffer.AsSpan(0, block.Length));
        if (_engine.Status.Locked)
            _unlockedSeconds = 0;
        else
        {
            _unlockedSeconds += (block.Length / 2.0) / Math.Max(_engine.SampleRate, 1);
            if (_unlockedSeconds > 8)
            {
                _engine.Reset();
                _unlockedSeconds = 0;
            }
        }
        Publish();
    }

    void Publish()
    {
        _published = new Snapshot(_engine.Status, _engine.SampleRate, Interlocked.Read(ref _dropped), _fault);
    }

    public readonly record struct Snapshot(LinkStatus Link, double SampleRate, long Dropped, string Fault);
}
