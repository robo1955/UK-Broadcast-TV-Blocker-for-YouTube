namespace SDRSharp.LBandAcars.Dsp;

/// <summary>
/// Resamples interleaved float IQ. The step is input samples per output sample.
/// </summary>
public sealed class IqResampler
{
    double _step = 1;
    double _srcPos;
    long _count;
    float _prevI;
    float _prevQ;
    bool _primed;

    public void Reset()
    {
        _srcPos = 0;
        _count = 0;
        _primed = false;
    }

    public void SetRates(double inputRate, double outputRate)
    {
        if (inputRate < 1 || outputRate < 1)
            return;
        var step = inputRate / outputRate;
        if (Math.Abs(step - _step) / _step > 0.001)
            Reset();
        _step = step;
    }

    public void Process(ReadOnlySpan<float> iq, Span<float> dst, out int produced)
    {
        produced = 0;
        var samples = iq.Length / 2;
        for (var n = 0; n < samples; n++)
        {
            var i = iq[n * 2];
            var q = iq[n * 2 + 1];
            if (!_primed)
            {
                _prevI = i;
                _prevQ = q;
                _primed = true;
                _count = 1;
                continue;
            }
            var at = _count;
            while (_srcPos <= at && produced + 2 <= dst.Length)
            {
                var t = _srcPos - (at - 1);
                if (t < 0) t = 0;
                if (t > 1) t = 1;
                dst[produced++] = (float)(_prevI + t * (i - _prevI));
                dst[produced++] = (float)(_prevQ + t * (q - _prevQ));
                _srcPos += _step;
            }
            _prevI = i;
            _prevQ = q;
            _count++;
        }
    }
}

static class Fft
{
    public static void Transform(Span<double> re, Span<double> im)
    {
        var n = re.Length;
        var j = 0;
        for (var i = 1; i < n; i++)
        {
            var bit = n >> 1;
            for (; (j & bit) != 0; bit >>= 1)
                j ^= bit;
            j ^= bit;
            if (i < j)
            {
                (re[i], re[j]) = (re[j], re[i]);
                (im[i], im[j]) = (im[j], im[i]);
            }
        }
        for (var len = 2; len <= n; len <<= 1)
        {
            var ang = -2 * Math.PI / len;
            var wRe = Math.Cos(ang);
            var wIm = Math.Sin(ang);
            for (var i = 0; i < n; i += len)
            {
                double cRe = 1, cIm = 0;
                var half = len >> 1;
                for (var k = 0; k < half; k++)
                {
                    var a = i + k;
                    var b = a + half;
                    var tRe = re[b] * cRe - im[b] * cIm;
                    var tIm = re[b] * cIm + im[b] * cRe;
                    re[b] = re[a] - tRe;
                    im[b] = im[a] - tIm;
                    re[a] += tRe;
                    im[a] += tIm;
                    var nRe = cRe * wRe - cIm * wIm;
                    cIm = cRe * wIm + cIm * wRe;
                    cRe = nRe;
                }
            }
        }
    }

    public static double BinHz(int bin, int n, double sampleRate)
    {
        if (bin >= n / 2)
            bin -= n;
        return bin * sampleRate / n;
    }
}

/// <summary>
/// Coherent-style A-BPSK / MSK demodulator. Aviation BPSK on the 600 and 1200
/// bit/s P-channels is filtered MSK: the phase steps ±π/2 each bit. Bits are
/// recovered from the sign of the integrated frequency deviation.
/// </summary>
public sealed class MskDemodulator
{
    public const int SamplesPerSymbol = 8;
    readonly float[] _dev = new float[SamplesPerSymbol * 128];
    readonly double[] _capRe = new double[4096];
    readonly double[] _capIm = new double[4096];
    readonly double[] _fftRe = new double[4096];
    readonly double[] _fftIm = new double[4096];
    int _cap;
    int _n;
    int _phase = -1;
    double _dc;
    double _omega;
    double _nco;
    float _zI, _zQ;
    bool _haveZ;
    double _power = 1;
    bool _acq = true;

    public int BitRate { get; }
    public double SampleRate => BitRate * (double)SamplesPerSymbol;
    public double FrequencyOffsetHz { get; private set; }
    public double ManualOffsetHz { get; set; }
    public event Action<byte>? Bit;

    public MskDemodulator(int bitRate) => BitRate = bitRate;

    public void Reset()
    {
        _cap = 0;
        _n = 0;
        _phase = -1;
        _dc = 0;
        _omega = 0;
        _nco = 0;
        _haveZ = false;
        _acq = true;
        FrequencyOffsetHz = 0;
    }

    public void Process(float i, float q)
    {
        _power = _power * 0.999 + (i * i + q * q) * 0.001;
        var g = 1.0 / Math.Sqrt(_power + 1e-12);
        i = (float)(i * g);
        q = (float)(q * g);

        if (_acq)
        {
            _capRe[_cap] = i;
            _capIm[_cap] = q;
            _cap++;
            if (_cap < _capRe.Length)
                return;
            Estimate();
            _acq = false;
            for (var n = 0; n < _capRe.Length; n++)
                Discriminate((float)_capRe[n], (float)_capIm[n]);
            _cap = 0;
            return;
        }
        Discriminate(i, q);
    }

    void Estimate()
    {
        var n = _fftRe.Length;
        for (var k = 0; k < n; k++)
        {
            var w = 0.5 - 0.5 * Math.Cos(2 * Math.PI * k / (n - 1));
            var i = _capRe[k];
            var q = _capIm[k];
            _fftRe[k] = (i * i - q * q) * w;
            _fftIm[k] = (2 * i * q) * w;
        }
        Fft.Transform(_fftRe, _fftIm);
        var sep = (int)Math.Round(BitRate / SampleRate * n);
        var half = Math.Max(1, sep / 2);
        var best = -1.0;
        var bestBin = 0;
        for (var c = 0; c < n; c++)
        {
            var a = Mag((c + half) % n);
            var b = Mag((c - half + n) % n);
            var score = a + b;
            if (score > best && a > 0.25 * score && b > 0.25 * score)
            {
                best = score;
                bestBin = c;
            }
        }
        var df = Fft.BinHz(bestBin, n, SampleRate) / 2;
        if (Math.Abs(df) < SampleRate * 0.45)
        {
            _omega = 2 * Math.PI * df / SampleRate;
            FrequencyOffsetHz = df;
        }
    }

    double Mag(int bin) => Math.Sqrt(_fftRe[bin] * _fftRe[bin] + _fftIm[bin] * _fftIm[bin]);

    void Discriminate(float i, float q)
    {
        var step = _omega + 2 * Math.PI * ManualOffsetHz / SampleRate;
        _nco += step;
        if (_nco > Math.PI * 2) _nco -= Math.PI * 2;
        else if (_nco < -Math.PI * 2) _nco += Math.PI * 2;
        var c = Math.Cos(_nco);
        var s = Math.Sin(_nco);
        var mi = (float)(i * c + q * s);
        var mq = (float)(q * c - i * s);
        if (!_haveZ)
        {
            _zI = mi;
            _zQ = mq;
            _haveZ = true;
            return;
        }
        var cross = mi * _zQ - mq * _zI;
        var dot = mi * _zI + mq * _zQ;
        var dphi = Math.Atan2(-cross, dot);
        _zI = mi;
        _zQ = mq;
        _dc = _dc * 0.995 + dphi * 0.005;
        _omega += _dc * 0.002;
        FrequencyOffsetHz = _omega * SampleRate / (2 * Math.PI) + ManualOffsetHz;
        Push((float)(dphi - _dc));
    }

    void Push(float dev)
    {
        if (_n >= _dev.Length)
            return;
        _dev[_n++] = dev;
        var sps = SamplesPerSymbol;
        if (_n < sps * 48)
            return;
        if (_phase < 0)
            _phase = BestPhase(_n);
        else
            _phase = Nudge(_phase, _n);
        var emitted = 0;
        while (_phase + (emitted + 1) * sps <= _n - sps)
        {
            double sum = 0;
            var o = _phase + emitted * sps;
            for (var k = 0; k < sps; k++)
                sum += _dev[o + k];
            Emit(sum);
            emitted++;
        }
        var start = _phase + emitted * sps;
        if (start > 0)
        {
            Array.Copy(_dev, start, _dev, 0, _n - start);
            _n -= start;
            _phase = 0;
        }
    }

    void Emit(double sum)
    {
        var bit = sum >= 0 ? 1 : 0;
        var mag = Math.Abs(sum);
        var expect = Math.PI / 2;
        var conf = Math.Clamp(mag / expect, 0, 1);
        var shape = 1 - Math.Clamp(Math.Abs(mag - expect) / expect, 0, 1);
        conf *= 0.3 + 0.7 * shape;
        var centered = bit == 1 ? 128 + (int)(127 * conf) : 128 - (int)(127 * conf);
        Bit?.Invoke((byte)Math.Clamp(centered, 0, 255));
    }

    int BestPhase(int n)
    {
        var sps = SamplesPerSymbol;
        var bestP = 0;
        var best = -1.0;
        for (var p = 0; p < sps; p++)
        {
            double energy = 0;
            for (var o = p; o + sps <= n; o += sps)
            {
                double sum = 0;
                for (var k = 0; k < sps; k++)
                    sum += _dev[o + k];
                energy += Math.Abs(sum);
            }
            if (energy > best)
            {
                best = energy;
                bestP = p;
            }
        }
        return bestP;
    }

    int Nudge(int phase, int n)
    {
        var sps = SamplesPerSymbol;
        double Score(int p)
        {
            if (p < 0) p += sps;
            double energy = 0;
            var count = 0;
            for (var o = p; o + sps <= n && count < 24; o += sps, count++)
            {
                double sum = 0;
                for (var k = 0; k < sps; k++)
                    sum += _dev[o + k];
                energy += Math.Abs(sum);
            }
            return count == 0 ? 0 : energy / count;
        }
        var here = Score(phase);
        var early = Score(phase - 1);
        var late = Score((phase + 1) % sps);
        if (early > here * 1.02 && early >= late)
            return (phase + sps - 1) % sps;
        if (late > here * 1.02)
            return (phase + 1) % sps;
        return phase;
    }
}

/// <summary>
/// OQPSK demodulator for the 10500 bit/s P-channel. Two bits per symbol, Q
/// delayed half a symbol from I. Carrier recovery uses the 4th-power line.
/// </summary>
public sealed class OqpskDemodulator
{
    public const int SamplesPerSymbol = 8;
    readonly float[] _i = new float[SamplesPerSymbol * 96];
    readonly float[] _q = new float[SamplesPerSymbol * 96];
    readonly double[] _capRe = new double[4096];
    readonly double[] _capIm = new double[4096];
    readonly double[] _fftRe = new double[4096];
    readonly double[] _fftIm = new double[4096];
    int _cap;
    int _n;
    int _phase = -1;
    double _omega;
    double _nco;
    double _power = 1;
    bool _acq = true;

    public int BitRate { get; }
    public double SymbolRate => BitRate / 2.0;
    public double SampleRate => SymbolRate * SamplesPerSymbol;
    public double FrequencyOffsetHz { get; private set; }
    public double ManualOffsetHz { get; set; }
    public event Action<byte>? Bit;

    public OqpskDemodulator(int bitRate) => BitRate = bitRate;

    public void Reset()
    {
        _cap = 0;
        _n = 0;
        _phase = -1;
        _omega = 0;
        _nco = 0;
        _acq = true;
        FrequencyOffsetHz = 0;
    }

    public void Process(float i, float q)
    {
        _power = _power * 0.999 + (i * i + q * q) * 0.001;
        var g = 1.0 / Math.Sqrt(_power + 1e-12);
        i = (float)(i * g);
        q = (float)(q * g);
        if (_acq)
        {
            _capRe[_cap] = i;
            _capIm[_cap] = q;
            _cap++;
            if (_cap < _capRe.Length)
                return;
            Estimate();
            _acq = false;
            for (var n = 0; n < _capRe.Length; n++)
                Mix((float)_capRe[n], (float)_capIm[n]);
            _cap = 0;
            return;
        }
        Mix(i, q);
    }

    void Estimate()
    {
        var n = _fftRe.Length;
        for (var k = 0; k < n; k++)
        {
            var w = 0.5 - 0.5 * Math.Cos(2 * Math.PI * k / (n - 1));
            var i = _capRe[k];
            var q = _capIm[k];
            var a = i * i - q * q;
            var b = 2 * i * q;
            var c = a * a - b * b;
            var d = 2 * a * b;
            _fftRe[k] = c * w;
            _fftIm[k] = d * w;
        }
        Fft.Transform(_fftRe, _fftIm);
        var best = -1.0;
        var bestBin = 0;
        for (var k = 0; k < n; k++)
        {
            var m = _fftRe[k] * _fftRe[k] + _fftIm[k] * _fftIm[k];
            if (m > best)
            {
                best = m;
                bestBin = k;
            }
        }
        var df = Fft.BinHz(bestBin, n, SampleRate) / 4;
        _omega = 2 * Math.PI * df / SampleRate;
        FrequencyOffsetHz = df;
    }

    void Mix(float i, float q)
    {
        _nco += _omega + 2 * Math.PI * ManualOffsetHz / SampleRate;
        if (_nco > Math.PI * 2) _nco -= Math.PI * 2;
        else if (_nco < 0) _nco += Math.PI * 2;
        var c = Math.Cos(_nco);
        var s = Math.Sin(_nco);
        Push((float)(i * c + q * s), (float)(q * c - i * s));
    }

    void Push(float i, float q)
    {
        if (_n >= _i.Length)
            return;
        _i[_n] = i;
        _q[_n] = q;
        _n++;
        var sps = SamplesPerSymbol;
        if (_n < sps * 40)
            return;
        if (_phase < 0)
            _phase = BestPhase();
        else
            _phase = Nudge(_phase);
        var emitted = 0;
        while (_phase + (emitted + 1) * sps + sps / 2 <= _n - sps / 2)
        {
            Slice(_phase + emitted * sps);
            emitted++;
        }
        var start = _phase + emitted * sps;
        if (start > 0)
        {
            Array.Copy(_i, start, _i, 0, _n - start);
            Array.Copy(_q, start, _q, 0, _n - start);
            _n -= start;
            _phase = 0;
        }
    }

    void Slice(int at)
    {
        var sps = SamplesPerSymbol;
        double si = 0, sq = 0;
        for (var k = 0; k < sps; k++)
            si += _i[at + k];
        var qAt = at + sps / 2;
        for (var k = 0; k < sps; k++)
            sq += _q[qAt + k];
        // Snap this symbol onto the nearest diagonal. A static phase offset is
        // not a frequency error, so it is not integrated into the NCO.
        var a = si * si - sq * sq;
        var b = 2 * si * sq;
        var c = a * a - b * b;
        var d = 2 * a * b;
        // A locked ±1±j symbol has fourth-power angle π, not 0. Fold around
        // that point so a locked symbol produces a near-zero correction.
        var ang = Math.Atan2(d, c);
        var folded = ang > 0 ? ang - Math.PI : ang + Math.PI;
        var theta = folded / 4;
        // theta is the carrier phase of this symbol. Snap the decision, but do
        // not integrate it into the frequency: a static phase is not a freq error
        // and walking it there unlocks the carrier. A slow residual is absorbed
        // by the next symbol's own snap as long as it stays well under 45 degrees.
        var rot = Math.Cos(-theta);
        var rs = Math.Sin(-theta);
        var ii = si * rot - sq * rs;
        var qq = si * rs + sq * rot;
        var scale = (Math.Abs(ii) + Math.Abs(qq)) * 0.5 + 1e-6;
        Emit(ii / scale);
        Emit(qq / scale);
    }

    void Emit(double v)
    {
        var bit = v >= 0 ? 1 : 0;
        var conf = Math.Clamp(Math.Abs(v), 0, 1);
        var centered = bit == 1 ? 128 + (int)(127 * conf) : 128 - (int)(127 * conf);
        Bit?.Invoke((byte)Math.Clamp(centered, 0, 255));
    }

    int BestPhase()
    {
        var sps = SamplesPerSymbol;
        var bestP = 0;
        var best = -1.0;
        for (var p = 0; p < sps; p++)
        {
            double energy = 0;
            for (var s = p; s + sps + sps / 2 <= _n; s += sps)
            {
                double si = 0, sq = 0;
                for (var k = 0; k < sps; k++)
                    si += _i[s + k];
                for (var k = 0; k < sps; k++)
                    sq += _q[s + sps / 2 + k];
                energy += Math.Abs(si) + Math.Abs(sq);
            }
            if (energy > best)
            {
                best = energy;
                bestP = p;
            }
        }
        return bestP;
    }

    int Nudge(int phase)
    {
        var sps = SamplesPerSymbol;
        double Score(int p)
        {
            if (p < 0) p += sps;
            double energy = 0;
            var count = 0;
            for (var s = p; s + sps + sps / 2 <= _n && count < 16; s += sps, count++)
            {
                double si = 0, sq = 0;
                for (var k = 0; k < sps; k++)
                    si += _i[s + k];
                for (var k = 0; k < sps; k++)
                    sq += _q[s + sps / 2 + k];
                energy += Math.Abs(si) + Math.Abs(sq);
            }
            return energy;
        }
        var here = Score(phase);
        var early = Score(phase - 1);
        var late = Score((phase + 1) % sps);
        if (early > here * 1.02 && early >= late)
            return (phase + sps - 1) % sps;
        if (late > here * 1.02)
            return (phase + 1) % sps;
        return phase;
    }
}

public sealed class LBandEngine
{
    readonly FrameDecoder _decoder = new();
    readonly IqResampler _resampler = new();
    float[] _resampled = new float[16384];
    MskDemodulator? _msk;
    OqpskDemodulator? _oq;
    double _sampleRate = 48000;
    double _level;
    long _samples;
    int _rate = 1200;

    public FrameDecoder Decoder => _decoder;
    public int BitRate => _rate;
    public double FrequencyOffsetHz => _msk?.FrequencyOffsetHz ?? _oq?.FrequencyOffsetHz ?? 0;
    public double LevelDb => _samples == 0 ? -120 : 10 * Math.Log10(_level + 1e-12);
    public bool SamplesFlowing => _samples > 0;

    public event Action<DecodedMessage>? Message;
    public event Action<byte>? SoftBit;

    public double ManualOffsetHz
    {
        get => _msk?.ManualOffsetHz ?? _oq?.ManualOffsetHz ?? 0;
        set
        {
            if (_msk != null) _msk.ManualOffsetHz = value;
            if (_oq != null) _oq.ManualOffsetHz = value;
        }
    }

    public double SampleRate
    {
        get => _sampleRate;
        set
        {
            if (value < 1000 || Math.Abs(value - _sampleRate) < 1)
                return;
            _sampleRate = value;
            _resampler.SetRates(_sampleRate, TargetRate);
        }
    }

    public double TargetRate => AeroLink.IsOqpsk(_rate) ? _rate * 4.0 : _rate * 8.0;

    public LBandEngine()
    {
        _decoder.Message += m => Message?.Invoke(m);
        SetRate(1200);
    }

    public void SetRate(int rate)
    {
        _rate = rate;
        _decoder.SetRate(rate);
        if (_msk != null) _msk.Bit -= OnBit;
        if (_oq != null) _oq.Bit -= OnBit;
        _msk = null;
        _oq = null;
        if (AeroLink.IsOqpsk(rate))
        {
            _oq = new OqpskDemodulator(rate);
            _oq.Bit += OnBit;
        }
        else
        {
            _msk = new MskDemodulator(rate);
            _msk.Bit += OnBit;
        }
        _resampler.Reset();
        _resampler.SetRates(_sampleRate, TargetRate);
    }

    public void Reset()
    {
        _decoder.Reset();
        _msk?.Reset();
        _oq?.Reset();
        _resampler.Reset();
        _samples = 0;
    }

    public LinkStatus Status => new()
    {
        Locked = _decoder.Locked,
        BitRate = _rate,
        FrequencyOffsetHz = FrequencyOffsetHz,
        UniqueWordErrors = _decoder.LastUniqueWordErrors,
        Frames = _decoder.Frames,
        CrcOk = _decoder.CrcOk,
        CrcBad = _decoder.CrcBad,
        FillUnits = _decoder.FillUnits,
        LevelDb = LevelDb,
        SamplesFlowing = SamplesFlowing,
        State = !_decoder.Locked ? "searching" : "locked"
    };

    public void ProcessIq(ReadOnlySpan<float> interleaved)
    {
        if (interleaved.Length < 2)
            return;
        double energy = 0;
        var n = interleaved.Length / 2;
        for (var i = 0; i < n; i++)
        {
            var re = interleaved[i * 2];
            var im = interleaved[i * 2 + 1];
            energy += re * re + im * im;
        }
        _level = _level * 0.9 + (energy / n) * 0.1;
        _samples += n;

        var offset = 0;
        while (offset < interleaved.Length)
        {
            var take = Math.Min(interleaved.Length - offset, 4096);
            var need = (int)(take / Math.Max(_sampleRate / TargetRate, 0.05)) + 64;
            if (_resampled.Length < need)
                Array.Resize(ref _resampled, need);
            _resampler.Process(interleaved.Slice(offset, take), _resampled, out var produced);
            offset += take;
            for (var i = 0; i + 1 < produced; i += 2)
            {
                if (_msk != null)
                    _msk.Process(_resampled[i], _resampled[i + 1]);
                else
                    _oq!.Process(_resampled[i], _resampled[i + 1]);
            }
        }
    }

    /// <summary>Inject already-decided soft samples (±127) to test the codec without the demodulator.</summary>
    public void InjectSoft(int soft) => _decoder.PushSoft(soft);

    void OnBit(byte soft)
    {
        SoftBit?.Invoke(soft);
        _decoder.PushSoft(soft - 128);
    }
}
