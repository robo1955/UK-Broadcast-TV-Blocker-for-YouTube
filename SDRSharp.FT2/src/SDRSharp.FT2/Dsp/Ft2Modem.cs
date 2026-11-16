namespace SDRSharp.FT2.Dsp;

/// <summary>
/// FT2 is the FT4 waveform at twice the speed: 0.024 s symbols, 41.6667 Hz spacing,
/// 105 tones, 2.52 s on the air. Symbol values, Costas arrays and the Gray map are FT4's.
/// Audio is resampled to 12 kHz (288 samples per symbol) and searched across the passband.
/// </summary>
public static class Ft2Modem
{
    public const int BaseRate = 12000;
    public const int SamplesPerSymbol = 288;
    public const int FrameSamples = Ft2Codec.FrameSymbols * SamplesPerSymbol;
    public const double ToneSpacing = BaseRate / (double)SamplesPerSymbol;

    const int FftSize = 1024;
    const int Hop = 72;
    const int Bins = 280;
    const double ScoreFloor = 0.50;

    static readonly int[] CostasAt = { 1, 34, 67, 100 };
    static readonly byte[][] Costas =
    {
        new byte[] { 0, 1, 3, 2 },
        new byte[] { 1, 0, 2, 3 },
        new byte[] { 2, 3, 1, 0 },
        new byte[] { 3, 2, 0, 1 }
    };

    public readonly record struct Hit(string Text, double AudioHz, double Score, double SnrDb, byte[] Payload);

    public static float[] Synthesize(ReadOnlySpan<byte> tones, int sampleRate, double centerHz, int delaySamples, double noiseSigma, int noiseSeed)
    {
        if (sampleRate != 12000 && sampleRate != 48000)
            throw new ArgumentException("Synthesizer supports 12000 and 48000 Hz.");
        var nsps = sampleRate / BaseRate * SamplesPerSymbol;
        var body = tones.Length * nsps;
        var total = delaySamples + body + nsps;
        var freq = new double[total];
        for (var s = 0; s < tones.Length; s++)
        {
            var toneHz = centerHz + (tones[s] - 1.5) * ToneSpacing;
            var a = delaySamples + s * nsps;
            var b = a + nsps;
            for (var i = a; i < b; i++)
                freq[i] = toneHz;
        }

        var sigma = 0.132505 * nsps;
        var radius = Math.Max(1, (int)Math.Ceiling(3 * sigma));
        var kernel = new double[radius * 2 + 1];
        var weight = 0.0;
        for (var k = -radius; k <= radius; k++)
        {
            var x = k / sigma;
            var g = Math.Exp(-0.5 * x * x);
            kernel[k + radius] = g;
            weight += g;
        }
        var smooth = new double[total];
        for (var i = 0; i < total; i++)
        {
            var acc = 0.0;
            var lo = Math.Max(0, i - radius);
            var hi = Math.Min(total - 1, i + radius);
            for (var j = lo; j <= hi; j++)
                acc += freq[j] * kernel[j - i + radius];
            smooth[i] = acc / weight;
        }

        var audio = new float[total];
        var rng = noiseSigma > 0 ? new Random(noiseSeed) : null;
        var phase = 0.0;
        for (var i = 0; i < total; i++)
        {
            double sample = 0;
            if (i >= delaySamples)
            {
                phase += 2 * Math.PI * smooth[i] / sampleRate;
                sample = 0.8 * Math.Cos(phase);
            }
            if (rng != null)
                sample += Gauss(rng) * noiseSigma;
            audio[i] = (float)sample;
        }
        return audio;
    }

    public static bool TryDemodulate(ReadOnlySpan<float> audio, int start, double centerHz, out Hit hit)
    {
        hit = default;
        if (start < 0 || start + FrameSamples > audio.Length)
            return false;
        if (!Measure(audio, start, centerHz, out var score, out var snr))
            return false;
        if (!SoftDecode(audio, start, centerHz, out var payload))
            return false;
        hit = new Hit(Ft2Text.Format(payload), centerHz, score, snr, payload);
        return true;
    }

    public static void Search(ReadOnlySpan<float> audio, double sampleRate, List<Hit> hits)
    {
        hits.Clear();
        if (sampleRate < 1000 || audio.Length < 32)
            return;
        float[] baseAudio;
        if (Math.Abs(sampleRate - BaseRate) < 1)
        {
            baseAudio = new float[audio.Length];
            audio.CopyTo(baseAudio);
        }
        else
        {
            var converter = new RateConverter();
            var produced = new List<float>(audio.Length * BaseRate / (int)sampleRate + 8);
            converter.Convert(audio, sampleRate, produced);
            baseAudio = produced.ToArray();
        }
        SearchBase(baseAudio, hits);
    }

    public static void SearchBase(ReadOnlySpan<float> audio, List<Hit> hits)
    {
        hits.Clear();
        if (audio.Length < FrameSamples + SamplesPerSymbol)
            return;

        var hops = (audio.Length - SamplesPerSymbol) / Hop;
        var lastStart = hops - 4 * (Ft2Codec.FrameSymbols - 1);
        if (lastStart <= 0)
            return;

        var spec = new float[hops * Bins];
        var re = new float[FftSize];
        var im = new float[FftSize];
        var binHz = BaseRate / (double)FftSize;
        for (var hop = 0; hop < hops; hop++)
        {
            Array.Clear(re);
            Array.Clear(im);
            var origin = hop * Hop;
            for (var n = 0; n < SamplesPerSymbol; n++)
                re[n] = audio[origin + n];
            Fft(re, im);
            var row = hop * Bins;
            for (var bin = 0; bin < Bins; bin++)
                spec[row + bin] = re[bin] * re[bin] + im[bin] * im[bin];
        }

        var rough = new List<Candidate>(16);
        var freq0 = (int)Math.Round(160 / binHz);
        var freq1 = (int)Math.Round(2850 / binHz);
        for (var start = 0; start < lastStart; start++)
        {
            for (var bin = freq0; bin <= freq1; bin++)
            {
                var score = CoarseScore(spec, hops, start, bin * binHz, binHz);
                if (score < ScoreFloor)
                    continue;
                rough.Add(new Candidate(start * Hop, bin * binHz, score));
            }
        }
        if (rough.Count == 0)
            return;

        rough.Sort((a, b) => b.Score.CompareTo(a.Score));
        var kept = new List<Candidate>(12);
        foreach (var item in rough)
        {
            var near = false;
            foreach (var prior in kept)
            {
                if (Math.Abs(item.Start - prior.Start) <= 8 * Hop && Math.Abs(item.Hz - prior.Hz) <= 8 * binHz)
                {
                    near = true;
                    break;
                }
            }
            if (near)
                continue;
            kept.Add(item);
            if (kept.Count == 12)
                break;
        }

        var seen = new HashSet<string>();
        foreach (var item in kept)
        {
            if (!Refine(audio, item, out var start, out var hz, out var score, out var snr))
                continue;
            if (!SoftDecode(audio, start, hz, out var payload))
                continue;
            var key = Convert.ToHexString(payload);
            if (!seen.Add(key))
                continue;
            hits.Add(new Hit(Ft2Text.Format(payload), hz, score, snr, payload));
        }
    }

    static bool Refine(ReadOnlySpan<float> audio, Candidate coarse, out int start, out double hz, out double score, out double snr)
    {
        start = coarse.Start;
        hz = coarse.Hz;
        score = 0;
        snr = 0;
        var best = -1.0;
        var bestStart = coarse.Start;
        var bestHz = coarse.Hz;
        for (var dt = -36; dt <= 36; dt += 12)
        {
            var at = coarse.Start + dt;
            if (at < 0 || at + FrameSamples > audio.Length)
                continue;
            for (var df = -12; df <= 12; df += 6)
            {
                var f = coarse.Hz + df;
                if (!Measure(audio, at, f, out var trial, out var trialSnr))
                    continue;
                if (trial > best)
                {
                    best = trial;
                    bestStart = at;
                    bestHz = f;
                    snr = trialSnr;
                }
            }
        }
        if (best < ScoreFloor)
            return false;

        var fineBest = best;
        var fineStart = bestStart;
        var fineHz = bestHz;
        var fineSnr = snr;
        for (var dt = -8; dt <= 8; dt += 4)
        {
            var at = bestStart + dt;
            if (at < 0 || at + FrameSamples > audio.Length)
                continue;
            for (var df = -4; df <= 4; df += 2)
            {
                var f = bestHz + df;
                if (!Measure(audio, at, f, out var trial, out var trialSnr))
                    continue;
                if (trial > fineBest)
                {
                    fineBest = trial;
                    fineStart = at;
                    fineHz = f;
                    fineSnr = trialSnr;
                }
            }
        }
        start = fineStart;
        hz = fineHz;
        score = fineBest;
        snr = fineSnr;
        return true;
    }

    static float CoarseScore(float[] spec, int hops, int startHop, double centerHz, double binHz)
    {
        var expected = 0.0;
        var total = 0.0;
        for (var group = 0; group < 4; group++)
        {
            for (var k = 0; k < 4; k++)
            {
                var hop = startHop + 4 * (CostasAt[group] + k);
                if (hop < 0 || hop >= hops)
                    return 0;
                var row = hop * Bins;
                double sum = 0;
                var want = 0f;
                for (var tone = 0; tone < 4; tone++)
                {
                    var bin = (int)Math.Round((centerHz + (tone - 1.5) * ToneSpacing) / binHz);
                    if (bin < 0 || bin >= Bins)
                        return 0;
                    var power = spec[row + bin];
                    sum += power;
                    if (tone == Costas[group][k])
                        want = power;
                }
                expected += want;
                total += sum;
            }
        }
        if (total <= 0)
            return 0;
        return (float)(expected / total);
    }

    static bool Measure(ReadOnlySpan<float> audio, int start, double centerHz, out double score, out double snr)
    {
        score = 0;
        snr = 0;
        var expected = 0.0;
        var other = 0.0;
        Span<double> power = stackalloc double[4];
        for (var group = 0; group < 4; group++)
        {
            for (var k = 0; k < 4; k++)
            {
                var at = start + (CostasAt[group] + k) * SamplesPerSymbol;
                if (at < 0 || at + SamplesPerSymbol > audio.Length)
                    return false;
                TonePower(audio, at, centerHz, power);
                var want = power[Costas[group][k]];
                var sum = power[0] + power[1] + power[2] + power[3];
                expected += want;
                other += sum - want;
            }
        }
        var total = expected + other;
        if (total <= 0)
            return false;
        score = expected / total;
        snr = 10 * Math.Log10((expected + 1e-12) / (other / 3.0 + 1e-12));
        return true;
    }

    static bool SoftDecode(ReadOnlySpan<float> audio, int start, double centerHz, out byte[] payload)
    {
        var llr = new float[Ft2Codec.CodeBits];
        var bit = 0;
        Span<double> power = stackalloc double[4];
        for (var symbol = 0; symbol < Ft2Codec.FrameSymbols; symbol++)
        {
            if (!Ft2Codec.IsData(symbol))
                continue;
            var at = start + symbol * SamplesPerSymbol;
            TonePower(audio, at, centerHz, power);
            llr[bit++] = BitLlr(power, 2);
            llr[bit++] = BitLlr(power, 1);
        }
        return Ft2Codec.TryDecodeLlrs(llr, out payload);
    }

    static float BitLlr(ReadOnlySpan<double> power, int mask)
    {
        var one = 1e-12;
        var zero = 1e-12;
        for (var tone = 0; tone < 4; tone++)
        {
            var bits = CostasBits(tone);
            if ((bits & mask) != 0)
                one = Math.Max(one, power[tone]);
            else
                zero = Math.Max(zero, power[tone]);
        }
        var llr = Math.Log(one) - Math.Log(zero);
        if (llr > 6) return 6;
        if (llr < -6) return -6;
        return (float)llr;
    }

    static int CostasBits(int tone) => tone switch
    {
        0 => 0,
        1 => 1,
        2 => 3,
        3 => 2,
        _ => 0
    };

    static void TonePower(ReadOnlySpan<float> audio, int start, double centerHz, Span<double> power)
    {
        Span<double> dRe = stackalloc double[4];
        Span<double> dIm = stackalloc double[4];
        Span<double> cRe = stackalloc double[4];
        Span<double> cIm = stackalloc double[4];
        Span<double> sumRe = stackalloc double[4];
        Span<double> sumIm = stackalloc double[4];
        for (var tone = 0; tone < 4; tone++)
        {
            var w = 2 * Math.PI * (centerHz + (tone - 1.5) * ToneSpacing) / BaseRate;
            dRe[tone] = Math.Cos(w);
            dIm[tone] = Math.Sin(w);
            cRe[tone] = 1;
        }
        for (var n = 0; n < SamplesPerSymbol; n++)
        {
            var s = audio[start + n];
            for (var tone = 0; tone < 4; tone++)
            {
                sumRe[tone] += s * cRe[tone];
                sumIm[tone] += s * cIm[tone];
                var nRe = cRe[tone] * dRe[tone] - cIm[tone] * dIm[tone];
                cIm[tone] = cRe[tone] * dIm[tone] + cIm[tone] * dRe[tone];
                cRe[tone] = nRe;
            }
        }
        for (var tone = 0; tone < 4; tone++)
            power[tone] = sumRe[tone] * sumRe[tone] + sumIm[tone] * sumIm[tone];
    }

    static void Fft(float[] re, float[] im)
    {
        var n = re.Length;
        var j = 0;
        for (var i = 1; i < n; i++)
        {
            var bit = n >> 1;
            while ((j & bit) != 0)
            {
                j ^= bit;
                bit >>= 1;
            }
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
            var half = len >> 1;
            for (var i = 0; i < n; i += len)
            {
                var cRe = 1.0;
                var cIm = 0.0;
                for (var k = 0; k < half; k++)
                {
                    var i0 = i + k;
                    var i1 = i0 + half;
                    var vRe = re[i1] * cRe - im[i1] * cIm;
                    var vIm = re[i1] * cIm + im[i1] * cRe;
                    var uRe = re[i0];
                    var uIm = im[i0];
                    re[i0] = (float)(uRe + vRe);
                    im[i0] = (float)(uIm + vIm);
                    re[i1] = (float)(uRe - vRe);
                    im[i1] = (float)(uIm - vIm);
                    var nRe = cRe * wRe - cIm * wIm;
                    cIm = cRe * wIm + cIm * wRe;
                    cRe = nRe;
                }
            }
        }
    }

    static double Gauss(Random rng)
    {
        var u = 1.0 - rng.NextDouble();
        var v = 1.0 - rng.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u)) * Math.Cos(2.0 * Math.PI * v);
    }

    readonly record struct Candidate(int Start, double Hz, float Score);
}

public sealed class RateConverter
{
    float[] _prev = new float[64];
    int _count;
    double _phase;

    public void Reset()
    {
        _count = 0;
        _phase = 0;
    }

    public void Convert(ReadOnlySpan<float> input, double sampleRate, List<float> output)
    {
        if (input.Length == 0 || sampleRate < 1000)
            return;
        const int guard = 24;
        var src = new float[_count + input.Length];
        Array.Copy(_prev, src, _count);
        input.CopyTo(src.AsSpan(_count));
        var step = sampleRate / Ft2Modem.BaseRate;
        var cutoff = Math.Min(0.45, 5200.0 / sampleRate);
        var pos = _phase;
        var limit = src.Length - guard;
        while (pos < limit)
        {
            output.Add(Interpolate(src, pos, cutoff));
            pos += step;
        }
        var keepFrom = Math.Max(0, (int)Math.Floor(pos) - guard);
        var keep = src.Length - keepFrom;
        if (keep < 0)
            keep = 0;
        if (_prev.Length < keep)
            _prev = new float[keep + 64];
        Array.Copy(src, keepFrom, _prev, 0, keep);
        _count = keep;
        _phase = pos - keepFrom;
    }

    static float Interpolate(float[] src, double pos, double cutoff)
    {
        const int taps = 32;
        var i0 = (int)Math.Floor(pos);
        var acc = 0.0;
        for (var k = -taps / 2; k < taps / 2; k++)
        {
            var idx = i0 + k;
            if ((uint)idx >= (uint)src.Length)
                continue;
            var t = idx - pos;
            var window = 0.5 + 0.5 * Math.Cos(Math.PI * k / (taps / 2.0));
            acc += src[idx] * Lowpass(cutoff, t) * window;
        }
        return (float)acc;
    }

    static double Lowpass(double cutoff, double t)
    {
        if (Math.Abs(t) < 1e-8)
            return 2 * cutoff;
        var x = 2 * cutoff * t;
        return 2 * cutoff * Math.Sin(Math.PI * x) / (Math.PI * x);
    }
}
