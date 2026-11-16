using SDRSharp.LBandAcars.Dsp;

var failed = 0;
failed += Test("interleaver", InterleaverRoundTrip);
failed += Test("scrambler", ScramblerRoundTrip);
failed += Test("viterbi", ViterbiRoundTrip);
failed += Test("codec 600", () => Codec(600));
failed += Test("codec 1200", () => Codec(1200));
failed += Test("codec 10500", () => Codec(10500));
failed += Test("msk 600", () => Radio(600, 80));
failed += Test("msk 1200", () => Radio(1200, 150));
failed += Test("oqpsk 10500", () => Radio(10500, 400));

Console.WriteLine(failed == 0 ? "ALL PASSED" : failed + " FAILED");
return failed == 0 ? 0 : 1;

static int Test(string name, Action body)
{
    try
    {
        body();
        Console.WriteLine("PASS  " + name);
        return 0;
    }
    catch (Exception ex)
    {
        Console.WriteLine("FAIL  " + name);
        Console.WriteLine("      " + ex.Message);
        return 1;
    }
}

static void InterleaverRoundTrip()
{
    foreach (var rate in new[] { 600, 1200, 10500 })
    {
        var cols = AeroLink.Columns(rate);
        var n = AeroLink.Rows * cols;
        var src = new int[n];
        var mid = new int[n];
        var back = new int[n];
        for (var i = 0; i < n; i++) src[i] = (i * 17 + rate) & 1;
        AeroInterleaver.Interleave(src, cols, mid);
        AeroInterleaver.Deinterleave(mid, cols, back);
        for (var i = 0; i < n; i++)
            if (src[i] != back[i]) throw new Exception($"{rate} mismatch at {i}");
    }
}

static void ScramblerRoundTrip()
{
    var a = new AeroScrambler();
    var bits = new int[2000];
    for (var i = 0; i < bits.Length; i++) bits[i] = (i * 3) & 1;
    var mixed = new int[bits.Length];
    for (var i = 0; i < bits.Length; i++) mixed[i] = bits[i] ^ a.Next();
    a.Reset();
    for (var i = 0; i < bits.Length; i++)
        if ((mixed[i] ^ a.Next()) != bits[i]) throw new Exception("descramble");
}

static void ViterbiRoundTrip()
{
    var rng = new Random(1);
    var bits = new int[400];
    for (var i = 0; i < bits.Length; i++) bits[i] = rng.Next(2);
    var coded = new int[bits.Length * 2 + 200];
    var reg = 0;
    ViterbiK7.Encode(bits, coded.AsSpan(0, bits.Length * 2), ref reg);
    ViterbiK7.Encode(new int[80], coded.AsSpan(bits.Length * 2, 160), ref reg);
    var dec = new ViterbiK7();
    var got = new List<int>();
    for (var i = 0; i < bits.Length * 2 + 160; i += 2)
    {
        var s0 = coded[i] == 1 ? 100 : -100;
        var s1 = coded[i + 1] == 1 ? 100 : -100;
        var bit = dec.Step(s0, s1);
        if (bit != null) got.Add(bit.Value);
    }
    if (got.Count < bits.Length) throw new Exception($"short decode {got.Count}");
    var errors = 0;
    for (var i = 0; i < bits.Length; i++)
        if (got[i] != bits[i]) errors++;
    if (errors != 0) throw new Exception($"{errors} viterbi errors, delay {ViterbiK7.Traceback}, first got {got[0]} want {bits[0]}");
}

static void Codec(int rate)
{
    var frame = BuildFrame(rate, out var reg, out var label, out var text);
    var engine = new LBandEngine();
    engine.SetRate(rate);
    var messages = new List<DecodedMessage>();
    engine.Message += messages.Add;
    // Three frames so the Viterbi tail of the first two is resolved.
    for (var copy = 0; copy < 3; copy++)
    {
        foreach (var bit in frame)
            engine.InjectSoft(bit == 1 ? 120 : -120);
    }
    var hit = messages.FirstOrDefault(m => m.Registration == reg && m.Text.Contains(text) && m.Label == label);
    if (hit == null)
    {
        var preview = string.Join(" | ", messages.Take(6).Select(m => m.Kind + " " + m.Registration + " " + m.Label + " " + m.Text));
        throw new Exception($"no ACARS. frames={engine.Decoder.Frames} crc={engine.Decoder.CrcOk}/{engine.Decoder.CrcBad} msgs={messages.Count} [{preview}]");
    }
}

static void Radio(int rate, double offsetHz)
{
    var frame = BuildFrame(rate, out var reg, out var label, out var text);
    var preamble = new int[rate >= 8400 ? 800 : 400];
    for (var i = 0; i < preamble.Length; i++) preamble[i] = (i / 7) & 1;
    var bits = new List<int>(preamble);
    for (var copy = 0; copy < 4; copy++) bits.AddRange(frame);

    var sps = 8;
    var symbolRate = AeroLink.IsOqpsk(rate) ? rate / 2.0 : rate;
    var fs = symbolRate * sps;
    var iq = Modulate(bits, AeroLink.IsOqpsk(rate), fs, offsetHz, timingOffset: 3);

    var engine = new LBandEngine();
    engine.SetRate(rate);
    engine.SampleRate = fs;
    var messages = new List<DecodedMessage>();
    engine.Message += messages.Add;
    var hard = new List<int>();
    engine.SoftBit += s => hard.Add(s >= 128 ? 1 : 0);
    engine.ProcessIq(iq);

    var hit = messages.FirstOrDefault(m => m.Registration == reg && m.Text.Contains(text));
    if (hit != null)
        return;

    var agree = Correlate(bits, hard);
    throw new Exception(
        $"no ACARS. offsetEst={engine.FrequencyOffsetHz:0} Hz bitsIn={bits.Count} bitsOut={hard.Count} bestAgree={agree:0.00} " +
        $"frames={engine.Decoder.Frames} crc={engine.Decoder.CrcOk}/{engine.Decoder.CrcBad} uwErr={engine.Decoder.LastUniqueWordErrors} locked={engine.Decoder.Locked}");
}

static double Correlate(List<int> sent, List<int> got)
{
    if (got.Count < 64 || sent.Count < 64) return 0;
    var best = 0.0;
    var window = Math.Min(sent.Count, 1500);
    var limit = Math.Min(got.Count - 200, 4000);
    for (var shift = 0; shift < limit; shift += 1)
    {
        var n = Math.Min(window, got.Count - shift);
        if (n < 200) break;
        var same = 0;
        for (var i = 0; i < n; i++)
            if (sent[i] == got[shift + i]) same++;
        var agree = same / (double)n;
        if (agree > best) best = agree;
        var inv = 1 - agree;
        if (inv > best) best = inv;
    }
    return best;
}

static float[] Modulate(List<int> bits, bool oqpsk, double fs, double offsetHz, int timingOffset)
{
    var sps = 8;
    var samples = new List<float>(bits.Count * sps + 32);
    for (var i = 0; i < timingOffset; i++)
    {
        samples.Add(0);
        samples.Add(0);
    }
    double carrier = 0.35;
    var t = 0.0;
    var dt = 1.0 / fs;
    if (!oqpsk)
    {
        foreach (var bit in bits)
        {
            var turn = bit == 1 ? Math.PI / 2 : -Math.PI / 2;
            for (var n = 0; n < sps; n++)
            {
                var a = carrier + turn * (n + 0.5) / sps + 2 * Math.PI * offsetHz * t;
                samples.Add((float)Math.Cos(a));
                samples.Add((float)Math.Sin(a));
                t += dt;
            }
            carrier += turn;
        }
    }
    else
    {
        double prevQ = -1;
        if ((bits.Count & 1) == 1) bits.Add(0);
        for (var s = 0; s < bits.Count; s += 2)
        {
            var ii = bits[s] == 1 ? 1.0 : -1.0;
            var qq = bits[s + 1] == 1 ? 1.0 : -1.0;
            for (var n = 0; n < sps; n++)
            {
                var q = n < sps / 2 ? prevQ : qq;
                var a = 2 * Math.PI * offsetHz * t;
                var c = Math.Cos(a);
                var sn = Math.Sin(a);
                samples.Add((float)(ii * c - q * sn));
                samples.Add((float)(ii * sn + q * c));
                t += dt;
            }
            prevQ = qq;
        }
    }
    return samples.ToArray();
}

static List<int> BuildFrame(int rate, out string reg, out string label, out string text)
{
    reg = "N12345";
    label = "H1";
    text = "TEST";
    var payload = new List<byte> { 0xFF, 0xFF, 0x01, Par((byte)'2') };
    foreach (var c in "N12345.") payload.Add(Par((byte)c));
    payload.Add(Par((byte)'Q'));
    payload.Add(Par((byte)'H'));
    payload.Add(Par((byte)'1'));
    payload.Add(Par((byte)'A'));
    payload.Add(0x02);
    foreach (var c in text) payload.Add(Par((byte)c));
    payload.Add(0x83);
    payload.Add(0xAB);
    payload.Add(0xCD);
    payload.Add(0x7F);

    var units = new List<byte[]>();
    var seq = 3;
    var ten = new byte[10];
    ten[0] = 0x71;
    ten[1] = 0x00;
    ten[2] = 0x12;
    ten[3] = 0x34;
    ten[4] = 0x44;
    ten[5] = 0x10;
    ten[6] = (byte)seq;
    ten[7] = (byte)(6 << 4);
    ten[8] = payload[0];
    ten[9] = payload[1];
    units.Add(AeroCrc.WithCrc(ten));

    var cursor = 2;
    for (var s = seq - 1; s >= 0; s--)
    {
        var count = s == 0 ? 6 : 8;
        var su = new byte[10];
        su[0] = (byte)(0xC0 | s);
        su[1] = 0x10;
        for (var i = 0; i < count; i++)
            su[2 + i] = payload[cursor++];
        units.Add(AeroCrc.WithCrc(su));
    }
    if (cursor != payload.Count)
        throw new Exception($"payload used {cursor} of {payload.Count}");

    var need = AeroLink.SignalUnits(rate);
    while (units.Count < need)
    {
        var fill = new byte[10];
        fill[0] = 0x01;
        units.Add(AeroCrc.WithCrc(fill));
    }

    var info = new List<int>();
    foreach (var su in units)
    {
        for (var i = 0; i < 12; i++)
            for (var b = 0; b < 8; b++)
                info.Add((su[i] >> b) & 1);
    }
    if (info.Count != AeroLink.InfoBits(rate))
        throw new Exception($"info bits {info.Count} != {AeroLink.InfoBits(rate)}");

    var scrambled = new int[info.Count];
    var scrambler = new AeroScrambler();
    for (var i = 0; i < info.Count; i++)
        scrambled[i] = info[i] ^ scrambler.Next();

    var coded = new int[scrambled.Length * 2];
    var register = 0;
    ViterbiK7.Encode(scrambled, coded, ref register);
    var cols = AeroLink.Columns(rate);
    var block = AeroLink.Rows * cols;
    var interleaved = new int[coded.Length];
    for (var off = 0; off < coded.Length; off += block)
        AeroInterleaver.Interleave(coded.AsSpan(off, block), cols, interleaved.AsSpan(off, block));

    var frame = new List<int>();
    if (AeroLink.IsOqpsk(rate))
    {
        for (var i = 31; i >= 0; i--)
        {
            var b = (int)((AeroLink.UniqueWord >> i) & 1);
            frame.Add(b);
            frame.Add(b);
        }
    }
    else
    {
        for (var i = 31; i >= 0; i--)
            frame.Add((int)((AeroLink.UniqueWord >> i) & 1));
    }
    for (var i = 15; i >= 0; i--)
        frame.Add((0x1000 >> i) & 1);
    for (var i = 0; i < AeroLink.DummyBits(rate); i++)
        frame.Add(0);
    frame.AddRange(interleaved);
    if (frame.Count != AeroLink.FrameBits(rate))
        throw new Exception($"frame {frame.Count} != {AeroLink.FrameBits(rate)}");
    return frame;
}

static byte Par(byte seven)
{
    var b = seven & 0x7F;
    var p = 0;
    for (var i = 0; i < 7; i++)
        p ^= (b >> i) & 1;
    if (p == 0) b |= 0x80;
    return (byte)b;
}
