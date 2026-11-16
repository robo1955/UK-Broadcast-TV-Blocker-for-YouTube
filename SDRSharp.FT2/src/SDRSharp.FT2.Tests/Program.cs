using System.Diagnostics;
using SDRSharp.FT2.Dsp;

var fails = 0;

void Check(bool ok, string name)
{
    if (ok)
        Console.WriteLine("OK   " + name);
    else
    {
        Console.WriteLine("FAIL " + name);
        fails++;
    }
}

ushort ShiftCrc(byte[] message, int numBits)
{
    var remainder = 0;
    var idx = 0;
    for (var bit = 0; bit < numBits; bit++)
    {
        if (bit % 8 == 0)
        {
            remainder ^= message[idx] << 6;
            idx++;
        }
        if ((remainder & 0x2000) != 0)
            remainder = ((remainder << 1) ^ 0x2757) & 0x3FFF;
        else
            remainder = (remainder << 1) & 0x3FFF;
    }
    return (ushort)remainder;
}

var crcBytes = new byte[] { 0x11, 0, 0, 0, 0, 0x0E, 0x10, 0x04, 0x01, 0x00 };
var crc = Ft2Codec.Crc14(crcBytes, 76);
Check(crc == ShiftCrc(crcBytes, 76), "CRC-14 matches the shift register (" + crc.ToString("X4") + ")");

Check(Ft2Text.TryPack28("K1ABC", out var n28, out var ip) && ip == 0 && n28 == 10214965, "pack K1ABC " + n28);

void RoundTrip(string to, string de, string extra, string expect)
{
    var payload = new byte[10];
    var packed = Ft2Text.TryStandard(to, de, extra, payload);
    var text = packed ? Ft2Text.Format(payload) : "(pack failed)";
    Check(packed && text == expect, "text " + expect + " -> " + text);
}

RoundTrip("CQ", "K1ABC", "FN42", "CQ K1ABC FN42");
RoundTrip("CQ 123", "K1ABC", "FN20", "CQ 123 K1ABC FN20");
RoundTrip("CQ JA", "LB2JK", "JO59", "CQ JA LB2JK JO59");
RoundTrip("K1ABC", "W9XYZ", "-07", "K1ABC W9XYZ -07");
RoundTrip("K1ABC", "W9XYZ", "R-07", "K1ABC W9XYZ R-07");
RoundTrip("K1ABC", "W9XYZ", "R FN42", "K1ABC W9XYZ R FN42");
RoundTrip("K1ABC/R", "W9XYZ", "RR73", "K1ABC/R W9XYZ RR73");
RoundTrip("DE", "W1AW", "73", "DE W1AW 73");
RoundTrip("W1A/R", "K1ABC", "", "W1A/R K1ABC");
RoundTrip("3DA0XYZ", "K1ABC", "FN42", "3DA0XYZ K1ABC FN42");
RoundTrip("PA3XYZ/P", "GM4ABC/P", "JO22", "PA3XYZ/P GM4ABC/P JO22");

var free = new byte[10];
Check(Ft2Text.TryFree("TNX BOB 73 GL", free) && Ft2Text.Format(free) == "TNX BOB 73 GL", "free text " + Ft2Text.Format(free));
Check(Ft2Text.TryFree("HELLO FT2", free) && Ft2Text.Format(free) == "HELLO FT2", "free hello " + Ft2Text.Format(free));

void Codec(string label, byte[] payload, string expect)
{
    var tones = Ft2Codec.EncodeTones(payload);
    var llr = new float[Ft2Codec.CodeBits];
    Ft2Codec.IdealLlrs(tones, llr);
    var ok = Ft2Codec.TryDecodeLlrs(llr, out var back);
    var text = ok ? Ft2Text.Format(back) : "(no decode)";
    Check(ok && text == expect, "codec " + label + " -> " + text);

    llr[10] = -llr[10];
    llr[40] = -llr[40];
    llr[90] = -llr[90];
    llr[140] = -llr[140];
    ok = Ft2Codec.TryDecodeLlrs(llr, out back);
    text = ok ? Ft2Text.Format(back) : "(no decode)";
    Check(ok && text == expect, "codec flips " + label + " -> " + text);
}

var cq = new byte[10];
Ft2Text.TryStandard("CQ", "K1ABC", "FN42", cq);
Codec("CQ", cq, "CQ K1ABC FN42");
Codec("free", free, "HELLO FT2");

void Aligned(string label, byte[] payload, string expect)
{
    var tones = Ft2Codec.EncodeTones(payload);
    var audio = Ft2Modem.Synthesize(tones, 12000, 737.5, 1500, 0, 1);
    var ok = Ft2Modem.TryDemodulate(audio, 1500, 737.5, out var hit);
    Check(ok && hit.Text == expect, "aligned " + label + " -> " + (ok ? hit.Text + " " + hit.Score.ToString("0.00") : "no"));
}

Aligned("CQ", cq, "CQ K1ABC FN42");
Aligned("free", free, "HELLO FT2");

void SearchOne(int rate, double hz, int delay, double noise, string expect)
{
    var tones = Ft2Codec.EncodeTones(cq);
    var audio = Ft2Modem.Synthesize(tones, rate, hz, delay, noise, 4);
    var hits = new List<Ft2Modem.Hit>();
    var clock = Stopwatch.StartNew();
    Ft2Modem.Search(audio, rate, hits);
    clock.Stop();
    var got = string.Join(" | ", hits.Select(h => h.Text + " @" + h.AudioHz.ToString("0") + "Hz " + h.Score.ToString("0.00")));
    var found = hits.Exists(h => h.Text == expect);
    Check(found, "search " + rate + " " + expect + " in " + clock.ElapsedMilliseconds + " ms [" + got + "]");
}

SearchOne(12000, 640, 900, 0.15, "CQ K1ABC FN42");
SearchOne(48000, 1860.5, 9000, 0.2, "CQ K1ABC FN42");

var other = new byte[10];
Ft2Text.TryStandard("K1ABC", "W9XYZ", "R-07", other);
var a = Ft2Modem.Synthesize(Ft2Codec.EncodeTones(cq), 48000, 520, 4000, 0.12, 7);
var b = Ft2Modem.Synthesize(Ft2Codec.EncodeTones(other), 48000, 2280, 16000, 0.12, 8);
var mix = new float[Math.Max(a.Length, b.Length)];
for (var i = 0; i < mix.Length; i++)
{
    if (i < a.Length) mix[i] += a[i];
    if (i < b.Length) mix[i] += b[i];
}
var both = new List<Ft2Modem.Hit>();
var mixClock = Stopwatch.StartNew();
Ft2Modem.Search(mix, 48000, both);
mixClock.Stop();
var mixText = string.Join(" | ", both.Select(h => h.Text + " @" + h.AudioHz.ToString("0")));
Check(both.Exists(h => h.Text == "CQ K1ABC FN42") && both.Exists(h => h.Text == "K1ABC W9XYZ R-07"),
    "two signals " + mixClock.ElapsedMilliseconds + " ms [" + mixText + "]");

var half = new List<float>();
var whole = new List<float>();
var left = new RateConverter();
var right = new RateConverter();
var tone = new float[48000];
for (var i = 0; i < tone.Length; i++)
    tone[i] = (float)Math.Sin(2 * Math.PI * 1000 * i / 48000);
left.Convert(tone.AsSpan(0, 20000), 48000, half);
left.Convert(tone.AsSpan(20000), 48000, half);
right.Convert(tone, 48000, whole);
var compare = Math.Min(half.Count, whole.Count);
var err = 0.0;
var skip = 64;
for (var i = skip; i < compare - skip; i++)
    err += Math.Abs(half[i] - whole[i]);
err /= Math.Max(1, compare - 2 * skip);
Check(compare > 10000 && err < 1e-3, "resampler halves err " + err.ToString("0.000000") + " n " + compare);

return fails == 0 ? 0 : 1;
