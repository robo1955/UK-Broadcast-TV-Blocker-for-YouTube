namespace SDRSharp.FT2.Dsp;

/// <summary>
/// FT4 channel code at the FT2 symbol rate. The 77-bit payload is scrambled, CRC-14
/// protected and LDPC(174,91) encoded into 105 4-GFSK tones. Positive LLR means bit 1.
/// </summary>
public static class Ft2Codec
{
    public const int PayloadBits = 77;
    public const int CodeBits = 174;
    public const int FrameSymbols = 105;
    public const int DataSymbols = 87;

    static readonly byte[] Xor = { 0x4A, 0x5E, 0x89, 0xB4, 0xB0, 0x8A, 0x79, 0x55, 0xBE, 0x28 };
    static readonly byte[] Gray = { 0, 1, 3, 2 };
    static readonly byte[][] Costas =
    {
        new byte[] { 0, 1, 3, 2 },
        new byte[] { 1, 0, 2, 3 },
        new byte[] { 2, 3, 1, 0 },
        new byte[] { 3, 2, 0, 1 }
    };
    static readonly int[] CostasAt = { 1, 34, 67, 100 };

    static readonly int[] EdgeVar;
    static readonly int[] CheckStart;
    static readonly int[] VarStart;
    static readonly int[] VarEdges;

    static Ft2Codec()
    {
        if (LdpcTables.Generator.Length != 83 * 12 || LdpcTables.Nm.Length != 83 * 7 || LdpcTables.NumRows.Length != 83)
            throw new InvalidOperationException("LDPC tables have the wrong length.");

        var vars = new List<int>(600);
        var perVar = new List<int>[CodeBits];
        for (var v = 0; v < CodeBits; v++)
            perVar[v] = new List<int>(3);

        for (var m = 0; m < 83; m++)
        {
            var n = LdpcTables.NumRows[m];
            for (var i = 0; i < n; i++)
            {
                var index = LdpcTables.Nm[m * 7 + i];
                if (index < 1 || index > CodeBits)
                    throw new InvalidOperationException("LDPC check points outside the codeword.");
                var edge = vars.Count;
                vars.Add(index - 1);
                perVar[index - 1].Add(edge);
            }
        }

        EdgeVar = vars.ToArray();
        CheckStart = new int[84];
        var cursor = 0;
        for (var m = 0; m < 83; m++)
        {
            CheckStart[m] = cursor;
            cursor += LdpcTables.NumRows[m];
        }
        CheckStart[83] = cursor;

        VarStart = new int[CodeBits + 1];
        VarEdges = new int[EdgeVar.Length];
        cursor = 0;
        for (var v = 0; v < CodeBits; v++)
        {
            VarStart[v] = cursor;
            for (var i = 0; i < perVar[v].Count; i++)
                VarEdges[cursor++] = perVar[v][i];
        }
        VarStart[CodeBits] = cursor;
    }

    public static ushort Crc14(ReadOnlySpan<byte> message, int numBits)
    {
        var remainder = 0;
        var idxByte = 0;
        for (var bit = 0; bit < numBits; bit++)
        {
            if (bit % 8 == 0)
            {
                remainder ^= message[idxByte] << 6;
                idxByte++;
            }
            if ((remainder & 0x2000) != 0)
                remainder = (remainder << 1) ^ 0x2757;
            else
                remainder <<= 1;
            remainder &= 0xFFFF;
        }
        return (ushort)(remainder & 0x3FFF);
    }

    public static byte[] EncodeTones(ReadOnlySpan<byte> payload)
    {
        if (payload.Length < 10)
            throw new ArgumentException("FT2 payload is 10 bytes (77 bits).");
        Span<byte> scrambled = stackalloc byte[10];
        for (var i = 0; i < 10; i++)
            scrambled[i] = (byte)(payload[i] ^ Xor[i]);
        scrambled[9] &= 0xF8;

        var a91 = new byte[12];
        AddCrc(scrambled, a91);
        var codeword = new byte[22];
        Encode174(a91, codeword);
        return MapTones(codeword);
    }

    public static bool TryDecodeLlrs(ReadOnlySpan<float> llr, out byte[] payload)
    {
        payload = Array.Empty<byte>();
        if (llr.Length < CodeBits)
            return false;

        var bits = new byte[CodeBits];
        var sum = Hard(llr, bits, channelOnly: true);
        if (sum != 0 && ParityErrors(bits) == 0 && TryPayload(bits, out payload))
            return true;
        if (!Believe(llr, bits))
            return false;
        return TryPayload(bits, out payload);
    }

    /// <summary>Build ideal LLRs from a tone frame. Used to separate the Gray map from the modem.</summary>
    public static void IdealLlrs(ReadOnlySpan<byte> tones, Span<float> llr)
    {
        var bit = 0;
        for (var i = 0; i < FrameSymbols; i++)
        {
            if (!IsData(i))
                continue;
            var pair = Gray[tones[i] & 3];
            llr[bit++] = (pair & 2) != 0 ? 8f : -8f;
            llr[bit++] = (pair & 1) != 0 ? 8f : -8f;
        }
    }

    public static bool IsData(int symbol)
    {
        if (symbol <= 0 || symbol >= 104)
            return false;
        if (symbol < 5 || (symbol >= 34 && symbol < 38) || (symbol >= 67 && symbol < 71) || symbol >= 100)
            return false;
        return true;
    }

    static void AddCrc(ReadOnlySpan<byte> payload, byte[] a91)
    {
        for (var i = 0; i < 10; i++)
            a91[i] = payload[i];
        a91[9] &= 0xF8;
        a91[10] = 0;
        a91[11] = 0;
        var crc = Crc14(a91, 82);
        a91[9] |= (byte)(crc >> 11);
        a91[10] = (byte)(crc >> 3);
        a91[11] = (byte)(crc << 5);
    }

    static void Encode174(byte[] message, byte[] codeword)
    {
        for (var j = 0; j < 12; j++)
            codeword[j] = message[j];
        var mask = (byte)(0x80 >> (91 % 8));
        var column = 11;
        for (var row = 0; row < 83; row++)
        {
            var parity = 0;
            var gen = row * 12;
            for (var j = 0; j < 12; j++)
                parity ^= Parity8((byte)(message[j] & LdpcTables.Generator[gen + j]));
            if (parity != 0)
                codeword[column] |= mask;
            mask >>= 1;
            if (mask == 0)
            {
                mask = 0x80;
                column++;
            }
        }
    }

    static int Parity8(byte x)
    {
        x ^= (byte)(x >> 4);
        x ^= (byte)(x >> 2);
        x ^= (byte)(x >> 1);
        return x & 1;
    }

    static byte[] MapTones(byte[] codeword)
    {
        var tones = new byte[FrameSymbols];
        for (var group = 0; group < 4; group++)
        {
            for (var k = 0; k < 4; k++)
                tones[CostasAt[group] + k] = Costas[group][k];
        }

        var mask = 0x80;
        var ib = 0;
        for (var i = 0; i < FrameSymbols; i++)
        {
            if (!IsData(i))
                continue;
            var pair = 0;
            if ((codeword[ib] & mask) != 0)
                pair |= 2;
            mask >>= 1;
            if (mask == 0)
            {
                mask = 0x80;
                ib++;
            }
            if ((codeword[ib] & mask) != 0)
                pair |= 1;
            mask >>= 1;
            if (mask == 0)
            {
                mask = 0x80;
                ib++;
            }
            tones[i] = Gray[pair];
        }
        return tones;
    }

    static int ParityErrors(byte[] bits)
    {
        var errors = 0;
        for (var m = 0; m < 83; m++)
        {
            var x = 0;
            var n = LdpcTables.NumRows[m];
            var row = m * 7;
            for (var i = 0; i < n; i++)
                x ^= bits[LdpcTables.Nm[row + i] - 1];
            if (x != 0)
                errors++;
        }
        return errors;
    }

    static int Hard(ReadOnlySpan<float> llr, byte[] bits, bool channelOnly)
    {
        var sum = 0;
        for (var i = 0; i < CodeBits; i++)
        {
            bits[i] = (byte)(llr[i] > 0 ? 1 : 0);
            sum += bits[i];
        }
        return channelOnly ? sum : sum;
    }

    static bool Believe(ReadOnlySpan<float> channel, byte[] bits)
    {
        // Messages use log(P0/P1). The channel LLR is log(P1/P0), so it is negated here
        // and the hard decision is inverted on the way out. Check degrees are 6 and 7,
        // and the tanh rule is not invariant under that sign flip.
        var edges = EdgeVar.Length;
        var v2c = new float[edges];
        var c2v = new float[edges];
        var post = new float[CodeBits];
        for (var e = 0; e < edges; e++)
            v2c[e] = -channel[EdgeVar[e]];

        for (var iter = 0; iter < 40; iter++)
        {
            for (var check = 0; check < 83; check++)
            {
                var a = CheckStart[check];
                var b = CheckStart[check + 1];
                for (var e = a; e < b; e++)
                {
                    var prod = 1.0;
                    for (var f = a; f < b; f++)
                    {
                        if (f == e)
                            continue;
                        var x = v2c[f];
                        if (x > 30f) x = 30f;
                        else if (x < -30f) x = -30f;
                        prod *= Math.Tanh(x / 2.0);
                    }
                    if (prod > 0.999999) prod = 0.999999;
                    else if (prod < -0.999999) prod = -0.999999;
                    c2v[e] = (float)(2.0 * Math.Atanh(prod));
                }
            }

            for (var v = 0; v < CodeBits; v++)
                post[v] = -channel[v];
            for (var e = 0; e < edges; e++)
                post[EdgeVar[e]] += c2v[e];

            var sum = 0;
            for (var v = 0; v < CodeBits; v++)
            {
                bits[v] = (byte)(post[v] < 0 ? 1 : 0);
                sum += bits[v];
            }
            if (sum != 0 && ParityErrors(bits) == 0)
                return true;

            for (var v = 0; v < CodeBits; v++)
            {
                var from = VarStart[v];
                var to = VarStart[v + 1];
                for (var i = from; i < to; i++)
                {
                    var e = VarEdges[i];
                    v2c[e] = post[v] - c2v[e];
                }
            }
        }
        return false;
    }

    static bool TryPayload(byte[] bits, out byte[] payload)
    {
        payload = Array.Empty<byte>();
        var a91 = new byte[12];
        for (var i = 0; i < 91; i++)
        {
            if (bits[i] != 0)
                a91[i >> 3] |= (byte)(0x80 >> (i & 7));
        }

        var stored = ((a91[9] & 0x07) << 11) | (a91[10] << 3) | (a91[11] >> 5);
        var tmp = new byte[12];
        Array.Copy(a91, tmp, 12);
        tmp[9] &= 0xF8;
        tmp[10] = 0;
        tmp[11] = 0;
        if (Crc14(tmp, 82) != stored)
            return false;

        payload = new byte[10];
        var any = false;
        for (var i = 0; i < 10; i++)
        {
            payload[i] = (byte)(tmp[i] ^ Xor[i]);
            if (payload[i] != 0)
                any = true;
        }
        payload[9] &= 0xF8;
        if (!any)
            return false;
        return true;
    }
}
