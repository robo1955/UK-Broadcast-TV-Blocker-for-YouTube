namespace SDRSharp.FT2.Dsp;

/// <summary>
/// 77-bit FT4/FT8 message packing. Standard calls, free text, and a hex fallback
/// for types this panel does not expand. Hashed calls are shown as &lt;hex&gt;.
/// </summary>
public static class Ft2Text
{
    const uint NTokens = 2063592;
    const uint Max22 = 4194304;
    const int MaxGrid4 = 32400;

    public static bool TryStandard(string callTo, string callDe, string extra, byte[] payload)
    {
        if (payload.Length < 10)
            return false;
        Array.Clear(payload, 0, 10);
        callTo = Normalize(callTo);
        callDe = Normalize(callDe);
        extra = Normalize(extra);
        if (!TryPack28(callTo, out var n28a, out var ipa))
            return false;
        if (!TryPack28(callDe, out var n28b, out var ipb))
            return false;

        var toP = callTo.EndsWith("/P", StringComparison.Ordinal);
        var deP = callDe.EndsWith("/P", StringComparison.Ordinal);
        var toR = callTo.EndsWith("/R", StringComparison.Ordinal);
        var deR = callDe.EndsWith("/R", StringComparison.Ordinal);
        var i3 = 1;
        if (toP || deP)
        {
            if (toR || deR)
                return false;
            i3 = 2;
        }
        if (!TryPackField(extra, out var ir, out var grid15))
            return false;

        var n29a = (n28a << 1) | (uint)ipa;
        var n29b = (n28b << 1) | (uint)ipb;
        var field = (uint)((ir << 15) | grid15);
        payload[0] = (byte)(n29a >> 21);
        payload[1] = (byte)(n29a >> 13);
        payload[2] = (byte)(n29a >> 5);
        payload[3] = (byte)((n29a << 3) | (n29b >> 26));
        payload[4] = (byte)(n29b >> 18);
        payload[5] = (byte)(n29b >> 10);
        payload[6] = (byte)(n29b >> 2);
        payload[7] = (byte)((n29b << 6) | (field >> 10));
        payload[8] = (byte)(field >> 2);
        payload[9] = (byte)((field << 6) | ((uint)i3 << 3));
        return true;
    }

    public static bool TryFree(string text, byte[] payload)
    {
        if (payload.Length < 10)
            return false;
        text = Normalize(text);
        if (text.Length > 13)
            return false;
        var b71 = new byte[9];
        for (var idx = 0; idx < 13; idx++)
        {
            var c = idx < text.Length ? text[idx] : ' ';
            var cid = FullIndex(c);
            if (cid < 0)
                return false;
            var rem = cid;
            for (var i = 8; i >= 0; i--)
            {
                rem += b71[i] * 42;
                b71[i] = (byte)(rem & 0xFF);
                rem >>= 8;
            }
        }

        Array.Clear(payload, 0, 10);
        var carry = 0;
        for (var i = 8; i >= 0; i--)
        {
            payload[i] = (byte)((b71[i] << 1) | (carry >> 7));
            carry = b71[i] & 0x80;
        }
        return true;
    }

    public static string Format(ReadOnlySpan<byte> payload)
    {
        if (payload.Length < 10)
            return "";
        var i3 = (payload[9] >> 3) & 0x07;
        if (i3 == 0)
        {
            var n3 = ((payload[8] & 1) << 2) | (payload[9] >> 6);
            if (n3 == 0)
                return FreeText(payload);
            if (n3 == 5)
                return "TEL " + Telemetry(payload);
            return "0." + n3.ToString() + " " + Hex(payload);
        }
        if (i3 == 1 || i3 == 2)
            return StandardText(payload, i3);
        if (i3 == 4)
            return NonstandardText(payload);
        return "i3=" + i3.ToString() + " " + Hex(payload);
    }

    public static bool TryPack28(string call, out uint n28, out int ip)
    {
        n28 = 0;
        ip = 0;
        call = Normalize(call);
        if (call == "DE") { n28 = 0; return true; }
        if (call == "QRZ") { n28 = 1; return true; }
        if (call == "CQ") { n28 = 2; return true; }
        if (call.StartsWith("CQ ", StringComparison.Ordinal) && call.Length < 8)
        {
            if (!TryCqModifier(call, out var modifier))
                return false;
            n28 = (uint)(3 + modifier);
            return true;
        }

        var length = call.Length;
        if (call.EndsWith("/P", StringComparison.Ordinal) || call.EndsWith("/R", StringComparison.Ordinal))
        {
            ip = 1;
            length -= 2;
        }
        var packed = PackBase(call, length);
        if (packed < 0)
            return false;
        n28 = NTokens + Max22 + (uint)packed;
        return true;
    }

    static string StandardText(ReadOnlySpan<byte> payload, int i3)
    {
        var n29a = ((uint)payload[0] << 21) | ((uint)payload[1] << 13) | ((uint)payload[2] << 5) | ((uint)payload[3] >> 3);
        var n29b = ((uint)(payload[3] & 7) << 26) | ((uint)payload[4] << 18) | ((uint)payload[5] << 10) |
                   ((uint)payload[6] << 2) | ((uint)payload[7] >> 6);
        var ir = (payload[7] >> 5) & 1;
        var grid = ((payload[7] & 0x1F) << 10) | (payload[8] << 2) | (payload[9] >> 6);
        var to = Unpack28(n29a >> 1, (int)(n29a & 1), i3);
        var de = Unpack28(n29b >> 1, (int)(n29b & 1), i3);
        return Join(to, de, UnpackField(grid, ir));
    }

    static string NonstandardText(ReadOnlySpan<byte> payload)
    {
        var n12 = (payload[0] << 4) | (payload[1] >> 4);
        ulong n58 = ((ulong)(payload[1] & 0x0F) << 54) |
                    ((ulong)payload[2] << 46) |
                    ((ulong)payload[3] << 38) |
                    ((ulong)payload[4] << 30) |
                    ((ulong)payload[5] << 22) |
                    ((ulong)payload[6] << 14) |
                    ((ulong)payload[7] << 6) |
                    ((ulong)payload[8] >> 2);
        var flip = (payload[8] >> 1) & 1;
        var report = ((payload[8] & 1) << 1) | (payload[9] >> 7);
        var icq = (payload[9] >> 6) & 1;
        var plain = Unpack58(n58);
        var hashed = "<" + n12.ToString("X") + ">";
        var first = flip != 0 ? plain : hashed;
        var second = flip != 0 ? hashed : plain;
        if (icq != 0)
            return Join("CQ", second, "");
        var extra = report switch
        {
            1 => "RRR",
            2 => "RR73",
            3 => "73",
            _ => ""
        };
        return Join(first, second, extra);
    }

    static string Join(string a, string b, string c)
    {
        var text = a;
        if (b.Length > 0)
            text = text.Length == 0 ? b : text + " " + b;
        if (c.Length > 0)
            text = text.Length == 0 ? c : text + " " + c;
        return text;
    }

    static string Unpack28(uint n28, int ip, int i3)
    {
        if (n28 < NTokens)
        {
            if (n28 == 0) return "DE";
            if (n28 == 1) return "QRZ";
            if (n28 == 2) return "CQ";
            if (n28 <= 1002)
                return "CQ " + (n28 - 3).ToString("000");
            if (n28 <= 532443)
            {
                var n = n28 - 1003;
                var chars = new char[4];
                for (var i = 3; i >= 0; i--)
                {
                    chars[i] = LetterSpace((int)(n % 27));
                    n /= 27;
                }
                return "CQ " + new string(chars).TrimStart(' ');
            }
            return "<" + n28.ToString("X") + ">";
        }

        n28 -= NTokens;
        if (n28 < Max22)
            return "<" + n28.ToString("X") + ">";

        var value = n28 - Max22;
        var call = new char[6];
        call[5] = LetterSpace((int)(value % 27)); value /= 27;
        call[4] = LetterSpace((int)(value % 27)); value /= 27;
        call[3] = LetterSpace((int)(value % 27)); value /= 27;
        call[2] = (char)('0' + (int)(value % 10)); value /= 10;
        call[1] = Alphanum((int)(value % 36)); value /= 36;
        call[0] = AlphanumSpace((int)(value % 37));
        var text = ApplyPrefix(new string(call));
        if (ip != 0)
            text += i3 == 2 ? "/P" : "/R";
        return text;
    }

    static string ApplyPrefix(string call)
    {
        var trimmed = call.Trim();
        if (call.StartsWith("3D0", StringComparison.Ordinal) && call.Length > 3 && call[3] != ' ')
            return "3DA0" + call.Substring(3).Trim();
        if (call.Length > 1 && call[0] == 'Q' && call[1] >= 'A' && call[1] <= 'Z')
            return "3X" + call.Substring(1).Trim();
        return trimmed;
    }

    static string UnpackField(int grid, int ir)
    {
        if (grid <= MaxGrid4)
        {
            var n = grid;
            var chars = new char[4];
            chars[3] = (char)('0' + n % 10); n /= 10;
            chars[2] = (char)('0' + n % 10); n /= 10;
            chars[1] = (char)('A' + n % 18); n /= 18;
            chars[0] = (char)('A' + n % 18);
            var locator = new string(chars);
            return ir > 0 ? "R " + locator : locator;
        }
        var report = grid - MaxGrid4;
        return report switch
        {
            1 => "",
            2 => "RRR",
            3 => "RR73",
            4 => "73",
            _ => (ir > 0 ? "R" : "") + Signed(report - 35)
        };
    }

    static string Signed(int value)
    {
        var sign = value < 0 ? "-" : "+";
        var mag = Math.Abs(value);
        return sign + mag.ToString("00");
    }

    static string FreeText(ReadOnlySpan<byte> payload)
    {
        var b71 = ShiftRight(payload);
        var chars = new char[13];
        for (var idx = 12; idx >= 0; idx--)
        {
            var rem = 0;
            for (var i = 0; i < 9; i++)
            {
                rem = (rem << 8) | b71[i];
                b71[i] = (byte)(rem / 42);
                rem %= 42;
            }
            chars[idx] = FullChar(rem);
        }
        return new string(chars).Trim();
    }

    static string Telemetry(ReadOnlySpan<byte> payload)
    {
        var b71 = ShiftRight(payload);
        var chars = new char[18];
        for (var i = 0; i < 9; i++)
        {
            chars[i * 2] = HexDigit(b71[i] >> 4);
            chars[i * 2 + 1] = HexDigit(b71[i] & 0x0F);
        }
        return new string(chars);
    }

    static byte[] ShiftRight(ReadOnlySpan<byte> payload)
    {
        var b71 = new byte[9];
        var carry = 0;
        for (var i = 0; i < 9; i++)
        {
            b71[i] = (byte)((carry << 7) | (payload[i] >> 1));
            carry = payload[i] & 1;
        }
        return b71;
    }

    static string Hex(ReadOnlySpan<byte> payload)
    {
        var chars = new char[20];
        for (var i = 0; i < 10; i++)
        {
            chars[i * 2] = HexDigit(payload[i] >> 4);
            chars[i * 2 + 1] = HexDigit(payload[i] & 0x0F);
        }
        return new string(chars);
    }

    static char HexDigit(int n) => (char)(n < 10 ? '0' + n : 'A' + n - 10);

    static bool TryPackField(string extra, out int ir, out int grid15)
    {
        ir = 0;
        grid15 = MaxGrid4 + 1;
        if (extra.Length == 0)
            return true;
        if (extra == "RRR") { grid15 = MaxGrid4 + 2; return true; }
        if (extra == "RR73") { grid15 = MaxGrid4 + 3; return true; }
        if (extra == "73") { grid15 = MaxGrid4 + 4; return true; }
        if (extra.StartsWith("R ", StringComparison.Ordinal) && IsGrid(extra.AsSpan(2)))
        {
            ir = 1;
            grid15 = PackGrid(extra.Substring(2));
            return true;
        }
        if (IsGrid(extra))
        {
            grid15 = PackGrid(extra);
            return true;
        }

        var report = extra;
        if (report.StartsWith("R", StringComparison.Ordinal))
        {
            ir = 1;
            report = report.Substring(1);
        }
        if (!TryReport(report, out var dd))
            return false;
        grid15 = MaxGrid4 + 35 + dd;
        return true;
    }

    static int PackGrid(string grid)
    {
        var value = (grid[0] - 'A') * 18 + (grid[1] - 'A');
        value = value * 10 + (grid[2] - '0');
        value = value * 10 + (grid[3] - '0');
        return value;
    }

    static bool IsGrid(ReadOnlySpan<char> grid)
    {
        return grid.Length == 4 &&
               grid[0] >= 'A' && grid[0] <= 'R' &&
               grid[1] >= 'A' && grid[1] <= 'R' &&
               grid[2] >= '0' && grid[2] <= '9' &&
               grid[3] >= '0' && grid[3] <= '9';
    }

    static bool TryReport(string text, out int value)
    {
        value = 0;
        if (text.Length == 0)
            return false;
        var i = 0;
        var sign = 1;
        if (text[0] == '-') { sign = -1; i++; }
        else if (text[0] == '+') i++;
        if (i >= text.Length)
            return false;
        var mag = 0;
        for (; i < text.Length; i++)
        {
            if (text[i] < '0' || text[i] > '9')
                return false;
            mag = mag * 10 + (text[i] - '0');
        }
        value = sign * mag;
        return value >= -30 && value <= 60;
    }

    static bool TryCqModifier(string call, out int modifier)
    {
        modifier = 0;
        var digits = 0;
        var letters = 0;
        var m = 0;
        for (var i = 3; i < call.Length; i++)
        {
            var c = call[i];
            if (c >= '0' && c <= '9') digits++;
            else if (c >= 'A' && c <= 'Z')
            {
                letters++;
                m = 27 * m + (c - 'A' + 1);
            }
            else return false;
        }
        if (digits == 3 && letters == 0)
        {
            modifier = int.Parse(call.Substring(3));
            return true;
        }
        if (digits == 0 && letters is >= 1 and <= 4)
        {
            modifier = 1000 + m;
            return true;
        }
        return false;
    }

    static int PackBase(string call, int length)
    {
        if (length <= 2 || length > call.Length)
            return -1;
        var c6 = new char[] { ' ', ' ', ' ', ' ', ' ', ' ' };
        if (length > 4 && length <= 7 && call.StartsWith("3DA0", StringComparison.Ordinal))
        {
            c6[0] = '3'; c6[1] = 'D'; c6[2] = '0';
            for (var i = 4; i < length && i - 1 < 6; i++)
                c6[i - 1] = call[i];
        }
        else if (length <= 7 && call.StartsWith("3X", StringComparison.Ordinal) && length >= 3 && call[2] >= 'A' && call[2] <= 'Z')
        {
            c6[0] = 'Q';
            for (var i = 2; i < length && i - 1 < 6; i++)
                c6[i - 1] = call[i];
        }
        else if (length >= 3 && call[2] >= '0' && call[2] <= '9' && length <= 6)
        {
            for (var i = 0; i < length; i++)
                c6[i] = call[i];
        }
        else if (length >= 2 && call[1] >= '0' && call[1] <= '9' && length <= 5)
        {
            for (var i = 0; i < length; i++)
                c6[i + 1] = call[i];
        }
        else
        {
            return -1;
        }

        var i0 = PrefixIndex(c6[0]);
        var i1 = AlphanumIndex(c6[1]);
        var i2 = c6[2] >= '0' && c6[2] <= '9' ? c6[2] - '0' : -1;
        var i3 = LetterIndex(c6[3]);
        var i4 = LetterIndex(c6[4]);
        var i5 = LetterIndex(c6[5]);
        if (i0 < 0 || i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 || i5 < 0)
            return -1;
        var n = i0;
        n = n * 36 + i1;
        n = n * 10 + i2;
        n = n * 27 + i3;
        n = n * 27 + i4;
        n = n * 27 + i5;
        return n;
    }

    static int PrefixIndex(char c)
    {
        if (c == ' ') return 0;
        if (c >= '0' && c <= '9') return 1 + c - '0';
        if (c >= 'A' && c <= 'Z') return 11 + c - 'A';
        return -1;
    }

    static int AlphanumIndex(char c)
    {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'Z') return 10 + c - 'A';
        return -1;
    }

    static int LetterIndex(char c)
    {
        if (c == ' ') return 0;
        if (c >= 'A' && c <= 'Z') return 1 + c - 'A';
        return -1;
    }

    static char AlphanumSpace(int n)
    {
        if (n <= 0) return ' ';
        if (n <= 10) return (char)('0' + n - 1);
        if (n <= 36) return (char)('A' + n - 11);
        return ' ';
    }

    static char Alphanum(int n)
    {
        if (n < 10) return (char)('0' + n);
        if (n < 36) return (char)('A' + n - 10);
        return '_';
    }

    static char LetterSpace(int n) => n <= 0 ? ' ' : (char)('A' + n - 1);

    static int FullIndex(char c)
    {
        if (c == ' ') return 0;
        if (c >= '0' && c <= '9') return 1 + c - '0';
        if (c >= 'A' && c <= 'Z') return 11 + c - 'A';
        return c switch { '+' => 37, '-' => 38, '.' => 39, '/' => 40, '?' => 41, _ => -1 };
    }

    static char FullChar(int n)
    {
        if (n <= 0) return ' ';
        if (n <= 10) return (char)('0' + n - 1);
        if (n <= 36) return (char)('A' + n - 11);
        return n switch { 37 => '+', 38 => '-', 39 => '.', 40 => '/', 41 => '?', _ => '_' };
    }

    static string Unpack58(ulong n)
    {
        var chars = new char[11];
        for (var i = 10; i >= 0; i--)
        {
            chars[i] = SlashChar((int)(n % 38));
            n /= 38;
        }
        return new string(chars).Trim();
    }

    static char SlashChar(int n)
    {
        if (n <= 0) return ' ';
        if (n <= 10) return (char)('0' + n - 1);
        if (n <= 36) return (char)('A' + n - 11);
        return n == 37 ? '/' : '_';
    }

    static string Normalize(string text) => (text ?? "").Trim().ToUpperInvariant();
}
