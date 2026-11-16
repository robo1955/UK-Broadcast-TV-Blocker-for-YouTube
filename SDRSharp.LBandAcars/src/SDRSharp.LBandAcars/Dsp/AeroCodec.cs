using System.Text;

namespace SDRSharp.LBandAcars.Dsp;

/// <summary>
/// Classic Aero P-channel constants. 600 and 1200 bit/s frames are 1 s and 2 s
/// of A-BPSK; the 10500 bit/s frame is 0.5 s of OQPSK. Figures follow the
/// Inmarsat SDM P-channel (unique word, 16-bit header, rate-1/2 K=7 code,
/// 64-row interleaver, 15-bit scrambler, 12-byte signal units).
/// </summary>
public static class AeroLink
{
    public const uint UniqueWord = 3780831379u;
    public const int Rows = 64;

    public static bool IsOqpsk(int rate) => rate >= 8400;

    public static int Columns(int rate) => rate switch
    {
        600 => 6,
        1200 => 9,
        10500 => 78,
        _ => throw new ArgumentOutOfRangeException(nameof(rate))
    };

    public static int CodedBits(int rate) => rate == 10500 ? 4992 : 1152;

    public static int InfoBits(int rate) => CodedBits(rate) / 2;

    public static int HeaderBits(int rate) => 16;

    public static int DummyBits(int rate) => rate == 10500 ? 178 : 0;

    public static int UniqueWordBits(int rate) => IsOqpsk(rate) ? 64 : 32;

    public static int FrameBits(int rate) =>
        UniqueWordBits(rate) + HeaderBits(rate) + DummyBits(rate) + CodedBits(rate);

    public static int SignalUnits(int rate) => InfoBits(rate) / 96;
}

public static class AeroCrc
{
    public static ushort Compute(ReadOnlySpan<byte> data)
    {
        ushort crc = 0xFFFF;
        foreach (var b in data)
        {
            var message = (int)b;
            for (var k = 0; k < 8; k++)
            {
                var messageBit = message & 1;
                message >>= 1;
                var crcBit = crc & 1;
                crc >>= 1;
                if ((crcBit ^ messageBit) != 0)
                    crc ^= 0x8408;
            }
        }
        return (ushort)~crc;
    }

    public static byte[] WithCrc(ReadOnlySpan<byte> ten)
    {
        if (ten.Length != 10)
            throw new ArgumentException("A signal unit payload is 10 bytes.", nameof(ten));
        var su = new byte[12];
        ten.CopyTo(su);
        var crc = Compute(ten);
        su[10] = (byte)crc;
        su[11] = (byte)(crc >> 8);
        return su;
    }
}

/// <summary>15-stage Aero scrambler. The same operation scrambles and descrambles.</summary>
public sealed class AeroScrambler
{
    readonly byte[] _state = new byte[15];

    public AeroScrambler() => Reset();

    public void Reset()
    {
        ReadOnlySpan<byte> init = stackalloc byte[] { 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1 };
        init.CopyTo(_state);
    }

    public int Next()
    {
        var v = _state[0] ^ _state[14];
        for (var i = 14; i > 0; i--)
            _state[i] = _state[i - 1];
        _state[0] = (byte)v;
        return v;
    }
}

public static class AeroInterleaver
{
    static int RowOrigin(int row) => (row * 27) % AeroLink.Rows;

    public static void Interleave(ReadOnlySpan<int> coded, int columns, Span<int> dst)
    {
        for (var col = 0; col < columns; col++)
        {
            for (var row = 0; row < AeroLink.Rows; row++)
                dst[RowOrigin(row) * columns + col] = coded[col * AeroLink.Rows + row];
        }
    }

    public static void Deinterleave(ReadOnlySpan<int> block, int columns, Span<int> dst)
    {
        var k = 0;
        for (var col = 0; col < columns; col++)
        {
            for (var row = 0; row < AeroLink.Rows; row++)
                dst[k++] = block[RowOrigin(row) * columns + col];
        }
    }
}

/// <summary>
/// Rate-1/2, constraint-length 7 Viterbi decoder.
/// Polynomials 109 and 79 (0x6D, 0x4F): the same code as the CCSDS (171, 133)
/// octal pair in the bit order used on the Aero P-channel. Newest bit is the
/// register LSB. The first coded bit of each pair is polynomial 0x6D.
/// </summary>
public sealed class ViterbiK7
{
    public const int States = 128;
    public const int Traceback = 48;
    const int Hist = 64;

    readonly int[] _metric = new int[States];
    readonly int[] _next = new int[States];
    readonly byte[,] _from = new byte[Hist, States];
    readonly byte[,] _input = new byte[Hist, States];
    int _time;
    bool _fresh = true;

    public void Reset()
    {
        _time = 0;
        _fresh = true;
    }

    /// <summary>
    /// Feed one information step (two coded soft values). Positive soft means
    /// the coded bit is more likely a 1. Returns the information bit delayed
    /// by <see cref="Traceback"/> steps, or null until the delay is filled.
    /// </summary>
    public int? Step(int soft0, int soft1)
    {
        if (_fresh)
        {
            // Equal start metrics: the register state at acquisition is unknown.
            Array.Clear(_metric);
            _fresh = false;
        }

        for (var i = 0; i < States; i++)
            _next[i] = int.MinValue / 4;

        var slot = _time & (Hist - 1);
        for (var reg = 0; reg < States; reg++)
        {
            var baseMetric = _metric[reg];
            if (baseMetric < -50_000_000)
                continue;
            for (var bit = 0; bit < 2; bit++)
            {
                var nr = ((reg << 1) | bit) & (States - 1);
                var g0 = Parity(nr & 0x6D);
                var g1 = Parity(nr & 0x4F);
                var branch = (g0 == 1 ? soft0 : -soft0) + (g1 == 1 ? soft1 : -soft1);
                var total = baseMetric + branch;
                if (total > _next[nr])
                {
                    _next[nr] = total;
                    _from[slot, nr] = (byte)reg;
                    _input[slot, nr] = (byte)bit;
                }
            }
        }

        var bestState = 0;
        var best = _next[0];
        for (var i = 1; i < States; i++)
        {
            if (_next[i] > best)
            {
                best = _next[i];
                bestState = i;
            }
        }
        if (best > 1_000_000 || best < -1_000_000)
        {
            for (var i = 0; i < States; i++)
                _next[i] -= best;
        }
        Array.Copy(_next, _metric, States);
        _time++;
        if (_time < Traceback)
            return null;

        var state = bestState;
        var decoded = 0;
        for (var age = 0; age < Traceback; age++)
        {
            var when = (_time - 1 - age) & (Hist - 1);
            decoded = _input[when, state];
            state = _from[when, state];
        }
        return decoded;
    }

    public static void Encode(ReadOnlySpan<int> bits, Span<int> coded, ref int register)
    {
        var reg = register;
        var n = 0;
        foreach (var bit in bits)
        {
            reg = ((reg << 1) | (bit & 1)) & (States - 1);
            coded[n++] = Parity(reg & 0x6D);
            coded[n++] = Parity(reg & 0x4F);
        }
        register = reg;
    }

    public static int Parity(int x)
    {
        x ^= x >> 4;
        x &= 0x0F;
        return (0x6996 >> x) & 1;
    }
}

public sealed class DecodedMessage
{
    public DateTime Utc { get; set; }
    public int BitRate { get; set; }
    public string Kind { get; set; } = "";
    public string Aes { get; set; } = "";
    public string Ges { get; set; } = "";
    public string Registration { get; set; } = "";
    public string Label { get; set; } = "";
    public string Mode { get; set; } = "";
    public string BlockId { get; set; } = "";
    public string Text { get; set; } = "";
    public bool Continued { get; set; }

    public string Format()
    {
        var clock = Utc.ToString("HH:mm:ss");
        var head = string.IsNullOrEmpty(Registration)
            ? $"{clock}  {BitRate,5}  {Kind}"
            : $"{clock}  {BitRate,5}  {Registration}  {Label}  {Mode}{BlockId}";
        if (!string.IsNullOrEmpty(Aes))
            head += $"  AES {Aes} GES {Ges}";
        return string.IsNullOrEmpty(Text) ? head : head + "\r\n" + Text;
    }
}

public sealed class LinkStatus
{
    public bool Locked { get; init; }
    public int BitRate { get; init; }
    public double FrequencyOffsetHz { get; init; }
    public int UniqueWordErrors { get; init; }
    public int Frames { get; init; }
    public int CrcOk { get; init; }
    public int CrcBad { get; init; }
    public int FillUnits { get; init; }
    public double LevelDb { get; init; }
    public bool SamplesFlowing { get; init; }
    public string State { get; init; } = "idle";
}

/// <summary>P-channel unique-word sync, deinterleave, Viterbi, descramble, signal-unit parse.</summary>
public sealed class FrameDecoder
{
    readonly ViterbiK7 _viterbi = new();
    readonly Queue<int> _stepFrame = new();
    readonly Dictionary<int, List<int>> _frameBits = new();
    readonly AcarsAssembler _acars = new();

    int _rate = 1200;
    bool _oqpsk;
    int _period;
    int _headerBits;
    int _dummyBits;
    int _codedBits;
    int _infoBits;
    int _columns;
    int _uwBits;

    uint _mskShift;
    uint _evenShift;
    uint _oddShift;
    long _bitIndex;
    long _lastSync = -1_000_000;
    bool _haveSync;
    int _misses;
    int _collectLeft;
    int _header;
    int _headerGot;
    int _dummyLeft;
    int[] _coded = new int[4992];
    int _codedGot;
    bool _invMsk;
    bool _invEven;
    bool _invOdd;
    int _frameSerial;
    int _activeFrame = -1;

    public int Frames { get; private set; }
    public int CrcOk { get; private set; }
    public int CrcBad { get; private set; }
    public int FillUnits { get; private set; }
    public int LastUniqueWordErrors { get; private set; }
    public bool Locked { get; private set; }
    public int Rate => _rate;

    public event Action<DecodedMessage>? Message;

    public void SetRate(int rate)
    {
        _rate = rate;
        _oqpsk = AeroLink.IsOqpsk(rate);
        _period = AeroLink.FrameBits(rate);
        _headerBits = AeroLink.HeaderBits(rate);
        _dummyBits = AeroLink.DummyBits(rate);
        _codedBits = AeroLink.CodedBits(rate);
        _infoBits = AeroLink.InfoBits(rate);
        _columns = AeroLink.Columns(rate);
        _uwBits = AeroLink.UniqueWordBits(rate);
        Reset();
    }

    public void Reset()
    {
        _viterbi.Reset();
        _stepFrame.Clear();
        _frameBits.Clear();
        _acars.Reset();
        _mskShift = 0;
        _evenShift = 0;
        _oddShift = 0;
        _bitIndex = 0;
        _lastSync = -1_000_000;
        _haveSync = false;
        _misses = 0;
        _collectLeft = 0;
        _header = 0;
        _headerGot = 0;
        _dummyLeft = 0;
        _codedGot = 0;
        _activeFrame = -1;
        Locked = false;
        LastUniqueWordErrors = 32;
    }

    public void PushSoft(int soft)
    {
        if (soft > 127) soft = 127;
        if (soft < -128) soft = -128;
        var hard = soft >= 0 ? 1 : 0;
        var index = _bitIndex++;

        var sync = false;
        var errors = 32;
        var invMsk = false;
        var invEven = false;
        var invOdd = false;

        if (_oqpsk)
        {
            if ((index & 1) == 0)
            {
                _evenShift = (_evenShift << 1) | (uint)hard;
            }
            else
            {
                _oddShift = (_oddShift << 1) | (uint)hard;
                if (index + 1 < _uwBits)
                    return;
                var evenDist = Hamming(_evenShift ^ AeroLink.UniqueWord);
                var evenInv = Hamming(_evenShift ^ ~AeroLink.UniqueWord);
                var oddDist = Hamming(_oddShift ^ AeroLink.UniqueWord);
                var oddInv = Hamming(_oddShift ^ ~AeroLink.UniqueWord);
                invEven = evenInv < evenDist;
                invOdd = oddInv < oddDist;
                var e = invEven ? evenInv : evenDist;
                var o = invOdd ? oddInv : oddDist;
                errors = Math.Max(e, o);
                sync = e <= 2 && o <= 2;
            }
        }
        else if (index >= 31)
        {
            _mskShift = (_mskShift << 1) | (uint)hard;
            var dist = Hamming(_mskShift ^ AeroLink.UniqueWord);
            var inv = Hamming(_mskShift ^ ~AeroLink.UniqueWord);
            invMsk = inv < dist;
            errors = invMsk ? inv : dist;
            sync = errors <= 2;
        }
        else
        {
            _mskShift = (_mskShift << 1) | (uint)hard;
        }

        if (_haveSync && index - _lastSync > _period + 8)
        {
            _misses++;
            _collectLeft = 0;
            _activeFrame = -1;
            if (_misses >= 2)
            {
                _haveSync = false;
                Locked = false;
                _viterbi.Reset();
                _stepFrame.Clear();
                _frameBits.Clear();
            }
        }

        if (sync && Accept(index))
        {
            LastUniqueWordErrors = errors;
            _lastSync = index;
            _haveSync = true;
            _misses = 0;
            _invMsk = invMsk;
            _invEven = invEven;
            _invOdd = invOdd;
            _collectLeft = _headerBits + _dummyBits + _codedBits;
            _header = 0;
            _headerGot = 0;
            _dummyLeft = 0;
            _codedGot = 0;
            _activeFrame = ++_frameSerial;
            _frameBits[_activeFrame] = new List<int>(_infoBits);
        }
        else if (_collectLeft > 0 && _activeFrame >= 0)
        {
            var sample = ApplyInversion(index, soft);
            Take(sample);
        }
    }

    bool Accept(long index)
    {
        if (!_haveSync)
            return true;
        var gap = index - _lastSync;
        return Math.Abs(gap - _period) <= 6;
    }

    int ApplyInversion(long index, int soft)
    {
        if (_oqpsk)
        {
            var invert = (index & 1) == 0 ? _invEven : _invOdd;
            return invert ? -soft : soft;
        }
        return _invMsk ? -soft : soft;
    }

    void Take(int soft)
    {
        _collectLeft--;
        if (_headerGot < _headerBits)
        {
            _header = (_header << 1) | (soft >= 0 ? 1 : 0);
            _headerGot++;
            if (_headerGot == _headerBits)
                _dummyLeft = _dummyBits;
            return;
        }
        if (_dummyLeft > 0)
        {
            _dummyLeft--;
            return;
        }
        _coded[_codedGot++] = soft;
        if (_codedGot == _codedBits)
        {
            var frame = _activeFrame;
            _activeFrame = -1;
            FeedCoded(frame, _coded.AsSpan(0, _codedBits));
        }
    }

    void FeedCoded(int frame, ReadOnlySpan<int> coded)
    {
        var block = AeroLink.Rows * _columns;
        var decoded = new int[block];
        for (var offset = 0; offset < coded.Length; offset += block)
        {
            AeroInterleaver.Deinterleave(coded.Slice(offset, block), _columns, decoded);
            for (var i = 0; i < decoded.Length; i += 2)
                Absorb(frame, decoded[i], decoded[i + 1]);
        }
    }

    void Absorb(int frame, int soft0, int soft1)
    {
        _stepFrame.Enqueue(frame);
        var bit = _viterbi.Step(soft0, soft1);
        if (bit == null)
            return;
        var id = _stepFrame.Dequeue();
        if (!_frameBits.TryGetValue(id, out var list))
            return;
        list.Add(bit.Value);
        if (list.Count == _infoBits)
        {
            _frameBits.Remove(id);
            Finish(list);
        }
    }

    void Finish(List<int> bits)
    {
        var scrambler = new AeroScrambler();
        var bytes = bits.Count / 8;
        var field = new byte[bytes];
        var acc = 0;
        var n = 0;
        for (var i = 0; i < bits.Count; i++)
        {
            var bit = bits[i] ^ scrambler.Next();
            acc |= bit << n;
            n++;
            if (n == 8)
            {
                field[i / 8] = (byte)acc;
                acc = 0;
                n = 0;
            }
        }

        Frames++;
        var units = field.Length / 12;
        var any = false;
        for (var u = 0; u < units; u++)
        {
            var su = field.AsSpan(u * 12, 12);
            var calc = AeroCrc.Compute(su[..10]);
            var rec = (ushort)(su[10] | (su[11] << 8));
            var sum = 0;
            for (var i = 0; i < 10; i++)
                sum += su[i];
            if (rec == 0 && calc != 0 && sum == 0)
                calc = 0;
            if (calc != rec)
            {
                CrcBad++;
                continue;
            }
            CrcOk++;
            any = true;
            Dispatch(su[..12].ToArray());
        }
        Locked = any || CrcOk > 0;
    }

    void Dispatch(byte[] su)
    {
        var type = su[0];
        if (type == 0x01 || type == 0x00)
        {
            FillUnits++;
            return;
        }
        if (type == 0x71 || (type & 0xC0) == 0xC0)
        {
            if (_acars.Push(su, out var message) && message.Kind.Length > 0)
                Publish(message);
            return;
        }
        Publish(Describe(su));
    }

    void Publish(DecodedMessage message)
    {
        message.BitRate = _rate;
        Message?.Invoke(message);
    }

    DecodedMessage Describe(byte[] su)
    {
        var type = su[0];
        var text = TypeName(type);
        var aes = "";
        var ges = "";
        switch (type)
        {
            case 0x05:
                text = ChannelTable(su);
                ges = su[3].ToString("X2");
                break;
            case 0x0C:
                text = SatelliteId(su);
                break;
            case 0x11:
            case 0x15:
                aes = Aes(su, 1);
                ges = su[4].ToString("X2");
                text = type == 0x11 ? "Log-on confirm" : "Log-on/off acknowledge";
                break;
            case 0x31:
            case 0x32:
            case 0x33:
            case 0x34:
            case 0x21:
                aes = Aes(su, 1);
                ges = su[4].ToString("X2");
                text = CChannel(su, type);
                break;
            case 0x40:
                ges = su[4].ToString("X2");
                text = DataChannel(su);
                break;
        }
        return new DecodedMessage
        {
            Utc = DateTime.UtcNow,
            Kind = text,
            Aes = aes,
            Ges = ges,
            Text = Hex(su, 10)
        };
    }

    static string Aes(byte[] su, int offset) =>
        ((su[offset] << 16) | (su[offset + 1] << 8) | su[offset + 2]).ToString("X6");

    static string Hex(byte[] su, int n)
    {
        var sb = new StringBuilder(n * 3);
        for (var i = 0; i < n; i++)
        {
            if (i > 0) sb.Append(' ');
            sb.Append(su[i].ToString("X2"));
        }
        return sb.ToString();
    }

    static string TypeName(byte type) => type switch
    {
        0x05 => "System table, P/R channels",
        0x07 => "System table, beam support",
        0x0A => "System table index",
        0x0C => "Satellite identification",
        0x10 => "Log-on request",
        0x11 => "Log-on confirm",
        0x12 => "Log-off request",
        0x13 => "Log-on reject",
        0x14 => "Log-on interrogation",
        0x15 => "Log-on/off acknowledge",
        0x16 => "Log-on prompt",
        0x17 => "Data channel reassignment",
        0x21 => "Call announcement",
        0x28 => "Data EIRP table",
        0x30 => "Call progress",
        0x31 => "C-channel assignment, distress",
        0x32 => "C-channel assignment, flight safety",
        0x33 => "C-channel assignment, other safety",
        0x34 => "C-channel assignment",
        0x40 => "P/R channel control",
        0x41 => "T channel control",
        0x51 => "T channel assignment",
        0x61 => "Request for acknowledgement",
        0x62 => "Acknowledge",
        0x71 => "User data",
        _ => $"Signal unit 0x{type:X2}"
    };

    static string ChannelTable(byte[] su)
    {
        var seq = (su[2] >> 2) & 0x3F;
        var lsu = su[2] & 0x03;
        var c1 = (su[4] << 8) | su[5];
        var c2 = (su[6] << 8) | su[7];
        var c3 = (su[8] << 8) | su[9];
        double F(int c) => 1510.0 + c * 0.0025 + (lsu >= 2 ? 101.5 : 0);
        var f1 = lsu <= 1 ? 1510.0 + c1 * 0.0025 : F(c1);
        var f2 = F(c2) + (lsu <= 1 ? 101.5 : 0);
        var f3 = F(c3) + (lsu <= 1 ? 101.5 : 0);
        if (lsu <= 1)
            return $"GES {su[3]:X2} seq {seq}  Psmc {f1:0.0000}  Rsmc {f2:0.0000} / {f3:0.0000} MHz";
        return $"GES {su[3]:X2} seq {seq}  Rsmc {f1:0.0000} / {f2:0.0000} / {f3:0.0000} MHz";
    }

    static string SatelliteId(byte[] su)
    {
        var sat = ((su[2] & 0x03) << 4) | (su[3] >> 4);
        var lon = su[5] * 1.5;
        var lonText = lon > 180 ? $"{360 - lon:0.#}W" : $"{lon:0.#}E";
        var c1 = ((su[6] & 0x7F) << 8) | su[7];
        var c2 = ((su[8] & 0x7F) << 8) | su[9];
        var f1 = 1510.0 + c1 * 0.0025;
        var spot1 = (su[6] & 0x80) != 0 ? " spot" : "";
        if (c2 == 0)
            return $"Satellite {sat} {lonText}  Psmc {f1:0.0000} MHz{spot1}";
        var f2 = 1510.0 + c2 * 0.0025;
        var spot2 = (su[8] & 0x80) != 0 ? " spot" : "";
        return $"Satellite {sat} {lonText}  Psmc {f1:0.0000}{spot1} / {f2:0.0000} MHz{spot2}";
    }

    static string DataChannel(byte[] su)
    {
        var rateCode = (su[7] >> 4) & 0x0F;
        var rate = rateCode switch
        {
            0 => 600,
            1 => 1200,
            2 => 2400,
            3 => 4800,
            4 => 6000,
            5 => 5250,
            6 => 10500,
            7 => 8400,
            9 => 21000,
            _ => 0
        };
        var channel = ((su[8] & 0x7F) << 8) | su[9];
        var mhz = 1510.0 + channel * 0.0025;
        var spot = (su[8] & 0x80) != 0 ? " spot beam" : "";
        return $"GES {su[4]:X2}  Pd {mhz:0.000} MHz at {rate} bit/s{spot}";
    }

    static string CChannel(byte[] su, byte type)
    {
        var rx = ((su[6] & 0x7F) << 8) | su[7];
        var tx = ((su[8] & 0x7F) << 8) | su[9];
        var rxF = 1510.0 + rx * 0.0025;
        var txF = 1611.5 + tx * 0.0025;
        return $"{TypeName(type)}  RX {rxF:0.0000}  TX {txF:0.0000} MHz";
    }

    static int Hamming(uint x)
    {
        x -= (x >> 1) & 0x55555555u;
        x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
        x = (x + (x >> 4)) & 0x0F0F0F0Fu;
        x += x >> 8;
        x += x >> 16;
        return (int)(x & 0x3F);
    }
}

/// <summary>Joins a 0x71 initial signal unit with its 0xC0 subsequent units, then reads ACARS.</summary>
public sealed class AcarsAssembler
{
    sealed class Partial
    {
        public int Aes;
        public byte Ges;
        public int Qno;
        public int Ref;
        public int SeqLeft;
        public int LastOctets;
        public readonly List<byte> Data = new();
        public int Age;
    }

    readonly List<Partial> _open = new();
    readonly List<Fragment> _fragments = new();

    sealed class Fragment
    {
        public string Reg = "";
        public string Label = "";
        public string Mode = "";
        public int Aes;
        public byte Ges;
        public string Block = "";
        public string Text = "";
        public bool More;
        public int Age;
    }

    public void Reset()
    {
        _open.Clear();
        _fragments.Clear();
    }

    public bool Push(byte[] su, out DecodedMessage message)
    {
        message = new DecodedMessage();
        AgeOut();
        if (su[0] == 0x71)
        {
            var item = new Partial
            {
                Aes = (su[1] << 16) | (su[2] << 8) | su[3],
                Ges = su[4],
                Qno = (su[5] >> 4) & 0x0F,
                Ref = su[5] & 0x0F,
                SeqLeft = su[6] & 0x3F,
                LastOctets = (su[7] >> 4) & 0x0F
            };
            item.Data.Add(su[8]);
            item.Data.Add(su[9]);
            var idx = Find(item.Aes, item.Ges, item.Qno, item.Ref);
            if (idx >= 0) _open[idx] = item;
            else _open.Add(item);
            return false;
        }
        if ((su[0] & 0xC0) != 0xC0)
            return false;

        var seq = su[0] & 0x3F;
        var qno = (su[1] >> 4) & 0x0F;
        var rf = su[1] & 0x0F;
        Partial? owner = null;
        foreach (var item in _open)
        {
            if (item.Qno == qno && item.Ref == rf && item.SeqLeft == seq + 1)
            {
                owner = item;
                break;
            }
        }
        if (owner == null)
            return false;
        owner.SeqLeft--;
        owner.Age = 0;
        if (owner.SeqLeft == 0)
        {
            var n = Math.Clamp(owner.LastOctets, 0, 8);
            for (var i = 0; i < n; i++)
                owner.Data.Add(su[2 + i]);
            _open.Remove(owner);
            message = Parse(owner);
            return true;
        }
        for (var i = 2; i <= 9; i++)
            owner.Data.Add(su[i]);
        return false;
    }

    int Find(int aes, byte ges, int qno, int rf)
    {
        for (var i = 0; i < _open.Count; i++)
        {
            var item = _open[i];
            if (item.Aes == aes && item.Ges == ges && item.Qno == qno && item.Ref == rf)
                return i;
        }
        return -1;
    }

    void AgeOut()
    {
        for (var i = _open.Count - 1; i >= 0; i--)
        {
            if (++_open[i].Age > 30)
                _open.RemoveAt(i);
        }
        for (var i = _fragments.Count - 1; i >= 0; i--)
        {
            if (++_fragments[i].Age > 30)
                _fragments.RemoveAt(i);
        }
    }

    DecodedMessage Parse(Partial item)
    {
        var data = item.Data;
        var acars = LooksLikeAcars(data);
        var reg = "";
        var label = "";
        var mode = "";
        var block = "";
        var text = "";
        var more = false;
        if (acars)
        {
            mode = Char7(data[3]).ToString();
            var regSb = new StringBuilder();
            for (var i = 4; i <= 10 && i < data.Count; i++)
                regSb.Append(Char7(data[i]));
            reg = regSb.ToString().Trim(' ', '.');
            if (data.Count > 14)
            {
                label = new string(new[] { Char7(data[12]), Char7(data[13]) });
                block = Char7(data[14]).ToString();
            }
            var hasText = data.Count > 15 && (data[15] & 0x7F) == 0x02;
            if (hasText && data.Count > 20)
            {
                var end = data.Count - 4;
                if (end > 16 && (data[end] & 0x7F) == 0x17)
                    more = true;
                var sb = new StringBuilder();
                for (var i = 16; i < end; i++)
                {
                    var c = Char7(data[i]);
                    if (c == 0x7F) sb.Append("<DEL>");
                    else if (c is >= (char)32 or '\r' or '\n') sb.Append(c);
                    else sb.Append('.');
                }
                text = sb.ToString();
            }
            if (!JoinFragments(item, reg, label, mode, block, ref text, more))
                return new DecodedMessage { Kind = "" };
        }
        else
        {
            text = BitConverter.ToString(data.ToArray());
        }

        return new DecodedMessage
        {
            Utc = DateTime.UtcNow,
            Kind = acars ? "ACARS" : "User data",
            Aes = item.Aes.ToString("X6"),
            Ges = item.Ges.ToString("X2"),
            Registration = reg,
            Label = label,
            Mode = mode,
            BlockId = block,
            Text = text,
            Continued = more
        };
    }

    bool JoinFragments(Partial item, string reg, string label, string mode, string block, ref string text, bool more)
    {
        Fragment? prev = null;
        foreach (var frag in _fragments)
        {
            if (frag.More && frag.Aes == item.Aes && frag.Ges == item.Ges &&
                frag.Reg == reg && frag.Label == label && frag.Mode == mode &&
                block.Length == 1 && frag.Block.Length == 1 &&
                NextBlock(frag.Block[0]) == block[0])
            {
                prev = frag;
                break;
            }
        }
        if (prev == null)
        {
            if (!more)
                return true;
            _fragments.Add(new Fragment
            {
                Reg = reg, Label = label, Mode = mode, Aes = item.Aes, Ges = item.Ges,
                Block = block, Text = text, More = true
            });
            return false;
        }
        prev.Text += text;
        prev.Block = block;
        prev.More = more;
        prev.Age = 0;
        if (more)
            return false;
        text = prev.Text;
        _fragments.Remove(prev);
        return true;
    }

    static char NextBlock(char b) => (char)('A' + ((b - 'A' + 1) % 26));

    static bool LooksLikeAcars(List<byte> data)
    {
        return data.Count > 16 && data[0] == 0xFF && data[1] == 0xFF &&
               ((data[15] & 0x7F) == 0x02 || (data[15] & 0x7F) == 0x03);
    }

    static char Char7(byte b) => (char)(b & 0x7F);
}
