using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using SDRSharp.Common;
using SDRSharp.LBandAcars.Dsp;
using SDRSharp.PanView;
using SDRSharp.Radio;

namespace SDRSharp.LBandAcars;

public sealed class AcarsPanel : UserControl
{
    const int LogLineCap = 500;

    readonly ISharpControl _control;
    readonly IqProcessor _processor;
    readonly DoubleBufferedComboBox _rate = new();
    readonly CheckBox _decode = new();
    readonly DoubleBufferedHintButton _arm = new();
    readonly DoubleBufferedNumericUpDown _offset = new();
    readonly Label _status = new();
    readonly DoubleBufferedTextBox _log = new();
    readonly DoubleBufferedHintButton _clear = new();
    readonly CheckBox _save = new();
    readonly System.Windows.Forms.Timer _timer = new();
    StreamWriter? _file;
    string _fileDay = "";
    int _lines;

    public AcarsPanel(ISharpControl control, IqProcessor processor)
    {
        _control = control;
        _processor = processor;
        _processor.Message += AppendMessage;
        Build();
        _control.PropertyChanged += OnThemeChanged;
        ApplyTheme();
        _timer.Interval = 250;
        _timer.Tick += (_, _) => RefreshStatus();
        _timer.Start();
        RefreshStatus();
    }

    public bool Decoding => _decode.Checked;

    public void Shutdown()
    {
        _timer.Stop();
        _control.PropertyChanged -= OnThemeChanged;
        _processor.Message -= AppendMessage;
        _processor.Enabled = false;
        CloseFile();
    }

    void Build()
    {
        MinimumSize = new Size(280, 320);
        Font = new Font("Segoe UI", 8.25f);

        var layout = new DoubleBufferedTableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 6,
            Padding = new Padding(6)
        };
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 92));
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        _rate.DropDownStyle = ComboBoxStyle.DropDownList;
        _rate.Items.AddRange(new object[] { "600 bit/s", "1200 bit/s", "10500 bit/s" });
        _rate.SelectedIndex = 1;
        _rate.Dock = DockStyle.Fill;
        _rate.SelectedIndexChanged += (_, _) => _processor.SetBitRate(SelectedRate());

        _decode.Text = "Decode";
        _decode.AutoSize = true;
        _decode.FlatStyle = FlatStyle.Flat;
        _decode.Margin = new Padding(0, 6, 0, 0);
        _decode.CheckedChanged += (_, _) => _processor.Enabled = _decode.Checked;

        _arm.Text = "Arm receiver";
        _arm.AutoSize = true;
        _arm.Dock = DockStyle.Fill;
        _arm.Click += (_, _) => Arm();

        _offset.Minimum = -8000;
        _offset.Maximum = 8000;
        _offset.Increment = 10;
        _offset.DecimalPlaces = 0;
        _offset.Dock = DockStyle.Fill;
        _offset.ValueChanged += (_, _) => _processor.SetManualOffset((double)_offset.Value);

        _status.AutoSize = false;
        _status.Dock = DockStyle.Fill;
        _status.TextAlign = ContentAlignment.TopLeft;

        _log.Multiline = true;
        _log.ReadOnly = true;
        _log.ScrollBars = ScrollBars.Both;
        _log.WordWrap = false;
        _log.Dock = DockStyle.Fill;
        _log.Font = new Font("Consolas", 8.25f);
        _log.HideSelection = false;

        _clear.Text = "Clear";
        _clear.AutoSize = true;
        _clear.Click += (_, _) =>
        {
            _log.Clear();
            _lines = 0;
        };

        _save.Text = "Log to file";
        _save.AutoSize = true;
        _save.FlatStyle = FlatStyle.Flat;
        _save.Margin = new Padding(0, 6, 0, 0);
        _save.CheckedChanged += (_, _) =>
        {
            if (!_save.Checked)
                CloseFile();
        };

        layout.Controls.Add(LabelOf("Rate"), 0, 0);
        layout.Controls.Add(_rate, 1, 0);
        layout.Controls.Add(_decode, 0, 1);
        layout.Controls.Add(_arm, 1, 1);
        layout.Controls.Add(LabelOf("Offset Hz"), 0, 2);
        layout.Controls.Add(_offset, 1, 2);
        layout.Controls.Add(_status, 0, 3);
        layout.SetColumnSpan(_status, 2);
        layout.Controls.Add(_log, 0, 4);
        layout.SetColumnSpan(_log, 2);
        layout.Controls.Add(_clear, 0, 5);
        layout.Controls.Add(_save, 1, 5);
        Controls.Add(layout);

        _processor.SetBitRate(SelectedRate());
    }

    static Label LabelOf(string text) => new()
    {
        Text = text,
        AutoSize = true,
        Margin = new Padding(0, 6, 0, 0)
    };

    int SelectedRate() => _rate.SelectedIndex switch
    {
        0 => 600,
        2 => 10500,
        _ => 1200
    };

    void Arm()
    {
        var rate = SelectedRate();
        var bandwidth = rate switch
        {
            600 => 4000,
            1200 => 6000,
            _ => 20000
        };
        var max = _control.MaximumFilterBandwidth;
        if (max > 0 && bandwidth > max)
            bandwidth = max;
        _control.DetectorType = DetectorType.RAW;
        _control.FilterBandwidth = bandwidth;
        Append("Armed RAW, filter " + bandwidth.ToString("N0") + " Hz. Center the VFO on the carrier.");
    }

    void RefreshStatus()
    {
        var snap = _processor.Read();
        var link = snap.Link;
        var state = _decode.Checked ? link.State : "idle";
        var iq = snap.SampleRate >= 1000 ? (snap.SampleRate / 1000.0).ToString("0.0") + " kHz" : "no IQ";
        var text =
            link.BitRate.ToString() + " bit/s   " + state + "   " +
            link.FrequencyOffsetHz.ToString("+0;-0;0") + " Hz" + Environment.NewLine +
            "UW " + link.UniqueWordErrors.ToString() +
            "   frames " + link.Frames.ToString() +
            "   CRC " + link.CrcOk.ToString() + "/" + link.CrcBad.ToString() +
            "   fill " + link.FillUnits.ToString() + Environment.NewLine +
            "IQ " + iq + "   " + link.LevelDb.ToString("0") + " dB   drop " + snap.Dropped.ToString();
        if (link.BitRate >= 8400 && snap.SampleRate >= 1000 && snap.SampleRate < 30000)
            text += Environment.NewLine + "IQ rate is low for 10500. Widen the filter toward 20 kHz.";
        if (!string.IsNullOrEmpty(snap.Fault))
            text += Environment.NewLine + snap.Fault;
        _status.Text = text;
    }

    public void AppendMessage(DecodedMessage message)
    {
        if (IsDisposed)
            return;
        if (InvokeRequired)
        {
            BeginInvoke(new Action<DecodedMessage>(AppendMessage), message);
            return;
        }
        Append(message.Format());
        if (_save.Checked)
            WriteFile(message.Format());
    }

    void Append(string line)
    {
        _log.AppendText(line + Environment.NewLine);
        _lines++;
        if (_lines <= LogLineCap)
            return;
        var body = _log.Text;
        var cut = body.IndexOf('\n', body.Length / 5);
        if (cut > 0)
            _log.Text = body.Substring(cut + 1);
        _log.SelectionStart = _log.TextLength;
        _log.ScrollToCaret();
        _lines = LogLineCap - 50;
    }

    void WriteFile(string line)
    {
        try
        {
            var day = DateTime.UtcNow.ToString("yyyy-MM-dd");
            if (_file == null || _fileDay != day)
            {
                CloseFile();
                var dir = Path.Combine(
                    Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                    "SDRSharp-LBandAcars");
                Directory.CreateDirectory(dir);
                var path = Path.Combine(dir, "acars-" + day + ".log");
                _file = new StreamWriter(new FileStream(path, FileMode.Append, FileAccess.Write, FileShare.Read))
                {
                    AutoFlush = true
                };
                _fileDay = day;
            }
            _file.WriteLine(line);
        }
        catch (Exception ex)
        {
            _save.Checked = false;
            Append("Log file failed: " + ex.Message);
        }
    }

    void CloseFile()
    {
        _file?.Dispose();
        _file = null;
        _fileDay = "";
    }

    void OnThemeChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e)
    {
        if (e.PropertyName is not (null or "ThemeName" or "ThemeForeColor" or "ThemeIsDark"))
            return;
        if (IsDisposed)
            return;
        if (InvokeRequired)
            BeginInvoke(new Action(ApplyTheme));
        else
            ApplyTheme();
    }

    void ApplyTheme()
    {
        var fore = _control.ThemeForeColor;
        var panel = _control.ThemePanelColor;
        var editor = _control.ThemeEditorColor;
        BackColor = panel;
        ForeColor = fore;
        PaintTree(this, fore, panel, editor);
    }

    static void PaintTree(Control root, Color fore, Color panel, Color editor)
    {
        foreach (Control child in root.Controls)
        {
            if (child is TextBox)
            {
                child.ForeColor = fore;
                child.BackColor = editor;
            }
            else if (child is Button or ComboBox or NumericUpDown or CheckBox)
            {
                child.ForeColor = fore;
                child.BackColor = panel;
            }
            else
            {
                child.ForeColor = fore;
                child.BackColor = panel;
            }
            if (child.HasChildren)
                PaintTree(child, fore, panel, editor);
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _timer.Dispose();
            CloseFile();
        }
        base.Dispose(disposing);
    }
}
