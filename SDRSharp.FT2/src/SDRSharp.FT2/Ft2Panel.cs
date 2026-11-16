using System.Drawing;
using System.IO;
using System.Windows.Forms;
using SDRSharp.Common;
using SDRSharp.FT2.Dsp;
using SDRSharp.PanView;
using SDRSharp.Radio;

namespace SDRSharp.FT2;

public sealed class Ft2Panel : UserControl
{
    const int LogLineCap = 500;

    readonly ISharpControl _control;
    readonly AudioProcessor _processor;
    readonly CheckBox _decode = new();
    readonly DoubleBufferedHintButton _arm = new();
    readonly Label _status = new();
    readonly DoubleBufferedTextBox _log = new();
    readonly DoubleBufferedHintButton _clear = new();
    readonly CheckBox _save = new();
    readonly System.Windows.Forms.Timer _timer = new();
    StreamWriter? _file;
    string _fileDay = "";
    int _lines;

    public Ft2Panel(ISharpControl control, AudioProcessor processor)
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
        MinimumSize = new Size(280, 280);
        Font = new Font("Segoe UI", 8.25f);

        var layout = new DoubleBufferedTableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 4,
            Padding = new Padding(6)
        };
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 64));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        _decode.Text = "Decode";
        _decode.AutoSize = true;
        _decode.FlatStyle = FlatStyle.Flat;
        _decode.Margin = new Padding(0, 6, 0, 0);
        _decode.CheckedChanged += (_, _) => _processor.Enabled = _decode.Checked;

        _arm.Text = "Arm USB";
        _arm.AutoSize = true;
        _arm.Dock = DockStyle.Fill;
        _arm.Click += (_, _) => Arm();

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

        layout.Controls.Add(_decode, 0, 0);
        layout.Controls.Add(_arm, 1, 0);
        layout.Controls.Add(_status, 0, 1);
        layout.SetColumnSpan(_status, 2);
        layout.Controls.Add(_log, 0, 2);
        layout.SetColumnSpan(_log, 2);
        layout.Controls.Add(_clear, 0, 3);
        layout.Controls.Add(_save, 1, 3);
        Controls.Add(layout);
    }

    void Arm()
    {
        var bandwidth = 3000;
        var max = _control.MaximumFilterBandwidth;
        if (max > 0 && bandwidth > max)
            bandwidth = max;
        _control.DetectorType = DetectorType.USB;
        _control.FilterBandwidth = bandwidth;
        Append("Armed USB, filter " + bandwidth.ToString("N0") + " Hz. Check Decode. FT2 is FT4 at twice the speed.");
    }

    void RefreshStatus()
    {
        var snap = _processor.Read();
        var audio = snap.SampleRate >= 1000 ? (snap.SampleRate / 1000.0).ToString("0.0") + " kHz" : "no audio";
        var state = _decode.Checked ? "on" : "idle";
        var text =
            audio + "   " + state + "   decodes " + snap.Decodes.ToString() + Environment.NewLine +
            "last pass " + snap.LastHits.ToString() + "   drop " + snap.Dropped.ToString() + Environment.NewLine +
            "USB, about 3 kHz. FT2 is FT4 at 2x (2.52 s).";
        if (!string.IsNullOrEmpty(snap.Fault))
            text += Environment.NewLine + snap.Fault;
        _status.Text = text;
    }

    public void AppendMessage(Ft2Modem.Hit hit)
    {
        if (IsDisposed)
            return;
        if (InvokeRequired)
        {
            BeginInvoke(new Action<Ft2Modem.Hit>(AppendMessage), hit);
            return;
        }
        var line = DateTime.Now.ToString("HH:mm:ss") + "  " +
                   hit.AudioHz.ToString("0") + " Hz  " +
                   hit.SnrDb.ToString("+0;-0;0") + " dB  " +
                   hit.Text;
        Append(line);
        if (_save.Checked)
            WriteFile(line);
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
                    "SDRSharp-FT2");
                Directory.CreateDirectory(dir);
                var path = Path.Combine(dir, "ft2-" + day + ".log");
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
            else if (child is Button or CheckBox)
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
