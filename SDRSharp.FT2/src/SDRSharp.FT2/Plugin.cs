using System.Windows.Forms;
using SDRSharp.Common;
using SDRSharp.Radio;

namespace SDRSharp.FT2;

public sealed class Ft2Plugin : ISharpPlugin, ICanLazyLoadGui, ISupportStatus, IExtendedNameProvider
{
    ISharpControl _control = null!;
    Ft2Panel? _gui;
    AudioProcessor? _processor;

    public string DisplayName => "FT2";

    public string Category => "Radio";

    public string MenuItemName => DisplayName;

    public bool IsActive => _gui != null && _gui.Visible && _gui.Decoding;

    public UserControl Gui
    {
        get
        {
            LoadGui();
            return _gui ?? throw new InvalidOperationException("FT2 is closed.");
        }
    }

    public void Initialize(ISharpControl control)
    {
        _control = control;
        _processor = new AudioProcessor();
        _control.RegisterStreamHook(_processor, ProcessorType.FilteredAudioOutput);
        _control.PropertyChanged += OnRadioChanged;
    }

    public void LoadGui()
    {
        if (_gui == null && _processor != null)
            _gui = new Ft2Panel(_control, _processor);
    }

    public void Close()
    {
        if (_control != null)
        {
            _control.PropertyChanged -= OnRadioChanged;
            if (_processor != null)
                _control.UnregisterStreamHook(_processor);
        }
        if (_gui != null)
        {
            _gui.Shutdown();
            _gui.Dispose();
            _gui = null;
        }
        _processor?.Dispose();
        _processor = null;
    }

    void OnRadioChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e)
    {
        if (_processor == null)
            return;
        if (e.PropertyName is "Frequency" or "CenterFrequency" or "FilterBandwidth" or "DetectorType")
            _processor.RequestReset();
    }
}
