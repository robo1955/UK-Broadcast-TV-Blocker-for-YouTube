using System.Windows.Forms;
using SDRSharp.Common;
using SDRSharp.LBandAcars.Dsp;
using SDRSharp.Radio;

namespace SDRSharp.LBandAcars;

public sealed class LBandAcarsPlugin : ISharpPlugin, ICanLazyLoadGui, ISupportStatus, IExtendedNameProvider
{
    ISharpControl _control = null!;
    AcarsPanel? _gui;
    IqProcessor? _processor;

    public string DisplayName => "L-Band ACARS";

    public string Category => "Radio";

    public string MenuItemName => DisplayName;

    public bool IsActive => _gui != null && _gui.Visible && _gui.Decoding;

    public UserControl Gui
    {
        get
        {
            LoadGui();
            return _gui ?? throw new System.InvalidOperationException("L-Band ACARS is closed.");
        }
    }

    public void Initialize(ISharpControl control)
    {
        _control = control;
        _processor = new IqProcessor(new LBandEngine());
        _control.RegisterStreamHook(_processor, ProcessorType.DecimatedAndFilteredIQ);
        _control.PropertyChanged += OnRadioChanged;
    }

    public void LoadGui()
    {
        if (_gui == null && _processor != null)
            _gui = new AcarsPanel(_control, _processor);
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
