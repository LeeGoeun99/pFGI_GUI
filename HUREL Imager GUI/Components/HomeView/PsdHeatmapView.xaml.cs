using System;
using System.Windows;
using System.Windows.Controls;
using HUREL_Imager_GUI.ViewModel;

namespace HUREL_Imager_GUI.Components
{
    /// <summary>
    /// PSD(Absorber E × counts) 2D 히트맵. <see cref="SpectrumViewModel.PsdHeatmap"/>·갱신 이벤트와 연동.
    /// </summary>
    public partial class PsdHeatmapView : UserControl
    {
        private SpectrumViewModel? _wiredSpectrumVm;

        public PsdHeatmapView()
        {
            InitializeComponent();
            Loaded += OnLoaded;
            Unloaded += OnUnloaded;
            DataContextChanged += OnDataContextChanged;
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            WireSpectrumViewModel(DataContext as SpectrumViewModel);
        }

        private void OnUnloaded(object sender, RoutedEventArgs e)
        {
            WireSpectrumViewModel(null);
        }

        private void OnDataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            WireSpectrumViewModel(DataContext as SpectrumViewModel);
        }

        private void WireSpectrumViewModel(SpectrumViewModel? vm)
        {
            if (_wiredSpectrumVm == vm)
            {
                return;
            }

            if (_wiredSpectrumVm != null)
            {
                _wiredSpectrumVm.PsdHeatmapRefreshNeeded -= OnPsdHeatmapRefreshNeeded;
            }

            _wiredSpectrumVm = vm;

            if (_wiredSpectrumVm != null)
            {
                _wiredSpectrumVm.PsdHeatmapRefreshNeeded += OnPsdHeatmapRefreshNeeded;
            }

            Dispatcher.BeginInvoke(new Action(UpdatePsdPlot));
        }

        private void OnPsdHeatmapRefreshNeeded(object? sender, EventArgs e)
        {
            Dispatcher.BeginInvoke(new Action(UpdatePsdPlot));
        }

        private void UpdatePsdPlot()
        {
            if (_wiredSpectrumVm == null)
            {
                return;
            }

            _wiredSpectrumVm.PsdHeatmap.UpdatePlot(PsdWpfPlot.Plot);
            PsdWpfPlot.Refresh();
        }
    }
}
