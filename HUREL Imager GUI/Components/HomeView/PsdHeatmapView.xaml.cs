using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Controls;
using HUREL_Imager_GUI.ViewModel;
using log4net;

namespace HUREL_Imager_GUI.Components
{
    /// <summary>
    /// PSD(Absorber E × counts) 2D 히트맵. <see cref="SpectrumViewModel.PsdHeatmap"/>·갱신 이벤트와 연동.
    /// </summary>
    public partial class PsdHeatmapView : UserControl
    {
        /// <summary>측정 직후 진단: 처음 N회 PSD 갱신만 UpdatePlot+Refresh ms 로그.</summary>
        private static int _psdDiagPlotsRemaining = 10;

        /// <summary>새 측정 세션마다 호출하면 PSD 진단 로그 카운터가 다시 10으로 맞춰짐.</summary>
        public static void ResetDiagnosticLogForNewSession()
        {
            _psdDiagPlotsRemaining = 10;
        }

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

            Stopwatch sw = Stopwatch.StartNew();
            _wiredSpectrumVm.PsdHeatmap.UpdatePlot(PsdWpfPlot.Plot);
            PsdWpfPlot.Refresh();
            if (_psdDiagPlotsRemaining > 0)
            {
                _psdDiagPlotsRemaining--;
                LogManager.GetLogger(typeof(PsdHeatmapView)).Info(
                    $"[SpectrumDiag] PSD UpdatePlot+Refresh {sw.ElapsedMilliseconds}ms (remaining={_psdDiagPlotsRemaining})");
            }
        }
    }
}
