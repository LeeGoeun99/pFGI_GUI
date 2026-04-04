using System;
using System.Linq;
using HUREL.Compton;
using ScottPlot;
using ScottPlot.Colormaps;
using ScottPlot.TickGenerators;

namespace HUREL_Imager_GUI.ViewModel
{
    /// <summary>
    /// <see cref="PsdAccumulator"/> 스냅샷을 ScottPlot 5 히트맵(Turbo)으로 그린다. View에서 Plot 인스턴스에 적용.
    /// </summary>
    public sealed class PsdHeatmapViewModel
    {
        private const double AxisEnergyMin = 0.0;
        private const double AxisEnergyMax = 5000.0;
        private const double AxisPsdMin = 0.4;
        private const double AxisPsdMax = 0.7;

        /// <summary>왼쪽 Y축 제목 픽셀 보정. 음수(예: -22)는 WPF에서 라벨이 잘려 안 보일 수 있음.</summary>
        private const float LeftAxisTitleOffsetX = 0f;

        /// <summary>컬러바 패널 최소 폭(눈금 문자 포함). 줄이면 데이터 영역 가로가 넓어짐(기존 72의 약 2/3).</summary>
        private const float ColorBarMinimumSizePixels = 48f;

        /// <summary>컬러바 막대 두께(px). ScottPlot 기본 30의 약 2/3.</summary>
        private const float ColorBarStripWidthPixels = 20f;

        public void UpdatePlot(Plot plot)
        {
            plot.Clear();

            plot.FigureBackground.Color = Colors.White;
            plot.DataBackground.Color = Colors.White;

            plot.Axes.Bottom.Label.Text = "Energy (keV)";
            // IYAxis에는 LabelStyle이 없고 Label(LabelStyle 별칭)만 노출됨(ScottPlot 5.1)
            plot.Axes.Left.Label.Text = "PSD";
            plot.Axes.Left.Label.OffsetX = LeftAxisTitleOffsetX;

            ApplyEnergyAxisTicks1000(plot);

            PsdHeatmapSnapshot snap = PsdAccumulator.Instance.GetHeatmapSnapshot();
            int nE = snap.Counts.GetLength(0);
            int nP = snap.Counts.GetLength(1);
            if (nE == 0 || nP == 0)
            {
                plot.Axes.SetLimits(AxisEnergyMin, AxisEnergyMax, AxisPsdMin, AxisPsdMax);
                return;
            }

            var data = new double[nP, nE];
            var alpha = new byte[nP, nE];
            double maxLog = 0.0;
            for (int ie = 0; ie < nE; ie++)
            {
                for (int ip = 0; ip < nP; ip++)
                {
                    double v = snap.Counts[ie, ip];
                    double logV = Math.Log10(1.0 + v);
                    data[ip, ie] = logV;
                    alpha[ip, ie] = v == 0 ? (byte)0 : (byte)255;
                    if (v > 0 && logV > maxLog)
                    {
                        maxLog = logV;
                    }
                }
            }

            double colorMax = maxLog > 0 ? maxLog : 1.0;
            var hm = plot.Add.Heatmap(data);
            hm.Colormap = new Turbo();
            hm.Rectangle = new CoordinateRect(snap.EnergyMinKeV, snap.EnergyMaxKeV, snap.PsdMin, snap.PsdMax);
            hm.Smooth = false;
            hm.FlipVertically = true;
            hm.AlphaMap = alpha;
            hm.ManualRange = new ScottPlot.Range(0, colorMax);

            var cb = plot.Add.ColorBar(hm);
            cb.Width = ColorBarStripWidthPixels;
            cb.MinimumSize = ColorBarMinimumSizePixels;
            cb.Label = string.Empty;
            cb.Axis.TickGenerator = new NumericAutomatic { LabelFormatter = FormatColorBarTickFromLog };

            plot.Axes.SetLimits(AxisEnergyMin, AxisEnergyMax, AxisPsdMin, AxisPsdMax);
        }

        /// <summary>컬러바 축 값은 log10(1+count); 표시는 카운트 정수에 가깝게.</summary>
        private static string FormatColorBarTickFromLog(double logValue)
        {
            if (logValue <= 0 || double.IsNaN(logValue))
            {
                return "0";
            }

            double c = Math.Pow(10, logValue) - 1.0;
            if (double.IsNaN(c) || double.IsInfinity(c))
            {
                return "0";
            }

            if (c < 10)
            {
                return Math.Round(c).ToString("0");
            }

            return Math.Round(c).ToString("0");
        }

        private static void ApplyEnergyAxisTicks1000(Plot plot)
        {
            double[] positions = Enumerable.Range(0, 6).Select(i => i * 1000.0).ToArray();
            string[] labels = positions.Select(x => x.ToString("0")).ToArray();
            plot.Axes.Bottom.SetTicks(positions, labels);
        }
    }
}
