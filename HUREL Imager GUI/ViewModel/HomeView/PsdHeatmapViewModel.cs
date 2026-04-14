using System;
using System.Linq;
using HUREL.Compton;
using ScottPlot;
using ScottPlot.Colormaps;
using ScottPlot.Panels;
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
        private const double AxisPsdMin = 0.1;
        private const double AxisPsdMax = 1.0;

        /// <summary>왼쪽 Y축 제목 픽셀 보정. 음수(예: -22)는 WPF에서 라벨이 잘려 안 보일 수 있음.</summary>
        private const float LeftAxisTitleOffsetX = 0f;

        /// <summary>컬러바 패널 최소 폭(눈금 문자 포함). 줄이면 데이터 영역 가로가 넓어짐(기존 72의 약 2/3).</summary>
        private const float ColorBarMinimumSizePixels = 48f;

        /// <summary>컬러바 막대 두께(px). ScottPlot 기본 30의 약 2/3.</summary>
        private const float ColorBarStripWidthPixels = 20f;

        public void UpdatePlot(Plot plot)
        {
            // ColorBar는 Plottable이 아니라 Axes 패널이라 Plot.Clear()로 제거되지 않음 → 갱신마다 누적됨
            RemoveColorBarPanels(plot);
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
                    int v = snap.Counts[ie, ip];
                    if (v <= 0)
                    {
                        data[ip, ie] = 0.0;
                        alpha[ip, ie] = 0;
                    }
                    else
                    {
                        double logV = Math.Log10(v);
                        data[ip, ie] = logV;
                        alpha[ip, ie] = 255;
                        if (logV > maxLog)
                        {
                            maxLog = logV;
                        }
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
            hm.Update();

            var cb = plot.Add.ColorBar(hm);
            cb.Width = ColorBarStripWidthPixels;
            cb.MinimumSize = ColorBarMinimumSizePixels;
            cb.Label = string.Empty;
            ConfigureColorBarAxisLogExponentTicks(cb);

            plot.Axes.SetLimits(AxisEnergyMin, AxisEnergyMax, AxisPsdMin, AxisPsdMax);
        }

        private static void RemoveColorBarPanels(Plot plot)
        {
            foreach (IPanel panel in plot.Axes.GetPanels().ToArray())
            {
                if (panel is ColorBar)
                {
                    plot.Remove(panel);
                }
            }
        }

        /// <summary>
        /// 색은 log10(N)에 매핑되므로 컬러바 눈금도 동일 좌표계의 지수(log10 값)를 표시한다.
        /// (이전: 10^pos를 반올림해 N으로만 보여 ‘그냥 count’처럼 느껴짐)
        /// </summary>
        private static void ConfigureColorBarAxisLogExponentTicks(ColorBar cb)
        {
            IAxis axis = cb.Axis;
            var auto = new NumericAutomatic
            {
                LabelFormatter = static pos =>
                {
                    if (double.IsNaN(pos) || double.IsInfinity(pos))
                    {
                        return string.Empty;
                    }

                    return pos.ToString("0.##");
                },
            };
            axis.TickGenerator = auto;
        }

        private static void ApplyEnergyAxisTicks1000(Plot plot)
        {
            double[] positions = Enumerable.Range(0, 6).Select(i => i * 1000.0).ToArray();
            string[] labels = positions.Select(x => x.ToString("0")).ToArray();
            plot.Axes.Bottom.SetTicks(positions, labels);
        }
    }
}
