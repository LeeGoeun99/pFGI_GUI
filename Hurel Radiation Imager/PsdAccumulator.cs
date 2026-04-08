using System.Globalization;
using System.IO;
using System.Text;
using log4net;

namespace HUREL.Compton
{
    /// <summary>
    /// SingleCoin1S: BD1만 — 채널 9~17 S/L로 PSD, Absorber 상호작용 에너지(keV)를 CSV·2D 히트맵에 적재한다.
    /// </summary>
    public sealed class PsdAccumulator
    {
        public static PsdAccumulator Instance { get; } = new PsdAccumulator();

        private static readonly ILog Log = LogManager.GetLogger(typeof(PsdAccumulator));

        private const int Bd1Flag = 2;

        private readonly object _sync = new();
        private readonly List<(double AbsorberEnergyKeV, double Psd)> _rows = new();

        private int[,] _heatmap;
        private int _nEnergyBins;
        private int _nPsdBins;
        private double _energyMinKeV;
        private double _energyMaxKeV;
        private double _psdMin;
        private double _psdMax;

        /// <summary>BD1 PSD 행이 실제로 쌓인 횟수(로그 샘플링·Reset 시 0으로).</summary>
        private long _psdAcceptedCount;

        private PsdAccumulator()
        {
            ConfigureHeatmap(256, 128, 0.0, 5000.0, 0.1, 1.0);
        }

        /// <summary>에너지·PSD 축 범위와 히스토그램 해상도(세션 시작 전·Reset 전에 호출 가능).</summary>
        public void ConfigureHeatmap(int energyBins, int psdBins, double energyMinKeV, double energyMaxKeV, double psdMin, double psdMax)
        {
            lock (_sync)
            {
                _nEnergyBins = Math.Max(1, energyBins);
                _nPsdBins = Math.Max(1, psdBins);
                _energyMinKeV = energyMinKeV;
                _energyMaxKeV = Math.Max(energyMinKeV + 1e-9, energyMaxKeV);
                _psdMin = psdMin;
                _psdMax = Math.Max(psdMin + 1e-12, psdMax);
                _heatmap = new int[_nEnergyBins, _nPsdBins];
            }
        }

        public int AccumulatedCount
        {
            get
            {
                lock (_sync)
                {
                    return _rows.Count;
                }
            }
        }

        public void Reset()
        {
            lock (_sync)
            {
                _rows.Clear();
                _psdAcceptedCount = 0;
                if (_heatmap != null)
                {
                    Array.Clear(_heatmap, 0, _heatmap.Length);
                }
            }
        }

        /// <summary>
        /// BD1 플래그·채널 9~17 Long 합이 0이 아닐 때만 적재. PSD = 1 - ΣS/ΣL.
        /// </summary>
        public bool TryAddEvent(double absorberInteractionEnergyKeV, Cs1sDetail detail)
        {
            if ((detail.BoardFlags & Bd1Flag) == 0)
            {
                return false;
            }

            long totalS = 0;
            long totalL = 0;
            for (int i = 9; i < 18; i++)
            {
                totalS += detail.ShortSums18[i];
                totalL += detail.LongSums18[i];
            }

            if (totalL == 0)
            {
                return false;
            }

            double psd = 1.0 - (double)totalS / totalL;
            lock (_sync)
            {
                _rows.Add((absorberInteractionEnergyKeV, psd));
                AddToHeatmapLocked(absorberInteractionEnergyKeV, psd);
                _psdAcceptedCount++;
            }

            return true;
        }

        private void AddToHeatmapLocked(double e, double p)
        {
            if (_heatmap == null)
            {
                return;
            }

            double eSpan = _energyMaxKeV - _energyMinKeV;
            int ie = (int)((e - _energyMinKeV) / eSpan * _nEnergyBins);
            if (ie < 0)
            {
                ie = 0;
            }
            else if (ie >= _nEnergyBins)
            {
                ie = _nEnergyBins - 1;
            }

            double pSpan = _psdMax - _psdMin;
            int ip = (int)((p - _psdMin) / pSpan * _nPsdBins);
            if (ip < 0)
            {
                ip = 0;
            }
            else if (ip >= _nPsdBins)
            {
                ip = _nPsdBins - 1;
            }

            _heatmap[ie, ip]++;
        }

        /// <summary>Counts[energyBin, psdBin] 카운트 복사본(ScottPlot Heatmap 등). 축 범위는 스냅샷 필드 참고.</summary>
        public PsdHeatmapSnapshot GetHeatmapSnapshot()
        {
            lock (_sync)
            {
                int w = _nEnergyBins;
                int h = _nPsdBins;
                var copy = new int[w, h];
                if (_heatmap != null)
                {
                    Buffer.BlockCopy(_heatmap, 0, copy, 0, w * h * sizeof(int));
                }

                return new PsdHeatmapSnapshot(copy, _energyMinKeV, _energyMaxKeV, _psdMin, _psdMax);
            }
        }

        /// <summary>
        /// 1열 Absorber 에너지(keV)(BD1), 2열 PSD. 행이 없어도 헤더만 있는 파일을 생성한다.
        /// </summary>
        public void SaveCsvIfAny(string path)
        {
            List<(double AbsorberEnergyKeV, double Psd)> snapshot;
            lock (_sync)
            {
                snapshot = _rows.Count == 0
                    ? new List<(double, double)>()
                    : new List<(double AbsorberEnergyKeV, double Psd)>(_rows);
            }

            var sb = new StringBuilder(Math.Max(32, snapshot.Count * 48));
            sb.AppendLine("AbsorberEnergy,PSD");
            IFormatProvider inv = CultureInfo.InvariantCulture;
            foreach (var row in snapshot)
            {
                sb.Append(row.AbsorberEnergyKeV.ToString("G17", inv));
                sb.Append(',');
                sb.AppendLine(row.Psd.ToString("G17", inv));
            }

            string? dir = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(dir))
            {
                Directory.CreateDirectory(dir);
            }

            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));
            Log.Info($"PSD CSV 저장(BD1만): {path} (행 수={snapshot.Count})");
            if (snapshot.Count == 0)
            {
                Log.Warn("PSD: BD1 적재 0건 — BoardFlags(BD1)·9~17 Long합·SingleCoin1S·흡수체 GetEcal(nan) 여부를 확인하세요. (에너지는 리스트모드 비동기가 아닌 TryGetAbsorberEnergyKeVFrom144Shorts로 동기 계산)");
            }
        }
    }

    public readonly struct PsdHeatmapSnapshot
    {
        public PsdHeatmapSnapshot(int[,] counts, double energyMinKeV, double energyMaxKeV, double psdMin, double psdMax)
        {
            Counts = counts;
            EnergyMinKeV = energyMinKeV;
            EnergyMaxKeV = energyMaxKeV;
            PsdMin = psdMin;
            PsdMax = psdMax;
        }

        /// <summary>인덱스 [energyBin, psdBin].</summary>
        public int[,] Counts { get; }

        public double EnergyMinKeV { get; }
        public double EnergyMaxKeV { get; }
        public double PsdMin { get; }
        public double PsdMax { get; }
    }
}
