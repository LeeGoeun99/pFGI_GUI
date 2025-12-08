using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using HUREL_Imager_GUI.Components;
using log4net;
using Syncfusion.CompoundFile.XlsIO.Native;
using Syncfusion.UI.Xaml.Charts;
using Syncfusion.XlsIO.Parser.Biff_Records.Formula;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Configuration;
using System.Diagnostics;
using System.Dynamic;
using System.Globalization;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using static HelixToolkit.Wpf.SharpDX.Model.Metadata;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Rebar;

namespace HUREL_Imager_GUI.ViewModel
{
    //240628 : peak에 대응되는 Max 값
    public class PeakToMax
    {
        public double peakE { get; set; } = 0.0;
        public double peakMax { get; set; } = 0.0;

        public PeakToMax(double peakE, double peakMax)
        {
            this.peakE = peakE;
            this.peakMax = peakMax;
        }
    }
    public class IsotopeInfo : ObservableObject
    {
        private Visibility _isSelected = Visibility.Hidden;

        public Visibility IsSelected
        {
            get => _isSelected;
            set
            {
                if (_isSelected != value)
                {
                    _isSelected = value;
                    if (value == Visibility.Visible)
                        //TextColor = Brushes.Blue;
                        TextColor = Brushes.Red;
                    else
                        TextColor = Brushes.Black;
                    OnPropertyChanged();
                }
            }
        }

        private Brush _textColor = Brushes.Black;
        public Brush TextColor
        {
            get => _textColor;
            set
            {
                _textColor = value;
                OnPropertyChanged();
            }
        }

        private bool _isManualSelection = false;
        public bool IsManualSelection
        {
            get => _isManualSelection;
            set
            {
                if (_isManualSelection != value)
                {
                    _isManualSelection = value;
                    OnPropertyChanged();
                }
            }
        }

        private bool _isDetected = false;
        public bool IsDetected
        {
            get => _isDetected;
            set
            {
                if (_isDetected != value)
                {
                    _isDetected = value;
                    OnPropertyChanged();
                }
            }
        }

        private bool _isAutoSelectDisabled = false;
        public bool IsAutoSelectDisabled
        {
            get => _isAutoSelectDisabled;
            set
            {
                if (_isAutoSelectDisabled != value)
                {
                    _isAutoSelectDisabled = value;
                    OnPropertyChanged();
                }
            }
        }

        public List<double> PeakEnergy { get; set; } = new List<double>();
        public IsotopeElement IsoElement { get; set; }

        public string Name { get; set; }
        public string Description { get; set; }
        public string Energy { get; set; }
        public string MLEMResult { get; set; }
        // public string Dose { get; set; }

        public IsotopeInfo(string name, string description, string energy, Visibility selected, Brush textcolor, IsotopeElement element, List<double> Peaks, string mLEMResult = "")
        {
            Name = name;
            Description = description;
            Energy = energy;
            IsSelected = selected;
            TextColor = textcolor;
            IsoElement = element;
            PeakEnergy = Peaks;
            MLEMResult = mLEMResult;
            IsManualSelection = false; // 기본값은 자동 선택
            //Dose = dose;
        }
    }
    public enum eSpectrumCases
    {
        Scatter,
        Absorber,
        All,
        ByChannel
    };

    //231024 sbkwon : Spetrumtype
    public enum eSpectrumType
    {
        Linear,
        Log
    };

    //231023 sbkwon : 핵종 이름 표시 경우의 수
    public enum eTextAnnotation
    {
        enAdd,  //추가
        enSelect,   //선택
        enDeselect, //선택 해제
        enDelete,   //삭제
    };

    //#nullable disable

    //231016 sbkwon : 소스 객체 찾기
    public class SpectrumViewModel : ViewModelBase
    {
        public TopButtonViewModel TopButtonVM { get; set; }
        /// <summary>소스 객체 찾기</summary>
        /// <remarks>traceLog를 true로 하면, System.Diagnostics.Trace로만 경로를 남김</remarks>
        /// <typeparam name="T">확인 객체형</typeparam>
        /// <param name="routedEventArgs">추적할 이벤트</param>
        /// <param name="traceLog">트리 추적 로그</param>
        /// <returns>소스 객체 검색 결과 반환</returns>
        public static (bool isExist, T resultObject) FindSourceObject<T>(RoutedEventArgs routedEventArgs)
            where T : DependencyObject
        {
            var hit = routedEventArgs.OriginalSource as DependencyObject;
            var source = routedEventArgs.Source as DependencyObject;

            if (source is T) return (true, source as T);

            while (null != hit && source != hit)
            {
                //Trace.WriteLine(hit);

                if (hit is T result) return (true, result);

                try { hit = VisualTreeHelper.GetParent(hit); }
                catch { break; }
            }

            return (false, default(T));
        }

        public SpectrumViewModel()
        {
            SpectrumTime = App.GlobalConfig.SpectrumEffectTime;
            IsSpectrumAnalysisShow = App.GlobalConfig.isSpectrumAnalysisShow;
            // 기본값을 All로 설정
            SpectrumCases = App.GlobalConfig.SpectrumCases;
            // 기본값이 Scatter인 경우(기존 설정 파일) All로 변경
            if (SpectrumCases == eSpectrumCases.Scatter)
            {
                SpectrumCases = eSpectrumCases.All;
                App.GlobalConfig.SpectrumCases = eSpectrumCases.All;
            }
            FpgaChannelNumber = App.GlobalConfig.FpgaChannelNumber;
            SpectrumType = App.GlobalConfig.SpectrumType;
            // 기본값: SetECal은 선택되지 않은 상태로 시작
            IsEcalUse = false;
            App.GlobalConfig.IsECalUse = false;  // Config에도 반영
            IntervalECalTime = App.GlobalConfig.ECalIntervalTime / 60000;    //화면 표시 분, config 단위 ms

            //231016 sbkwon : S 마우스 이벤트
            MouseEnterCommand = new AsyncCommand<MouseEventArgs>(OnMouseEnter);
            MouseLeaveCommand = new AsyncCommand<MouseEventArgs>(OnMouseLeave);
            PreviewMouseDownCommand = new AsyncCommand<MouseButtonEventArgs>(OnPreviewMouseDown);
            PreviewMouseUpCommand = new AsyncCommand<MouseButtonEventArgs>(OnPreviewMouseUp);
            PreviewMouseMoveCommand = new AsyncCommand<MouseEventArgs>(OnPreviewMouseMove);
            PreviewMouseDoubleClickCommand = new AsyncCommand<MouseButtonEventArgs>(OnPreviewMouseDoubleClick);
            //231016 sbkwon : E 마우스 이벤트

            LahgiApi.StatusUpdate += StatusUpdate;
            EnergySpectrum = new ObservableCollection<HistoEnergy>();
            IsotopeInfos = new ObservableCollection<IsotopeInfo>();
        }
        Mutex StatusUpdateMutex = new Mutex();
        static int IsotopeInfosCount = 0;

        //231016 sbkwon : S 마우스 이벤트
        public ICommand MouseEnterCommand { get; }
        public ICommand MouseLeaveCommand { get; }
        public ICommand PreviewMouseDownCommand { get; }
        public ICommand PreviewMouseUpCommand { get; }
        public ICommand PreviewMouseMoveCommand { get; }
        public ICommand PreviewMouseDoubleClickCommand { get; }

        /// <summary>마우스 겹쳐짐 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnMouseEnter(MouseEventArgs e) { }

        /// <summary>마우스 벗어남 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnMouseLeave(MouseEventArgs e) { }

        /// <summary>마우스 버튼 눌림 터널링 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnPreviewMouseDown(MouseButtonEventArgs e)
        {
            var (isExists, obj) = FindSourceObject<Syncfusion.UI.Xaml.Charts.ChartRootPanel>(e);

            if (isExists is true)
            {
                Point p = e.GetPosition((IInputElement)obj);
                int X = (int)p.X;
                int Y = (int)p.Y;
                //Trace.WriteLine($"{X}, {Y}");
                if ((X - MouseXPointZero) > 0 && PeakLine.Count() > 0)
                {
                    double xRatio = 3000.0 / (obj.ActualWidth - MouseXPointZero);   //좌표 변환 상수
                    int nHitEnergy = (int)((X - MouseXPointZero) * xRatio);           //에너지 값

                    //PeakLine hit 판단
                    foreach (var peak in PeakLine)
                    {
                        //Trace.WriteLine($" {peak.X}, HistE : {nHitEnergy}, Ratio : {PeakLine.Count()} W : {obj.ActualWidth}, H : {obj.ActualHeight}");
                        if (Math.Abs(peak.X - nHitEnergy) <= HitRange)   //line check.
                        {
                            //핵종 선택에 대한 energy gate 설정
                            AddSelectPeak(peak);
                            //Trace.WriteLine($" hit : {peak.X}, HitE : {nHitEnergy} count : {PeakLine.Count()} W : {obj.ActualWidth}, H : {obj.ActualHeight}");
                        }
                    }
                }
            }
        }

        /// <summary>마우스 버튼 올라옴 터널링 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnPreviewMouseUp(MouseButtonEventArgs e) { }

        /// <summary>마우스 움직임 터널링 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnPreviewMouseMove(MouseEventArgs e) { }

        /// <summary>마우스 버튼 두번 누름 터널링 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnPreviewMouseDoubleClick(MouseButtonEventArgs e) { }
        //231016 sbkwon : E

        //231016 sbkwon : 선택한 Peak Linear 리스트
        private ObservableCollection<GraphData> _selectPeakLine = new ObservableCollection<GraphData>(); //Spectrum 에서 Peak Linear 선택 리스트
        public ObservableCollection<GraphData> SelectPeakLine
        {
            get { return _selectPeakLine; }
            set { _selectPeakLine = value; OnPropertyChanged(nameof(SelectPeakLine)); }
        }

        // 탐지된 핵종의 Peak Line (빨간색)
        private ObservableCollection<GraphData> _selectPeakLineRed = new ObservableCollection<GraphData>();
        public ObservableCollection<GraphData> SelectPeakLineRed
        {
            get { return _selectPeakLineRed; }
            set { _selectPeakLineRed = value; OnPropertyChanged(nameof(SelectPeakLineRed)); }
        }

        // 탐지되지 않은 핵종의 Peak Line (초록색)
        private ObservableCollection<GraphData> _selectPeakLineGreen = new ObservableCollection<GraphData>();
        public ObservableCollection<GraphData> SelectPeakLineGreen
        {
            get { return _selectPeakLineGreen; }
            set { _selectPeakLineGreen = value; OnPropertyChanged(nameof(SelectPeakLineGreen)); }
        }

        //231023 sbkwon : Isotopes Name Annotation
        private readonly object _chartAnnotationLock = new object();
        private AnnotationCollection _chartAnnotation = new AnnotationCollection();
        public AnnotationCollection chartAnnotation
        {
            get => _chartAnnotation;
            set
            {
                _chartAnnotation = value;
                //BindingOperations.EnableCollectionSynchronization(_chartAnnotation, _chartAnnotationLock);
                OnPropertyChanged(nameof(chartAnnotation));
            }
        }

        //231100-GUI sbkwon
        private readonly object _chartAnnotationLockLinear = new object();
        private AnnotationCollection _chartAnnotationLinear = new AnnotationCollection();
        public AnnotationCollection chartAnnotationLinear
        {
            get => _chartAnnotationLinear;
            set
            {
                _chartAnnotationLinear = value;
                //BindingOperations.EnableCollectionSynchronization(_chartAnnotationLinear, _chartAnnotationLockLinear);
                OnPropertyChanged(nameof(chartAnnotation));
            }
        }

        //231023 sbkwon : Isotopes Name Annotation 추가, 선택, 삭제
        //231100-GUI
        public void IsotopesTextAnnotation(string name, int x, int y, eTextAnnotation bSelect)
        {
            switch (bSelect)
            {
                case eTextAnnotation.enAdd:
                    {
                        //Trace.WriteLine($"==========================================Annotation Add - {Thread.CurrentThread.ManagedThreadId}");

                        int nIndex = CheckTextAnnotationExist(x);

                        if (nIndex == -1)
                        {
                            chartAnnotation.Add(new TextAnnotation()
                            {
                                Text = name,
                                Foreground = Brushes.Red,
                                FontSize = 15,
                                FontWeight = FontWeights.Bold,
                                X1 = x,
                                Y1 = y,
                                HorizontalAlignment = HorizontalAlignment.Center,
                                VerticalAlignment = VerticalAlignment.Top,
                            });

                            chartAnnotationLinear.Add(new TextAnnotation()
                            {
                                Text = name,
                                Foreground = Brushes.Red,
                                FontSize = 15,
                                FontWeight = FontWeights.Bold,
                                X1 = x,
                                Y1 = y,
                                HorizontalAlignment = HorizontalAlignment.Center,
                                VerticalAlignment = VerticalAlignment.Top,
                            });
                        }

                        //Trace.WriteLine($"-----------------------------------------------Annotation Add End - {Thread.CurrentThread.ManagedThreadId}");
                    }
                    break;
                case eTextAnnotation.enSelect:
                    foreach (var item in chartAnnotation)
                    {
                        if ((int)item.X1 == x)
                        {
                            item.Foreground = Brushes.Cyan; break;
                        }
                        //Trace.WriteLine("Annotation Select");
                    }
                    foreach (var item in chartAnnotationLinear)
                    {
                        if ((int)item.X1 == x)
                        {
                            item.Foreground = Brushes.Cyan; break;
                        }
                        //Trace.WriteLine("Annotation Select");
                    }
                    break;
                case eTextAnnotation.enDeselect:
                    foreach (var item in chartAnnotation)
                    {
                        if ((int)item.X1 == x)
                        {
                            item.Foreground = Brushes.Red; break;
                        }
                        //Trace.WriteLine("Annotation Deselect");
                    }
                    foreach (var item in chartAnnotationLinear)
                    {
                        if ((int)item.X1 == x)
                        {
                            item.Foreground = Brushes.Red; break;
                        }
                        //Trace.WriteLine("Annotation Deselect");
                    }
                    break;
                case eTextAnnotation.enDelete:
                    {
                        foreach (var item in PeakLine)
                        {
                            int nIndex = CheckTextAnnotationExist(x);

                            if (nIndex != -1)
                            {
                                chartAnnotation.RemoveAt(nIndex);
                                chartAnnotationLinear.RemoveAt(nIndex);
                            }
                        }
                    }
                    break;
                default:
                    break;
            }
        }

        /// <summary>
        /// 231025 sbkwon : chart text annotation에서 특정 값이 존재하는지 확인
        /// </summary>
        /// <param name="x">찾을려고하는 포인트 X 값</param>
        /// <returns>존재할 경우 해당 인덱스, 존재하지 않을 경우 -1</returns>
        private int CheckTextAnnotationExist(int x)
        {
            int nIndex = -1;

            if (chartAnnotation.Count() <= 0)
                return nIndex;

            foreach (var item in chartAnnotation.Select((value, index) => (value, index)))
            {
                if ((int)(item.value.X1) == x)
                {
                    nIndex = item.index;
                    break;
                }
            }

            return nIndex;
        }

        //231017 sbkwon : 선택한 Peak Linear 추가
        public void AddSelectPeak(GraphData select)
        {
            bool bExist = false;    //기존에 선택여부 확인

            foreach (var peak in SelectPeakLine)
            {
                if (peak.X == select.X)
                {
                    bExist = true; break;
                }
            }

            if (bExist)
            {
                SelectPeakLine.Remove(select);  //기존 선택된것은 삭제
                //IsotopesTextAnnotation("", (int)select.X, (int)select.Y, eTextAnnotation.enDeselect);
            }
            else
            {
                SelectPeakLine.Add(select);     //기존 선택없으면 추가
                //IsotopesTextAnnotation("", (int)select.X, (int)select.Y, eTextAnnotation.enSelect);
            }

            Trace.WriteLine($"AddSelect : {select.X}, {select.Y}");

            //선택된 Peak를 영상화 하기 위해 Echk 계산 (Min , Max)
            var tempEchk = new List<AddListModeDataEchk>();
            foreach (var Echk in SelectPeakLine)
            {
                double fwhm = PeakSearching.CalcFWHM(Echk.X, Ref_x, Ref_fwhm, Ref_at_0);
                double MinE = Echk.X - fwhm;
                double MaxE = Echk.X + fwhm;


                AddListModeDataEchk addListModeDataEchk = new AddListModeDataEchk(MinE, MaxE);
                tempEchk.Add(addListModeDataEchk);

                //Trace.WriteLine($"{Echk.X}, {MinE}, {MaxE}");
            }

            //Trace.WriteLine("----------------------");

            //lahgi Echks에 추가
            LahgiApi.Echks = tempEchk;
        }

        //231025-1 sbkwon : 우선 순위 중복 방지
        private bool _Priority = true;
        public bool Priority { get => _Priority; set => _Priority = value; }

        //231016 sbkwon : Spectrum X=0 좌표 마우스 포인트 값 설정
        private int _mouseXPointZero = 45;
        public int MouseXPointZero
        {
            get => _mouseXPointZero;
            set => _mouseXPointZero = value;
        }

        //231016 sbkwon : Hit Peak X Range (Peak value를 기준으로 일정 범위 내에 hit로 간주할 값)
        private int _hitRange = 30;
        public int HitRange
        {
            get => _hitRange; set => _hitRange = value;
        }

        public bool SpectrumStart { get; set; } = false;    //231100-GUI

        private static bool isSpectrumLoaded = false;   //240926 spectrum upadte 만 진행되도록 - 프로그램 시작시 최초 1회 진행해야함. python load 때문

        private bool _spectrumSelectUpdate = false;

        public async void StatusUpdate(object? obj, EventArgs eventArgs)
        {
            // UI 스레드에서 실행되도록 Dispatcher 사용
            if (App.mainDispatcher != null && !App.mainDispatcher.CheckAccess())
            {
                App.mainDispatcher.Invoke(() => StatusUpdate(obj, eventArgs));
                return;
            }

            if (Monitor.TryEnter(this) is false)
            {
                return;
            }

            try
            {
                #region Test
                ////511kev check test
                //List<double> peaks = new List<double>
                //{
                //    1173.23, 1332.49,  511, 1274.53, 661.66
                //    //511
                //};
                //List<Isotope> DetectedIso = PeakSearching.GetIsotopesFromPeaks(peaks, 1, Ref_x, Ref_fwhm, Ref_at_0);
                //ObservableCollection<IsotopeInfo> isotopeInfos = new ObservableCollection<IsotopeInfo>();
                //PeakLine = new ObservableCollection<GraphData>();

                ////chartAnnotation.Clear();

                //foreach (Isotope iso in DetectedIso)
                //{
                //    string energy = "";
                //    foreach (double e in iso.PeakEnergy)
                //    {
                //        energy += e.ToString("0.");
                //        energy += " ";
                //    }

                //    double dY = 150;

                //    var newInfo = new IsotopeInfo(iso.IsotopeName, iso.IsotopeDescription, energy, Visibility.Hidden, Brushes.Black, new GraphData(iso.selectE, 150));//Y : Maxcount 찾아서 대입
                //    newInfo.PropertyChanged += isotopelInfoChanged;

                //    isotopeInfos.Add(newInfo);
                //    foreach (double e in iso.PeakEnergy)
                //    {
                //        GraphData peakData = new GraphData(e, dY);
                //        PeakLine.Add(peakData);

                //        //Text Annotation 추가
                //        _ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                //        {
                //            IsotopesTextAnnotation(iso.IsotopeName, (int)peakData.X, (int)peakData.Y, eTextAnnotation.enAdd);
                //        }));
                //    }

                //    //231017 sbkwon : sub Peak Energy => Peak Linear 추가
                //    foreach (double e in iso.SubPeakEnergy)
                //    {
                //        double fwhm = PeakSearching.CalcFWHM(e, Ref_x, Ref_fwhm, Ref_at_0);
                //        foreach (var data in peaks)
                //        {
                //            if (e - fwhm < data && e + fwhm > data)
                //            {
                //                GraphData peakData = new GraphData(e, dY);
                //                PeakLine.Add(peakData);

                //                //Text Annotation 추가
                //                // IsotopesTextAnnotation(iso.IsotopeName, (int)peakData.X, (int)peakData.Y, eTextAnnotation.enAdd);
                //                _ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                //                {
                //                    IsotopesTextAnnotation(iso.IsotopeName, (int)peakData.X, (int)peakData.Y, eTextAnnotation.enAdd);
                //                }));
                //            }
                //        }
                //    }
                //}

                //if (IsotopeInfos is not null)
                //{ 
                //    foreach (var info in IsotopeInfos)
                //    {
                //        info.PropertyChanged -= isotopelInfoChanged;
                //    }
                //}

                //IsotopeInfos = isotopeInfos;

                ////if (IsotopeInfos.ElementAtOrDefault(1) is not null)
                ////{
                ////    IsotopeInfos[1].IsSelected = Visibility.Visible;
                ////}

                ////231017 sbkwon : 우선순위
                //if (DetectedIso.Count > 0 && SelectPeakLine.Count == 0)
                //{
                //    int nIndex = 0;
                //    int nIsoIndex = 0;
                //    double selectE = 0;
                //    int priority = 9999;
                //    foreach (var item in DetectedIso)
                //    {
                //        bool bFind = false;
                //        foreach (var e in item.Priority.Select((value, index) => (value, index)))
                //        {
                //            if (priority > e.value)
                //            {
                //                priority = e.value;
                //                selectE = item.PeakEnergy[e.index];
                //                bFind = true;
                //            }
                //        }
                //        if (bFind) 
                //            nIndex = nIsoIndex;

                //        nIsoIndex++;
                //    }

                //    if (selectE != 0)
                //    {
                //        //GraphData selectData = new GraphData(selectE, 150 * 1.5);
                //        //_ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                //        //{
                //        //    AddSelectPeak(selectData);
                //        //}));

                //        if (IsotopeInfos.ElementAtOrDefault(nIndex) is not null)
                //        {
                //            IsotopeInfos[nIndex].IsSelected = Visibility.Visible;
                //        }
                //    }
                //}
                #endregion

                if (eventArgs is LahgiApiEnvetArgs)
                {
                    LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                    if (isSpectrumLoaded == false ||/*lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading || */
                        lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum)
                    {
                        isSpectrumLoaded = true;

                        //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"Spectrum start : {((LahgiApiEnvetArgs)eventArgs).State}");

                        SpectrumEnergyNasa? spectrum = null;
                        switch (_spectrumCases)
                        {
                            case eSpectrumCases.Scatter:
                                spectrum = LahgiApi.GetScatterSumSpectrumByTime(SpectrumTime);  //231100-GUI sbkwon
                                break;
                            case eSpectrumCases.Absorber:
                                spectrum = LahgiApi.GetAbsorberSumSpectrumByTime(SpectrumTime);  //231100-GUI sbkwon
                                break;
                            case eSpectrumCases.All:
                                spectrum = LahgiApi.GetSumSpectrumEnergyByTime(SpectrumTime);  //231100-GUI sbkwon
                                break;
                            case eSpectrumCases.ByChannel:
                                // 2채널 모드: UI 채널 번호를 실제 FPGA 채널 번호로 변환
                                spectrum = LahgiApi.GetSpectrumByTime(GetActualFpgaChannelNumber(), SpectrumTime);
                                break;
                        }

                        // 에너지 스펙트럼 디버깅 로그
                        var logger = LogManager.GetLogger(typeof(SpectrumViewModel));
                        if (spectrum == null)
                        {
                            logger.Warn($"에너지 스펙트럼 획득 실패: spectrum이 null입니다. SpectrumCases={_spectrumCases}, TimerBoolSpectrum={LahgiApi.TimerBoolSpectrum}");
                        }

                        if (spectrum != null && LahgiApi.TimerBoolSpectrum)
                        {
                            //logger.Info($"EnergySpectrum 업데이트 시작: 기존 Count={EnergySpectrum.Count}, 새 Count={spectrum.HistoEnergies.Count}");
                            EnergySpectrum = new ObservableCollection<HistoEnergy>(spectrum.HistoEnergies);
                            int maxCount = 0;
                            int sumCount = 0;
                            for (int i = 0; i < spectrum.HistoEnergies.Count; i++)
                            {
                                sumCount += spectrum.HistoEnergies[i].Count;
                                if (spectrum.HistoEnergies[i].Count > maxCount)
                                {
                                    maxCount = spectrum.HistoEnergies[i].Count;
                                }
                            }

                            if (SpectrumStart == false && sumCount > 50)
                            {
                                if (LahgiApi.SessionStopwatch != null)
                                {
                                    LahgiApi.SessionStopwatch.Restart();
                                }
                                if (TopButtonVM != null)
                                {
                                    TopButtonVM.StopText = "종료";
                                }
                                SpectrumStart = true;
                                LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"Spectrum Max count : {maxCount}");
                            }

                            MaxPeakCount = maxCount * 1.5;
                            MaxCount = maxCount + 100;    //231100-GUI sbkwon

                            //if (eventArgs is LahgiApiEnvetArgs)
                            //{
                            //    LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                            //    if (/*lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading || */lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum)
                            //    {
                                    
                            var espect = spectrum;

                            ObservableCollection<IsotopeInfo> isotopeTemp = new ObservableCollection<IsotopeInfo>();
                            List<Isotope> DetectedIso = PeakSearching.GetIsotopesFromPeaks(espect.FindPeaks(Ref_x, Ref_fwhm, Ref_at_0, Min_snr), 1, Ref_x, Ref_fwhm, Ref_at_0);

                            if (IsSpectrumAnalysisShow)
                            {
                                List<GraphData> graphDatas = new List<GraphData>();
                                for (int i = 0; i < espect.SnrData.Count; i++)
                                {
                                    GraphData graphData = espect.SnrData[i];
                                    graphDatas.Add(graphData);
                                }
                                SnrSpectrum = new ObservableCollection<GraphData>(graphDatas);

                                List<GraphData> graphDatas2 = new List<GraphData>();

                                for (int i = 0; i < espect.SnrData.Count; i++)
                                {
                                    GraphData graphData = new GraphData(espect.SnrData[i].X, espect.SnrData[i].Y);
                                    graphData.Y = Min_snr;
                                    graphDatas2.Add(graphData);
                                }
                                MinSnrLine = new ObservableCollection<GraphData>(graphDatas2);

                                PeakLine = new ObservableCollection<GraphData>(espect.PeakData);
                            }
                            else
                            {
                                SnrSpectrum = new ObservableCollection<GraphData>();
                                MinSnrLine = new ObservableCollection<GraphData>();
                                PeakLine = new ObservableCollection<GraphData>();
                            }

                            PeakLine = new ObservableCollection<GraphData>();
                            peakToMaxes = new List<PeakToMax>();    //240618
                            bool isBa133Found = false;
                            bool isCo60Found = false;

                            foreach (Isotope iso in DetectedIso)
                            {
                                List<double> Peak = new List<double>();
                                foreach (var item in iso.PeakEnergy)
                                {
                                    Peak.Add(item);
                                }
                                var newInfo = new IsotopeInfo(iso.IsotopeName, iso.IsotopeDescription, iso.sEnerge, Visibility.Hidden, Brushes.Black, iso.IsotopeElement, Peak/*iso.PeakEnergy*/);
                                newInfo.IsDetected = true; // 탐지된 핵종
                                // 재측정 시마다 자동 선택 해제 상태 리셋 (새로운 측정이므로)
                                // 단, 현재 측정에서 사용자가 해제한 경우는 유지
                                newInfo.IsAutoSelectDisabled = false;

                                if (IsotopeInfos.Count > 0)
                                {
                                    foreach (var item in IsotopeInfos)
                                    {
                                        if (item.IsoElement == newInfo.IsoElement)
                                        {
                                            // 수동 선택 상태는 유지 (사용자가 수동으로 선택한 경우)
                                            if (item.IsManualSelection && item.IsSelected == Visibility.Visible)
                                            {
                                                newInfo.IsSelected = item.IsSelected;
                                                newInfo.IsManualSelection = item.IsManualSelection;
                                            }
                                            // 사용자가 해제한 경우 IsAutoSelectDisabled 상태 유지
                                            if (item.IsAutoSelectDisabled)
                                            {
                                                newInfo.IsAutoSelectDisabled = true;
                                                LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"핵종 자동 선택 비활성화 상태 유지: {newInfo.Name}");
                                            }
                                            break;
                                        }
                                    }
                                }

                                newInfo.PropertyChanged += isotopelInfoChanged;

                                isotopeTemp.Add(newInfo);

                                foreach (double e in iso.PeakEnergy)
                                {
                                    //240618 S : max peak find : peak에서 찾아야 한다.
                                    double fwhm = PeakSearching.CalcFWHM(e, Ref_x, Ref_fwhm, Ref_at_0);
                                    double findPeak = e;
                                    double sigma = 1.0;
                                    double maxE = e + sigma * fwhm;
                                    double minE = e - sigma * fwhm;
                                    double maxY = 0;

                                    //peak 내에서 찾기.
                                    foreach (var gpeak in espect.PeakData)
                                    {
                                        if (gpeak.X > minE && gpeak.X < maxE) // 범위에 있는지
                                        {
                                            if (maxY <= gpeak.Y)
                                            {
                                                findPeak = gpeak.X;

                                                maxY = gpeak.Y;
                                            }
                                        }
                                        else if (maxE < gpeak.X)//상한 벗어날 경우 루프 종료
                                        {
                                            break;
                                        }
                                    }
                                    //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"SNR : {e}, {findPeak}, {minE}, {maxE}");


                                    GraphData peakData = new GraphData(findPeak, MaxPeakCount);
                                    peakToMaxes.Add(new PeakToMax(e, findPeak));
                                    //240618 E : max peak find : peak에서 찾아야 한다.
                                    //GraphData peakData = new GraphData(e, MaxPeakCount);  //기존
                                    PeakLine.Add(peakData);

                                    //_ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    //{
                                    //    IsotopesTextAnnotation(iso.IsotopeName, (int)peakData.X, (int)peakData.Y, eTextAnnotation.enAdd);
                                    //}));
                                }


                                if (iso.IsotopeElement == IsotopeElement.Co60)
                                {
                                    isCo60Found = true;
                                }
                                if (iso.IsotopeElement == IsotopeElement.Ba133)
                                {
                                    isBa133Found = true;
                                }
                            }

                            //240126 : 탐지 핵종이 없을 경우
                            if (DetectedIso.Count <= 0 && IsotopeInfos.Count > 0)
                            {
                                foreach (var item in IsotopeInfos)
                                {
                                    if (App.mainDispatcher is not null)
                                        _ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                        {
                                            item.IsSelected = Visibility.Hidden;
                                        }));
                                }
                            }

                            // 기존 IsotopeInfos에서 선택된 핵종(수동 선택 및 자동 선택 포함) 유지
                            // 또한 해제된 핵종의 IsAutoSelectDisabled 상태도 유지
                            foreach (var existing in IsotopeInfos)
                            {
                                // 선택된 핵종의 상태 유지
                                if (existing.IsSelected == Visibility.Visible)
                                {
                                    // 선택된 핵종이 isotopeTemp에 없으면 추가 (탐지되지 않은 핵종)
                                    // 또는 isotopeTemp에 있으면 선택 상태 유지
                                    bool found = false;
                                    foreach (var newInfo in isotopeTemp)
                                    {
                                        if (newInfo.Name == existing.Name)
                                        {
                                            // 기존 선택 상태 유지 (수동/자동 모두)
                                            // IsDetected는 isotopeTemp의 값(true)을 유지 (탐지된 핵종이므로)
                                            newInfo.IsSelected = existing.IsSelected;
                                            newInfo.IsManualSelection = existing.IsManualSelection;
                                            // IsDetected는 newInfo가 이미 true로 설정되어 있으므로 유지
                                            // existing.IsDetected는 수동 선택 시 false로 설정되었을 수 있으므로 무시
                                            newInfo.IsAutoSelectDisabled = existing.IsAutoSelectDisabled;
                                            found = true;
                                            break;
                                        }
                                    }
                                    if (!found)
                                    {
                                        // 탐지되지 않은 핵종은 isotopeTemp에 추가
                                        // IsDetected는 existing의 값(false)을 유지
                                        existing.PropertyChanged += isotopelInfoChanged;
                                        isotopeTemp.Add(existing);
                                    }
                                }
                                
                                // 해제된 핵종의 IsAutoSelectDisabled 상태도 유지
                                // (선택 여부와 관계없이 사용자가 해제한 상태는 유지)
                                if (existing.IsDetected && existing.IsAutoSelectDisabled)
                                {
                                    foreach (var newInfo in isotopeTemp)
                                    {
                                        if (newInfo.Name == existing.Name)
                                        {
                                            newInfo.IsAutoSelectDisabled = true;
                                            LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"해제된 핵종의 자동 선택 비활성화 상태 유지: {existing.Name}");
                                            break;
                                        }
                                    }
                                }
                            }

                            IsotopeInfos = isotopeTemp;
                            
                            // IsotopeTable UI 업데이트를 위한 이벤트 발생
                            OnPropertyChanged(nameof(IsotopeInfos));
                            
                            // 선택된 핵종들의 peak line과 echk를 다시 설정
                            // StatusUpdate에서 IsotopeInfos를 덮어씌울 때 이벤트가 발생하지 않을 수 있음
                            if (LahgiApi.MLEMRun == false)
                            {
                                foreach (var iso in IsotopeInfos)
                                {
                                    if (iso.IsSelected == Visibility.Visible)
                                    {
                                        // PropertyChanged 이벤트 핸들러 연결 확인
                                        iso.PropertyChanged -= isotopelInfoChanged;
                                        iso.PropertyChanged += isotopelInfoChanged;
                                        
                                        // peak line과 echk 설정
                                        var peakColor = iso.IsDetected ? System.Windows.Media.Brushes.Red : System.Windows.Media.Brushes.Green;
                                        
                                        foreach (var energy in iso.PeakEnergy)
                                        {
                                            SetSelectPeakLine(energy, true, iso.IsoElement, peakColor);
                                        }
                                        
                                        SetSelectEChk(iso.IsoElement, true);
                                        //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"StatusUpdate 후 선택 상태 복원 - 핵종 : {iso.Name}, index : {iso.IsoElement}, 탐지여부: {iso.IsDetected}, 수동선택: {iso.IsManualSelection}");
                                    }
                                }
                            }

                            //240123 : List Mode Data ECheck
                            bool bChangeFindIso = false;

                            if (DetectedIso.Count != IsotopesOld.Count)
                            {
                                bChangeFindIso = true;
                            }
                            else
                            {
                                for (int index = 0; index < DetectedIso.Count; index++)
                                {
                                    if (DetectedIso[index].IsotopeElement != IsotopesOld[index].IsotopeElement)
                                    {
                                        bChangeFindIso = true;
                                    }
                                }
                            }

                            if (bChangeFindIso)
                            {
                                var tempEchk = new List<AddListModeDataEchk>();
                                foreach (Isotope iso in DetectedIso)
                                {
                                    if (iso.IsotopeElement == IsotopeElement.K40 || iso.IsotopeElement == IsotopeElement.Tl208)
                                        continue;

                                    foreach (var energe in iso.PeakEnergy)
                                    {
                                        double fwhm = PeakSearching.CalcFWHM(energe, Ref_x, Ref_fwhm, Ref_at_0);
                                        double MinE = energe - fwhm;
                                        double MaxE = energe + fwhm;

                                        //Trace.WriteLine($"핵종 리스트 변경 : !!!!!!!!! :::: {energe}");
                                        //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"핵종 리스트 변경 - 핵종 수 : {DetectedIso.Count}, 에너지 : {energe}");
                                        tempEchk.Add(new AddListModeDataEchk(MinE, MaxE, iso.IsotopeElement));
                                    }
                                }

                                LahgiApi.Echks = tempEchk;
                            }
                            IsotopesOld = DetectedIso;
                            ////////

                            ////231017 sbkwon : 우선순위
                            //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"자동 핵종 선택 체크: DetectedIso.Count={DetectedIso.Count}, SelectPeakLineRed.Count={SelectPeakLineRed.Count}, SelectPeakLineGreen.Count={SelectPeakLineGreen.Count}");
                            
                            // 수동으로 선택된 핵종이 있는지 확인 (탐지되지 않은 핵종 포함)
                            bool hasManualSelection = false;
                            foreach (var iso in IsotopeInfos)
                            {
                                if (iso.IsSelected == Visibility.Visible && iso.IsManualSelection)
                                {
                                    hasManualSelection = true;
                                    LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"수동 선택된 핵종 발견: {iso.Name}, IsDetected={iso.IsDetected}");
                                    break;
                                }
                            }
                            
                            // 수동 선택이 없고, 탐지된 핵종이 있으며, 선택된 peak line이 없을 때만 자동 선택
                            // 단, 사용자가 해제한 핵종이 있는 경우 자동 선택하지 않음 (IsAutoSelectDisabled 체크)
                            bool hasAutoSelectDisabled = false;
                            foreach (var iso in IsotopeInfos)
                            {
                                if (iso.IsDetected && iso.IsAutoSelectDisabled)
                                {
                                    hasAutoSelectDisabled = true;
                                    LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"자동 선택 비활성화된 핵종 발견: {iso.Name}");
                                    break;
                                }
                            }
                            
                            if (DetectedIso.Count > 0 && !hasManualSelection && !hasAutoSelectDisabled && SelectPeakLineRed.Count == 0 && SelectPeakLineGreen.Count == 0)
                            {
                                LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"자동 핵종 선택 조건 만족: DetectedIso.Count={DetectedIso.Count}");
                                var tempSelectEchk = new List<SelectEchk>();
                                LahgiApi.SelectEchks = tempSelectEchk;

                                if (DetectedIso.Count == 1)
                                {
                                    if (IsotopeInfos.Count > 0 && DetectedIso[0].IsotopeElement != IsotopeElement.K40 && DetectedIso[0].IsotopeElement != IsotopeElement.Tl208)
                                    {
                                        if (App.mainDispatcher is not null)
                                            _ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                            {
                                                // 자동 선택이 해제되지 않은 경우에만 자동 선택
                                                if (!IsotopeInfos[0].IsAutoSelectDisabled)
                                                {
                                                    IsotopeInfos[0].IsSelected = Visibility.Visible;
                                                    IsotopeInfos[0].IsManualSelection = false; // 자동 선택
                                                }
                                            }));
                                        //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"자동 핵종 선택 : {DetectedIso[0].IsotopeName}");
                                    }
                                }
                                else
                                {
                                    double maxCountY = 0;
                                    IsotopeElement element = DetectedIso[0].IsotopeElement;

                                    foreach (var iso in DetectedIso)
                                    {
                                        if (iso.IsotopeElement == IsotopeElement.K40 || iso.IsotopeElement == IsotopeElement.Tl208)
                                            continue;

                                        double sum = 0;
                                        foreach (var energy in iso.PeakEnergy)
                                        {
                                            double fwhm = PeakSearching.CalcFWHM(energy, Ref_x, Ref_fwhm, Ref_at_0);
                                            double MinE = energy - fwhm;
                                            double MaxE = energy + fwhm;

                                            foreach (var pead in espect.PeakData)
                                            {
                                                if (MinE <= pead.X && MaxE >= pead.X)
                                                {
                                                    sum += pead.Y;
                                                    break;
                                                }
                                            }
                                        }

                                        if (sum > maxCountY)
                                        {
                                            maxCountY = sum;
                                            element = iso.IsotopeElement;
                                        }
                                        //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"자동 핵종 선택 bb : {iso.IsotopeElement}");
                                    }

                                    foreach (var iso in IsotopeInfos)
                                    {
                                        if (iso.IsoElement == element && element != IsotopeElement.K40 && maxCountY > 0 && element != IsotopeElement.Tl208)
                                        {
                                            if (App.mainDispatcher is not null)
                                                _ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                                {
                                                    // 자동 선택이 해제되지 않은 경우에만 자동 선택
                                                    if (!iso.IsAutoSelectDisabled)
                                                    {
                                                        iso.IsSelected = Visibility.Visible;
                                                        iso.IsManualSelection = false; // 자동 선택
                                                    }
                                                }));

                                            LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"자동 핵종 선택 : {iso.IsoElement}");
                                        }
                                    }
                                }
                            }

                            //selectpeak : 240912 느려짐 현상
                            //if (_spectrumSelectUpdate)
                            //{
                            if (App.mainDispatcher is not null)
                            {
                                _ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                {
                                    SelectPeakLineUpdate();
                                }));
                            }
                            //}

                            //_spectrumSelectUpdate = !_spectrumSelectUpdate;
                        }
                    }
                    else if(lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.MLEM)
                    {
                        if (LahgiApi.MLEMSelectNo > IsotopeInfos.Count || LahgiApi.MLEMSelectNo < 0)
                            return;

                        SpectrumEnergyNasa? spectrum = null;

                        EnergySpectrum = new ObservableCollection<HistoEnergy>();
                        
                        // 2채널 모드: UI 채널 번호를 실제 FPGA 채널 번호로 변환
                        spectrum = LahgiApi.GetSpectrumData((uint)SpectrumCases, (uint)GetActualFpgaChannelNumber());

                        if (spectrum != null)
                        {
                            EnergySpectrum = new ObservableCollection<HistoEnergy>(spectrum.HistoEnergies);

                            int maxCount = 0;
                            for (int i = 0; i < spectrum.HistoEnergies.Count; i++)
                            {
                                if (spectrum.HistoEnergies[i].Count > maxCount)
                                {
                                    maxCount = spectrum.HistoEnergies[i].Count;
                                }
                            }

                            MaxPeakCount = maxCount * 1.5;
                            MaxCount = maxCount + 100;    //231100-GUI sbkwon

                            var espectM = spectrum;

                            ObservableCollection<IsotopeInfo> isotopeTempM = new ObservableCollection<IsotopeInfo>();
                            List<Isotope> DetectedIsoM = PeakSearching.GetIsotopesFromPeaks(espectM.FindPeaks(Ref_x, Ref_fwhm, Ref_at_0, Min_snr), 1, Ref_x, Ref_fwhm, Ref_at_0);

                            PeakLine = new ObservableCollection<GraphData>();
                            peakToMaxes = new List<PeakToMax>();

                            foreach (Isotope iso in DetectedIsoM)
                            {
                                List<double> Peak = new List<double>();
                                foreach (var item in iso.PeakEnergy)
                                {
                                    Peak.Add(item);
                                }
                             
                                foreach (double e in iso.PeakEnergy)
                                {
                                    //240618 S : max peak find : peak에서 찾아야 한다.
                                    double fwhm = PeakSearching.CalcFWHM(e, Ref_x, Ref_fwhm, Ref_at_0);
                                    double findPeak = e;
                                    double sigma = 1.0;
                                    double maxE = e + sigma * fwhm;
                                    double minE = e - sigma * fwhm;
                                    double maxY = 0;

                                    //peak 내에서 찾기.
                                    foreach (var gpeak in espectM.PeakData)
                                    {
                                        if (gpeak.X > minE && gpeak.X < maxE) // 범위에 있는지
                                        {
                                            if (maxY <= gpeak.Y)
                                            {
                                                findPeak = gpeak.X;

                                                maxY = gpeak.Y;
                                            }
                                        }
                                        else if (maxE < gpeak.X)//상한 벗어날 경우 루프 종료
                                        {
                                            break;
                                        }
                                    }
                                    //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"SNR : {e}, {findPeak}, {minE}, {maxE}");


                                    GraphData peakData = new GraphData(findPeak, MaxPeakCount);
                                    peakToMaxes.Add(new PeakToMax(e, findPeak));

                                    PeakLine.Add(peakData);
                                }
                            }
                        }

                        //select list
                        IsotopeInfo select = IsotopeInfos[LahgiApi.MLEMSelectNo];
                        List<PeakToMax> tempMax = peakToMaxes;

                        SelectPeakLine = new ObservableCollection<GraphData>();

                        foreach (double e in select.PeakEnergy)
                        {
                            double peakMax = e;
                            foreach (var max in tempMax)
                            {
                                if (Math.Abs(max.peakE - e) < 0.5)
                                {
                                    peakMax = max.peakMax;
                                    break;
                                }
                            }

                            SelectPeakLine.Add(new GraphData(peakMax, MaxPeakCount)); 
                        }

                        if (App.mainDispatcher is not null)
                        _ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                        {
                            IsotopeInfos[LahgiApi.MLEMSelectNo].IsSelected = Visibility.Visible;
                        }));

                        if (App.mainDispatcher is not null)
                        {
                            _ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                            {
                                SelectPeakLineUpdate();
                            }));
                        }
                    }
                }
            }
            finally
            {
                //StatusUpdateMutex.ReleaseMutex();

                Monitor.Exit(this);
            }
        }

        private void SelectPeakLineUpdate()
        {
            if ((SelectPeakLineRed.Count > 0 || SelectPeakLineGreen.Count > 0) || LahgiApi.MLEMRun)   //250428 LahgiApi.MLEMRun 추가
            {
                if (IsotopeInfos.Count > 0)
                {
                    // 탐지된 핵종과 탐지되지 않은 핵종을 분리하여 업데이트
                    var redPeaks = new ObservableCollection<GraphData>();
                    var greenPeaks = new ObservableCollection<GraphData>();

                    List<PeakToMax> tempMax = peakToMaxes;
                    foreach (var iso in IsotopeInfos)
                    {
                        if(iso.IsSelected == Visibility.Visible)
                        {
                            var targetCollection = iso.IsDetected ? redPeaks : greenPeaks;
                            var peakColor = iso.IsDetected ? System.Windows.Media.Brushes.Red : System.Windows.Media.Brushes.Green;
                            
                            foreach (double e in iso.PeakEnergy)
                            {
                                double peakMax = e;
                                foreach (var max in tempMax)
                                {
                                    if (Math.Abs(max.peakE - e) < 0.5)
                                    {
                                        peakMax = max.peakMax;
                                        break;
                                    }
                                }                        
                                
                                targetCollection.Add(new GraphData(peakMax, MaxPeakCount, peakColor));
                            }
                        }
                    }
                    
                    SelectPeakLineRed = redPeaks;
                    SelectPeakLineGreen = greenPeaks;
                }
            }
        }

        private void SelectPeakLineAdd(double energe)
        {
            SelectPeakLine.Add(new GraphData(energe, MaxPeakCount));
        }

        private void SelectPeakLineDelete(double energe)
        {
            //Trace.WriteLine($"Peak Delete : {energe}");
            List<double> list = new List<double>();

            foreach (var item in SelectPeakLine)
            {
                if (Math.Abs(energe - item.X) < 0.5)
                    continue;

                list.Add(item.X);
            }

            foreach (var item in list)
            {
                SelectPeakLine.Add(new GraphData(item, MaxPeakCount));
            }
        }

        private IsotopeInfo? _selectedInfo;

        public IsotopeInfo? SelectedInfo
        {
            get => _selectedInfo;
            set
            {
                if (_selectedInfo != value)
                {
                    _selectedInfo = value;

                    OnPropertyChanged();

                    if (value is not null)
                    {
                        value.IsSelected = value.IsSelected is Visibility.Visible ? Visibility.Hidden : Visibility.Visible;

                        if (App.mainDispatcher is not null)
                            _ = App.mainDispatcher.InvokeAsync(() => SelectedInfo = null);
                    }
                }
            }
        }

        //240429 MLEM 추가
        public void isotopelInfoChanged(object? sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(IsotopeInfo.IsSelected) && sender is IsotopeInfo info)
            {
                if (LahgiApi.MLEMRun == false)
                {
                    if (info is not null)
                    {
                        if (info.IsSelected == Visibility.Visible)
                        {
                            // 탐지되지 않은 핵종은 초록색, 탐지된 핵종은 빨간색
                            var peakColor = info.IsDetected ? System.Windows.Media.Brushes.Red : System.Windows.Media.Brushes.Green;
                            
                            foreach (var item in info.PeakEnergy)
                            {
                                SetSelectPeakLine(item, true, info.IsoElement, peakColor);
                            }

                            //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"선택 - 핵종 : {info.Name}, index : {info.IsoElement}, 탐지여부: {info.IsDetected}");
                            SetSelectEChk(info.IsoElement, true);
                        }
                        else
                        {
                            // 탐지되지 않은 핵종은 초록색, 탐지된 핵종은 빨간색
                            var peakColor = info.IsDetected ? System.Windows.Media.Brushes.Red : System.Windows.Media.Brushes.Green;
                            
                            foreach (var item in info.PeakEnergy)
                            {
                                SetSelectPeakLine(item, false, info.IsoElement, peakColor);
                            }

                            LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"해제 - 핵종 : {info.Name}, index : {info.IsoElement}");
                            SetSelectEChk(info.IsoElement, false);
                            
                            // 방사선 영상 reconstruction 중지
                            StopReconstructionForIsotope(info.Name);
                        }
                    }
                }
                else
                {
                    // MLEMRun이 true일 때의 처리
                    // 탐지되지 않은 핵종의 수동 선택은 MLEM과 무관하므로 이 블록에서 처리하지 않음
                    if (info is not null && info.IsDetected)
                    {
                        // 탐지된 핵종만 MLEM 처리
                        if (TopButtonVM.IsMLEMRun == false && info.IsSelected == Visibility.Visible && info.MLEMResult.Equals("완료"))
                        {
                            for (var i = 0; i < IsotopeInfos.Count; i++)
                            {
                                if (info.IsoElement == IsotopeInfos[i].IsoElement)
                                {
                                    LahgiApi.MLEMSelectNo = i;
                                    //break;
                                }
                                else
                                {
                                    if (IsotopeInfos[i].IsSelected == Visibility.Visible)
                                        IsotopeInfos[i].IsSelected = Visibility.Hidden;
                                }
                            }
                            LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"Select MLEM - 핵종 : {info.Name}, index : {LahgiApi.MLEMSelectNo}");
                            LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.MLEM);
                        }
                        else if (info.IsSelected == Visibility.Visible && !info.MLEMResult.Equals("완료"))
                        {
                            // MLEMRun이 true이고 MLEMResult가 "완료"가 아닌 탐지된 핵종만 해제
                            info.IsSelected = Visibility.Hidden;
                        }
                    }
                    // 탐지되지 않은 핵종의 수동 선택은 MLEM과 무관하므로 아무 처리도 하지 않음
                }
            }
        }

        private void SetSelectEChk(IsotopeElement isoElement, bool bAdd)
        {
            var tempSelectEchk = new List<SelectEchk>();

            if (bAdd)
            {
                //기존 항목 있는지 확인
                bool find = false;
                foreach (var item in LahgiApi.SelectEchks)
                {
                    tempSelectEchk.Add(new SelectEchk(item.element));

                    if (item.element == isoElement)
                        find = true;
                }

                if (find == false)
                {
                    tempSelectEchk.Add(new SelectEchk(isoElement));

                    //Trace.WriteLine($"SelectEchk Add : ============= {tempSelectEchk.Count} :::: {isoElement}");
                    LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"SelectEchk Add - 핵종 : {isoElement}, SelectEchks 수: {tempSelectEchk.Count}");
                }
            }
            else
            {
                foreach (var item in LahgiApi.SelectEchks)
                {
                    if (item.element == isoElement)
                    {
                        //Trace.WriteLine($"SelectEchk Delete : ============= :::: {isoElement}");
                        LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"SelectEchk Delete - 핵종 : {isoElement}");
                        continue;
                    }
                    tempSelectEchk.Add(new SelectEchk(item.element));
                }
            }

            //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"SetSelectEChk 완료 - 핵종 수 : {tempSelectEchk.Count}, 핵종: {string.Join(", ", tempSelectEchk.Select(e => e.element))}");

            LahgiApi.SelectEchks = tempSelectEchk;
        }

        /// <summary>
        /// Echeck 설정
        /// </summary>
        /// <param name="energe"> 에너지 </param>
        /// <param name="bAdd"> true : 추가,, false : 삭제</param>
        private void SetSelectPeakLine(double energe, bool bAdd, IsotopeElement element, System.Windows.Media.Brush color = null)
        {
            // 색상이 지정되지 않으면 기본값(빨간색) 사용
            var peakColor = color ?? System.Windows.Media.Brushes.Red;
            bool isGreen = peakColor == System.Windows.Media.Brushes.Green;
            var targetCollection = isGreen ? SelectPeakLineGreen : SelectPeakLineRed;

            if (bAdd)
            {
                //240618 S :
                double peakE = energe;

                List<PeakToMax> tempMax = peakToMaxes;

                foreach (var e in tempMax)
                {
                    if(Math.Abs(e.peakE - energe) < 1)
                    {
                        peakE = e.peakMax;
                        break;
                    }
                }
                
                targetCollection.Add(new GraphData(peakE, MaxPeakCount+100, peakColor));
                //240618 E :
                //LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"Select Peak Add - 핵종 : {element}, 에너지 : {energe} => {peakE}, 색상: {(isGreen ? "초록" : "빨강")}");
            }
            else
            {
                List<double> list = new List<double>();

                //240618 S :
                double peakE = energe;

                List<PeakToMax> tempMax = peakToMaxes;

                foreach (var e in tempMax)
                {
                    if (Math.Abs(e.peakE - energe) < 1)
                    {
                        peakE = e.peakMax;
                        break;
                    }
                }

                foreach (var item in targetCollection)
                {
                    if (Math.Abs(peakE - item.X) < 0.5)
                    {
                        LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"Select Peak Delete - 핵종 : {element}, 에너지 : {energe}");
                        continue;
                    }
                    list.Add(item.X);
                }

                //240618 E :

                var newCollection = new ObservableCollection<GraphData>();
                foreach (var item in list)
                {
                    newCollection.Add(new GraphData(item, MaxPeakCount, peakColor));
                }

                if (isGreen)
                {
                    SelectPeakLineGreen = newCollection;
                }
                else
                {
                    SelectPeakLineRed = newCollection;
                }
            }
        }


        private ObservableCollection<HistoEnergy> _energySpetrum = new ObservableCollection<HistoEnergy>();

        public ObservableCollection<HistoEnergy> EnergySpectrum
        {
            get { return _energySpetrum; }
            set
            {
                _energySpetrum = value;
                OnPropertyChanged(nameof(EnergySpectrum));
            }
        }

        private ObservableCollection<HistoEnergy> _imagingEnergySpetrum = new ObservableCollection<HistoEnergy>();

        public ObservableCollection<HistoEnergy> ImagingEnergySpectrum
        {
            get { return _imagingEnergySpetrum; }
            set
            {
                _imagingEnergySpetrum = value;
                OnPropertyChanged(nameof(ImagingEnergySpectrum));
            }
        }
        public ObservableCollection<IsotopeInfo> _isotopeInfos = new ObservableCollection<IsotopeInfo>();
        public ObservableCollection<IsotopeInfo> IsotopeInfos
        {
            get
            {
                return _isotopeInfos;
            }
            set
            {
                if (_isotopeInfos.Count != value.Count)
                {
                    _isotopeInfos = value;
                    OnPropertyChanged(nameof(IsotopeInfos));
                }
                else
                {
                    for (int i = 0; i < IsotopeInfos.Count; i++)
                    {
                        if (value[i].Name != IsotopeInfos[i].Name)
                        {
                            _isotopeInfos = value;
                            OnPropertyChanged(nameof(IsotopeInfos));
                        }
                    }
                }
            }
        }
        private ObservableCollection<GraphData> _minSnrLine = new ObservableCollection<GraphData>();

        public ObservableCollection<GraphData> MinSnrLine
        {
            get { return _minSnrLine; }
            set
            {
                _minSnrLine = value;
                OnPropertyChanged(nameof(MinSnrLine));
            }
        }

        private ObservableCollection<GraphData> _snrSpectrum = new ObservableCollection<GraphData>();

        public ObservableCollection<GraphData> SnrSpectrum
        {
            get { return _snrSpectrum; }
            set
            {
                _snrSpectrum = value;
                OnPropertyChanged(nameof(SnrSpectrum));
            }
        }

        private ObservableCollection<GraphData> _peakLine = new ObservableCollection<GraphData>();

        public ObservableCollection<GraphData> PeakLine
        {
            get { return _peakLine; }
            set
            {
                _peakLine = value;
                OnPropertyChanged(nameof(PeakLine));
            }
        }

        private eSpectrumCases _spectrumCases = eSpectrumCases.Scatter;
        public eSpectrumCases SpectrumCases
        {
            get { return _spectrumCases; }
            set
            {
                _spectrumCases = value;
                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
                OnPropertyChanged(nameof(SpectrumCases));
                OnPropertyChanged(nameof(SpectrumMaxEnergy)); // 축 범위 변경 알림
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.SpectrumCases = value;
            }
        }

        // Scatter는 3000, Absorber와 All은 4000
        public double SpectrumMaxEnergy
        {
            get
            {
                return (_spectrumCases == eSpectrumCases.Absorber || _spectrumCases == eSpectrumCases.All) ? 4000 : 3000;
            }
        }

        //231024 sbkwon : Spectrum Type
        private eSpectrumType _spectrumType = eSpectrumType.Log;
        public eSpectrumType SpectrumType
        {
            get => _spectrumType;
            set
            {
                if (value == eSpectrumType.Log)
                {
                    VisibilityLog = Visibility.Visible;
                    VisibilityLinear = Visibility.Hidden;
                }
                else
                {
                    VisibilityLog = Visibility.Hidden;
                    VisibilityLinear = Visibility.Visible;
                }
                _spectrumType = value;
                OnPropertyChanged(nameof(SpectrumType));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.SpectrumType = value;
            }
        }

        //231100-GUI sbkwon
        private Visibility _visibilityLog = Visibility.Collapsed;
        public Visibility VisibilityLog
        {
            get => _visibilityLog;
            set
            {
                _visibilityLog = value;
                OnPropertyChanged(nameof(VisibilityLog));
            }
        }
        private Visibility _visibilityLinear = Visibility.Collapsed;
        public Visibility VisibilityLinear
        {
            get => _visibilityLinear;
            set
            {
                _visibilityLinear = value;
                OnPropertyChanged(nameof(VisibilityLinear));
            }
        }

        //231109-2 sbkwon
        private bool _isEcalUse = false;
        public bool IsEcalUse
        {
            get => _isEcalUse;
            set 
            { 
                _isEcalUse = value; 
                OnPropertyChanged(nameof(IsEcalUse));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.IsECalUse = value;
            }
        }

        // SelectIsEcalUse Command 추가
        private AsyncCommand? _selectIsEcalUse = null;
        public ICommand SelectIsEcalUse
        {
            get { return _selectIsEcalUse ?? (_selectIsEcalUse = new AsyncCommand(SelIsEcalUse)); }
        }

        private Task SelIsEcalUse() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectIsEcalUse Command 실행됨. 현재 값: {IsEcalUse}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                IsEcalUse = !IsEcalUse;
                System.Diagnostics.Debug.WriteLine($"SelectIsEcalUse Command 실행 후 값: {IsEcalUse}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.IsECalUse = IsEcalUse;
            });
        });

        //231109-2 sbkwon
        private int _intervalECalTime = 1;
        public int IntervalECalTime
        {
            get { return _intervalECalTime; }
            set
            {
                _intervalECalTime = value;
                App.GlobalConfig.ECalIntervalTime = value * 60000;
                OnPropertyChanged(nameof(IntervalECalTime));
            }
        }

        private int fpgaChannelNumber = 0;
        public int FpgaChannelNumber
        {
            get { return fpgaChannelNumber; }
            set
            {
                // 2채널 모드: 0(Scatter), 1(Absorber)만 유효
                if (value >= 0 && value <= 1)
                {
                    fpgaChannelNumber = value;
                    OnPropertyChanged(nameof(FpgaChannelNumber));
                    
                    // App.GlobalConfig에 자동 저장
                    App.GlobalConfig.FpgaChannelNumber = value;
                }
            }
        }

        // 2채널 모드: UI 채널 번호를 실제 FPGA 채널 번호로 변환
        public int GetActualFpgaChannelNumber()
        {
            // 0 → 4 (Scatter), 1 → 12 (Absorber)
            return fpgaChannelNumber == 0 ? 4 : 12;
        }

        public float Ref_x
        {
            get { return LahgiApi.Ref_x; }
            set
            {
                LahgiApi.Ref_x = value;
                OnPropertyChanged(nameof(Ref_x));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.Ref_x = value;
            }
        }

        public float Ref_fwhm
        {
            get { return LahgiApi.Ref_fwhm; }
            set
            {
                LahgiApi.Ref_fwhm = value;
                OnPropertyChanged(nameof(Ref_fwhm));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.Ref_fwhm = value;
            }
        }
        public float Ref_at_0
        {
            get { return LahgiApi.Ref_at_0; }
            set
            {
                LahgiApi.Ref_at_0 = value;
                OnPropertyChanged(nameof(Ref_at_0));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.Ref_at_0 = value;
            }
        }
        public float Min_snr
        {
            get { return LahgiApi.Min_snr; }
            set
            {
                LahgiApi.Min_snr = value;
                OnPropertyChanged(nameof(Min_snr));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.Min_snr = value;
            }
        }

        private bool _isSpectrumAnalysisShow = false;
        public bool IsSpectrumAnalysisShow
        {
            get => _isSpectrumAnalysisShow;
            set 
            { 
                // 값이 실제로 변경된 경우에만 처리
                if (_isSpectrumAnalysisShow == value)
                {
                    return;
                }
                
                _isSpectrumAnalysisShow = value; 
                
                // OnPropertyChanged 호출 제거 - Command에서만 UI 업데이트 처리
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.isSpectrumAnalysisShow = value;
            }
        }

        // SelectIsSpectrumAnalysisShow Command 추가
        private AsyncCommand? _selectIsSpectrumAnalysisShow = null;
        public ICommand SelectIsSpectrumAnalysisShow
        {
            get { return _selectIsSpectrumAnalysisShow ?? (_selectIsSpectrumAnalysisShow = new AsyncCommand(SelIsSpectrumAnalysisShow)); }
        }

        private Task SelIsSpectrumAnalysisShow() => Task.Run(() =>
        {
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                bool oldValue = IsSpectrumAnalysisShow;
                
                // 단순하게 토글 (중복 저장 제거)
                IsSpectrumAnalysisShow = !oldValue;
            });
        });

        // SelectSpectrumTypeLinear Command 추가
        private AsyncCommand? _selectSpectrumTypeLinear = null;
        public ICommand SelectSpectrumTypeLinear
        {
            get { return _selectSpectrumTypeLinear ?? (_selectSpectrumTypeLinear = new AsyncCommand(SelSpectrumTypeLinear)); }
        }

        private Task SelSpectrumTypeLinear() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectSpectrumTypeLinear Command 실행됨. 현재 값: {SpectrumType}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                SpectrumType = eSpectrumType.Linear;
                System.Diagnostics.Debug.WriteLine($"SelectSpectrumTypeLinear Command 실행 후 값: {SpectrumType}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.SpectrumType = eSpectrumType.Linear;
            });
        });

        // SelectSpectrumTypeLog Command 추가
        private AsyncCommand? _selectSpectrumTypeLog = null;
        public ICommand SelectSpectrumTypeLog
        {
            get { return _selectSpectrumTypeLog ?? (_selectSpectrumTypeLog = new AsyncCommand(SelSpectrumTypeLog)); }
        }

        private Task SelSpectrumTypeLog() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectSpectrumTypeLog Command 실행됨. 현재 값: {SpectrumType}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                SpectrumType = eSpectrumType.Log;
                System.Diagnostics.Debug.WriteLine($"SelectSpectrumTypeLog Command 실행 후 값: {SpectrumType}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.SpectrumType = eSpectrumType.Log;
            });
        });

        // SelectSpectrumCasesScatter Command 추가
        private AsyncCommand? _selectSpectrumCasesScatter = null;
        public ICommand SelectSpectrumCasesScatter
        {
            get { return _selectSpectrumCasesScatter ?? (_selectSpectrumCasesScatter = new AsyncCommand(SelSpectrumCasesScatter)); }
        }

        private Task SelSpectrumCasesScatter() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectSpectrumCasesScatter Command 실행됨. 현재 값: {SpectrumCases}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                SpectrumCases = eSpectrumCases.Scatter;
                System.Diagnostics.Debug.WriteLine($"SelectSpectrumCasesScatter Command 실행 후 값: {SpectrumCases}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.SpectrumCases = eSpectrumCases.Scatter;
            });
        });

        // SelectSpectrumCasesAbsorber Command 추가
        private AsyncCommand? _selectSpectrumCasesAbsorber = null;
        public ICommand SelectSpectrumCasesAbsorber
        {
            get { return _selectSpectrumCasesAbsorber ?? (_selectSpectrumCasesAbsorber = new AsyncCommand(SelSpectrumCasesAbsorber)); }
        }

        private Task SelSpectrumCasesAbsorber() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectSpectrumCasesAbsorber Command 실행됨. 현재 값: {SpectrumCases}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                SpectrumCases = eSpectrumCases.Absorber;
                System.Diagnostics.Debug.WriteLine($"SelectSpectrumCasesAbsorber Command 실행 후 값: {SpectrumCases}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.SpectrumCases = eSpectrumCases.Absorber;
            });
        });

        // SelectSpectrumCasesAll Command 추가
        private AsyncCommand? _selectSpectrumCasesAll = null;
        public ICommand SelectSpectrumCasesAll
        {
            get { return _selectSpectrumCasesAll ?? (_selectSpectrumCasesAll = new AsyncCommand(SelSpectrumCasesAll)); }
        }

        private Task SelSpectrumCasesAll() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectSpectrumCasesAll Command 실행됨. 현재 값: {SpectrumCases}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                SpectrumCases = eSpectrumCases.All;
                System.Diagnostics.Debug.WriteLine($"SelectSpectrumCasesAll Command 실행 후 값: {SpectrumCases}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.SpectrumCases = eSpectrumCases.All;
            });
        });

        // SelectSpectrumCasesByChannel Command 추가
        private AsyncCommand? _selectSpectrumCasesByChannel = null;
        public ICommand SelectSpectrumCasesByChannel
        {
            get { return _selectSpectrumCasesByChannel ?? (_selectSpectrumCasesByChannel = new AsyncCommand(SelSpectrumCasesByChannel)); }
        }

        private Task SelSpectrumCasesByChannel() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectSpectrumCasesByChannel Command 실행됨. 현재 값: {SpectrumCases}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                SpectrumCases = eSpectrumCases.ByChannel;
                System.Diagnostics.Debug.WriteLine($"SelectSpectrumCasesByChannel Command 실행 후 값: {SpectrumCases}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.SpectrumCases = eSpectrumCases.ByChannel;
            });
        });

        //231109-2 sbkwon
        private int _maxCount = 200;
        public int MaxCount
        {
            get => _maxCount;
            set
            {
                _maxCount = value;
                OnPropertyChanged(nameof(MaxCount));
            }
        }

        private double MaxPeakCount { get; set; } = 100;

        private uint _spectrumTime = 10;
        public uint SpectrumTime
        {
            get => _spectrumTime;
            set
            {
                _spectrumTime = value;
                OnPropertyChanged(nameof(SpectrumTime));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.SpectrumEffectTime = value;
            }
        }

        public List<Isotope> IsotopesOld { get; set; } = new List<Isotope>();

        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= StatusUpdate;

            LogManager.GetLogger(typeof(SpectrumViewModel)).Info("Unhandle");
        }

        public void ChangeIsotopeColor(int nNo = 0)
        {
            if (nNo > IsotopeInfos.Count || nNo < 0)
                return;

            IsotopeInfos[nNo].PropertyChanged += isotopelInfoChanged;

            if (App.mainDispatcher is not null)
                _ = App.mainDispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                {
                    IsotopeInfos[nNo].IsSelected = Visibility.Visible;
                }));
        }

        //240618
        public List<PeakToMax> peakToMaxes { get; set; } = new List<PeakToMax>();

        // 수동 핵종 선택 처리
        public void OnIsotopeManualSelection(string isotopeName)
        {
            try
            {
                LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"OnIsotopeManualSelection 호출됨: isotopeName={isotopeName}");
                // Energy Spectrum.cs에서 해당 핵종 정보 찾기
                var isotope = FindIsotopeByName(isotopeName);
                if (isotope != null)
                {
                    LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"핵종 정보 찾음: {isotope.IsotopeName}, PeakEnergy.Count={isotope.PeakEnergy.Count}");
                    // 수동 선택된 핵종을 위한 IsotopeInfo 생성
                    var manualIsotopeInfo = new IsotopeInfo(
                        isotope.IsotopeName,
                        isotope.IsotopeDescription,
                        isotope.sEnerge,
                        Visibility.Visible,
                        Brushes.Black,
                        isotope.IsotopeElement,
                        isotope.PeakEnergy
                    );

                    // 기존 IsotopeInfos에 추가 (중복 방지)
                    bool exists = false;
                    IsotopeInfo targetIsotopeInfo = null;
                    
                    foreach (var existing in IsotopeInfos)
                    {
                        if (existing.Name == isotopeName)
                        {
                            exists = true;
                            targetIsotopeInfo = existing;
                            // PropertyChanged 이벤트 핸들러가 연결되어 있는지 확인
                            existing.PropertyChanged -= isotopelInfoChanged;
                            existing.PropertyChanged += isotopelInfoChanged;
                            
                            existing.IsManualSelection = true; // 수동 선택
                            // IsDetected는 기존 값을 유지 (탐지된 핵종을 수동 선택한 경우 true 유지)
                            // 탐지되지 않은 핵종만 false로 설정
                            // existing.IsDetected는 이미 올바른 값이므로 변경하지 않음
                            
                            // IsSelected가 이미 Visible이면 이벤트가 발생하지 않으므로 강제로 변경
                            if (existing.IsSelected == Visibility.Visible)
                            {
                                // 이미 선택된 상태이므로 이벤트를 강제로 발생시키기 위해 Hidden으로 먼저 설정
                                existing.IsSelected = Visibility.Hidden;
                            }
                            existing.IsSelected = Visibility.Visible; // 이벤트 발생하여 isotopelInfoChanged 호출
                            LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"기존 핵종 선택 업데이트: {isotopeName}, IsSelected={existing.IsSelected}, IsDetected={existing.IsDetected}");
                            break;
                        }
                    }

                    if (!exists)
                    {
                        manualIsotopeInfo.IsManualSelection = true; // 수동 선택
                        manualIsotopeInfo.IsDetected = false; // 탐지되지 않은 핵종
                        manualIsotopeInfo.MLEMResult = ""; // MLEMResult 초기화 (탐지되지 않은 핵종은 MLEM 결과 없음)
                        manualIsotopeInfo.PropertyChanged += isotopelInfoChanged;
                        IsotopeInfos.Add(manualIsotopeInfo);
                        targetIsotopeInfo = manualIsotopeInfo;
                        LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"새 핵종 추가: {isotopeName}, IsSelected={manualIsotopeInfo.IsSelected}");
                    }

                    // isotopelInfoChanged가 호출되지 않을 수 있으므로 직접 peak line과 echk 설정
                    if (targetIsotopeInfo != null && targetIsotopeInfo.IsSelected == Visibility.Visible)
                    {
                        // 탐지 여부에 따라 색상 결정
                        var peakColor = targetIsotopeInfo.IsDetected ? System.Windows.Media.Brushes.Red : System.Windows.Media.Brushes.Green;
                        
                        foreach (var energy in targetIsotopeInfo.PeakEnergy)
                        {
                            SetSelectPeakLine(energy, true, targetIsotopeInfo.IsoElement, peakColor);
                        }
                        
                        LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"수동 선택 - 핵종 : {targetIsotopeInfo.Name}, index : {targetIsotopeInfo.IsoElement}, 탐지여부: {targetIsotopeInfo.IsDetected}, 색상: {(targetIsotopeInfo.IsDetected ? "빨강" : "초록")}");
                        SetSelectEChk(targetIsotopeInfo.IsoElement, true);
                    }

                    // 핵종 선택에 따른 방사선 영상 reconstruction 시작
                    LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"StartReconstructionForIsotope 호출 시작: {isotope.IsotopeName}");
                    StartReconstructionForIsotope(isotope);
                    LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"StartReconstructionForIsotope 호출 완료: {isotope.IsotopeName}");
                }
                else
                {
                    LogManager.GetLogger(typeof(SpectrumViewModel)).Warn($"핵종 정보를 찾을 수 없음: isotopeName={isotopeName}");
                }
            }
            catch (Exception ex)
            {
                LogManager.GetLogger(typeof(SpectrumViewModel)).Error($"수동 핵종 선택 중 오류: {ex.Message}, StackTrace: {ex.StackTrace}");
            }
        }

        // 수동 핵종 해제 처리
        public void OnIsotopeManualDeselection(string isotopeName)
        {
            try
            {
                // 기존 IsotopeInfos에서 해당 핵종 찾아서 해제
                foreach (var existing in IsotopeInfos)
                {
                    if (existing.Name == isotopeName)
                    {
                        existing.IsSelected = Visibility.Hidden;
                        // 탐지된 핵종을 사용자가 해제한 경우, 현재 측정에서만 자동 선택 비활성화
                        // 재측정 시에는 리셋되므로 다음 측정에서는 다시 자동 선택됨
                        if (existing.IsDetected)
                        {
                            existing.IsAutoSelectDisabled = true;
                            LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"핵종 해제로 인한 자동 선택 비활성화: {isotopeName}, IsDetected={existing.IsDetected}, IsManualSelection={existing.IsManualSelection}");
                        }
                        break;
                    }
                }

                // 방사선 영상 reconstruction 중지
                StopReconstructionForIsotope(isotopeName);
            }
            catch (Exception ex)
            {
                LogManager.GetLogger(typeof(SpectrumViewModel)).Error($"수동 핵종 해제 중 오류: {ex.Message}");
            }
        }

        // 핵종 이름으로 Isotope 정보 찾기
        private Isotope? FindIsotopeByName(string isotopeName)
        {
            try
            {
                // Energy Spectrum.cs의 IsotopeList에서 찾기
                var isotopeList = HUREL.Compton.RadioisotopeAnalysis.PeakSearching.IsotopeList;
                return isotopeList.FirstOrDefault(iso => iso.IsotopeName == isotopeName);
            }
            catch (Exception ex)
            {
                LogManager.GetLogger(typeof(SpectrumViewModel)).Error($"핵종 정보 찾기 중 오류: {ex.Message}");
                return null;
            }
        }

        // 선택된 핵종에 대한 방사선 영상 reconstruction 시작
        private void StartReconstructionForIsotope(Isotope isotope)
        {
            try
            {
                // 1. LMdata 생성 (List Mode Data)
                var lmData = new List<AddListModeDataEchk>();
                
                foreach (var energy in isotope.PeakEnergy)
                {
                    double fwhm = PeakSearching.CalcFWHM(energy, Ref_x, Ref_fwhm, Ref_at_0);
                    double minE = energy - fwhm;
                    double maxE = energy + fwhm;
                    
                    lmData.Add(new AddListModeDataEchk(minE, maxE, isotope.IsotopeElement));
                }

                // 2. LahgiApi에 LMdata 설정
                LahgiApi.Echks = lmData;
                
                // 3. 방사선 영상 reconstruction 시작
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Reconstruction);
                
                LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"핵종 {isotope.IsotopeName}에 대한 방사선 영상 reconstruction 시작");
            }
            catch (Exception ex)
            {
                LogManager.GetLogger(typeof(SpectrumViewModel)).Error($"방사선 영상 reconstruction 시작 중 오류: {ex.Message}");
            }
        }

        // 선택된 핵종에 대한 방사선 영상 reconstruction 중지
        private void StopReconstructionForIsotope(string isotopeName)
        {
            try
            {
                // Echks는 유지하고, SelectEchks만 업데이트하여 화면에서만 숨김
                // Reconstruction 이벤트를 발생시켜 ReconstructionImageViewModel에서 SelectEchks를 확인하여 영상 표시 여부 결정
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Reconstruction);
                LogManager.GetLogger(typeof(SpectrumViewModel)).Info($"핵종 {isotopeName}에 대한 방사선 영상 화면 숨김 처리 (Echks는 유지)");
            }
            catch (Exception ex)
            {
                LogManager.GetLogger(typeof(SpectrumViewModel)).Error($"방사선 영상 reconstruction 중지 중 오류: {ex.Message}");
            }
        }
    }
}
