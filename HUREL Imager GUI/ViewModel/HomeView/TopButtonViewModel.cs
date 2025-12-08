using AsyncAwaitBestPractices.MVVM;
using HelixToolkit.Wpf.SharpDX;
using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using HUREL_Imager_GUI.Components;
using log4net;
using Microsoft.Win32;
using OpenCvSharp;
using SharpDX;
using Syncfusion.Windows.PdfViewer;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Diagnostics.Tracing;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;

namespace HUREL_Imager_GUI.ViewModel
{
    public enum eMeasuremetType
    {
        Infinite,
        SettingTime,
    };

    public enum eFaultCheckType
    {
        None,
        RealTime,
        Precision,
    };

    public class TopButtonViewModel : ViewModelBase
    {
        public SpectrumViewModel SpectrumVM { get; set; }
        public ReconstructionImageViewModel ReconstructionVM { get; set; }
        public ReconstructionImageViewModel ReconstructionImageVM { get; set; }

        public ThreeDimensionalViewModel ThreeDimensionalVM { get; set; }

        //240206-Gain : 실시간 검사 문구 표시
        public DoseRateViewModel DoseRateVM { get; set; }

        private static readonly ILog logger = LogManager.GetLogger(typeof(TopButtonViewModel));

        private bool _startButtonEnabled = true;
        public bool StartButtonEnabled
        {
            get { return _startButtonEnabled; }
            set { _startButtonEnabled = value; OnPropertyChanged(nameof(StartButtonEnabled)); }
        }

        private bool _stopButtonEnabled = false;
        public bool StopButtonEnabled
        {
            get { return _stopButtonEnabled; }
            set { _stopButtonEnabled = value; OnPropertyChanged(nameof(StopButtonEnabled)); }
        }

        // 토글 버튼 활성화 상태
        public bool StartStopButtonEnabled
        {
            get { return StartButtonEnabled || StopButtonEnabled; }
        }

        // 리셋 버튼 활성화 상태 (측정 완료 상태에서만 활성화)
        public bool ResetButtonEnabled
        {
            get { return !IsRunning; }
        }

        private bool _isRunning = false;
        public bool IsRunning
        {
            get { return _isRunning; }
            set { _isRunning = value; OnPropertyChanged(nameof(IsRunning)); }
        }

        // 타이머 관련 속성들
        private string _timerDisplay = "00:00:00";
        public string TimerDisplay
        {
            get { return _timerDisplay; }
            set { _timerDisplay = value; OnPropertyChanged(nameof(TimerDisplay)); }
        }

        private DispatcherTimer _timer;

        private void InitializeTimer()
        {
            _timer = new DispatcherTimer();
            _timer.Interval = TimeSpan.FromSeconds(1);
            _timer.Tick += Timer_Tick;
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            if (IsRunning)
            {
                // LahgiApi의 SessionStopwatch에서 경과 시간을 가져와서 표시
                var elapsed = LahgiApi.SessionStopwatch.Elapsed;
                TimerDisplay = elapsed.ToString(@"hh\:mm\:ss");
            }
        }

        public void StartTimer()
        {
            if (_timer != null)
            {
                _timer.Start();
            }
        }

        public void StopTimer()
        {
            if (_timer != null)
            {
                _timer.Stop();
                TimerDisplay = "00:00:00";
            }
        }

        public TopButtonViewModel()
        {
            // ViewModel 초기화
            SpectrumVM = new SpectrumViewModel();
            ReconstructionVM = new ReconstructionImageViewModel();
            ReconstructionImageVM = ReconstructionVM; // 같은 인스턴스 사용
            ThreeDimensionalVM = new ThreeDimensionalViewModel();
            DoseRateVM = new DoseRateViewModel();

            //231100-GUI sbkwon
            FileName = App.GlobalConfig.SaveFileName;
            MeasuremetType = App.GlobalConfig.MeasurementType;
            MeasurementTime = App.GlobalConfig.MeasurementTime;
            RealTimeCheck = App.GlobalConfig.UseRealTimeCheck;  //24006
            RealTimeCheckCycleTime = App.GlobalConfig.RealTimeCycleTime;    //240206
            FaultDiagnosis = App.GlobalConfig.UseFaultDiagnosis;    //240206
            FaultDiagnosisMeasurementTime = App.GlobalConfig.FaultDiagnosisMeasurementTime; //240206

            LahgiApi.StatusUpdate += updateSatus;
            updateSatus(null, EventArgs.Empty);
            logger.Info("TopButtonViewModel Loaded");

            // 타이머 초기화
            InitializeTimer();

            StartPeakToValleyClear = true;

            System.Timers.Timer timer = new System.Timers.Timer();
            timer.Interval = 500;
            timer.Elapsed += UpdateTimerInvoker;
            timer.Start();
        }

        private void UpdateTimerInvoker(object? sender, ElapsedEventArgs e)
        {
            //231017 sbkwon : 경과 시간 측정
            if (IsRunning is true)
            {
                ElapsedTime = (uint)(LahgiApi.SessionStopwatch.ElapsedMilliseconds);

                //240222
                //TimeSpan ts = LahgiApi.SessionStopwatch.Elapsed;
                //MeasureTime = String.Format("{0:00}:{1:00}:{2:00}.{3:00}", ts.Hours, ts.Minutes, ts.Seconds, ts.Milliseconds / 10);

                //231017 sbkwon : 측정 시간 자동 종료 사용
                if (MeasuremetType == eMeasuremetType.SettingTime && ElapsedTime >= (MeasurementTime * 1000))
                    StopSession();
            }
        }

        private void updateSatus(object? obj, EventArgs args)
        {
            StartButtonEnabled = (LahgiApi.IsInitiate && LahgiApi.IsFpgaAvailable) && !LahgiApi.IsSessionStarting && !IsMLEMRun && !IsRunning;    //240429
            StopButtonEnabled = IsRunning;  // 측정 중일 때만 종료 버튼 활성화
            OnPropertyChanged(nameof(StartStopButtonEnabled));  // 토글 버튼 활성화 상태 업데이트
            OnPropertyChanged(nameof(ResetButtonEnabled));  // 리셋 버튼 활성화 상태 업데이트
            
            bool wasRunning = IsRunning;
            IsRunning = LahgiApi.IsSessionStart;
            
            // IsRunning 상태가 변경되었을 때 타이머 제어
            if (wasRunning != IsRunning)
            {
                if (IsRunning)
                {
                    StartTimer();
                }
                else
                {
                    StopTimer();
                }
                // IsRunning 변경 시 리셋 버튼 상태도 업데이트
                OnPropertyChanged(nameof(ResetButtonEnabled));
            }
            
            IsSaveBinary = LahgiApi.IsSavingBinary;

            if (args is LahgiApiEnvetArgs)
            {
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)args;
                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.FPGAUpdate)
                    LahgiStatusUpdate();
            }
        }

        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= updateSatus;
            StopTimer(); // 타이머 정리 추가
            StopSession();
            logger.Info("Unhandle StatusUpdate");
        }

        private string _fileName = "Default";
        public string FileName
        {
            get { return _fileName; }
            set
            {
                _fileName = value;
                OnPropertyChanged(nameof(FileName));
            }
        }

        private CancellationTokenSource? _sessionCancle;
        private AsyncCommand? startSessionCommand = null;
        public ICommand StartSessionCommand
        {
            get { return startSessionCommand ?? (startSessionCommand = new AsyncCommand(StartSession)); }
        }

        // 토글 버튼 Command (시작/종료를 하나의 Command로)
        private AsyncCommand? _startStopSessionCommand = null;
        public ICommand StartStopSessionCommand
        {
            get 
            { 
                return _startStopSessionCommand ?? (_startStopSessionCommand = new AsyncCommand(StartStopSession)); 
            }
        }

        private async Task StartStopSession()
        {
            if (IsRunning)
            {
                await StopSession();
            }
            else
            {
                await StartSession();
            }
        }

        // 토글 버튼 Command (시작/종료를 하나의 Command로)
        private AsyncCommand? _startStopSessionCommand = null;
        public ICommand StartStopSessionCommand
        {
            get 
            { 
                return _startStopSessionCommand ?? (_startStopSessionCommand = new AsyncCommand(StartStopSession)); 
            }
        }

        private async Task StartStopSession()
        {
            if (IsRunning)
            {
                await StopSession();
            }
            else
            {
                await StartSession();
            }
        }

        private bool bClicked = false;
        private async Task StartSession()
        {
            if (bClicked == false)
            {
                bClicked = true;

                IsMLEMEnable = true;

                StopText = "대기중";

                _sessionCancle = new CancellationTokenSource();
                SpectrumVM.SpectrumStart = false;


                SpectrumVM.SelectPeakLine.Clear();
                SpectrumVM.PeakLine.Clear();
                SpectrumVM.IsotopeInfos.Clear();
                SpectrumVM.IsotopesOld.Clear();

                LahgiApi.Echks = new List<AddListModeDataEchk>();
                ReconstructionVM.ComptonImgRGB = new BitmapImage();
                ReconstructionVM.CodedImgRGB = new BitmapImage();
                ReconstructionVM.HybridImgRGB = new BitmapImage();

                ReconstructionVM.MLEM2DVisibility = false;  //250107 MLEM 2D visibility init
                ReconstructionVM.SetVisibitity();   //250107 recon init

                LahgiApi.StatusCalMLEM = false;
                ReconstructionVM.MLEM2DRGB = false; //250115 init
                LahgiApi.MLEMDataLoad = false;

                ReconstructionVM.RGBDisplay();

                DoseRateVM.ClearDoseRateAlarm();  //240228 측정 시작시 Alarm clear

                //240105
                var line = new LineBuilder();
                line.AddLine(new Vector3(0, 0, 0), new Vector3(0, 0, 0));
                ThreeDimensionalVM.SLAMPoseInfo = line.ToLineGeometry3D();

                LahgiApi.SessionStopwatch.Reset();
                LahgiApi.SessionStopwatch.Start();

                // 타이머 시작
                StartTimer();

                //LahgiStatusUpdate();

                StartFaultDiagnosis = false;

                LahgiApi.MLEMRun = false;    //240429

                //240228 : 고장 검사 여부에 따라 호출 변경
                if (FaultDiagnosis == false)
                {
                    // 조건부로 Task 리스트 생성
                    var tasks = new List<Task> { LahgiApi.StartSessionAsync(FileName, _sessionCancle) };
                    
                    // 자동교정 실행 체크박스가 선택된 경우만 SetECal 실행
                    if (SpectrumVM.IsEcalUse)
                    {
                        tasks.Add(SetECal(_sessionCancle.Token));
                        logger.Info("SetECal 시작: 자동교정 실행 체크박스가 선택됨");
                    }
                    else
                    {
                        logger.Info("SetECal 건너뜀: 자동교정 실행 체크박스가 선택되지 않음");
                    }
                    
                    // 실시간 검사가 체크된 경우만 PeakToValley 실행
                    if (RealTimeCheck)
                    {
                        tasks.Add(PeakToValley(_sessionCancle.Token));
                        logger.Info("PeakToValley 시작: 실시간 검사가 체크됨");
                    }
                    else
                    {
                        logger.Info("PeakToValley 건너뜀: 실시간 검사가 체크되지 않음");
                    }
                    
                    await Task.WhenAll(tasks);
                }
                else
                {
                    ReconstructionVM.ClearFaultColor(); //240228

                    StartFaultDiagnosis = true;

                    //await Task.WhenAll(LahgiApi.StartSessionAsyncFD(FileName, _sessionCancle), FaultDiagnosisTask(_sessionCancle.Token));    //240228
                    await LahgiApi.StartSessionAsyncFD(FileName, _sessionCancle); //240228
                }
            }
        }

        private AsyncCommand? _stopSessionCommand = null;
        public ICommand StopSessionCommand
        {
            get { return _stopSessionCommand ?? (_stopSessionCommand = new AsyncCommand(StopSession)); }
        }

        private AsyncCommand? _screenshotCommand = null;
        public ICommand ScreenshotCommand
        {
            get { return _screenshotCommand ?? (_screenshotCommand = new AsyncCommand(Screenshot)); }
        }
        private Task Screenshot() => Task.Run(() =>
        {
            try
            {
                string saveFilePath;
                string saveFileName = FileName;
                if (string.IsNullOrWhiteSpace(saveFileName))
                {
                    saveFileName = "Screenshot";
                }

                // 측정 중이면 측정 데이터 저장 폴더에, 종료 상태면 이전 측정 데이터 저장 폴더에 저장
                string? directoryPath = System.IO.Path.GetDirectoryName(LahgiApi.GetFileSavePath());
                if (string.IsNullOrEmpty(directoryPath))
                {
                    // 측정 데이터 폴더가 없으면 기본 경로 사용
                    directoryPath = System.Windows.Forms.Application.StartupPath;
                }

                saveFilePath = directoryPath + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + saveFileName + "_Screenshot.png";

                // UI 스레드에서 창의 위치와 크기 가져오기 (DPI 스케일링 고려)
                int windowX = 0, windowY = 0, windowWidth = 0, windowHeight = 0;
                Application.Current.Dispatcher.Invoke(() =>
                {
                    if (App.CurrentMainWindow != null)
                    {
                        // DPI 스케일링 팩터 가져오기
                        PresentationSource? source = PresentationSource.FromVisual(App.CurrentMainWindow);
                        double dpiX = 1.0, dpiY = 1.0;
                        if (source != null && source.CompositionTarget != null)
                        {
                            System.Windows.Media.Matrix transform = source.CompositionTarget.TransformToDevice;
                            dpiX = transform.M11;
                            dpiY = transform.M22;
                        }

                        // 논리적 픽셀을 물리적 픽셀로 변환
                        windowX = (int)(App.CurrentMainWindow.Left * dpiX);
                        windowY = (int)(App.CurrentMainWindow.Top * dpiY);
                        windowWidth = (int)(App.CurrentMainWindow.ActualWidth * dpiX);
                        windowHeight = (int)(App.CurrentMainWindow.ActualHeight * dpiY);
                    }
                });

                // 측정 중이면 측정 데이터 저장 폴더에, 종료 상태면 이전 측정 데이터 저장 폴더에 저장
                string? directoryPath = System.IO.Path.GetDirectoryName(LahgiApi.GetFileSavePath());
                if (string.IsNullOrEmpty(directoryPath))
                {
                    // 측정 데이터 폴더가 없으면 기본 경로 사용
                    directoryPath = System.Windows.Forms.Application.StartupPath;
                }

                saveFilePath = directoryPath + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + saveFileName + "_Screenshot.png";

                // UI 스레드에서 창의 위치와 크기 가져오기 (DPI 스케일링 고려)
                int windowX = 0, windowY = 0, windowWidth = 0, windowHeight = 0;
                Application.Current.Dispatcher.Invoke(() =>
                {
                    if (App.CurrentMainWindow != null)
                    {
                        // DPI 스케일링 팩터 가져오기
                        PresentationSource? source = PresentationSource.FromVisual(App.CurrentMainWindow);
                        double dpiX = 1.0, dpiY = 1.0;
                        if (source != null && source.CompositionTarget != null)
                        {
                            System.Windows.Media.Matrix transform = source.CompositionTarget.TransformToDevice;
                            dpiX = transform.M11;
                            dpiY = transform.M22;
                        }

                        // 논리적 픽셀을 물리적 픽셀로 변환
                        windowX = (int)(App.CurrentMainWindow.Left * dpiX);
                        windowY = (int)(App.CurrentMainWindow.Top * dpiY);
                        windowWidth = (int)(App.CurrentMainWindow.ActualWidth * dpiX);
                        windowHeight = (int)(App.CurrentMainWindow.ActualHeight * dpiY);
                    }
                });

                // GUI 화면 창 전체 캡처
                ImgCapture imgCapture = new ImgCapture(windowX, windowY, windowWidth, windowHeight);
                imgCapture.SetPath(saveFilePath);
                imgCapture.DoCaptureImage();

                logger.Info($"Screenshot captured: {saveFilePath}");
            }
            catch (Exception ex)
            {
                logger.Error($"Screenshot error: {ex.Message}");
            }
        });

        // 리셋 Command 추가
        private AsyncCommand? _resetDisplayCommand = null;
        public ICommand ResetDisplayCommand
        {
            get { return _resetDisplayCommand ?? (_resetDisplayCommand = new AsyncCommand(ResetDisplay)); }
        }

        private Task ResetDisplay() => Task.Run(() =>
        {
            try
            {
                Application.Current.Dispatcher.Invoke(() =>
                {
                    // 측정 중이면 리셋하지 않음 (이미 버튼이 비활성화되어 있지만 이중 확인)
                    if (IsRunning)
                    {
                        logger.Warn("ResetDisplay called during measurement - ignored");
                        return;
                    }

                    // 에너지 스펙트럼 초기화 (초기 GUI 상태로 복원)
                    // 실제 화면에 표시되는 스펙트럼 데이터 초기화
                    SpectrumVM.EnergySpectrum = new ObservableCollection<HistoEnergy>();
                    SpectrumVM.ImagingEnergySpectrum = new ObservableCollection<HistoEnergy>();
                    
                    // 스펙트럼 분석 데이터 초기화
                    SpectrumVM.SnrSpectrum = new ObservableCollection<GraphData>();
                    SpectrumVM.MinSnrLine = new ObservableCollection<GraphData>();
                    
                    // 피크 라인 초기화
                    SpectrumVM.SelectPeakLine.Clear();
                    SpectrumVM.SelectPeakLineRed.Clear();
                    SpectrumVM.SelectPeakLineGreen.Clear();
                    SpectrumVM.PeakLine = new ObservableCollection<GraphData>();
                    
                    // 핵종 정보 초기화
                    SpectrumVM.IsotopeInfos = new ObservableCollection<IsotopeInfo>();
                    SpectrumVM.IsotopesOld.Clear();

                    // 스펙트럼 주석 초기화
                    SpectrumVM.chartAnnotation.Clear();
                    SpectrumVM.chartAnnotationLinear.Clear();
                    
                    // 스펙트럼 관련 값 초기화
                    SpectrumVM.MaxPeakCount = 0;
                    SpectrumVM.MaxCount = 0;

                    // 방사선 영상 초기화 (초기 상태로 복원)
                    ReconstructionVM.ComptonImgRGB = new BitmapImage();
                    ReconstructionVM.CodedImgRGB = new BitmapImage();
                    ReconstructionVM.HybridImgRGB = new BitmapImage();
                    ReconstructionVM.MLEM2DVisibility = false;
                    ReconstructionVM.MLEM2DRGB = false;
                    
                    // RGB 표시 업데이트 (초기 상태로 복원)
                    ReconstructionVM.RGBDisplay();

                    // 핵종 표 초기화는 IsotopeInfos를 Clear하면 자동으로 UpdateIsotopeDisplay가 호출됨
                    // SpectrumVM.PropertyChanged 이벤트가 발생하여 IsotopeTable이 자동으로 업데이트됨

                    logger.Info("Display reset completed to initial GUI state (data not deleted)");
                });
            }
            catch (Exception ex)
            {
                logger.Error($"Reset display error: {ex.Message}");
            }
        });

        private Task StopSession() => Task.Run(() =>
        {
            bClicked = false;

            IsMLEMEnable = false;

            //240228 : 고장 검사 여부에 따라 호출 변경
            if (FaultDiagnosis == false)    //일반 측정
            {
                //231019 sbkwon : spectrum capture - 종료시 delay 발생하여 위치 이동
                if (IsRunning)
                {
                    // GUI 화면 창 전체 캡처
                    string saveFileName = System.IO.Path.GetDirectoryName(LahgiApi.GetFileSavePath()) + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + FileName;
                    
                    // UI 스레드에서 창의 위치와 크기 가져오기 (DPI 스케일링 고려)
                    int windowX = 0, windowY = 0, windowWidth = 0, windowHeight = 0;
                    Application.Current.Dispatcher.Invoke(() =>
                    {
                        if (App.CurrentMainWindow != null)
                        {
                            // DPI 스케일링 팩터 가져오기
                            PresentationSource? source = PresentationSource.FromVisual(App.CurrentMainWindow);
                            double dpiX = 1.0, dpiY = 1.0;
                            if (source != null && source.CompositionTarget != null)
                            {
                                System.Windows.Media.Matrix transform = source.CompositionTarget.TransformToDevice;
                                dpiX = transform.M11;
                                dpiY = transform.M22;
                            }

                            // 논리적 픽셀을 물리적 픽셀로 변환
                            windowX = (int)(App.CurrentMainWindow.Left * dpiX);
                            windowY = (int)(App.CurrentMainWindow.Top * dpiY);
                            windowWidth = (int)(App.CurrentMainWindow.ActualWidth * dpiX);
                            windowHeight = (int)(App.CurrentMainWindow.ActualHeight * dpiY);
                        }
                    });

                    // 창 전체 캡처
                    ImgCapture imgCapture = new ImgCapture(windowX, windowY, windowWidth, windowHeight);
                    imgCapture.SetPath(saveFileName + "_screenshot.png");
                    imgCapture.DoCaptureImage();

                    LahgiApi.SessionStopwatch.Stop();
                    
                    // 타이머 정지
                    StopTimer();
                }

                ////test
                //IsRunning = false;

                //LahgiApi.IsSessionStarting = false; 
                _sessionCancle?.Cancel();
            }
            else    //고장 검사
            {
                LahgiApi.IsSessionStarting = false; _sessionCancle?.Cancel();

                if (StartFaultDiagnosis == true)
                {
                    BrokenTestByBkgGain();
                    StartFaultDiagnosis = false;
                }
            }
        });

        //231109-1 sbkwon : ECal
        private Task SetECal(CancellationToken cancellationToken) => Task.Run(async () =>
        {
            // StartSession에서 이미 IsEcalUse를 체크했으므로, 여기서는 실행 중 취소만 처리
            logger.Info($"SetECal 시작: cancellationToken.IsCancellationRequested={cancellationToken.IsCancellationRequested}");

            while (cancellationToken.IsCancellationRequested is false)
            {
                try
                {
                    await Task.Delay(App.GlobalConfig.ECalIntervalTime, cancellationToken);
                }
                catch (OperationCanceledException)
                {
                    logger.Info("SetECal 취소됨: Task.Delay 중 취소 요청");
                    return;
                }

                // Task.Delay 후 취소 확인
                if (cancellationToken.IsCancellationRequested)
                {
                    logger.Info("SetECal 취소됨: Delay 후 취소 확인");
                    return;
                }

                logger.Info("Update SetECal Start");
                //Scatter
                try
                {
                    // 취소 확인
                    if (cancellationToken.IsCancellationRequested)
                    {
                        logger.Info("SetECal 취소됨: ECal 업데이트 시작 전 취소 확인");
                        return;
                    }

                    uint setTime = (uint)SpectrumVM.IntervalECalTime * 60;  //단위 : 초
                    //logger.Info($"Update SetECal : {App.GlobalConfig.ECalIntervalTime}");
                    for (int i = 0; i < 1; ++i)
                    {
                        // 취소 확인
                        if (cancellationToken.IsCancellationRequested)
                        {
                            logger.Info("SetECal 취소됨: Scatter 채널 처리 중 취소 확인");
                            return;
                        }

                        //Scatter
                        //
                        // Energy
                        double refEnergy = 1461.0;
                        double refEnergyMin = refEnergy - 350;
                        double refEnergyMax = refEnergy + 350;

                        double k40Peak = -10.0;

                        SpectrumEnergyNasa? spectrum = null;
                        spectrum = LahgiApi.GetSpectrumByTime(i + 1, setTime);


                        var peaks = spectrum.FindPeaks(LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0, LahgiApi.Min_snr);

                        //logger.Info($"ECal CH{i + 4} : Peaks count - {peaks.Count()}");

                        //K40 Peak find
                        foreach (var e in peaks)
                        {
                            //logger.Info($"ECal CH{i + 4} : Find Peaks - {e}");
                            if (k40Peak < 0)
                            {
                                if (e > refEnergyMin & e < refEnergyMax)
                                {
                                    k40Peak = e;
                                }
                            }
                            else
                            {
                                if (Math.Abs(e - refEnergy) < Math.Abs(k40Peak - refEnergy))  //240131 : if (Math.Abs(e - k40Peak) < Math.Abs(k40Peak - refEnergy))
                                {
                                    k40Peak = e;
                                }
                            }
                        }

                        if (k40Peak > 0)
                        {
                            double ecalA = 0;
                            double ecalB = 0;
                            double ecalC = 0;

                            // 채널 1을 직접 사용 (C++ 코드에서 채널 1과 4 모두 지원)
                            uint ecalChannel = Convert.ToUInt32(i + 1);
                            LahgiApi.GetEcal(ecalChannel, ref ecalA, ref ecalB, ref ecalC);

                            //Only linearity affect
                            double diffPortion = refEnergy / k40Peak;

                            double newA = ecalA * diffPortion * diffPortion;
                            double newB = ecalB * diffPortion;
                            double newC = ecalC;

                            LahgiApi.SetEcal(ecalChannel, newA, newB, newC);
                            logger.Info($"Update Ecal Ch : {ecalChannel}, k40Peak : {k40Peak}, Old : {ecalA}, {ecalB}, {ecalC}");
                            logger.Info($"Update Ecal Ch : {ecalChannel}, k40Peak : {k40Peak}, New : {newA}, {newB}, {newC}");
                        }

                        Thread.Sleep(100);
                    }

                    // 취소 확인
                    if (cancellationToken.IsCancellationRequested)
                    {
                        logger.Info("SetECal 취소됨: Absorber 채널 처리 전 취소 확인");
                        return;
                    }

                    //Absorber
                    for (int i = 0; i < 1; ++i)
                    {
                        // 취소 확인
                        if (cancellationToken.IsCancellationRequested)
                        {
                            logger.Info("SetECal 취소됨: Absorber 채널 처리 중 취소 확인");
                            return;
                        }

                        //Scatter
                        // K40 Energy
                        double refEnergy = 1461.0;
                        double refEnergyMin = refEnergy - 350;
                        double refEnergyMax = refEnergy + 350;

                        double k40Peak = -10.0;

                        //logger.Info($"ECal CH{i + 12} : GetSpectrumByTime");

                        SpectrumEnergyNasa? spectrum = null;
                        spectrum = LahgiApi.GetSpectrumByTime(i + 9, setTime);

                        //logger.Info($"ECal CH{i + 12} : FindPeaks");
                        var peaks = spectrum.FindPeaks(LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0, LahgiApi.Min_snr);

                        //logger.Info($"ECal CH{i + 12} : Peaks count - {peaks.Count()}");
                        //K40 Peak find
                        foreach (var e in peaks)
                        {
                            //logger.Info($"ECal CH{i + 12} : Find Peaks - {e}");
                            if (k40Peak < 0)
                            {
                                if (e > refEnergyMin & e < refEnergyMax)
                                {
                                    k40Peak = e;
                                }
                            }
                            else
                            {
                                if (Math.Abs(e - refEnergy) < Math.Abs(k40Peak - refEnergy))    //240131
                                {
                                    k40Peak = e;
                                }
                            }
                        }

                        if (k40Peak > 0)
                        {
                            double ecalA = 0;
                            double ecalB = 0;
                            double ecalC = 0;

                            // 채널 9를 직접 사용 (C++ 코드에서 채널 9와 12 모두 지원)
                            uint ecalChannel = Convert.ToUInt32(i + 9);
                            LahgiApi.GetEcal(ecalChannel, ref ecalA, ref ecalB, ref ecalC);

                            //Only linearity affect
                            double diffPortion = refEnergy / k40Peak;

                            double newA = ecalA * diffPortion * diffPortion;
                            double newB = ecalB * diffPortion;
                            double newC = ecalC;

                            LahgiApi.SetEcal(ecalChannel, newA, newB, newC);
                            logger.Info($"Update Ecal Ch : {ecalChannel}, k40Peak : {k40Peak}, Old : {ecalA}, {ecalB}, {ecalC}");
                            logger.Info($"Update Ecal Ch : {ecalChannel}, k40Peak : {k40Peak}, New : {newA}, {newB}, {newC}");
                        }
                        Thread.Sleep(100);
                    }
                }
                catch (OperationCanceledException)
                {
                    logger.Info("SetECal 취소됨: ECal 업데이트 중 취소 요청");
                    return;
                }
                catch (Exception e)
                {
                    logger.Info($"Update Ecal Error" + e.Message);
                    // 예외 발생 시에도 취소 확인
                    if (cancellationToken.IsCancellationRequested)
                    {
                        logger.Info("SetECal 취소됨: 예외 처리 후 취소 확인");
                        return;
                    }
                }

            }
            logger.Info("SetECal 종료: while 루프 종료");
        },
        cancellationToken);

        //240206-Gain :
        private Task PeakToValley(CancellationToken cancellationToken) => Task.Run(async () =>
        {
            logger.Info($"Use PeakToValley : {RealTimeCheck}");
            if (RealTimeCheck == false) return;

            while (cancellationToken.IsCancellationRequested is false)
            {
                try
                {
                    await Task.Delay(RealTimeCheckCycleTime * 60 * 1000, cancellationToken);
                }
                catch (OperationCanceledException)
                {
                    logger.Info("PeakToValley 취소됨: Task.Delay 중 취소 요청");
                    return;
                }

                // Task.Delay 후 취소 확인
                if (cancellationToken.IsCancellationRequested)
                {
                    logger.Info("PeakToValley 취소됨: Delay 후 취소 확인");
                    return;
                }

                logger.Info("Start PeakToValley");

                //240228 : 측정 중 검사여부 변경 적용
                if (RealTimeCheck == false)
                {
                    DoseRateVM.ClearPeakToValleyWarning();
                    logger.Info("Termination PeakToValley");
                    break;
                }

                try
                {
                    // 취소 확인
                    if (cancellationToken.IsCancellationRequested)
                    {
                        logger.Info("PeakToValley 취소됨: 처리 시작 전 취소 확인");
                        return;
                    }

                    double refEnergy = 1461.0;
                    double refEnergyMin = refEnergy - 100;
                    double refEnergyMax = refEnergy + 100;

                    double refRate = 2.2;

                    uint setTime = (uint)RealTimeCheckCycleTime * 60;  //단위 : 초

                    int warning = 0;

                    int errorChennal = 0;   //240228

                    //Scatter
                    for (int i = 0; i < 1; ++i)
                    {
                        // 취소 확인
                        if (cancellationToken.IsCancellationRequested)
                        {
                            logger.Info("PeakToValley 취소됨: Scatter 채널 처리 중 취소 확인");
                            return;
                        }
                        //Scatter         
                        SpectrumEnergyNasa? spectrum = null;
                        spectrum = LahgiApi.GetSpectrumByTime(i + 1, setTime);

                        var peaks = spectrum.FindPeaks(LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0, LahgiApi.Min_snr);

                        double k40Peak = -10.0;

                        if (peaks.Count <= 0)
                            logger.Info($"PeakToValley - peaks count : {peaks.Count}, scatter CH{i + 1} skip");

                        //K40 Peak 찾기
                        foreach (var e in peaks)
                        {
                            if (k40Peak < 0)
                            {
                                if (e > refEnergyMin & e < refEnergyMax)
                                {
                                    k40Peak = e;
                                }
                            }
                            else
                            {
                                if (Math.Abs(e - refEnergy) < Math.Abs(k40Peak - refEnergy))    //240131
                                {
                                    k40Peak = e;
                                }
                            }
                        }

                        if (k40Peak > 0)
                        {
                            double binSize = spectrum.BinSize;


                            int k40PeakIndex = (int)Math.Floor(k40Peak / binSize) - 1;  //K40 Index 구하기 : FindPeaks에서의 BinEnerge는 bin 구간의 max값을 가지기때문에 -1을 해준다.
                            List<HistoEnergy> HistoEnergies = spectrum.HistoEnergies;

                            //logger.Info($"1 PeakToValley : {k40PeakIndex}, {HistoEnergies.Count}, {HistoEnergies[k40PeakIndex - 1].Count}, {HistoEnergies[k40PeakIndex].Count}, {HistoEnergies[k40PeakIndex + 1].Count}, {HistoEnergies[k40PeakIndex].Energy}");

                            if (k40PeakIndex > 0 && k40PeakIndex < (HistoEnergies.Count - 1))   //peak 찾은 경우
                            {
                                double k40PeakE = 0;
                                k40PeakE = spectrum.MaxPeakE(HistoEnergies[k40PeakIndex - 1].Count, HistoEnergies[k40PeakIndex].Count, HistoEnergies[k40PeakIndex + 1].Count, HistoEnergies[k40PeakIndex].Energy);

                                ////logger.Info("2 PeakToValley");

                                //logger.Info($"PeakToValley scatter ch{i + 1} - K40 Max PeakE : {k40PeakE}, {k40Peak}");

                                //Valley 찾기 : k40 피크 왼쪽으로 첫번째 찾기
                                if (k40PeakE > 0)
                                {
                                    //범위 설정
                                    double minE = (0.06 * Math.Sqrt(k40PeakE * 0.001)) * 1000 * 4;
                                    int sIndex = (int)Math.Floor(k40PeakE / binSize);   //k40
                                    int eIndex = (int)Math.Floor((k40PeakE - minE) / binSize) - 1;  //k40 왼쪽 한계


                                    logger.Info($"PeakToValley : {k40Peak} - {k40PeakE}, {minE}, {sIndex}, {eIndex}");

                                    if (eIndex < 0)
                                        eIndex = 0;

                                    bool valleyFind = false;
                                    double valleyY = 0.0;
                                    double k40PeakY = HistoEnergies[sIndex].Count;

                                    //k40 기준으로 왼쪽으로 검색하여 처음 나오는 Valley 찾기
                                    for (int index = sIndex - 1; index > eIndex; index--)
                                    {


                                        logger.Info($"PeakToValley : {index}, {HistoEnergies[index - 1].Count}, {HistoEnergies[index].Count}, {HistoEnergies[index + 1].Count}");


                                        if (HistoEnergies[index].Count < HistoEnergies[index - 1].Count &&
                                            HistoEnergies[index].Count < HistoEnergies[index + 1].Count)
                                        {
                                            valleyY = HistoEnergies[index].Count;
                                            valleyFind = true;

                                            logger.Info($"Find Peak : {k40PeakY}");
                                            logger.Info($"Find Valley : {valleyY}");

                                            break;
                                        }
                                    }

                                    //valley 찾은 경우
                                    if (valleyFind)
                                    {
                                        double peakTovalleyR = k40PeakY / valleyY;

                                        //2402228
                                        if (peakTovalleyR < refRate)    //문제
                                        {
                                            warning++;
                                            errorChennal |= 1 << (i + 1);   //240228 문제 채널 bit on
                                        }
                                    }
                                    else
                                    {
                                        logger.Info($"PeakToValley scatter ch{i + 1} - Valley not find");
                                        //logger.Info($"PeakToValley scatter ch{i + 1} - Valley not find : {sIndex} - {eIndex}");
                                    }
                                }
                            }
                        }
                        else
                            logger.Info($"PeakToValley scatter ch{i + 1} - Not Find K40 Peak, peaks count : {peaks.Count}");

                        Thread.Sleep(100);
                    }

                    // 취소 확인
                    if (cancellationToken.IsCancellationRequested)
                    {
                        logger.Info("PeakToValley 취소됨: Absorber 채널 처리 전 취소 확인");
                        return;
                    }

                    //Absorber
                    for (int i = 0; i < 1; ++i)
                    {
                        // 취소 확인
                        if (cancellationToken.IsCancellationRequested)
                        {
                            logger.Info("PeakToValley 취소됨: Absorber 채널 처리 중 취소 확인");
                            return;
                        }
                        SpectrumEnergyNasa? spectrum = null;
                        spectrum = LahgiApi.GetSpectrumByTime(i + 9, setTime);

                        var peaks = spectrum.FindPeaks(LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0, LahgiApi.Min_snr);

                        double k40Peak = -10.0;

                        if (peaks.Count <= 0)
                            logger.Info($"PeakToValley - peaks count : {peaks.Count}, absorber CH{i + 1} skip");

                        //K40 Peak 찾기
                        foreach (var e in peaks)
                        {
                            if (k40Peak < 0)
                            {
                                if (e > refEnergyMin & e < refEnergyMax)
                                {
                                    k40Peak = e;
                                }
                            }
                            else
                            {
                                if (Math.Abs(e - refEnergy) < Math.Abs(k40Peak - refEnergy))    //240131
                                {
                                    k40Peak = e;
                                }
                            }
                        }

                        if (k40Peak > 0)
                        {
                            double binSize = spectrum.BinSize;

                            int k40PeakIndex = (int)Math.Floor(k40Peak / binSize) - 1;  //K40 Index 구하기 : FindPeaks에서의 BinEnerge는 bin 구간의 max값을 가지기때문에 -1을 해준다.
                            List<HistoEnergy> HistoEnergies = spectrum.HistoEnergies;

                            if (k40PeakIndex > 0 && k40PeakIndex < (HistoEnergies.Count - 1))   //peak 찾은 경우
                            {
                                double k40PeakE = 0;
                                k40PeakE = spectrum.MaxPeakE(HistoEnergies[k40PeakIndex - 1].Count, HistoEnergies[k40PeakIndex].Count, HistoEnergies[k40PeakIndex + 1].Count, HistoEnergies[k40PeakIndex].Energy);

                                logger.Info($"PeakToValley absorber ch{i + 1} - K40 Max PeakE : {k40PeakE}, {k40Peak}");

                                //Valley 찾기 : k40 피크 왼쪽으로 첫번째 찾기
                                if (k40PeakE > 0)
                                {
                                    //범위 설정
                                    double minE = (0.06 * Math.Sqrt(k40PeakE * 0.001)) * 1000 * 4;
                                    int sIndex = (int)Math.Floor(k40PeakE / binSize);   //k40
                                    int eIndex = (int)Math.Floor((k40PeakE - minE) / binSize) - 1;  //k40 왼쪽 한계

                                    if (eIndex < 0)
                                        eIndex = 0;

                                    bool valleyFind = false;
                                    double valleyY = 0.0;
                                    double k40PeakY = HistoEnergies[sIndex].Count;

                                    //k40 기준으로 왼쪽으로 검색하여 처음 나오는 Valley 찾기
                                    for (int index = sIndex - 1; index > eIndex; index--)
                                    {
                                        if (HistoEnergies[index].Count < HistoEnergies[index - 1].Count &&
                                            HistoEnergies[index].Count < HistoEnergies[index + 1].Count)
                                        {
                                            valleyY = HistoEnergies[index].Count;
                                            valleyFind = true;
                                            break;
                                        }
                                    }

                                    //valley 찾은 경우
                                    if (valleyFind)
                                    {
                                        double peakTovalleyR = k40PeakY / valleyY;

                                        //2402228
                                        if (peakTovalleyR < refRate)    //정상
                                        {
                                            warning++;
                                            errorChennal |= 1 << (i + 9);   //240228 문제 채널 bit on
                                        }
                                    }
                                    else
                                        logger.Info($"PeakToValley absorver ch{i + 1} - Valley not find : {sIndex} - {eIndex}");
                                }
                            }
                        }
                        else
                            logger.Info($"PeakToValley absorver ch{i + 1} - Not Find K40 Peak, peaks count : {peaks.Count}");

                        Thread.Sleep(100);
                    }

                    //240228 PeakToValley
                    ErrorPeakToValleyCount = warning;
                    if (warning == 0)    //모두 정상
                    {
                        DoseRateVM.ClearPeakToValleyWarning();
                    }
                    else
                    {
                        //경고 문구 표시
                        DoseRateVM.SetPeakToValleyWarning();
                    }

                    SavePeakToValley(errorChennal);

                    logger.Info("End PeakToValley");
                }
                catch (OperationCanceledException)
                {
                    logger.Info("PeakToValley 취소됨: 처리 중 취소 요청");
                    return;
                }
                catch (Exception e)
                {
                    logger.Info($"Update PeakToValley Error" + e.Message);
                    // 예외 발생 시에도 취소 확인
                    if (cancellationToken.IsCancellationRequested)
                    {
                        logger.Info("PeakToValley 취소됨: 예외 처리 후 취소 확인");
                        return;
                    }
                }
            }
            logger.Info("PeakToValley 종료: while 루프 종료");
        },
        cancellationToken);

        /// <summary>
        /// 240228 : PeakToValley 검사 결과값 저장
        /// file name : 시간 + 통합결과(OK or FAIL).txt
        /// </summary>
        /// <param name="result"></param>
        private void SavePeakToValley(int result)
        {
            try
            {
                string totalResult = result > 0 ? "FAIL" : "OK";
                string saveFileName = System.IO.Path.GetDirectoryName(LahgiApi.GetFileSavePath()) + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + totalResult + ".txt";

                using (StreamWriter file = new StreamWriter(saveFileName))
                {
                    //Scatter
                    file.WriteLine($"Scatter 결과");
                    for (int i = 0; i < 1; i++)
                    {
                        int mask = 1 << (i + 1);
                        string chresult = ((result & mask) == mask) ? "FAIL" : "OK";
                        file.WriteLine($" CH{i} : {chresult}");
                    }
                    //Absorber
                    file.WriteLine($"Absorber 결과");
                    for (int i = 0; i < 1; i++)
                    {
                        int mask = 1 << (i + 9);
                        string chresult = ((result & mask) == mask) ? "FAIL" : "OK";
                        file.WriteLine($" CH{i} : {chresult}");
                    }
                }
            }
            catch (Exception e)
            {
                logger.Info($"Save PeakToValley Error" + e.Message);
            }
        }

        private Task FaultDiagnosisTask(CancellationToken cancellationToken) => Task.Run(async () =>
        {
            logger.Info($"Use FaultDiagnosisTask : {FaultDiagnosis}");
            if (FaultDiagnosis == false) return;

            while (cancellationToken.IsCancellationRequested is false)
            {
                await Task.Delay((FaultDiagnosisMeasurementTime * 60 * 1000));

                logger.Info("Start FaultDiagnosisTask");

                try
                {
                    BrokenTestByBkgGain();

                    logger.Info("End FaultDiagnosisTask");
                }
                catch (Exception e)
                {

                    logger.Info($"FaultDiagnosisTask Error" + e.Message);
                }
            }
        },
        cancellationToken);

        //240228 고장검사
        private void BrokenTestByBkgGain()
        {
            logger.Info($"Start BrokenTestByBkgGain : {FaultDiagnosis}");

            if (bStartBrokenTestByBkgGain == true)
                return;

            bStartBrokenTestByBkgGain = true;

            try
            {
                DoseRateVM.ClearFaultDiagnosis();   //문구 초기화

                string filepath;
                filepath = System.Windows.Forms.Application.StartupPath + "\\" + "Gain" + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss");
                Directory.CreateDirectory(filepath);

                //PMT Data Copy
                LahgiApi.CopyPMTData();

                //체널별측정 데이터를 가져온다.
                //Scatter
                logger.Info("----- Scatter -----");
                for (int i = 0; i < 1; ++i)
                {
                    GainOpt_bkg(i + 4, filepath);
                }

                logger.Info("----- Absorber -----");
                //Absorber
                for (int i = 0; i < 1; ++i)
                {
                    GainOpt_bkg(i + 12, filepath);
                }

                logger.Info($"End BrokenTestByBkgGain : {FaultDiagnosis}");
            }
            catch (Exception e)
            {

                logger.Error($"BrokenTestByBkgGain Error" + e.Message);
            }
            finally
            {
                bStartBrokenTestByBkgGain = false;
            }

        }

        //240228 고장검사 중복 방지
        private bool bStartBrokenTestByBkgGain { get; set; } = false;

        //240228 
        private void GainOpt_bkg(int nCh, string filepath)
        {
            try
            {
                logger.Info($"GainOPt_bkg CH{nCh} start");

                SpectrumEnergyNasa? spectrum = null;
                spectrum = LahgiApi.GetPMTEnergyData(nCh);  //선택 채널의 energy count 값 획득

                if (spectrum == null)
                {
                    logger.Warn($"CH {nCh} : Spectrum Energy null");
                    return;
                }

                double k40_min = 1260;
                double k40_max = 1660;
                double k40Peak = -10.0;

                //k40 피크 찾기 : min_snr을 50에서 1씩 감소하면서 찾기
                for (int min_snr = 50; min_snr >= 1; min_snr--)
                {
                    var peaks = spectrum.FindPeaks(LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0, min_snr);

                    bool bFindK40 = false;

                    if (peaks.Count > 0)
                    {
                        foreach (var e in peaks)
                        {
                            if (e >= k40_min && e <= k40_max) //일전 범위내에서 K40 피크 찾기
                            {
                                k40Peak = e;
                                bFindK40 = true;
                                logger.Info($"Find K40 Peak : CH{nCh}, min_snr : {min_snr}, K40 Peak : {k40Peak}");
                                break;
                            }
                        }

                        if (bFindK40)
                            break;
                    }
                }

                //k40 피크 찾은 경우
                if (k40Peak > 0)
                {
                    int[] usedPeak = new int[4] { 242, 609, 1461, 2615 };   //배경방사선의 242, 609, 1461, 2615 keV 피크가 위치한 채널을 선택하는 과정

                    double res_ADC = 0.065; //이벤트 범위를 선택하기 위한 ADC의 분해능
                    double[] lowerBound = new double[4];    //이벤트 선택할 때 채널의 하한값 4개
                    double[] upperBound = new double[4];    //이벤트 선택할 때 채널의 상한값 4개

                    //범위 설정
                    double refEnergy = 1461.0;
                    for (int k = 0; k < 1; k++)
                    {
                        lowerBound[k] = (usedPeak[k] / refEnergy) * k40Peak * (1 - res_ADC);
                        upperBound[k] = (usedPeak[k] / refEnergy) * k40Peak * (1 + res_ADC);
                    }

                    //usedPeak 4개 찾기
                    //float SNR_criteria = 0.05f; // 피크 탐지 4번 수행할 때의 기준 SNR
                    float SNR_criteria = 1.0f; // 피크 탐지 4번 수행할 때의 기준 SNR

                    var peaks = spectrum.FindPeaks(LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0, SNR_criteria);

                    if (peaks.Count > 0)
                    {
                        bool[] findUsedPeak = new bool[4] { false, false, false, false };

                        for (int kk = 0; kk < 1; kk++)
                        {
                            foreach (var e in peaks)
                            {
                                if (e >= lowerBound[kk] && e <= upperBound[kk])
                                {
                                    findUsedPeak[kk] = true;
                                    logger.Info($"Find used Peak : {usedPeak[kk]}");
                                    break;
                                }
                            }
                        }

                        if (findUsedPeak[0] && findUsedPeak[1] && findUsedPeak[2] && findUsedPeak[3])   //모든 used peak를 찾은 경우
                        {
                            double[] erange_bkg = new double[8];        //lower - upper bound 각각 2개씩
                            for (int kk = 0; kk < 1; kk++)
                            {
                                int nIndex = kk * 2;
                                erange_bkg[nIndex] = lowerBound[kk];
                                erange_bkg[nIndex + 1] = upperBound[kk];
                            }

                            List<double> CorrMatIn = LahgiApi.GetPMTCorrMatIn(nCh, usedPeak, erange_bkg);//usedPeak range를 이용하여 gain 획득

                            if (CorrMatIn.Count > 0)
                            {
                                List<double> CorrMatOut = new List<double>();
                                bool resultOK = false;
                                bool res = false;
                                //20회 반복
                                for (int kk = 1; kk <= 20; kk++)
                                {
                                    (CorrMatOut, res) = GainIter(nCh, CorrMatIn, usedPeak);   //일단 usdePeak 4개 모두 찾았다고 가정한다

                                    if (res == true) //gain 정상 계산 완료, CorrMatOut / CorrMain 비율이 일점 범위안에 들어왔는지 판단,
                                    {
                                        resultOK = true;
                                        for (int kk2 = 0; kk2 < 10; kk2++)
                                        {
                                            double ratio = CorrMatOut[kk2] / CorrMatIn[kk2];
                                            if (ratio > 1.003 && ratio < 0.997) //하나라도 비율이 3% 이상 차이나면 재 연산
                                            {
                                                resultOK = false;
                                                logger.Info($"gain 3% 이하에 들어오지 않음, 횟수 : {kk}");
                                                break;
                                            }
                                        }

                                        CorrMatIn = CorrMatOut;
                                    }
                                    else  //문제 발생 : usedPeak 4개를 찾지 못한 경우
                                    {
                                        resultOK = false;
                                        logger.Info("usedPeak 4개를 찾지 못함");
                                        break;
                                    }

                                    if (resultOK == true)   //정상일 경우 반복문 탈출
                                        break;
                                }

                                if (resultOK == true)    //검사 정상 완료
                                {
                                    //CorrMatOut 을 이용하여 현재 사용하고 있는 gain과 비교하여 화면에 표시 및 파일 저장
                                    CheckBroken(nCh, CorrMatIn);
                                    SaveGain(nCh, CorrMatIn, filepath);
                                }
                                else //검사 오류 
                                {
                                    //해당 체널 더 측정하라는 문구 출력
                                    DoseRateVM.SetFaultDiagnosisWarning();
                                }
                            }
                            else
                            {
                                //해당 체널 더 측정하라는 문구 출력
                                logger.Warn("CorrMatIn 계산 오류 : 1");
                                DoseRateVM.SetFaultDiagnosisWarning();
                            }
                        }
                        else
                        {
                            //해당 체널 더 측정하라는 문구 출력
                            logger.Warn($"Not Find Use Peak : {findUsedPeak[0]}, {findUsedPeak[1]}, {findUsedPeak[2]}, {findUsedPeak[3]}");
                            DoseRateVM.SetFaultDiagnosisWarning();
                        }
                    }
                    else
                    {
                        //해당 체널 더 측정하라는 문구 출력
                        logger.Info("Not Find Use Peak : peak count 0");
                        DoseRateVM.SetFaultDiagnosisWarning();
                    }

                    logger.Warn($"GainOPt_bkg CH{nCh} End");
                }
                else
                {
                    //해당 체널 더 측정하라는 문구 출력
                    logger.Warn("Not Find K40 Peak");
                    DoseRateVM.SetFaultDiagnosisWarning();
                }

            }
            catch (Exception e)
            {
                logger.Error($"GainOPt_bkg Error" + e.Message);
            }
        }

        //240228 계산된 gain을 가지고 다시 gain을 계산함.
        private (List<double>, bool) GainIter(int nCh, List<double> CorrMatIn, int[] usedPeak)
        {
            List<double> CorrMatOut = new List<double>();
            bool ret = false;

            //CorrMatin 을 이용하여 Spectrum(count) Data 획득
            SpectrumEnergyNasa? spectrum = null;
            spectrum = LahgiApi.GetPMTEnergyData(nCh, CorrMatIn);

            if (spectrum != null)
            {
                //usedPeak 찾기
                //float SNR_criteria = 2.0f;
                float SNR_criteria = 1.0f;
                var peaks = spectrum.FindPeaks(LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0, SNR_criteria);

                if (peaks.Count > 0)
                {
                    double[] lowerBound = new double[4];
                    double[] upperBound = new double[4];
                    bool[] findUsedPeak = new bool[4] { false, false, false, false };

                    double[] findPeak = new double[4];

                    for (int i = 0; i < 4; i++)
                    {
                        lowerBound[i] = usedPeak[i] * 0.95;
                        upperBound[i] = usedPeak[i] * 1.05;
                    }

                    //usedPeak 4개 찾기
                    for (int i = 0; i < 4; i++)
                    {
                        foreach (double peak in peaks)
                        {
                            if (peak > lowerBound[i] && peak < upperBound[i])
                            {
                                findPeak[i] = peak;
                                findUsedPeak[i] = true;

                                logger.Info($"GainIter : {peak}");

                                break;
                            }
                        }
                    }

                    if (findUsedPeak[0] && findUsedPeak[1] && findUsedPeak[2] && findUsedPeak[3])   //4개의 usedPeak를 찾은 경우
                    {
                        double[] erange_bkg = new double[8];        //lower - upper bound 각각 2개씩
                        for (int kk = 0; kk < 4; kk++)
                        {
                            int nIndex = kk * 2;    //nIndex : 0, 2, 4, 6
                            erange_bkg[nIndex] = findPeak[kk] - (0.06 * Math.Sqrt(findPeak[kk] * 0.001) * 1000); //단위 변환 필요 : lower //240315
                            erange_bkg[nIndex + 1] = findPeak[kk] + (0.06 * Math.Sqrt(findPeak[kk] * 0.001) * 1000); //단위 변환 필요 : upper //240315
                        }

                        CorrMatOut = LahgiApi.GetPMTCorrMatInBeforGain(nCh, usedPeak, erange_bkg, CorrMatIn);//240312 usedPeak range를 이용하여 gain 획득
                        ret = true;
                    }
                    else
                        logger.Info("GainIter : usedPeak 4개를 찾지 못함");
                }
                else
                    logger.Info("GainIter : FindPeaks fail");
            }

            return (CorrMatOut, ret);
        }

        //240228 : 고장 검사 완료 후 이상유무 판단
        private void CheckBroken(int nCh, List<double> CorrMat_bkg)
        {
            //load gain 호출
            var CorrMat_ref = LahgiApi.GetGainref(nCh);

            double[] changedratio = new double[9] { 1, 1, 1, 1, 1, 1, 1, 1, 1 };  //게인이 변화한 정도
            int brokenNo = 0;   // 변화량이 > 0.2 인 개수
            //gain 9개의 비율을 확인
            for (int i = 0; i < CorrMat_ref.Count; i++)
            {
                logger.Info($"Gain bkg : {CorrMat_bkg[i]} / Gain ref : {CorrMat_ref[i]}");
                changedratio[i] = (Math.Abs((CorrMat_bkg[i] * CorrMat_ref[i]) - CorrMat_ref[i])) / CorrMat_ref[i]*5.5;
                if (changedratio[i] > 0.2)  //범위
                    brokenNo++;
            }

            logger.Info($"Broken PMT number for Ch {nCh} : {brokenNo}");
            logger.Info($"Ratio : {changedratio[0]} : {changedratio[1]} : {changedratio[2]} : {changedratio[3]} : {changedratio[4]} : {changedratio[5]} : {changedratio[6]} : {changedratio[7]} : {changedratio[8]}");

            //ratio 별 결과 분류
            System.Windows.Media.Brush[] result = new System.Windows.Media.Brush[9];  //검사 결과 저장
            for (int i = 0; i < 9; i++)
            {
                if (changedratio[i] < 0.2)
                    result[i] = System.Windows.Media.Brushes.Green;
                else if (changedratio[i] >= 0.2 && changedratio[i] < 0.5)
                    result[i] = System.Windows.Media.Brushes.Yellow;
                else
                    result[i] = System.Windows.Media.Brushes.Red;
            }

            //화면 표시 - 2채널 모드: 채널 4(Scatter), 채널 12(Absorber)만 처리
            switch (nCh)
            {
                case 4:  //Scatter Ch1 (채널 4)
                    ReconstructionVM.ScatterCH1 = result;
                    break;
                case 12: //Absorber Ch1 (채널 12)
                    ReconstructionVM.AbsorberCH1 = result;
                    break;
            }

            if (brokenNo > 0) //결과 문구
                DoseRateVM.SetFaultDiagnosisError();
            else
                DoseRateVM.ClearFaultDiagnosis();
        }

        private void GainWindowClear()
        {
            System.Windows.Media.Brush[] result = new System.Windows.Media.Brush[9];  //검사 결과 저장
            for (int i = 0; i < 9; i++)
            {               
                   result[i] = System.Windows.Media.Brushes.White;
            }

            // 2채널 모드: 채널 4(Scatter), 채널 12(Absorber)만 초기화
            ReconstructionVM.ScatterCH1 = result;   // 채널 4
            ReconstructionVM.AbsorberCH1 = result;  // 채널 12
        }

        //240228 결과 저장 : Gain 폴더에 시분초 추가하여 넣기 - 측정중 여러번 계산하기 때문
        private void SaveGain(int nCh, List<double> CorrMat_bkg, string filepath)
        {
            string scatterSerial = "50777";
            string absorberSerial = "51516";

            if (nCh < 8)
            {
                int no = nCh - 4;

                filepath += "\\" + scatterSerial + "_Scint" + no.ToString() + ".csv";
            }
            else
            {
                int no = nCh - 12;
                filepath += "\\" + absorberSerial + "_Scint" + no.ToString() + ".csv";
            }

            logger.Info($"SaveGain Ch {nCh} : {filepath}");

            using (StreamWriter file = new StreamWriter(filepath))
            {
                file.WriteLine($"{CorrMat_bkg[0]},{CorrMat_bkg[1]},{CorrMat_bkg[2]},{CorrMat_bkg[3]},{CorrMat_bkg[4]}," +
                               $"{CorrMat_bkg[5]},{CorrMat_bkg[6]},{CorrMat_bkg[7]},{CorrMat_bkg[8]},{CorrMat_bkg[9]}");
            }
        }


        //240228 2D 영상 화면 표시 설정 : RGB, Recon Image 표시 여부
        private Visibility _visibitityReconstruction = Visibility.Visible;
        public Visibility VisibitityReconstruction
        {
            get => _visibitityReconstruction;
            set { _visibitityReconstruction = value; OnPropertyChanged(nameof(VisibitityReconstruction)); }
        }

        //240311
        private bool StartFaultDiagnosis { get; set; } = false;

        //240228 2D 영상 화면 표시 설정 : 고장검사 표시 여부
        private Visibility _visibitityFaultDiagnosis = Visibility.Hidden;
        public Visibility VisibitityFaultDiagnosis
        {
            get => _visibitityFaultDiagnosis;
            set { _visibitityFaultDiagnosis = value; OnPropertyChanged(nameof(VisibitityFaultDiagnosis)); }
        }

        //240228 화면표시 설정
        private void SetVisibitity()
        {
            VisibitityReconstruction = Visibility.Hidden;
            VisibitityFaultDiagnosis = Visibility.Hidden;

            if (FaultDiagnosis == true)
            {
                VisibitityFaultDiagnosis = Visibility.Visible;
            }
            else
                VisibitityReconstruction = Visibility.Visible;
        }

        private bool _isSaveBinary;
        public bool IsSaveBinary
        {
            get { return _isSaveBinary; }
            set
            {
                _isSaveBinary = value;
                LahgiApi.IsSavingBinary = value;
                OnPropertyChanged(nameof(IsSaveBinary));
            }
        }

        private AsyncCommand? _loadDataCommand;
        public ICommand LoadDataCommand
        {
            get { return _loadDataCommand ?? (_loadDataCommand = new AsyncCommand(LoadData)); }
        }

        //250605 - Status 버튼 기능
        private AsyncCommand? _statusCommand;
        public ICommand StatusCommand
        {
            get { return _statusCommand ?? (_statusCommand = new AsyncCommand(ShowStatus)); }
        }

        private bool _statusPopupShowFlag = false;
        public bool StatusPopupShowFlag
        {
            get { return _statusPopupShowFlag; }
            set { _statusPopupShowFlag = value; OnPropertyChanged(nameof(StatusPopupShowFlag)); }
        }

        //250605 - Setting 버튼 기능
        private AsyncCommand? _settingCommand;
        public ICommand SettingCommand
        {
            get { return _settingCommand ?? (_settingCommand = new AsyncCommand(ShowSetting)); }
        }

        private bool _settingPopupShowFlag = false;
        public bool SettingPopupShowFlag
        {
            get { return _settingPopupShowFlag; }
            set { _settingPopupShowFlag = value; OnPropertyChanged(nameof(SettingPopupShowFlag)); }
        }



        //240108 - 화면 캡쳐
        private AsyncCommand? _capture;
        public ICommand Capture
        {
            get { return _capture ?? (_capture = new AsyncCommand(ManualCapture)); }
        }

        //250605 - 파일 관련 기능
        private AsyncCommand? _fileCommand;
        public ICommand FileCommand
        {
            get { return _fileCommand ?? (_fileCommand = new AsyncCommand(FileOperations)); }
        }

        private Task FileOperations() => Task.Run(() =>
        {
            // 파일 관련 기능 구현
            // 예: 파일 열기, 저장, 설정 등
            logger.Info("File operations menu clicked");
        });

        private Task ShowStatus() => Task.Run(() =>
        {
            StatusPopupShowFlag = !StatusPopupShowFlag;
            logger.Info("Status popup toggled");
        });

        private Task ShowSetting() => Task.Run(() =>
        {
            SettingPopupShowFlag = !SettingPopupShowFlag;
            logger.Info("Setting popup toggled");
        });

        //250605 - Save 버튼 기능
        private AsyncCommand? _saveCommand;
        public ICommand SaveCommand
        {
            get { return _saveCommand ?? (_saveCommand = new AsyncCommand(ShowSave)); }
        }

        private bool _savePopupShowFlag = false;
        public bool SavePopupShowFlag
        {
            get { return _savePopupShowFlag; }
            set { _savePopupShowFlag = value; OnPropertyChanged(nameof(SavePopupShowFlag)); }
        }

        private Task ShowSave() => Task.Run(() =>
        {
            SavePopupShowFlag = !SavePopupShowFlag;
            logger.Info("Save popup toggled");
        });

        //250605 - Save 데이터 저장 및 취소 명령
        private AsyncCommand? _saveDataCommand;
        public ICommand SaveDataCommand
        {
            get { return _saveDataCommand ?? (_saveDataCommand = new AsyncCommand(SaveData)); }
        }

        private AsyncCommand? _cancelSaveCommand;
        public ICommand CancelSaveCommand
        {
            get { return _cancelSaveCommand ?? (_cancelSaveCommand = new AsyncCommand(CancelSave)); }
        }

        private Task SaveData() => Task.Run(() =>
        {
            // 폴더명이 비어있으면 기본값 설정
            if (string.IsNullOrWhiteSpace(FileName))
            {
                FileName = "Default";
            }

            // 설정 파일에 저장
            App.GlobalConfig.SaveFileName = FileName;
            
            // Save popup 닫기
            SavePopupShowFlag = false;
            
            logger.Info($"Data save settings updated. Folder name: {FileName}");
        });

        private Task CancelSave() => Task.Run(() =>
        {
            // Save popup 닫기
            SavePopupShowFlag = false;
            logger.Info("Save popup cancelled");
        });



        private Task ManualCapture() => Task.Run(() =>
        {
            string saveFileName;
            if (string.IsNullOrEmpty(System.IO.Path.GetDirectoryName(LahgiApi.GetFileSavePath())))
                saveFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + FileName;
            else
                saveFileName = System.IO.Path.GetDirectoryName(LahgiApi.GetFileSavePath()) + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + FileName;
            //string saveFileName = DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + FileName;



            ImgCapture imgCapture = new ImgCapture(0, 0, 1920, 1035);
            imgCapture.SetPath(saveFileName + ".png");
            imgCapture.DoCaptureImage();
        });

        //240429 : 정밀영상 진행 진행 여부
        public bool IsMLEMRun { get; set; } = false;

        //240429 : 정밀영상
        private AsyncCommand? _MLEM;
        public ICommand MLEM
        {
            get { return _MLEM ?? (_MLEM = new AsyncCommand(MLEMStart)); }
        }

        //250605 정밀영상 버튼 enable 
        private bool isMLEMEnable = false;
        public bool IsMLEMEnable
        {
            get { return isMLEMEnable; }
            set { isMLEMEnable = value; OnPropertyChanged(nameof(IsMLEMEnable)); }
        }

        private Task MLEMStart1() => Task.Run(() =>
        {
            LahgiApi.MLEMRun = true;
            //선택 핵종 나열
            ObservableCollection<IsotopeInfo> isotopeSel = new ObservableCollection<IsotopeInfo>();
            ObservableCollection<IsotopeInfo> isotopeNone = new ObservableCollection<IsotopeInfo>();

            foreach (var item in SpectrumVM.IsotopeInfos)
            {
                if (item.IsSelected == Visibility.Visible)
                    isotopeSel.Add(item);
                else
                    isotopeNone.Add(item);
            }

            MessageBox.Show($"선택 핵종 수 : {isotopeSel.Count}");

            //선택된 핵종만 실행

            //연산 완료
            foreach (var item in isotopeSel)
            {
                item.MLEMResult = "완료";
                item.IsSelected = Visibility.Hidden;
            }

            //결과 표시
            ObservableCollection<IsotopeInfo> isotopeTemp = new ObservableCollection<IsotopeInfo>();

            foreach (var item in isotopeSel)
            {
                isotopeTemp.Add(item);
            }
            foreach (var item in isotopeNone)
            {
                isotopeTemp.Add(item);
            }
            SpectrumVM.IsotopeInfos = new ObservableCollection<IsotopeInfo>();

            SpectrumVM.IsotopeInfos = isotopeTemp;
        });

        //240429 : 정밀영상
        private async Task MLEMStart()
        {
            if (FaultDiagnosis == false)    //일반 측정
            {
                IsMLEMEnable = false;

                bClicked = false;

                IsMLEMRun = true;
                LahgiApi.MLEMRun = true;
                //231019 sbkwon : spectrum capture - 종료시 delay 발생하여 위치 이동
                string saveFileName = "";
                if (IsRunning)
                {
                    // GUI 화면 창 전체 캡처
                    saveFileName = System.IO.Path.GetDirectoryName(LahgiApi.GetFileSavePath()) + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + FileName;
                    
                    // UI 스레드에서 창의 위치와 크기 가져오기 (DPI 스케일링 고려)
                    int windowX = 0, windowY = 0, windowWidth = 0, windowHeight = 0;
                    await Application.Current.Dispatcher.InvokeAsync(() =>
                    {
                        if (App.CurrentMainWindow != null)
                        {
                            // DPI 스케일링 팩터 가져오기
                            PresentationSource? source = PresentationSource.FromVisual(App.CurrentMainWindow);
                            double dpiX = 1.0, dpiY = 1.0;
                            if (source != null && source.CompositionTarget != null)
                            {
                                System.Windows.Media.Matrix transform = source.CompositionTarget.TransformToDevice;
                                dpiX = transform.M11;
                                dpiY = transform.M22;
                            }

                            // 논리적 픽셀을 물리적 픽셀로 변환
                            windowX = (int)(App.CurrentMainWindow.Left * dpiX);
                            windowY = (int)(App.CurrentMainWindow.Top * dpiY);
                            windowWidth = (int)(App.CurrentMainWindow.ActualWidth * dpiX);
                            windowHeight = (int)(App.CurrentMainWindow.ActualHeight * dpiY);
                        }
                    });

                    // 창 전체 캡처
                    ImgCapture imgCapture = new ImgCapture(windowX, windowY, windowWidth, windowHeight);
                    imgCapture.SetPath(saveFileName + "_screenshot.png");
                    imgCapture.DoCaptureImage();

                    LahgiApi.SessionStopwatch.Stop();
                }

                //LahgiApi.IsSessionStarting = false; 
                _sessionCancle?.Cancel();
                StartFaultDiagnosis = false;

                await Task.Delay(1000);

                //system matrix file 존재 확인
                //string systemMPath = System.Windows.Forms.Application.StartupPath + "\\" + "config" + "\\" + "AngleInteractionProbData(NaIScatter,130,1,300,5,dist,3m).bin"; //20250416
                string systemMPath = System.Windows.Forms.Application.StartupPath + "\\" + "config" + "\\" + "AngleInteractionProbData(NaIScatter,120,1,300,5).bin";

                

                if (File.Exists(systemMPath) == false)
                {
                    MessageBox.Show("Not exist system matrix file.");
                    IsMLEMRun = false;
                    return;
                }

                //정밀영상 검사
                //선택된 핵종별 점검 시작
                //선택 핵종 나열
                ObservableCollection<IsotopeInfo> isotopeSel = new ObservableCollection<IsotopeInfo>();
                ObservableCollection<IsotopeInfo> isotopeNone = new ObservableCollection<IsotopeInfo>();

                foreach (var item in SpectrumVM.IsotopeInfos)
                {
                    if (item.IsSelected == Visibility.Visible)
                        isotopeSel.Add(item);
                    else
                        isotopeNone.Add(item);
                }

                //역순으로
                //for (int i = SpectrumVM.IsotopeInfos.Count -1; i >= 0; i--)
                //{
                //    var item = SpectrumVM.IsotopeInfos[i];
                //    if (item.IsSelected == Visibility.Visible)
                //        isotopeSel.Add(item);
                //    else
                //        isotopeNone.Add(item);
                //}

                logger.Info($"정밀영상 핵종 개수 : {isotopeSel.Count}");    //test log
                //1. Data Load

                bool breturn = await Task.Run(() => LahgiApi.LoadMLEMData(saveFileName, "", false));

                logger.Info($"정밀영상 Data Load 완료");    //test log

                //2. 연산
                int nCount = 0;
                LahgiApi.MLEMSelectNo = 0;
                foreach (var Echk in isotopeSel)
                {
                    LahgiApi.UIMessageUpdateInvoke(null, $"{nCount + 1}번째 정밀 영상 계산 중 ({Echk.Name})");

                    List<double> energy = new List<double>();
                    List<double> min = new List<double>();
                    List<double> max = new List<double>();

                    foreach (var item in Echk.PeakEnergy)
                    {
                        double fwhm = PeakSearching.CalcFWHM(item, LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0);
                        double MinE = item - fwhm;
                        double MaxE = item + fwhm;

                        energy.Add(item);
                        min.Add(MinE);
                        max.Add(MaxE);
                    }

                    //energy 역으로
                    //for (int i = Echk.PeakEnergy.Count - 1; i >= 0; i--)
                    //{
                    //    var item = Echk.PeakEnergy[i];
                    //    double fwhm = PeakSearching.CalcFWHM(item, LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0);
                    //    double MinE = item - fwhm;
                    //    double MaxE = item + fwhm;

                    //    energy.Add(item);
                    //    min.Add(MinE);
                    //    max.Add(MaxE);
                    //}

                    bool showresult = true;

                    if (isotopeSel.Count > 1 && (isotopeSel.Count - 1) != nCount)
                        showresult = false;

                    LahgiApi.MLEMSelectNo = 0;

                    await LahgiApi.CalMLEM(systemMPath, energy, min, max, ReconstructionVM.MinValuePortion, showresult);


                    if (breturn == false)
                    {
                        LahgiApi.UIMessageUpdateInvoke(null, "정밀 영상 데이터 로드 실패");
                        continue;
                    }

                    if(LahgiApi.StatusCalMLEM == false) //250107
                    {
                        LahgiApi.UIMessageUpdateInvoke(null, "정밀 영상 연산 실패");
                        continue;
                    }

                    logger.Info($"정밀영상 연산 완료");    //test log

                    nCount++;

                    Echk.MLEMResult = "완료";

                    //완료 확인 필요                 
                    ObservableCollection<IsotopeInfo> isotopeTemp = new ObservableCollection<IsotopeInfo>();

                    foreach (var item in isotopeSel)
                    {
                        //item.PropertyChanged += SpectrumVM.isotopelInfoChanged;
                        isotopeTemp.Add(item);
                    }

                    isotopeTemp[0].IsSelected = Visibility.Visible;

                    foreach (var item in isotopeNone)
                    {
                        isotopeTemp.Add(item);
                    }
                    SpectrumVM.IsotopeInfos = new ObservableCollection<IsotopeInfo>();


                    SpectrumVM.IsotopeInfos = isotopeTemp;

                    //SpectrumVM.IsotopeInfos[0].IsSelected = Visibility.Visible;
                    //SpectrumVM.ChangeIsotopeColor(LahgiApi.MLEMSelectNo);


                    //3. 화면 출력 : list, 3D, 2D

                }

                LahgiApi.UIMessageUpdateInvoke(null, "정밀 영상 계산 완료");

                ObservableCollection<IsotopeInfo> isotopeTemp1 = new ObservableCollection<IsotopeInfo>();

                foreach (var item in isotopeSel)
                {
                    item.IsSelected = Visibility.Hidden;
                    isotopeTemp1.Add(item);
                }
                foreach (var item in isotopeNone)
                {
                    item.IsSelected = Visibility.Hidden;
                    isotopeTemp1.Add(item);
                }
                SpectrumVM.IsotopeInfos = new ObservableCollection<IsotopeInfo>();

                SpectrumVM.IsotopeInfos = isotopeTemp1;

                IsMLEMRun = false;
                StartButtonEnabled = (LahgiApi.IsInitiate && LahgiApi.IsFpgaAvailable) && !LahgiApi.IsSessionStarting && !IsMLEMRun;
                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.MLEM);
            }
            else    //고장 검사
            {
                MessageBox.Show("고장 검사 설정되어 있습니다. 설정 해제 후 다시 시도하세요.");
            }
        }

        //231100-GUI sbkwon
        private AsyncCommand? _popupSetting;
        public ICommand PopupSetting
        {
            get { return _popupSetting ?? (_popupSetting = new AsyncCommand(PopupShow)); }
        }

        private bool _popupSettingFlag = false;
        public bool PopupSettingFlag
        {
            get => _popupSettingFlag;
            set { _popupSettingFlag = value; OnPropertyChanged(nameof(PopupSettingFlag)); }
        }

        private bool _popupShowFlag = false;
        public bool PopupShowFlag
        {
            get => _popupShowFlag;
            set { _popupShowFlag = value; OnPropertyChanged(nameof(PopupShowFlag)); }
        }

        private Task PopupShow() => Task.Run(() =>
        {
            PopupSettingFlag = !PopupSettingFlag;
        });

        //2404 : MLEM
        private async Task LoadData()
        {
            string systemMPath, PLYPath, LMDPath;

            OpenFileDialog dlg = new OpenFileDialog();
            //dlg.Filter = "ply files(*.ply)| *.ply";
            //dlg.Multiselect = false;
            //dlg.Title = "Select pointcloud file";

            //LahgiApi.MLEMSelectNo = 0;

            //if (dlg.ShowDialog() == true)
            //{
            //    //await Task.Run(() => LahgiApi.LoadPlyFile(dlg.FileName));
            //    int nIndex = dlg.FileName.LastIndexOf('_');
            //    PLYPath = dlg.FileName.Substring(0, nIndex);
            //}
            //else
            //{
            //    return;
            //}



            //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Loading);

            dlg = new OpenFileDialog();
            dlg.Filter = "csv files(*.csv)| *.csv";
            dlg.Multiselect = false;
            dlg.Title = "Select list mode data file";

            if (dlg.ShowDialog() == true)
            {
                //await Task.Run(() => LahgiApi.LoadListModeData(dlg.FileName));
                LMDPath = dlg.FileName;

                int nIndex = dlg.FileName.LastIndexOf('_');
                PLYPath = dlg.FileName.Substring(0, nIndex);
            }
            else
            {
                return;
            }

            //dlg = new OpenFileDialog();
            //dlg.Filter = "bin files(*.bin)| *.bin";
            //dlg.Multiselect = false;
            //dlg.Title = "Select System Matrix data file";

            //if (dlg.ShowDialog() == true)
            //{
            //    //await Task.Run(() => LahgiApi.LoadListModeData(dlg.FileName));
            //    systemMPath = dlg.FileName;
            //}
            //else
            //{
            //    return;
            //}

            //250428
            string systemMFile = "AngleInteractionProbData(NaIScatter,120,1,300,5).bin";
            string exePath = Assembly.GetExecutingAssembly().Location;
            string? exeFolder = Path.GetDirectoryName(exePath);
            if(exeFolder != null)
            {
                systemMPath = Path.Combine(exeFolder, "config", systemMFile);
                if(!File.Exists(systemMPath))
                {
                    LahgiApi.UIMessageUpdateInvoke(null, "시스템 메트릭스 파일 확인 필요");
                    return;
                }
            }
            else
            {
                LahgiApi.UIMessageUpdateInvoke(null, "데이터 경로 확인 필요");
                return;
            }

            LahgiApi.UIMessageUpdateInvoke(null, "정밀 영상 데이터 로드 시작");

            bool breturn = await Task.Run(() => LahgiApi.LoadMLEMData(PLYPath, LMDPath, true));

            if (breturn == false)
            {
                LahgiApi.UIMessageUpdateInvoke(null, "정밀 영상 데이터 로드 실패");
                return;
            }

            //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Loading);

            await Task.Delay(1000);

            //250410 : S manual load 정밀 영상 피크 찾기 추가

            //선택된 PMT 에 따라 분류
            SpectrumEnergyNasa? spectrum = null;

            spectrum = LahgiApi.GetSpectrumData((uint)SpectrumVM.SpectrumCases, (uint)SpectrumVM.FpgaChannelNumber);
           
            ObservableCollection<IsotopeInfo> isotopeSel = new ObservableCollection<IsotopeInfo>();

            if (spectrum != null)
            {
                //var espect = spectrum;

               // ObservableCollection<IsotopeInfo> isotopeTemp = new ObservableCollection<IsotopeInfo>();
                List<Isotope> DetectedIso = PeakSearching.GetIsotopesFromPeaks(
                    spectrum.FindPeaks(SpectrumVM.Ref_x, SpectrumVM.Ref_fwhm, SpectrumVM.Ref_at_0, SpectrumVM.Min_snr), 
                    1, SpectrumVM.Ref_x, SpectrumVM.Ref_fwhm, SpectrumVM.Ref_at_0);

                logger.Info($"메뉴얼 정밀영상 핵종 개수 : {DetectedIso.Count}");    //test log

                List<Isotope> CalIso = new List<Isotope>();

                foreach (Isotope iso in DetectedIso)
                {
                    if (iso.IsotopeElement == IsotopeElement.K40 || iso.IsotopeElement == IsotopeElement.Tl208
                        || iso.IsotopeElement == IsotopeElement.Bi214 || iso.IsotopeElement == IsotopeElement.Pb212)
                        continue;

                    CalIso.Add(iso);
                }

                int nCount = 0;

                foreach (Isotope iso in CalIso)
                {
                    if (iso.IsotopeElement == IsotopeElement.K40 || iso.IsotopeElement == IsotopeElement.Tl208
                        || iso.IsotopeElement == IsotopeElement.Bi214 || iso.IsotopeElement == IsotopeElement.Pb212)
                        continue;

                    List<double> Peak = new List<double>();
                    foreach (var item in iso.PeakEnergy)
                    {
                        Peak.Add(item);
                    }

                    var newInfo = new IsotopeInfo(iso.IsotopeName, iso.IsotopeDescription, iso.sEnerge, Visibility.Hidden, Brushes.Black, iso.IsotopeElement, Peak/*iso.PeakEnergy*/);
                    newInfo.PropertyChanged += SpectrumVM.isotopelInfoChanged;
                    isotopeSel.Add(newInfo);

                    //2. 연산
                    LahgiApi.UIMessageUpdateInvoke(null, $"{nCount + 1}번째 정밀 영상 계산 중 ({iso.IsotopeName})");

                    List<double> energy = new List<double>();
                    List<double> min = new List<double>();
                    List<double> max = new List<double>();

                    foreach (var item in iso.PeakEnergy)
                    {
                        double fwhm = PeakSearching.CalcFWHM(item, LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0);
                        double MinE = item - fwhm;
                        double MaxE = item + fwhm;

                        energy.Add(item);
                        min.Add(MinE);
                        max.Add(MaxE);
                    }

                    //역으로
                    //for (int i = iso.PeakEnergy.Count - 1; i >= 0; i--)
                    //{
                    //    var item = iso.PeakEnergy[i];
                    //    double fwhm = PeakSearching.CalcFWHM(item, LahgiApi.Ref_x, LahgiApi.Ref_fwhm, LahgiApi.Ref_at_0);
                    //    double MinE = item - fwhm;
                    //    double MaxE = item + fwhm;

                    //    energy.Add(item);
                    //    min.Add(MinE);
                    //    max.Add(MaxE);
                    //}


                    bool showresult = true;

                    if (CalIso.Count > 1 && (CalIso.Count - 1) != nCount)
                        showresult = false;

                    LahgiApi.MLEMSelectNo = 0;

                    await LahgiApi.CalMLEM(systemMPath, energy, min, max, ReconstructionVM.MinValuePortion, showresult);

                    if (LahgiApi.StatusCalMLEM == false) //250107
                    {
                        LahgiApi.UIMessageUpdateInvoke(null, "정밀 영상 연산 실패");
                        continue;
                    }

                    logger.Info($"정밀영상 연산 완료");    //test log

                    isotopeSel[nCount].MLEMResult = "완료";

                    nCount++;
                }

                LahgiApi.UIMessageUpdateInvoke(null, "정밀 영상 연산 완료");

                SpectrumVM.IsotopeInfos = new ObservableCollection<IsotopeInfo>();

                foreach (var item in isotopeSel)
                {
                    item.MLEMResult = "완료";
                }


                SpectrumVM.IsotopeInfos = isotopeSel;

                LahgiApi.MLEMRun = true;
                //250410 : E
            }
        }

        private AsyncCommand? _testFuctionCommand;
        public ICommand TestFunctionCommand
        {
            get { return _testFuctionCommand ?? (_testFuctionCommand = new AsyncCommand(TestFunction)); }
        }
        private async Task TestFunction()
        {
            TestFunctionCommand.CanExecute(false);

            await Task.Run(() => {
                LahgiApi.TestAddingListModeData(1000_000);


            }
            );

            TestFunctionCommand.CanExecute(true);
        }

        //240228 
        private bool StartPeakToValleyClear { get; set; } = false;

        //240206-Gain : 실시간 검사
        private bool _realTimeCheck = false;
        public bool RealTimeCheck
        {
            get { return _realTimeCheck; }
            set 
            { 
                _realTimeCheck = value; 
                OnPropertyChanged(nameof(RealTimeCheck));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.UseRealTimeCheck = value;
            }
        }

        //240206-Gain : 실시간 검사 주기(분)
        private int _realTimeCheckCycleTime = 3;
        public int RealTimeCheckCycleTime
        {
            get { return _realTimeCheckCycleTime; }
            set 
            { 
                _realTimeCheckCycleTime = value; 
                OnPropertyChanged(nameof(RealTimeCheckCycleTime));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.RealTimeCycleTime = value;
            }
        }

        //240206-Gain : 정밀 고장 검사(fault diagnosis)
        private bool _faultDiagnosis = false;
        public bool FaultDiagnosis
        {
            get { return _faultDiagnosis; }
            set 
            { 
                _faultDiagnosis = value; 
                OnPropertyChanged(nameof(FaultDiagnosis));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.UseFaultDiagnosis = value;
            }
        }

        // 고장 검사 타입 설정
        private eFaultCheckType _faultCheckType = eFaultCheckType.None;
        public eFaultCheckType FaultCheckType
        {
            get => _faultCheckType;
            set 
            { 
                logger.Info($"FaultCheckType 속성 변경: {_faultCheckType} -> {value}");
                _faultCheckType = value; 
                OnPropertyChanged(nameof(FaultCheckType));
                
                // 기존 boolean 속성들과 동기화
                RealTimeCheck = (value == eFaultCheckType.RealTime);
                FaultDiagnosis = (value == eFaultCheckType.Precision);
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.UseRealTimeCheck = RealTimeCheck;
                App.GlobalConfig.UseFaultDiagnosis = FaultDiagnosis;
                logger.Info($"App.GlobalConfig 업데이트: RealTimeCheck={RealTimeCheck}, FaultDiagnosis={FaultDiagnosis}");
            }
        }

        //240206-Gain : 실시간 검사 주기(분)
        private int _faultDiagnosisMeasurementTime = 20;
        public int FaultDiagnosisMeasurementTime
        {
            get { return _faultDiagnosisMeasurementTime; }
            set
            {
                _faultDiagnosisMeasurementTime = value;
                OnPropertyChanged(nameof(FaultDiagnosisMeasurementTime));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.FaultDiagnosisMeasurementTime = value;
            }
        }

        //240228-PeakToValley : 실시간 검사에서 발생한 문제 채널의 개수
        public int ErrorPeakToValleyCount { get; set; } = 0;

        //231100-GUI sbkwon : 측정 시간 type 설정
        private eMeasuremetType _measuremetType = eMeasuremetType.Infinite;
        public eMeasuremetType MeasuremetType
        {
            get => _measuremetType;
            set 
            { 
                logger.Info($"MeasuremetType 속성 변경: {_measuremetType} -> {value}");
                _measuremetType = value; 
                OnPropertyChanged(nameof(MeasuremetType));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.MeasurementType = value;
                logger.Info($"App.GlobalConfig.MeasurementType 저장됨: {App.GlobalConfig.MeasurementType}");
            }
        }

        private AsyncCommand? _selectInfinite = null;

        public ICommand SelectInfinite
        {
            get { return _selectInfinite ?? (_selectInfinite = new AsyncCommand(SelInfinite)); }
        }

        private Task SelInfinite() => Task.Run(() =>
        {
            logger.Info($"SelectInfinite Command 실행됨. 현재 MeasuremetType: {MeasuremetType}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                MeasuremetType = eMeasuremetType.Infinite;
                logger.Info($"SelectInfinite Command 실행 후 MeasuremetType: {MeasuremetType}");
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.MeasurementType = MeasuremetType;
                logger.Info($"App.GlobalConfig.MeasurementType 업데이트: {App.GlobalConfig.MeasurementType}");
            });
        });

        private AsyncCommand? _selectSettingTime = null;

        public ICommand SelectSettingTime
        {
            get { return _selectSettingTime ?? (_selectSettingTime = new AsyncCommand(SelSettingTime)); }
        }

        private Task SelSettingTime() => Task.Run(() =>
        {
            logger.Info($"SelectSettingTime Command 실행됨. 현재 MeasuremetType: {MeasuremetType}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                MeasuremetType = eMeasuremetType.SettingTime;
                logger.Info($"SelectSettingTime Command 실행 후 MeasuremetType: {MeasuremetType}");
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.MeasurementType = MeasuremetType;
                logger.Info($"App.GlobalConfig.MeasurementType 업데이트: {App.GlobalConfig.MeasurementType}");
            });
        });

        // 고장 검사 타입 선택 Command들
        private AsyncCommand? _selectFaultCheckNone = null;
        public ICommand SelectFaultCheckNone
        {
            get { return _selectFaultCheckNone ?? (_selectFaultCheckNone = new AsyncCommand(SelFaultCheckNone)); }
        }

        private Task SelFaultCheckNone() => Task.Run(() =>
        {
            logger.Info($"SelectFaultCheckNone Command 실행됨. 현재 FaultCheckType: {FaultCheckType}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                FaultCheckType = eFaultCheckType.None;
                logger.Info($"SelectFaultCheckNone Command 실행 후 FaultCheckType: {FaultCheckType}");
            });
        });

        private AsyncCommand? _selectFaultCheckRealTime = null;
        public ICommand SelectFaultCheckRealTime
        {
            get { return _selectFaultCheckRealTime ?? (_selectFaultCheckRealTime = new AsyncCommand(SelFaultCheckRealTime)); }
        }

        private Task SelFaultCheckRealTime() => Task.Run(() =>
        {
            logger.Info($"SelectFaultCheckRealTime Command 실행됨. 현재 FaultCheckType: {FaultCheckType}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                FaultCheckType = eFaultCheckType.RealTime;
                logger.Info($"SelectFaultCheckRealTime Command 실행 후 FaultCheckType: {FaultCheckType}");
            });
        });

        private AsyncCommand? _selectFaultCheckPrecision = null;
        public ICommand SelectFaultCheckPrecision
        {
            get { return _selectFaultCheckPrecision ?? (_selectFaultCheckPrecision = new AsyncCommand(SelFaultCheckPrecision)); }
        }

        private Task SelFaultCheckPrecision() => Task.Run(() =>
        {
            logger.Info($"SelectFaultCheckPrecision Command 실행됨. 현재 FaultCheckType: {FaultCheckType}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                FaultCheckType = eFaultCheckType.Precision;
                logger.Info($"SelectFaultCheckPrecision Command 실행 후 FaultCheckType: {FaultCheckType}");
            });
        });

        //231016 sbkwon : 측정 시간
        private int _measurementTime = 60;
        public int MeasurementTime
        {
            get { return _measurementTime; }
            set
            {
                _measurementTime = value;
                OnPropertyChanged(nameof(MeasurementTime));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.MeasurementTime = value;
            }
        }

        //231016 sbkwon : 경과 시간
        private long _elapsedTime = 0;
        public long ElapsedTime
        {
            get { return _elapsedTime; }
            set
            {
                _elapsedTime = value;
                LahgiApi.ElapsedTime = (uint)(value * 0.001);

                OnPropertyChanged(nameof(ElapsedTime));
            }
        }

        //240222 측정시간
        private string _measureTime = "";
        public string MeasureTime
        {
            get => _measureTime;
            set
            {
                _measureTime = value;

                OnPropertyChanged(nameof(MeasureTime));
            }
        }

        //240222 stop button text
        private string _stopText = "종료";
        public string StopText
        {
            get => _stopText;
            set
            {
                _stopText = value;
                OnPropertyChanged(nameof(StopText));
            }
        }

        public void LahgiStatusUpdate()
        {
            // COM 포트 상태 체크
            if (LahgiSerialControl.PortsName.Count == 0 || LahgiSerialControl.SelectedPortName == null)
            {
                // COM 포트가 없으면 검출기 상태를 점검 필요로 설정
                FPGAStatus = "점검 필요";
                FPGATextColor = Brushes.Red;
                HVModuleStatus = "점검 필요";
                HVModuleTextColor = Brushes.Red;
                return;
            }

            // COM 포트가 있으면 정상 체크
            LahgiSerialControl.CheckParams();

            // FPGA 연결 상태 체크
            bool isFPGAConnected = LahgiSerialControl.IsFPGAOn & LahgiApi.IsFPGAStart;
            IsFPGAOn = isFPGAConnected;
            
            if (!isFPGAConnected)
            {
                // FPGA 연결이 안되면 신호처리 시스템 상태를 점검 필요로 설정
                HVModuleStatus = "점검 필요";
                HVModuleTextColor = Brushes.Red;
            }
            else
            {
                // FPGA가 연결되면 정상 체크
                HVModule = LahgiSerialControl.HvModuleVoltage > 850.0 ? true : false;
            }
        }

        //231208 sbkwon : 검출기 상태
        private bool _isFPGAOn = false;
        public bool IsFPGAOn
        {
            get => _isFPGAOn;
            set
            {
                _isFPGAOn = value;
                if (value)
                {
                    FPGAStatus = "정상";
                    FPGATextColor = Brushes.Black;
                }
                else
                {
                    FPGAStatus = "점검 필요";
                    FPGATextColor = Brushes.Red;
                }
            }
        }
        private string _fpgaStatus = "";
        public string FPGAStatus
        {
            get => _fpgaStatus;
            set
            {
                _fpgaStatus = value;
                OnPropertyChanged(nameof(FPGAStatus));
            }
        }

        private Brush _fpgaTexttColor = Brushes.Black;
        public Brush FPGATextColor
        {
            get => _fpgaTexttColor;
            set
            {
                _fpgaTexttColor = value;
                OnPropertyChanged();
            }
        }

        //231208 sbkwon : 신호처리 상태
        private bool _hvModule = false;
        public bool HVModule
        {
            get => _hvModule;
            set
            {
                _hvModule = value;
                if (value)
                {
                    HVModuleStatus = "정상";
                    HVModuleTextColor = Brushes.Black;
                }
                else
                {
                    HVModuleStatus = "점검 필요";
                    HVModuleTextColor = Brushes.Red;
                }
            }
        }
        private string _hvModuleStatus = "";
        public string HVModuleStatus
        {
            get => _hvModuleStatus;
            set
            {
                _hvModuleStatus = value;
                OnPropertyChanged(nameof(HVModuleStatus));
            }
        }

        private Brush _hvModuleTexttColor = Brushes.Black;
        public Brush HVModuleTextColor
        {
            get => _hvModuleTexttColor;
            set
            {
                _hvModuleTexttColor = value;
                OnPropertyChanged();
            }
        }

        // 추가된 Command들
        private AsyncCommand? _selectRealTimeCheck = null;
        public ICommand SelectRealTimeCheck
        {
            get { return _selectRealTimeCheck ?? (_selectRealTimeCheck = new AsyncCommand(SelRealTimeCheck)); }
        }

        private Task SelRealTimeCheck() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectRealTimeCheck Command 실행됨. 현재 값: {RealTimeCheck}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                RealTimeCheck = !RealTimeCheck;
                System.Diagnostics.Debug.WriteLine($"SelectRealTimeCheck Command 실행 후 값: {RealTimeCheck}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.UseRealTimeCheck = RealTimeCheck;
            });
        });

        // SelectFaultDiagnosis Command 추가
        private AsyncCommand? _selectFaultDiagnosis = null;
        public ICommand SelectFaultDiagnosis
        {
            get { return _selectFaultDiagnosis ?? (_selectFaultDiagnosis = new AsyncCommand(SelFaultDiagnosis)); }
        }

        private Task SelFaultDiagnosis() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectFaultDiagnosis Command 실행됨. 현재 값: {FaultDiagnosis}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                FaultDiagnosis = !FaultDiagnosis;
                System.Diagnostics.Debug.WriteLine($"SelectFaultDiagnosis Command 실행 후 값: {FaultDiagnosis}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.UseFaultDiagnosis = FaultDiagnosis;
            });
        });


    }
}
