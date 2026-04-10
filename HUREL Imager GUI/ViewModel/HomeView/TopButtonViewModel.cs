using AsyncAwaitBestPractices.MVVM;
using HelixToolkit.Wpf.SharpDX;
using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using HUREL_Imager_GUI.Components;
using HUREL_Imager_GUI.ViewModel.ObjectDetection;
using HUREL_Imager_GUI.ViewModel.ObjectDetection.Models;
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

    public enum eMeasurementMode
    {
        Moving,
        Static,
        ObjectDetection,
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
            set 
            { 
                _isRunning = value; 
                OnPropertyChanged(nameof(IsRunning));
                OnPropertyChanged(nameof(IsStatusButtonEnabled));
                OnPropertyChanged(nameof(IsMeasurementModeChangeEnabled)); // 측정 모드 변경 활성화 상태 업데이트
                OnPropertyChanged(nameof(IsS2MEnabled)); // 정지 모드 측정 중 영상 거리 비활성화 반영
            }
        }

        public bool IsStatusButtonEnabled
        {
            get { return !IsRunning && !LahgiApi.IsSessionStarting; }
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
            SpectrumVM.TopButtonVM = this; // SpectrumViewModel에 TopButtonVM 연결
            ReconstructionVM = new ReconstructionImageViewModel();
            ReconstructionVM.TopButtonVM = this; // TopButtonVM 설정 (Loop 시작 전에 설정 필요)
            ReconstructionVM.StartLoop(); // TopButtonVM 설정 후 Loop 시작
            ReconstructionImageVM = ReconstructionVM; // 같은 인스턴스 사용
            ThreeDimensionalVM = new ThreeDimensionalViewModel();
            DoseRateVM = new DoseRateViewModel();
            DoseRateVM.TopButtonVM = this; // DoseRateViewModel에 TopButtonVM 연결

            //231100-GUI sbkwon
            FileName = App.GlobalConfig.SaveFileName;
            MeasuremetType = App.GlobalConfig.MeasurementType;
            MeasurementTime = App.GlobalConfig.MeasurementTime;
            MeasurementMode = App.GlobalConfig.MeasurementMode;
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
            // 테스트 모드에서는 연결 상태와 무관하게 버튼 활성화
            bool isTestMode = TestModeConfig.IsTestMode;
            if (isTestMode)
            {
                // 테스트 모드: 연결 상태 체크 없이 측정 시작 버튼 활성화
                StartButtonEnabled = !LahgiApi.IsSessionStarting && !IsMLEMRun && !IsRunning;
            }
            else
            {
                // 일반 모드: 연결 상태 체크
                StartButtonEnabled = (LahgiApi.IsInitiate && LahgiApi.IsFpgaAvailable) && !LahgiApi.IsSessionStarting && !IsMLEMRun && !IsRunning;    //240429
            }
            StopButtonEnabled = IsRunning;  // 측정 중일 때만 종료 버튼 활성화
            OnPropertyChanged(nameof(StartStopButtonEnabled));  // 토글 버튼 활성화 상태 업데이트
            OnPropertyChanged(nameof(ResetButtonEnabled));  // 리셋 버튼 활성화 상태 업데이트
            OnPropertyChanged(nameof(IsStatusButtonEnabled));
            
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

        // 객체탐지 서비스
        private ObjectDetectionService? _objectDetectionService;

        /// <summary>측정 시작 시 발생 (모드 전달). 객체 탐지 모드일 때 GUI 테이블 초기화용.</summary>
        public event EventHandler<eMeasurementMode>? SessionStarted;
        /// <summary>탐지된 객체 목록 갱신 시 발생. GUI 테이블 동기화용.</summary>
        public event EventHandler<List<TrackedPerson>>? TrackedPersonsUpdated;

        /// <summary>탐지된 객체 목록을 구독자에게 전달 (ReconstructionImageViewModel에서 호출)</summary>
        public void NotifyTrackedPersonsUpdated(List<TrackedPerson> persons) => TrackedPersonsUpdated?.Invoke(this, persons);

        private CancellationTokenSource? _sessionCancle;
        /// <summary>시작 버튼 중복 클릭 방지.</summary>
        private bool bClicked = false;
        /// <summary>측정 세션 비동기 수명주기(StartSessionAsync 등). UI 스레드에서 await 하지 않음.</summary>
        private Task? _sessionLifecycleTask;
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

        private async Task ObserveSessionLifecycleAsync(Task lifecycleTask, Stopwatch swStartSession)
        {
            try
            {
                await lifecycleTask.ConfigureAwait(false);
                logger.Info($"[StartDiag] Session lifecycle finished (total={swStartSession.ElapsedMilliseconds}ms)");
            }
            catch (OperationCanceledException)
            {
                logger.Info($"[StartDiag] Session lifecycle canceled (total={swStartSession.ElapsedMilliseconds}ms)");
            }
            catch (Exception ex)
            {
                logger.Error($"[StartDiag] Session lifecycle failed after {swStartSession.ElapsedMilliseconds}ms: {ex.Message}");
            }
        }

        private async Task StartSession()
        {
            if (bClicked == false)
            {
                Stopwatch swStartSession = Stopwatch.StartNew();
                logger.Info($"[StartDiag] StartSession entered at {DateTime.Now:O} (mode={MeasurementMode}, FaultDiagnosis={FaultDiagnosis}, RealTimeCheck={RealTimeCheck}, IsEcalUse={SpectrumVM.IsEcalUse})");
                bClicked = true;

                PsdHeatmapView.ResetDiagnosticLogForNewSession();

                IsMLEMEnable = true;

                StopText = "대기중";

                _sessionCancle = new CancellationTokenSource();
                SpectrumVM.SpectrumStart = false;

                // 정지 모드: 측정 시작 시 누적 버퍼 초기화 (Option A 기준 시각 설정)
                if (MeasurementMode == eMeasurementMode.Static)
                {
                    SpectrumVM.ClearStaticModeAccumulators();
                    ReconstructionVM.ClearStaticModeRadiationAccumulators();
                }

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

                // 객체탐지 모드일 경우 ObjectDetectionService 초기화 (RGBDisplay 호출 전에 초기화 필요)
                if (MeasurementMode == eMeasurementMode.ObjectDetection)
                {
                    logger.Info("StartSession: 객체탐지 모드 감지 - ObjectDetectionService 초기화 시작");
                    Stopwatch swObjectDetectionInit = Stopwatch.StartNew();
                    await Task.Run(() => InitializeObjectDetectionService());
                    logger.Info($"[StartDiag] ObjectDetection init done in {swObjectDetectionInit.ElapsedMilliseconds}ms");
                    
                    // 초기화 결과 확인
                    if (_objectDetectionService != null && _objectDetectionService.IsInitialized)
                    {
                        logger.Info("StartSession: ObjectDetectionService 초기화 성공");
                    }
                    else
                    {
                        logger.Warn("StartSession: ObjectDetectionService 초기화 실패 또는 모델 파일을 찾을 수 없음");
                    }
                    // 객체 탐지 모드 측정 시작 시 GUI 테이블 초기화를 위해 알림
                    SessionStarted?.Invoke(this, MeasurementMode);
                }

                // GetRealTimeRGB + Bitmap/BitmapImage 변환이 UI 스레드를 수백 ms~수 초 점유할 수 있어,
                // 측정 시작 직후 스펙트럼/PSD BeginInvoke 큐와 입력 처리가 막히지 않도록 한 프레임 뒤에 RGB 갱신.
                var dispRgb = Application.Current?.Dispatcher;
                if (dispRgb != null)
                    dispRgb.BeginInvoke(DispatcherPriority.ApplicationIdle, new Action(() => ReconstructionVM.RGBDisplay()));
                else
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
                    logger.Info($"[StartDiag] Session task list initialized in {swStartSession.ElapsedMilliseconds}ms");
                    
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
                    
                    logger.Info($"[StartDiag] Run Task.WhenAll in background (taskCount={tasks.Count}) at +{swStartSession.ElapsedMilliseconds}ms");
                    _sessionLifecycleTask = Task.WhenAll(tasks);
                    _ = ObserveSessionLifecycleAsync(_sessionLifecycleTask, swStartSession);
                }
                else
                {
                    ReconstructionVM.ClearFaultColor(); //240228

                    StartFaultDiagnosis = true;

                    logger.Info($"[StartDiag] Run StartSessionAsyncFD in background at +{swStartSession.ElapsedMilliseconds}ms");
                    _sessionLifecycleTask = LahgiApi.StartSessionAsyncFD(FileName, _sessionCancle); //240228
                    _ = ObserveSessionLifecycleAsync(_sessionLifecycleTask, swStartSession);
                }

                await Dispatcher.Yield(DispatcherPriority.Background);
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

                string saveFilePath = directoryPath + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + saveFileName + "_Screenshot.png";

                // UI 스레드에서 창의 실제 화면 좌표 가져오기 (PointToScreen 사용)
                int windowX = 0, windowY = 0, windowWidth = 0, windowHeight = 0;
                Application.Current.Dispatcher.Invoke(() =>
                {
                    if (App.CurrentMainWindow != null)
                    {
                        // 창의 왼쪽 상단 모서리를 화면 좌표로 변환
                        System.Windows.Point topLeft = App.CurrentMainWindow.PointToScreen(new System.Windows.Point(0, 0));
                        
                        // 창의 오른쪽 하단 모서리를 화면 좌표로 변환
                        System.Windows.Point bottomRight = App.CurrentMainWindow.PointToScreen(
                            new System.Windows.Point(App.CurrentMainWindow.ActualWidth, App.CurrentMainWindow.ActualHeight));

                        // 화면 좌표를 정수로 변환
                        windowX = (int)topLeft.X;
                        windowY = (int)topLeft.Y;
                        windowWidth = (int)(bottomRight.X - topLeft.X);
                        windowHeight = (int)(bottomRight.Y - topLeft.Y);
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
                    
                    // 스펙트럼 관련 값 초기화 (MaxPeakCount는 private이므로 접근하지 않음)
                    SpectrumVM.MaxCount = 0;

                    PsdAccumulator.Instance.Reset();
                    SpectrumVM.NotifyPsdHeatmapRefresh();

                    // 방사선 영상 초기화 (초기 상태로 복원)
                    ReconstructionVM.ComptonImgRGB = new BitmapImage();
                    ReconstructionVM.CodedImgRGB = new BitmapImage();
                    ReconstructionVM.HybridImgRGB = new BitmapImage();
                    ReconstructionVM.MLEM2DVisibility = false;
                    ReconstructionVM.MLEM2DRGB = false;
                    
                    // RGB 표시 업데이트 (초기 상태로 복원)
                    ReconstructionVM.RGBDisplay();

                    // 객체탐지 모드에서는 초기화 버튼 시 객체탐지 테이블/추적 상태도 함께 초기화
                    if (MeasurementMode == eMeasurementMode.ObjectDetection)
                    {
                        ReconstructionVM?.WaitForObjectDetectionPipelineIdle(TimeSpan.FromSeconds(5));
                        _objectDetectionService?.ClearData();
                        LahgiApi.ClearAllObjectAccumulations();
                        ReconstructionVM?.ResetObjectDetectionVisualState();
                        ReconstructionVM.PersonTableItemsRef?.Clear();
                        NotifyTrackedPersonsUpdated(new List<TrackedPerson>());
                        CleanupObjectDetectionService(); // 다음 측정 시작 시 새로 초기화되도록 완전 정리
                        logger.Info("Object detection table and tracking data reset completed");
                    }

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
                // 종료 요청은 항상 최우선 반영한다.
                // 후속 정리(스크린샷/파일 저장)에서 예외가 나더라도 측정 루프는 반드시 취소되어야 한다.
                _sessionCancle?.Cancel();
                // Stop 처리 중 재측정 시작을 막아 FPGA 파이프라인 정리/재시작 경합을 방지한다.
                LahgiApi.IsSessionStarting = true;

                // StartSessionAsync 수명주기 종료를 잠시 기다려 StartMeasurement false 처리 등이 완료되도록 보장한다.
                try
                {
                    var lifecycleTask = _sessionLifecycleTask;
                    if (lifecycleTask != null && !lifecycleTask.IsCompleted)
                    {
                        if (!lifecycleTask.Wait(3000))
                            logger.Warn("StopSession: session lifecycle 종료 대기 타임아웃(3초) - 후속 정리 계속 진행");
                    }
                }
                catch (Exception ex)
                {
                    logger.Warn($"StopSession: session lifecycle 종료 대기 중 예외 (무시): {ex.Message}");
                }

                //231019 sbkwon : spectrum capture - 종료시 delay 발생하여 위치 이동
                if (IsRunning)
                {
                    try
                    {
                        // GUI 화면 창 전체 캡처
                        string? directoryPath = System.IO.Path.GetDirectoryName(LahgiApi.GetFileSavePath());
                        string saveFileName = directoryPath + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + FileName;

                        // UI 스레드에서 창의 실제 화면 좌표 가져오기 (PointToScreen 사용)
                        int windowX = 0, windowY = 0, windowWidth = 0, windowHeight = 0;
                        Application.Current.Dispatcher.Invoke(() =>
                        {
                            if (App.CurrentMainWindow != null)
                            {
                                // 창의 왼쪽 상단 모서리를 화면 좌표로 변환
                                System.Windows.Point topLeft = App.CurrentMainWindow.PointToScreen(new System.Windows.Point(0, 0));

                                // 창의 오른쪽 하단 모서리를 화면 좌표로 변환
                                System.Windows.Point bottomRight = App.CurrentMainWindow.PointToScreen(
                                    new System.Windows.Point(App.CurrentMainWindow.ActualWidth, App.CurrentMainWindow.ActualHeight));

                                // 화면 좌표를 정수로 변환
                                windowX = (int)topLeft.X;
                                windowY = (int)topLeft.Y;
                                windowWidth = (int)(bottomRight.X - topLeft.X);
                                windowHeight = (int)(bottomRight.Y - topLeft.Y);
                            }
                        });

                        // 창 전체 캡처
                        ImgCapture imgCapture = new ImgCapture(windowX, windowY, windowWidth, windowHeight);
                        imgCapture.SetPath(saveFileName + "_screenshot.png");
                        imgCapture.DoCaptureImage();
                    }
                    catch (Exception ex)
                    {
                        logger.Warn($"StopSession: 종료 스크린샷 저장 실패 (측정 종료는 계속 진행): {ex.Message}");
                    }

                    LahgiApi.SessionStopwatch.Stop();
                    
                    // 타이머 정지
                    StopTimer();

                // 객체탐지 모드일 경우 데이터 저장, C++ 누적 버퍼 삭제, 정리
                if (MeasurementMode == eMeasurementMode.ObjectDetection)
                {
                    SaveObjectDetectionData();
                    // RGBDisplay의 Task.Run(ProcessObjectDetection)이 YOLO Run 중일 때 Dispose하면 AV/즉시 종료 가능
                    ReconstructionVM?.WaitForObjectDetectionPipelineIdle(TimeSpan.FromSeconds(20));
                    LahgiApi.ClearAllObjectAccumulations();
                    CleanupObjectDetectionService();
                }
                }

                ////test
                //IsRunning = false;

                // 테스트 모드 체크
                bool isTestMode = TestModeConfig.IsTestMode;
                if (isTestMode)
                {
                    // 테스트 모드: SLAM 정지 및 세션 상태 업데이트
                    LahgiApi.StopSlam();
                    // IsSessionStart는 private setter이므로 직접 설정할 수 없음
                    // StartSessionAsync가 완료되면 자동으로 false가 되지만,
                    // 테스트 모드에서는 취소 시 수동으로 처리 필요
                    // StatusUpdate를 통해 상태 변경 알림
                    LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                }
            }
            else    //고장 검사
            {
                LahgiApi.IsSessionStarting = true;
                _sessionCancle?.Cancel();

                try
                {
                    var lifecycleTask = _sessionLifecycleTask;
                    if (lifecycleTask != null && !lifecycleTask.IsCompleted)
                    {
                        if (!lifecycleTask.Wait(3000))
                            logger.Warn("StopSession(FD): session lifecycle 종료 대기 타임아웃(3초) - 후속 정리 계속 진행");
                    }
                }
                catch (Exception ex)
                {
                    logger.Warn($"StopSession(FD): session lifecycle 종료 대기 중 예외 (무시): {ex.Message}");
                }

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

            //화면 표시 - 2채널 모드: 채널 0(Scatter), 채널 1(Absorber)만 처리
            switch (nCh)
            {
            case 0:  //Scatter Ch1 (채널 0)
                    ReconstructionVM.ScatterCH1 = result;
                    break;
            case 1: //Absorber Ch1 (채널 1)
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

            // 2채널 모드: 채널 0(Scatter), 채널 1(Absorber)만 초기화
            ReconstructionVM.ScatterCH1 = result;   // 채널 0
            ReconstructionVM.AbsorberCH1 = result;  // 채널 1
        }

        //240228 결과 저장 : Gain 폴더에 시분초 추가하여 넣기 - 측정중 여러번 계산하기 때문
        private void SaveGain(int nCh, List<double> CorrMat_bkg, string filepath)
        {
            string scatterSerial = "50777";
            string absorberSerial = "51516";

            if (nCh == 0)
            {
                int no = 0;

                filepath += "\\" + scatterSerial + "_Scint" + no.ToString() + ".csv";
            }
            else if (nCh == 1)
            {
                int no = 0;
                filepath += "\\" + absorberSerial + "_Scint" + no.ToString() + ".csv";
            }
            else
            {
                logger.Warn($"SaveGain skipped. Unsupported channel: {nCh}");
                return;
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
            // 측정 중/초기화 중에는 상태창 및 시리얼 질의를 막아 통신 간섭을 방지
            if (!IsStatusButtonEnabled || LahgiApi.IsSessionStarting || LahgiApi.IsSessionStart)
            {
                logger.Info("Status popup blocked: running or initializing");
                return;
            }

            bool willOpen = !StatusPopupShowFlag;
            if (willOpen)
            {
                RefreshStatusByCheckParams();
            }

            StatusPopupShowFlag = willOpen;
            logger.Info("Status popup toggled");
        });

        private void RefreshStatusByCheckParams()
        {
            if (LahgiApi.IsSessionStarting || LahgiApi.IsSessionStart)
            {
                return;
            }

            if (LahgiSerialControl.PortsName.Count == 0 || LahgiSerialControl.SelectedPortName == null)
            {
                IsFPGAOn = false;
                HVModule = false;
                return;
            }

            // 기존 상태 갱신 루프(LahgiStatusUpdate)와 CheckParams 호출이 겹치지 않도록 직렬화
            if (Interlocked.CompareExchange(ref _serialCheckInProgress, 1, 0) != 0)
            {
                logger.Info("RefreshStatusByCheckParams: serial check already in progress");
                return;
            }

            bool checkSuccess = false;
            try
            {
                checkSuccess = LahgiSerialControl.CheckParams();
            }
            catch (Exception ex)
            {
                logger.Warn($"RefreshStatusByCheckParams: CheckParams 실행 중 예외: {ex.Message}");
            }
            finally
            {
                Interlocked.Exchange(ref _serialCheckInProgress, 0);
            }

            if (!checkSuccess)
            {
                FPGAStatus = "응답 없음";
                FPGATextColor = Brushes.Red;
                HVModule = false;
                return;
            }

            IsFPGAOn = LahgiSerialControl.IsFPGAOn;
            HVModule = LahgiSerialControl.HvModuleVoltage > 850.0;
        }

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
                    
                    // UI 스레드에서 창의 실제 화면 좌표 가져오기 (PointToScreen 사용)
                    int windowX = 0, windowY = 0, windowWidth = 0, windowHeight = 0;
                    await Application.Current.Dispatcher.InvokeAsync(() =>
                    {
                        if (App.CurrentMainWindow != null)
                        {
                            // 창의 왼쪽 상단 모서리를 화면 좌표로 변환
                            System.Windows.Point topLeft = App.CurrentMainWindow.PointToScreen(new System.Windows.Point(0, 0));
                            
                            // 창의 오른쪽 하단 모서리를 화면 좌표로 변환
                            System.Windows.Point bottomRight = App.CurrentMainWindow.PointToScreen(
                                new System.Windows.Point(App.CurrentMainWindow.ActualWidth, App.CurrentMainWindow.ActualHeight));

                            // 화면 좌표를 정수로 변환
                            windowX = (int)topLeft.X;
                            windowY = (int)topLeft.Y;
                            windowWidth = (int)(bottomRight.X - topLeft.X);
                            windowHeight = (int)(bottomRight.Y - topLeft.Y);
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
                // 테스트 모드에서는 연결 상태와 무관하게 버튼 활성화
                bool isTestMode = TestModeConfig.IsTestMode;
                if (isTestMode)
                {
                    StartButtonEnabled = !LahgiApi.IsSessionStarting && !IsMLEMRun;
                }
                else
                {
                    StartButtonEnabled = (LahgiApi.IsInitiate && LahgiApi.IsFpgaAvailable) && !LahgiApi.IsSessionStarting && !IsMLEMRun;
                }
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

        // 측정 모드 설정
        private eMeasurementMode _measurementMode = eMeasurementMode.Moving;
        public eMeasurementMode MeasurementMode
        {
            get => _measurementMode;
            set 
            { 
                logger.Info($"MeasurementMode 속성 변경: {_measurementMode} -> {value}");
                
                // 이전 모드가 객체탐지 모드였고, 새로운 모드가 객체탐지 모드가 아니면 정리
                if (_measurementMode == eMeasurementMode.ObjectDetection && value != eMeasurementMode.ObjectDetection)
                {
                    CleanupObjectDetectionService();
                }
                
                _measurementMode = value; 
                OnPropertyChanged(nameof(MeasurementMode));
                OnPropertyChanged(nameof(IsTimeInputEnabled)); // 시간 입력 활성화 상태 업데이트
                OnPropertyChanged(nameof(IsS2MEnabled)); // 영상 거리 입력 활성화 상태 업데이트
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.MeasurementMode = value;
                logger.Info($"App.GlobalConfig.MeasurementMode 저장됨: {App.GlobalConfig.MeasurementMode}");
            }
        }

        // 정지모드일 때 시간 입력 비활성화
        public bool IsTimeInputEnabled
        {
            get => _measurementMode != eMeasurementMode.Static;
        }

        /// <summary>정지 모드에서 측정 중일 때 영상 거리(S2M) 변경 불가. 그 외에는 활성화.</summary>
        public bool IsS2MEnabled => !(_measurementMode == eMeasurementMode.Static && _isRunning);

        // 측정 중일 때 측정 모드 변경 비활성화
        public bool IsMeasurementModeChangeEnabled
        {
            get => !IsRunning;
        }

        private AsyncCommand? _selectMovingMode = null;
        public ICommand SelectMovingMode
        {
            get { return _selectMovingMode ?? (_selectMovingMode = new AsyncCommand(SelMovingMode)); }
        }

        private Task SelMovingMode() => Task.Run(() =>
        {
            logger.Info($"SelectMovingMode Command 실행됨. 현재 MeasurementMode: {MeasurementMode}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                MeasurementMode = eMeasurementMode.Moving;
                logger.Info($"SelectMovingMode Command 실행 후 MeasurementMode: {MeasurementMode}");
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.MeasurementMode = MeasurementMode;
                logger.Info($"App.GlobalConfig.MeasurementMode 업데이트: {App.GlobalConfig.MeasurementMode}");
            });
        });

        private AsyncCommand? _selectStaticMode = null;
        public ICommand SelectStaticMode
        {
            get { return _selectStaticMode ?? (_selectStaticMode = new AsyncCommand(SelStaticMode)); }
        }

        private Task SelStaticMode() => Task.Run(() =>
        {
            logger.Info($"SelectStaticMode Command 실행됨. 현재 MeasurementMode: {MeasurementMode}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                MeasurementMode = eMeasurementMode.Static;
                logger.Info($"SelectStaticMode Command 실행 후 MeasurementMode: {MeasurementMode}");
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.MeasurementMode = MeasurementMode;
                logger.Info($"App.GlobalConfig.MeasurementMode 업데이트: {App.GlobalConfig.MeasurementMode}");
            });
        });

        private AsyncCommand? _selectObjectDetectionMode = null;
        public ICommand SelectObjectDetectionMode
        {
            get { return _selectObjectDetectionMode ?? (_selectObjectDetectionMode = new AsyncCommand(SelObjectDetectionMode)); }
        }

        private Task SelObjectDetectionMode() => Task.Run(() =>
        {
            logger.Info($"SelectObjectDetectionMode Command 실행됨. 현재 MeasurementMode: {MeasurementMode}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                MeasurementMode = eMeasurementMode.ObjectDetection;
                logger.Info($"SelectObjectDetectionMode Command 실행 후 MeasurementMode: {MeasurementMode}");
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.MeasurementMode = MeasurementMode;
                logger.Info($"App.GlobalConfig.MeasurementMode 업데이트: {App.GlobalConfig.MeasurementMode}");
            });
        });

        // 객체탐지 서비스 초기화
        private void InitializeObjectDetectionService()
        {
            try
            {
                // 이미 초기화되어 있으면 건너뛰기
                if (_objectDetectionService != null && _objectDetectionService.IsInitialized)
                {
                    logger.Debug("ObjectDetectionService가 이미 초기화되어 있습니다.");
                    return;
                }
                
                // 기존 서비스가 있지만 초기화되지 않은 경우 정리
                if (_objectDetectionService != null)
                {
                    _objectDetectionService.Dispose();
                    _objectDetectionService = null;
                }
                
                // 모델 경로 설정 (여러 위치에서 찾기)
                // 1. 실행 파일과 같은 폴더의 Models 폴더
                // 2. 실행 파일과 같은 폴더
                // 3. 소스 코드의 Models 폴더 (개발 환경)
                string[] possibleFileNames = { "yolo11n.onnx", "yolov11n.onnx", "yolo11.onnx" };
                string? modelPath = null;
                
                // 검색할 경로 목록 (더 많은 경로 추가)
                List<string> searchPaths = new List<string>
                {
                    Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Models"),
                    AppDomain.CurrentDomain.BaseDirectory,
                    // 소스 코드 경로 직접 확인
                    Path.Combine(Directory.GetCurrentDirectory(), "HUREL Imager GUI", "Models"),
                    Path.Combine(Directory.GetCurrentDirectory(), "Models"),
                    // 상대 경로로 소스 코드 찾기
                    Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "..", "..", "..", "HUREL Imager GUI", "Models"),
                    Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "..", "..", "HUREL Imager GUI", "Models"),
                    Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "..", "HUREL Imager GUI", "Models"),
                    // 실행 파일 위치 기준
                    Path.Combine(Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location) ?? "", "Models"),
                };
                
                // 각 경로에서 파일 찾기
                foreach (var searchPath in searchPaths)
                {
                    try
                    {
                        string fullPath = Path.GetFullPath(searchPath);
                        foreach (var fileName in possibleFileNames)
                        {
                            string testPath = Path.Combine(fullPath, fileName);
                            if (File.Exists(testPath))
                            {
                                modelPath = Path.GetFullPath(testPath); // 절대 경로로 변환
                                logger.Info($"모델 파일 발견: {modelPath}");
                                break;
                            }
                        }
                        if (modelPath != null)
                            break;
                    }
                    catch
                    {
                        // 경로가 유효하지 않으면 건너뛰기
                        continue;
                    }
                }

                if (modelPath == null || !File.Exists(modelPath))
                {
                    logger.Warn($"YOLOv11 ONNX 모델 파일을 찾을 수 없습니다.");
                    logger.Warn($"실행 파일 위치: {AppDomain.CurrentDomain.BaseDirectory}");
                    logger.Warn($"현재 작업 디렉토리: {Directory.GetCurrentDirectory()}");
                    logger.Warn($"검색한 경로들:");
                    foreach (var searchPath in searchPaths)
                    {
                        try
                        {
                            string fullPath = Path.GetFullPath(searchPath);
                            bool exists = Directory.Exists(fullPath);
                            logger.Warn($"  - {fullPath} (폴더 존재: {exists})");
                            if (exists)
                            {
                                var files = Directory.GetFiles(fullPath, "*.onnx");
                                logger.Warn($"    파일 목록: {string.Join(", ", files.Select(f => Path.GetFileName(f)))}");
                            }
                        }
                        catch (Exception ex)
                        {
                            logger.Warn($"  - {searchPath} (경로 오류: {ex.Message})");
                        }
                    }
                    logger.Warn("Models 폴더에 yolo11n.onnx 파일이 있는지 확인해주세요.");
                    logger.Warn($"소스 코드 Models 폴더: {Path.Combine(Directory.GetCurrentDirectory(), "HUREL Imager GUI", "Models")}");
                    return;
                }
                
                logger.Info($"YOLOv11 모델 파일 발견: {modelPath}");

                // 초기화가 끝날 때까지 _objectDetectionService에 할당하지 않음 (백그라운드 RGB 루프가
                // IsInitialized==false인 인스턴스를 잡는 레이스 방지).
                var candidate = new ObjectDetectionService(modelPath, confidenceThreshold: 0.5f, nmsThreshold: 0.4f, useCpuOnly: false);
                if (!candidate.Initialize())
                {
                    logger.Error("ObjectDetectionService 초기화 실패");
                    candidate.Dispose();
                    return;
                }

                _objectDetectionService = candidate;
                logger.Info($"ObjectDetectionService 초기화 완료: 모델 경로={modelPath}");
            }
            catch (Exception ex)
            {
                logger.Error($"ObjectDetectionService 초기화 중 오류 발생: {ex.Message}", ex);
                _objectDetectionService?.Dispose();
                _objectDetectionService = null;
            }
        }

        // 객체탐지 데이터 저장
        private void SaveObjectDetectionData()
        {
            if (_objectDetectionService == null)
            {
                logger.Debug("ObjectDetectionService가 null이므로 데이터 저장 건너뜀");
                return;
            }

            try
            {
                string? directoryPath = null;
                
                // 테스트 모드 체크
                bool isTestMode = TestModeConfig.IsTestMode;
                
                // 테스트 모드와 일반 모드 모두 측정 데이터가 생성되는 경로 사용
                // 실행 파일이 있는 경로 (bin\x64\Release\net6.0-windows)
                directoryPath = AppDomain.CurrentDomain.BaseDirectory;
                
                // 테스트 모드와 일반 모드 모두 LahgiApi.GetFileSavePath()의 디렉토리를 사용
                // 측정 시작 시 생성된 폴더에 저장
                string? fileSavePath = LahgiApi.GetFileSavePath();
                if (!string.IsNullOrEmpty(fileSavePath))
                {
                    string? fileSaveDir = Path.GetDirectoryName(fileSavePath);
                    if (!string.IsNullOrEmpty(fileSaveDir) && Directory.Exists(fileSaveDir))
                    {
                        directoryPath = fileSaveDir;
                    }
                }
                
                logger.Info($"객체탐지 데이터 저장 경로 = {directoryPath}");

                // 파일명: yolo_result.csv
                string saveFilePath = Path.Combine(directoryPath, "yolo_result.csv");
                logger.Info($"객체탐지 데이터 저장 시도: {saveFilePath}");
                
                if (_objectDetectionService.SaveData(saveFilePath))
                {
                    logger.Info($"객체탐지 데이터 저장 완료: {saveFilePath}");
                }
                else
                {
                    logger.Warn($"객체탐지 데이터 저장 실패: {saveFilePath}");
                }
            }
            catch (Exception ex)
            {
                logger.Error($"객체탐지 데이터 저장 중 오류 발생: {ex.Message}", ex);
            }
        }

        // 객체탐지 서비스 정리
        private void CleanupObjectDetectionService()
        {
            if (_objectDetectionService != null)
            {
                _objectDetectionService.Dispose();
                _objectDetectionService = null;
                logger.Debug("ObjectDetectionService 정리 완료");
            }
        }

        // 객체탐지 서비스 접근자 (ReconstructionImageViewModel에서 사용)
        public ObjectDetectionService? GetObjectDetectionService()
        {
            return _objectDetectionService;
        }

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
        private int _serialCheckInProgress = 0;
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

            // COM 포트가 있으면 정상 체크 (UI 스레드 블로킹 방지: 백그라운드에서 단일 실행)
            if (Interlocked.CompareExchange(ref _serialCheckInProgress, 1, 0) == 0)
            {
                Task.Run(() =>
                {
                    try
                    {
                        LahgiSerialControl.CheckParams();
                    }
                    catch (Exception ex)
                    {
                        logger.Warn($"LahgiStatusUpdate: CheckParams 백그라운드 실행 중 예외: {ex.Message}");
                    }
                    finally
                    {
                        Interlocked.Exchange(ref _serialCheckInProgress, 0);
                    }
                });
            }

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
                // 단순 토글만 수행
                RealTimeCheck = !RealTimeCheck;
                
                System.Diagnostics.Debug.WriteLine($"SelectRealTimeCheck Command 실행 후 값: {RealTimeCheck}");
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
                // 단순 토글만 수행
                FaultDiagnosis = !FaultDiagnosis;
                
                System.Diagnostics.Debug.WriteLine($"SelectFaultDiagnosis Command 실행 후 값: {FaultDiagnosis}");
            });
        });


    }
}
