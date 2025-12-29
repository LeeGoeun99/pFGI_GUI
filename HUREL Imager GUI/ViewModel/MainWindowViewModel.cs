using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using HUREL_Imager_GUI.State.Navigator;
using log4net;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;

namespace HUREL_Imager_GUI.ViewModel
{
    internal class MainWindowViewModel : ViewModelBase
    {
        private bool _isClosing = false; // 중복 실행 방지 플래그
        Stopwatch sw = new Stopwatch();

        // D455 카메라 업데이트 Task 및 종료 플래그 (ReconstructionImageViewModel과 동일한 패턴)
        private Task? _d455CameraUpdateTask;
        private bool _runD455CameraUpdate = true;

        private string _currentTime = "";
        public string CurrentTime
        {
            get => _currentTime;
            set
            {
                _currentTime = value;
                OnPropertyChanged(nameof(CurrentTime));
            }
        }

        //231017 sbkwon : Alarm 문구
        private string _alarmContent = "";
        public string AlarmContent
        {
            get => _alarmContent;
            set
            {
                if (_alarmContent != value)
                {
                    _alarmContent = value;
                    OnPropertyChanged(nameof(AlarmContent));
                }
            }
        }

        //230921 sbkwon : Alarm 화면 투명도
        private double _alarmOpacity = 0.0;
        public double AlramOpacity
        {
            get => _alarmOpacity;
            set
            {
                if (_alarmOpacity != value)
                {
                    _alarmOpacity = value;
                    OnPropertyChanged(nameof(AlramOpacity));
                }
            }
        }

        //230921 sbkwon : Alarm 화면 표시 여부 (visible / Collapsed)
        private Visibility _alramVisiblity = Visibility.Collapsed;
        public Visibility AlramVisibility
        {
            get => _alramVisiblity;
            set
            {
                if (_alramVisiblity != value)
                {
                    if (value is Visibility.Visible)
                        sw.Restart();
                    else
                        sw.Stop();

                    _alramVisiblity = value;
                    OnPropertyChanged(nameof(AlramVisibility));
                }
            }
        }

        //230921 sbkwon : Alarm 화면 배경 색
        private Brush _alramColor = Brushes.Yellow;
        public Brush AlramColor
        {
            get => _alramColor;
            set
            {
                if (_alramColor != value)
                {
                    _alramColor = value;
                    OnPropertyChanged(nameof(AlramColor));
                }
            }
        }

        //230921 sbkwon : Alarm 화면 깜빡임 주기
        private long _alarmCycleTime = 500;
        public long AlramCycleTime
        {
            get => _alarmCycleTime;
            set
            {
                if (value != _alarmCycleTime)
                {
                    _alarmCycleTime = value;
                    OnPropertyChanged(nameof(AlramCycleTime));
                }
            }
        }

        private readonly ILog logger = LogManager.GetLogger(nameof(MainWindowViewModel));

        //public INavigator Navigator { get; set; } = new Navigator();

        //private BottomSatusViewModel bottomStatusViewModel;
        //public BottomSatusViewModel BottomStatusViewModel
        //{
        //    get { return bottomStatusViewModel; }
        //    set
        //    {
        //        bottomStatusViewModel = value;
        //        OnPropertyChanged(nameof(BottomStatusViewModel));
        //    }
        //}

        public MainWindowViewModel()
        {
            try
            {
                var logger = log4net.LogManager.GetLogger(typeof(MainWindowViewModel));
                
                // 간단한 테스트 로그 추가
                System.Diagnostics.Debug.WriteLine("=== MainWindowViewModel 생성자 시작 (Debug.WriteLine) ===");
                logger.Info("MainWindowViewModel 생성자 시작 (log4net)");
                
                // 콘솔에도 직접 출력
                Console.WriteLine("=== MainWindowViewModel 생성자 시작 (Console.WriteLine) ===");

            // ViewModel 초기화
            TopButtonVM = new TopButtonViewModel();
            SpectrumVM = TopButtonVM.SpectrumVM;
            ThreeDimensionalVM = TopButtonVM.ThreeDimensionalVM;
            DoseRateVM = TopButtonVM.DoseRateVM;
            ReconstructionImageVM = TopButtonVM.ReconstructionVM;
            
            // 디버깅: ViewModel 초기화 상태 확인
            logger.Info($"TopButtonVM 초기화 완료: {TopButtonVM != null}");
            logger.Info($"SpectrumVM 초기화 완료: {SpectrumVM != null}");
            logger.Info($"ThreeDimensionalVM 초기화 완료: {ThreeDimensionalVM != null}");
            logger.Info($"DoseRateVM 초기화 완료: {DoseRateVM != null}");
            logger.Info($"ReconstructionImageVM 초기화 완료: {ReconstructionImageVM != null}");
            
            // SpectrumVM의 IsSpectrumAnalysisShow 초기값 확인
            if (SpectrumVM != null)
            {
                logger.Info($"SpectrumVM.IsSpectrumAnalysisShow 초기값: {SpectrumVM.IsSpectrumAnalysisShow}");
                logger.Info($"SpectrumVM.SelectIsSpectrumAnalysisShow Command: {SpectrumVM.SelectIsSpectrumAnalysisShow != null}");
                
                // 에너지 탭 바인딩 상태 상세 로그
                logger.Info("=== 에너지 탭 바인딩 상태 확인 ===");
                logger.Info($"SpectrumVM.SpectrumType: {SpectrumVM.SpectrumType}");
                logger.Info($"SpectrumVM.SpectrumCases: {SpectrumVM.SpectrumCases}");
                logger.Info($"SpectrumVM.FpgaChannelNumber: {SpectrumVM.FpgaChannelNumber}");
                logger.Info($"SpectrumVM.IsEcalUse: {SpectrumVM.IsEcalUse}");
                logger.Info($"SpectrumVM.Ref_x: {SpectrumVM.Ref_x}");
                logger.Info($"SpectrumVM.Ref_fwhm: {SpectrumVM.Ref_fwhm}");
                logger.Info($"SpectrumVM.Ref_at_0: {SpectrumVM.Ref_at_0}");
                logger.Info($"SpectrumVM.Min_snr: {SpectrumVM.Min_snr}");
                
                // Command 존재 여부 확인
                logger.Info($"SpectrumVM.SelectSpectrumTypeLinear Command: {SpectrumVM.SelectSpectrumTypeLinear != null}");
                logger.Info($"SpectrumVM.SelectSpectrumTypeLog Command: {SpectrumVM.SelectSpectrumTypeLog != null}");
                logger.Info($"SpectrumVM.SelectSpectrumCasesScatter Command: {SpectrumVM.SelectSpectrumCasesScatter != null}");
                logger.Info($"SpectrumVM.SelectSpectrumCasesAbsorber Command: {SpectrumVM.SelectSpectrumCasesAbsorber != null}");
                logger.Info($"SpectrumVM.SelectSpectrumCasesAll Command: {SpectrumVM.SelectSpectrumCasesAll != null}");
                logger.Info($"SpectrumVM.SelectSpectrumCasesByChannel Command: {SpectrumVM.SelectSpectrumCasesByChannel != null}");
                logger.Info($"SpectrumVM.SelectIsEcalUse Command: {SpectrumVM.SelectIsEcalUse != null}");
                logger.Info("=== 에너지 탭 바인딩 상태 확인 완료 ===");
            }

            // 기본 상태 업데이트
            TopButtonVM.LahgiStatusUpdate();
            ReconstructionImageVM.CheckD455CameraConnection();

            // D455 카메라 기본 이미지 초기화
            try
            {
                // not_connected.jpg 파일 경로 확인 및 설정
                string notConnectedPath = "pack://application:,,,/HUREL Imager GUI;component/Resource/not_connected.jpg";
                
                // 파일 존재 여부 확인 (런타임에서는 확인할 수 없으므로 try-catch로 처리)
                D455CameraImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(notConnectedPath));
                D455CameraStatusColor = System.Windows.Media.Brushes.Yellow;
                D455CameraStatusText = "초기화 대기 중...";
                IsD455CameraConnected = false;
                logger.Info("D455 카메라 기본 이미지 초기화 완료");
            }
            catch (Exception ex)
            {
                logger.Warn($"기본 이미지 초기화 실패: {ex.Message} - 색상 이미지로 대체");
                
                // not_connected.jpg 파일이 없을 때 간단한 색상 이미지 생성
                try
                {
                    D455CameraImage = CreateDefaultColorImage();
                    D455CameraStatusColor = System.Windows.Media.Brushes.Yellow;
                    D455CameraStatusText = "초기화 대기 중...";
                    IsD455CameraConnected = false;
                    logger.Info("색상 기본 이미지 생성 완료");
                }
                catch (Exception colorEx)
                {
                    logger.Warn($"색상 이미지 생성도 실패: {colorEx.Message} - null로 설정");
                    D455CameraImage = null;
                    D455CameraStatusColor = System.Windows.Media.Brushes.Yellow;
                    D455CameraStatusText = "초기화 대기 중...";
                    IsD455CameraConnected = false;
                }
            }

            // D455 카메라 업데이트 시작 (원래 방식: Task.Run으로 백그라운드 스레드에서 실행)
            _d455CameraUpdateTask = Task.Run(() => StartD455CameraUpdate());

            // LahgiApi에서 이미 RTAB-Map 초기화가 완료되었으므로 중복 초기화 제거
            // _ = Task.Run(async () => await InitializeRtabmapAsync());

            // 에너지 스펙트럼은 실제 측정 데이터가 들어올 때까지 비어있는 상태로 유지
            // AddSampleDataToSpectrum(); // 제거됨

            logger.Info("MainWindowViewModel 생성자 완료");
            }
            catch (Exception ex)
            {
                // logger가 null일 수 있으므로 안전하게 처리
                try
                {
                    var logger = log4net.LogManager.GetLogger(typeof(MainWindowViewModel));
                    logger.Error($"MainWindowViewModel 생성자에서 예외 발생: {ex.Message}");
                    logger.Error($"예외 타입: {ex.GetType().Name}");
                    logger.Error($"스택 트레이스: {ex.StackTrace}");
                }
                catch
                {
                    // 로그 출력도 실패한 경우 콘솔에 출력
                    Console.WriteLine($"MainWindowViewModel 생성자에서 예외 발생: {ex.Message}");
                    Console.WriteLine($"예외 타입: {ex.GetType().Name}");
                    Console.WriteLine($"스택 트레이스: {ex.StackTrace}");
                }
                
                // 예외를 다시 던져서 상위에서 처리할 수 있도록 함
                throw;
            }
        }


        //231100-GUI sbkwon : 프로그램 종료시
        private void WriteConfig()
        {
            App.GlobalConfig.SpectrumEffectTime = SpectrumVM.SpectrumTime;
            App.GlobalConfig.isSpectrumAnalysisShow = SpectrumVM.IsSpectrumAnalysisShow;
            App.GlobalConfig.SpectrumCases = SpectrumVM.SpectrumCases;
            App.GlobalConfig.FpgaChannelNumber = SpectrumVM.FpgaChannelNumber;
            App.GlobalConfig.SpectrumType = SpectrumVM.SpectrumType;
            App.GlobalConfig.IsECalUse = SpectrumVM.IsEcalUse;
            App.GlobalConfig.ECalIntervalTime = SpectrumVM.IntervalECalTime * 60000;  //231113-1 sbkwon : 화면 표시 분, config 단위 ms
            App.GlobalConfig.Ref_x = SpectrumVM.Ref_x;
            App.GlobalConfig.Ref_fwhm = SpectrumVM.Ref_fwhm;
            App.GlobalConfig.Ref_at_0 = SpectrumVM.Ref_at_0;
            App.GlobalConfig.Min_snr = SpectrumVM.Min_snr;

                    App.GlobalConfig.ReconSpaceAuto = ReconstructionImageVM.ReconSpaceAuto;
        App.GlobalConfig.ReconSpaceManual = ReconstructionImageVM.ReconSpaceManual;
        App.GlobalConfig.ReconSpace = ReconstructionImageVM.ReconSpace;
        App.GlobalConfig.RGBImageType = ReconstructionImageVM.RGBType;
            App.GlobalConfig.OpacityValue = ReconstructionImageVM.OpacityIntValue;
            App.GlobalConfig.MinValuePortion = ReconstructionImageVM.MinValuePortionint;
            App.GlobalConfig.ReconType = ReconstructionImageVM.ReconType;
            App.GlobalConfig.ReconMeasurTime = ReconstructionImageVM.ReconMeasurTime;
            App.GlobalConfig.ReconMeasurCount = ReconstructionImageVM.ReconMeasurCount;
            App.GlobalConfig.S2M = ReconstructionImageVM.S2M;
            App.GlobalConfig.ReconMaxValue = ReconstructionImageVM.ReconMaxValue;
            App.GlobalConfig.UseLabelingCheck = ReconstructionImageVM.LabelingCheck; //241021
            App.GlobalConfig.VisualizationRange = ReconstructionImageVM.VisualizationRange;

            App.GlobalConfig.SaveFileName = TopButtonVM.FileName;
            App.GlobalConfig.MeasurementTime = TopButtonVM.MeasurementTime;
            App.GlobalConfig.MeasurementType = TopButtonVM.MeasuremetType;
            App.GlobalConfig.UseRealTimeCheck = TopButtonVM.RealTimeCheck;  //240206
            App.GlobalConfig.RealTimeCycleTime = TopButtonVM.RealTimeCheckCycleTime;  //240206
                    App.GlobalConfig.UseFaultDiagnosis = TopButtonVM.FaultDiagnosis;    //240206
        App.GlobalConfig.FaultDiagnosisMeasurementTime = TopButtonVM.FaultDiagnosisMeasurementTime; //240206
        App.GlobalConfig.Type3DView = TopButtonVM.ThreeDimensionalVM.Type3DView;
        }

        private AsyncCommand? closingCommand = null;
        public ICommand ClosingCommand
        {
            get
            {
                return closingCommand ?? (closingCommand = new AsyncCommand(Closing));
            }
        }
        internal async Task Closing()
        {
            // 중복 실행 방지
            if (_isClosing)
            {
                return;
            }
            _isClosing = true;
            
            var logger = LogManager.GetLogger(typeof(MainWindowViewModel));
            logger.Info("Closing 시작");
            
            try
            {
                //Navigator?.CurrentViewModel?.Unhandle();
                LahgiApi.StopUpdateInvoker();

                WriteConfig();

                LahgiApi.StopAll();

                // StopFPGA는 타임아웃 없이 실행 (시리얼 포트 타임아웃은 내부에서 처리)
                try
                {
                    LahgiApi.StopFPGA();
                }
                catch (Exception ex)
                {
                    logger.Warn($"StopFPGA 중 예외 발생 (무시): {ex.Message}");
                }

                //240315 : stop_usb() - 타임아웃 추가
                try
                {
                    var stopUsbTask = LahgiApi.StopUSBAsync();
                    if (await Task.WhenAny(stopUsbTask, Task.Delay(5000)).ConfigureAwait(false) == stopUsbTask)
                    {
                        await stopUsbTask.ConfigureAwait(false);
                        logger.Info("StopUSBAsync 완료");
                    }
                    else
                    {
                        logger.Warn("StopUSBAsync 타임아웃 (5초 초과) - 강제 종료 진행");
                    }
                }
                catch (Exception ex)
                {
                    logger.Warn($"StopUSBAsync 중 예외 발생 (무시): {ex.Message}");
                }

                         
                Unhandle();
                logger.Info("Closing 완료");
            }
            catch (Exception ex)
            {
                logger.Error($"Closing 중 예외 발생: {ex.Message}");
                logger.Error($"스택 트레이스: {ex.StackTrace}");
            }
        }

        public override void Unhandle()
        {
            LogManager.GetLogger(typeof(MainWindowViewModel)).Info("Unhandle");

            // D455 카메라 업데이트 중단 (ReconstructionImageViewModel과 동일한 패턴)
            if (_d455CameraUpdateTask != null)
            {
                _runD455CameraUpdate = false;
                try
                {
                    // 최대 1초 대기 (Thread.Sleep(100)이 최대 10번 실행될 수 있으므로)
                    if (!_d455CameraUpdateTask.Wait(1000))
                    {
                        LogManager.GetLogger(typeof(MainWindowViewModel)).Warn("D455 카메라 업데이트 Task 종료 대기 시간 초과");
                    }
                }
                catch (Exception ex)
                {
                    LogManager.GetLogger(typeof(MainWindowViewModel)).Error($"D455 카메라 업데이트 Task 종료 중 예외: {ex.Message}");
                }
            }

            TopButtonVM.Unhandle();
            SpectrumVM.Unhandle();
            ThreeDimensionalVM.Unhandle();
            DoseRateVM.Unhandle();
            ReconstructionImageVM.Unhandle();

            if (SoundPlayer != null)    //230921 sbkwon : 종료시
            {
                SoundPlayer.Stop();
                SoundPlayer.Dispose();
                SoundPlayer = null;
            }

            DoseRateViewModel.AlarmUpdate -= AlarmUpdate;   //231025-1 sbkwon
        }

        private TopButtonViewModel _topButtonVM;
        public TopButtonViewModel TopButtonVM
        {
            get => _topButtonVM;
            set
            {
                _topButtonVM = value;
                OnPropertyChanged(nameof(TopButtonVM));
            }
        }

        private SpectrumViewModel _spectrumVM;

        public SpectrumViewModel SpectrumVM
        {
            get { return _spectrumVM; }
            set { _spectrumVM = value; OnPropertyChanged(nameof(SpectrumVM)); }
        }

        private ThreeDimensionalViewModel _threeDimensionalVM;
        public ThreeDimensionalViewModel ThreeDimensionalVM
        {
            get { return _threeDimensionalVM; }
            set
            {
                _threeDimensionalVM = value;
                OnPropertyChanged(nameof(ThreeDimensionalVM));
            }
        }

        private DoseRateViewModel _doseRateVM;
        public DoseRateViewModel DoseRateVM
        {
            get { return _doseRateVM; }
            set
            {
                _doseRateVM = value;
                OnPropertyChanged(nameof(DoseRateVM));
            }
        }

        private ReconstructionImageViewModel _reconstructionImageVM;
        public ReconstructionImageViewModel ReconstructionImageVM
        {
            get { return _reconstructionImageVM; }
            set
            {
                _reconstructionImageVM = value;
                OnPropertyChanged(nameof(ReconstructionImageVM));
            }
        }

        //230920 sbkwon : DoseRate Alarm Event
        //231100-GUI 위치 이동 : HomeViewModel => ManinWindowViewModel
        private void AlarmUpdate(object? sender, EventArgs e)
        {
            if (e is AlarmEventArgs)
            {
                AlarmEventArgs alarmEventArgs = (AlarmEventArgs)e;

                AlarmDisplay(alarmEventArgs.AlarmStatus);
                AlarmSound(alarmEventArgs.AlarmStatus);
            }
        }

        //230911 sbkwon : 선량률에 따른 경보 설정 4~5단계
        private System.Media.SoundPlayer? SoundPlayer = null;

        //230911 sbkwon : 경보 단계 별 설정
        public void AlarmSound(enAlarm enLevel)
        {
            if (SoundPlayer != null)
            {
                SoundPlayer.Stop();
                SoundPlayer.Dispose();
                SoundPlayer = null;
            }

            if (enLevel < enAlarm.enAlarm3 || enLevel > enAlarm.enAlarm5)
                return;

            string? filepath = null;

            if (enLevel == enAlarm.enAlarm3)
            {
                filepath = "Sound\\Alarm3.wav";
            }
            else if (enLevel == enAlarm.enAlarm4)
            {
                filepath = "Sound\\Alarm4.wav";
            }
            else if (enLevel == enAlarm.enAlarm5)
            {
                filepath = "Sound\\Alarm5.wav";
            }

            if (!File.Exists(filepath))
            {
                LogManager.GetLogger(typeof(MainWindowViewModel)).Info($"not exist File : {enLevel}");
                return;
            }

            SoundPlayer = new System.Media.SoundPlayer(filepath ?? string.Empty);

            SoundPlayer.PlayLooping();
        }

        //231100-GUI 위치 이동 : HomeViewModel => ManinWindowViewModel
        //240206-Gain : warning 추가 및 일부 수정
        private void AlarmDisplay(enAlarm alarm)
        {
            //if (App.MainVM is not null)
            {
                AlramVisibility = Visibility.Collapsed;
                AlarmContent = "";

                if (alarm == enAlarm.enAlarm3)
                {
                    AlramColor = System.Windows.Media.Brushes.Yellow;
                    AlramCycleTime = 1000;
                    AlramVisibility = Visibility.Visible;
                    AlarmContent = "일반인 구역 설계기준 선량율 이상의 선원이 발견되었습니다";
                }
                else if (alarm == enAlarm.enAlarm4)
                {
                    AlramColor = System.Windows.Media.Brushes.Orange;
                    AlramCycleTime = 800;
                    AlramVisibility = Visibility.Visible;
                    AlarmContent = "Alarm 4";
                }
                else if (alarm == enAlarm.enAlarm5)
                {
                    AlramColor = System.Windows.Media.Brushes.Red;
                    AlramCycleTime = 500;
                    AlramVisibility = Visibility.Visible;
                    AlarmContent = "방사선 관리 구역 설계기준 선량율 이상의 선원이 발견되었습니다"; //문구
                }
                else if (alarm == enAlarm.enPeakToValleyWarning)
                {
                    AlarmContent = $"{TopButtonVM.ErrorPeakToValleyCount}개의 채널에서 문제가 발견되었습니다. 정밀검사를 진행하세요"; //240228 : 문구
                }
                else if (alarm == enAlarm.enFaultDiagnosisWarning) //240228
                {
                    AlarmContent = "측정이 더 필요합니다.";
                }
                else if (alarm == enAlarm.enFaultDiagnosisError) //240228
                {
                    AlarmContent = "AS 필요.";
                }
            }
        }

        //2404 : MLEM
        private void UIMessageUpdate(object? sender, EventArgs e)
        {
            AlramVisibility = Visibility.Collapsed;
            AlarmContent = LahgiApi.UIMessage;
        }

        // D455 카메라 영상 바인딩을 위한 속성들
        private System.Windows.Media.Imaging.BitmapImage _d455CameraImage = null;
        public System.Windows.Media.Imaging.BitmapImage D455CameraImage
        {
            get => _d455CameraImage;
            set
            {
                _d455CameraImage = value;
                OnPropertyChanged(nameof(D455CameraImage));
            }
        }

        private bool _isD455CameraConnected = false;
        public bool IsD455CameraConnected
        {
            get => _isD455CameraConnected;
            set
            {
                _isD455CameraConnected = value;
                OnPropertyChanged(nameof(IsD455CameraConnected));
            }
        }

        private System.Windows.Media.Brush _d455CameraStatusColor = System.Windows.Media.Brushes.Red;
        public System.Windows.Media.Brush D455CameraStatusColor
        {
            get => _d455CameraStatusColor;
            set
            {
                _d455CameraStatusColor = value;
                OnPropertyChanged(nameof(D455CameraStatusColor));
            }
        }

        private string _d455CameraStatusText = "점검 필요";
        public string D455CameraStatusText
        {
            get => _d455CameraStatusText;
            set
            {
                _d455CameraStatusText = value;
                OnPropertyChanged(nameof(D455CameraStatusText));
            }
        }



        // D455 카메라 영상 주기적 업데이트 시작 (원래 방식: ReconstructionImageViewModel과 동일한 패턴)
        private void StartD455CameraUpdate()
        {
            var logger = log4net.LogManager.GetLogger(typeof(MainWindowViewModel));
            
            // not_connected.jpg 경로 정의
            string notConnectedPath = "pack://application:,,,/HUREL Imager GUI;component/Resource/not_connected.jpg";
            
            // 기본 이미지 로드 시도
            System.Windows.Media.Imaging.BitmapImage defaultImage = null;
            try
            {
                defaultImage = new System.Windows.Media.Imaging.BitmapImage(new Uri(notConnectedPath));
                logger.Info("not_connected.jpg 기본 이미지 로드 성공");
            }
            catch (Exception ex)
            {
                logger.Warn($"not_connected.jpg 기본 이미지 로드 실패: {ex.Message} - 기본 이미지 없이 진행");
                defaultImage = null;
            }

            // 원래 방식: while 루프에서 직접 실행 (async/await 제거, Thread.Sleep 사용)
            // ReconstructionImageViewModel과 동일한 패턴: 종료 플래그 사용
            while (_runD455CameraUpdate)
            {
                try
                {
                    // RTAB-Map이 초기화된 경우에만 이미지 획득 시도
                    if (LahgiApi.IsRtabmapInitiate)
                    {
                        // 원래 방식: LahgiApi.GetRgbImage() 직접 호출 (파라미터 없음, 기본값 사용)
                        var rgbImage = LahgiApi.GetRgbImage();
                        
                        if (rgbImage != null && rgbImage.Width > 0 && rgbImage.Height > 0)
                        {
                            // 원래 방식: ReconstructionImageViewModel과 동일하게 직접 할당 (Dispatcher 없이)
                            D455CameraImage = rgbImage;
                            IsD455CameraConnected = true;
                            D455CameraStatusColor = System.Windows.Media.Brushes.Green;
                            D455CameraStatusText = "연결됨";
                            
                            // 원래 방식: Thread.Sleep 사용 (ReconstructionImageViewModel과 동일)
                            Thread.Sleep(100);
                        }
                        else
                        {
                            // RTAB-Map이 초기화되었지만 이미지가 없는 경우 - 기본 이미지 표시
                            // 원래 방식: ReconstructionImageViewModel과 동일하게 직접 할당 (Dispatcher 없이)
                            if (defaultImage != null)
                            {
                                D455CameraImage = defaultImage;
                            }
                            else
                            {
                                D455CameraImage = null;
                            }
                            
                            IsD455CameraConnected = true;
                            D455CameraStatusColor = System.Windows.Media.Brushes.Orange;
                            D455CameraStatusText = "영상 스트림 없음";
                            
                            Thread.Sleep(100);
                        }
                    }
                    else
                    {
                        // RTAB-Map이 초기화되지 않은 경우 - 기본 이미지 표시
                        // 원래 방식: ReconstructionImageViewModel과 동일하게 직접 할당 (Dispatcher 없이)
                        if (defaultImage != null)
                        {
                            D455CameraImage = defaultImage;
                        }
                        else
                        {
                            D455CameraImage = null;
                        }
                        
                        IsD455CameraConnected = false;
                        D455CameraStatusColor = System.Windows.Media.Brushes.Yellow;
                        D455CameraStatusText = "초기화 대기 중...";
                        
                        Thread.Sleep(100);
                    }
                }
                catch (Exception ex)
                {
                    logger.Error($"D455 카메라 업데이트 중 예외: {ex.Message}");
                    
                    // 예외 발생 시 기본 이미지 표시
                    // 원래 방식: ReconstructionImageViewModel과 동일하게 직접 할당 (Dispatcher 없이)
                    try
                    {
                        if (defaultImage != null)
                        {
                            D455CameraImage = defaultImage;
                        }
                        else
                        {
                            D455CameraImage = null;
                        }
                        
                        IsD455CameraConnected = false;
                        D455CameraStatusColor = System.Windows.Media.Brushes.Red;
                        D455CameraStatusText = "오류 발생";
                    }
                    catch (Exception uiEx)
                    {
                        logger.Error($"예외 처리 중 기본 이미지 설정 실패: {uiEx.Message}");
                    }
                    
                    Thread.Sleep(100);
                }
            }
        }

        private async Task InitializeRtabmapAsync()
        {
            var logger = log4net.LogManager.GetLogger(typeof(MainWindowViewModel));
            int retryCount = 0;
            const int maxRetries = 10; // 최대 10번까지 재시도

            try
            {
                logger.Info("RTAB-Map 자동 초기화 시작...");
                
                // 프로그램 시작 후 잠시 대기 (하드웨어 초기화 시간 확보)
                await Task.Delay(3000);
                
                while (retryCount < maxRetries && !LahgiApi.IsRtabmapInitiate)
                {
                    retryCount++;
                    logger.Info($"RTAB-Map 초기화 시도 {retryCount}/{maxRetries}...");
                    
                    // RTAB-Map 초기화 시도
                    bool rtabmapInitResult = LahgiApi.InititateRtabmap();
                    if (rtabmapInitResult)
                    {
                        logger.Info("RTAB-Map 초기화 성공!");
                        
                        // 초기화 성공 후 D455 카메라 상태 업데이트
                        await Application.Current.Dispatcher.InvokeAsync(() =>
                        {
                            try
                            {
                                D455CameraStatusColor = System.Windows.Media.Brushes.Green;
                                D455CameraStatusText = "초기화 완료";
                                IsD455CameraConnected = true;
                                logger.Info("D455 카메라 상태 업데이트 완료");
                            }
                            catch (Exception uiEx)
                            {
                                logger.Error($"D455 카메라 상태 업데이트 중 예외: {uiEx.Message}");
                            }
                        });
                        break;
                    }
                    else
                    {
                        logger.Warn($"RTAB-Map 초기화 실패 (시도 {retryCount}/{maxRetries}): {LahgiApi.StatusMsg}");
                        
                        if (retryCount < maxRetries)
                        {
                            // 다음 시도 전 3초 대기 (더 긴 대기 시간)
                            logger.Info("3초 후 재시도...");
                            await Task.Delay(3000);
                        }
                    }
                }

                if (!LahgiApi.IsRtabmapInitiate)
                {
                    logger.Error($"RTAB-Map 초기화 실패: 최대 재시도 횟수({maxRetries}) 초과");
                    
                    // 최종 실패 시 상태 업데이트
                    await Application.Current.Dispatcher.InvokeAsync(() =>
                    {
                        try
                        {
                            D455CameraStatusColor = System.Windows.Media.Brushes.Red;
                            D455CameraStatusText = "RTAB-Map 초기화 실패";
                            IsD455CameraConnected = false;
                        }
                        catch (Exception uiEx)
                        {
                            logger.Error($"최종 상태 업데이트 중 예외: {uiEx.Message}");
                        }
                    });
                }
            }
            catch (Exception ex)
            {
                logger.Error($"RTAB-Map 초기화 중 예외 발생: {ex.Message}");
            }
        }

        private System.Windows.Media.Imaging.BitmapImage CreateDefaultColorImage()
        {
            try
            {
                // 가장 간단하고 안전한 방법으로 기본 이미지 생성
                // 복잡한 그래픽 작업 대신 단순한 색상 브러시 사용
                var bitmapImage = new System.Windows.Media.Imaging.BitmapImage();
                
                // 1x1 픽셀의 단순한 이미지 생성 (메모리 사용량 최소화)
                var writeableBitmap = new System.Windows.Media.Imaging.WriteableBitmap(
                    1, 1, 96, 96, System.Windows.Media.PixelFormats.Bgra32, null);
                
                // 단일 픽셀 데이터 (회색)
                byte[] pixelData = { 128, 128, 128, 255 }; // BGRA
                writeableBitmap.WritePixels(new System.Windows.Int32Rect(0, 0, 1, 1), pixelData, 4, 0);
                
                // WriteableBitmap을 BitmapImage로 변환
                var encoder = new System.Windows.Media.Imaging.PngBitmapEncoder();
                encoder.Frames.Add(System.Windows.Media.Imaging.BitmapFrame.Create(writeableBitmap));
                
                using (var stream = new System.IO.MemoryStream())
                {
                    encoder.Save(stream);
                    stream.Position = 0;
                    
                    bitmapImage.BeginInit();
                    bitmapImage.CacheOption = System.Windows.Media.Imaging.BitmapCacheOption.OnLoad;
                    bitmapImage.StreamSource = stream;
                    bitmapImage.EndInit();
                    bitmapImage.Freeze();
                }

                return bitmapImage;
            }
            catch (Exception ex)
            {
                // 이미지 생성 실패 시 null 반환
                var logger = log4net.LogManager.GetLogger(typeof(MainWindowViewModel));
                logger.Warn($"기본 이미지 생성 실패: {ex.Message}");
                return null;
            }
        }


    }
}
