using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using log4net;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Reflection.Emit;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Imaging;

namespace HUREL_Imager_GUI.ViewModel
{
    public class HomeViewModel :ViewModelBase
    {
        private static readonly ILog logger = LogManager.GetLogger(typeof(HomeViewModel));

        private TopButtonViewModel _topButtonViewModel;
        public TopButtonViewModel TopButtonViewModel
        {
            get 
            { 
                return _topButtonViewModel; 
            } 
            set
            {
                _topButtonViewModel = value;
                OnPropertyChanged(nameof(TopButtonViewModel));
            }
        }
        private SpectrumViewModel _spectrumViewModel;
        
        public SpectrumViewModel SpectrumViewModel
        {
            get { return _spectrumViewModel; }
            set { _spectrumViewModel = value; OnPropertyChanged(nameof(SpectrumViewModel)); }
        }

        private ThreeDimensionalViewModel _threeDimensionalViewModel;
        public ThreeDimensionalViewModel ThreeDimensionalViewModel
        {
            get
            {
                return _threeDimensionalViewModel;
            }
            set
            {
                _threeDimensionalViewModel = value;
                OnPropertyChanged(nameof(ThreeDimensionalViewModel));
            }
        }

        private SourceDirectionViewModel _sourceDirectionViewModel;
        public SourceDirectionViewModel SourceDirectionViewModel
        {
            get
            {
               return _sourceDirectionViewModel; 
            }
            set
            {
                _sourceDirectionViewModel = value;
                OnPropertyChanged(nameof(SourceDirectionViewModel));
            }
        }

        private DoseRateViewModel _doseRateViewModel;
        public DoseRateViewModel DoseRateViewModel
        {
            get
            {
                return _doseRateViewModel;
            }
            set
            {
                _doseRateViewModel = value;
                OnPropertyChanged(nameof(DoseRateViewModel));
            }
        }

        private ReconstructionImageViewModel _reconstructionImageViewModel;
        public ReconstructionImageViewModel ReconstructionImageViewModel
        {
            get
            {
                return _reconstructionImageViewModel;
            }
            set
            {
                _reconstructionImageViewModel = value;
                OnPropertyChanged(nameof(ReconstructionImageViewModel));
            }
        }

        // 객체 탐지 모드용 Table 항목
        private ObservableCollection<HUREL_Imager_GUI.ViewModel.ObjectDetection.Models.PersonTableItem>? _personTableItems;
        public ObservableCollection<HUREL_Imager_GUI.ViewModel.ObjectDetection.Models.PersonTableItem> PersonTableItems
        {
            get
            {
                if (_personTableItems == null)
                {
                    _personTableItems = new ObservableCollection<HUREL_Imager_GUI.ViewModel.ObjectDetection.Models.PersonTableItem>();
                }
                return _personTableItems;
            }
            set
            {
                _personTableItems = value;
                OnPropertyChanged(nameof(PersonTableItems));
            }
        }

        // 객체 탐지 모드 여부
        private bool _isObjectDetectionMode = false;
        public bool IsObjectDetectionMode
        {
            get => _isObjectDetectionMode;
            set
            {
                _isObjectDetectionMode = value;
                OnPropertyChanged(nameof(IsObjectDetectionMode));
            }
        }
        
        public HomeViewModel()
        {
            // Will be not null!
            _topButtonViewModel = null!;
            _spectrumViewModel = null!;
            _testValue = null!;
            _threeDimensionalViewModel = null!;
            _sourceDirectionViewModel = null!;
            _doseRateViewModel= null!;

            TopButtonViewModel = new TopButtonViewModel();
            SpectrumViewModel = new SpectrumViewModel();
            ThreeDimensionalViewModel = new ThreeDimensionalViewModel();
            SourceDirectionViewModel = new SourceDirectionViewModel();
            _doseRateViewModel = new DoseRateViewModel();
            _doseRateViewModel.TopButtonVM = TopButtonViewModel; // DoseRateViewModel에 TopButtonVM 연결

            // SpectrumViewModel에도 TopButtonVM 연결 (선량 계산 및 측정 모드 판별에 필요)
            SpectrumViewModel.TopButtonVM = TopButtonViewModel;

            ReconstructionImageViewModel= new ReconstructionImageViewModel();
            ReconstructionImageViewModel.TopButtonVM = TopButtonViewModel; // ReconstructionImageViewModel에 TopButtonVM 연결

            // 객체 탐지 모드 변경 감지 - TopButtonViewModel의 PropertyChanged 이벤트 구독
            logger.Info($"HomeViewModel: TopButtonViewModel 인스턴스 생성 완료, HashCode={TopButtonViewModel.GetHashCode()}");
            logger.Info($"HomeViewModel: 초기 MeasurementMode={TopButtonViewModel.MeasurementMode}");
            
            TopButtonViewModel.PropertyChanged += (s, e) =>
            {
                logger.Info($"HomeViewModel: TopButtonViewModel PropertyChanged 이벤트 발생: {e.PropertyName}, HashCode={s?.GetHashCode()}");
                if (e.PropertyName == nameof(TopButtonViewModel.MeasurementMode))
                {
                    var newMode = TopButtonViewModel.MeasurementMode == eMeasurementMode.ObjectDetection;
                    logger.Info($"HomeViewModel: 객체 탐지 모드 변경 감지: {TopButtonViewModel.MeasurementMode}, IsObjectDetectionMode={newMode}");
                    
                    // UI 스레드에서 업데이트
                    if (Application.Current != null && Application.Current.Dispatcher != null)
                    {
                        Application.Current.Dispatcher.Invoke(() =>
                        {
                            IsObjectDetectionMode = newMode;
                            logger.Info($"HomeViewModel: IsObjectDetectionMode 업데이트 완료: {IsObjectDetectionMode}");
                        });
                    }
                    else
                    {
                        // Dispatcher가 없으면 직접 업데이트
                        IsObjectDetectionMode = newMode;
                        logger.Info($"HomeViewModel: IsObjectDetectionMode 업데이트 완료 (Dispatcher 없음): {IsObjectDetectionMode}");
                    }
                }
            };
            
            // 초기 객체 탐지 모드 상태 설정
            IsObjectDetectionMode = TopButtonViewModel.MeasurementMode == eMeasurementMode.ObjectDetection;
            logger.Info($"HomeViewModel: 초기 객체 탐지 모드 상태: {TopButtonViewModel.MeasurementMode}, IsObjectDetectionMode={IsObjectDetectionMode}");
            
            // 주기적으로 MeasurementMode 확인 (이벤트가 발생하지 않는 경우를 대비)
            var timer = new System.Timers.Timer(1000); // 1초마다 확인
            timer.Elapsed += (s, e) =>
            {
                try
                {
                    var currentMode = TopButtonViewModel.MeasurementMode == eMeasurementMode.ObjectDetection;
                    if (currentMode != IsObjectDetectionMode)
                    {
                        logger.Info($"HomeViewModel: Timer에서 객체 탐지 모드 변경 감지: {TopButtonViewModel.MeasurementMode}, IsObjectDetectionMode={currentMode}");
                        if (Application.Current != null && Application.Current.Dispatcher != null)
                        {
                            Application.Current.Dispatcher.Invoke(() =>
                            {
                                IsObjectDetectionMode = currentMode;
                            });
                        }
                        else
                        {
                            IsObjectDetectionMode = currentMode;
                        }
                    }
                }
                catch (Exception ex)
                {
                    logger.Error($"HomeViewModel: Timer에서 오류 발생: {ex.Message}");
                }
            };
            timer.Start();

            TestValue = "Hello World";
            logger.Info("HomeViewModel Loaded");

            LahgiApi.StatusUpdate += StatusUpdate;
        }

        private string statusMsg = "test";
        public string StatusMsg
        {
            get { return statusMsg; }
            set { statusMsg = value; OnPropertyChanged(nameof(StatusMsg)); }
        }

        Mutex StatusUpdateMutex = new Mutex();
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {
            if (!StatusUpdateMutex.WaitOne(0))
            {
                return;
            }
            if (eventArgs is LahgiApiEnvetArgs)
            {
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Massage)
                {
                    StatusMsg = LahgiApi.StatusMsg;
                }
            }
            StatusUpdateMutex.ReleaseMutex();

        }

      

        private string _testValue;
        public string TestValue
        {
            get { return _testValue; }
            set { _testValue = value; OnPropertyChanged(nameof(TestValue)); }
        }

        public override void Unhandle()
        {
            SpectrumViewModel.Unhandle();
            TopButtonViewModel.Unhandle();
            DoseRateViewModel.Unhandle();
            ThreeDimensionalViewModel.Unhandle();
            ReconstructionImageViewModel.Unhandle();

            LahgiApi.StatusUpdate -= StatusUpdate;
        }
        
    }
}
