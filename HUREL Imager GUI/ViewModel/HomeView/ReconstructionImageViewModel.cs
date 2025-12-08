using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using log4net;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Brush = System.Windows.Media.Brush;
using Brushes = System.Windows.Media.Brushes;

namespace HUREL_Imager_GUI.ViewModel
{
    //231025-1 sbkwon : 영상화 방법 (Plane or Pointcloud)
    public enum eReconSpace
    {
        Plane,
        Pointcloud
    };

    //240326 : 영상 정합 표시 방법
    public enum eReconOption
    {
        type1,  //RGB 영상에 재구성영상을 크롭해서 표시
        type2,  //재구성 영상(360*180)에 RGB FOV 고려 표시
        type3,  //재구성 영상(360*180)만 표시
    }

    //231025-2 sbkwon : 영상 정합 방법
    public enum eReconType
    {
        None,
        ComptonImage,
        CodedImage,
        HybridImage
    };

    public enum eRGBType
    {
        None,
        Color,
        Gray
    };

    public class ReconstructionImageViewModel : ViewModelBase
    {
        public TopButtonViewModel TopButtonVM { get; set; } //240228
        public ReconstructionImageViewModel()
        {
            //231100-GUI sbkwon
            ReconSpaceAuto = App.GlobalConfig.ReconSpaceAuto;
            ReconSpaceManual = App.GlobalConfig.ReconSpaceManual;
            ReconSpace = App.GlobalConfig.ReconSpace;
            
                         // 영상모드 디폴트: 자동
             if (!ReconSpaceAuto && !ReconSpaceManual)
             {
                 ReconSpaceAuto = true;
                 ReconSpaceManual = false;
             }
             // 초기화 시 자동모드가 true이면 수동모드를 false로 설정
             else if (ReconSpaceAuto)
             {
                 ReconSpaceManual = false;
             }
             // ReconSpaceAuto가 false일 때만 ReconSpaceManual을 true로 설정
             else
             {
                 ReconSpaceManual = true;
             }
             
             // RGBType 기본값: 흑백 (Gray)
             if (App.GlobalConfig.RGBImageType == eRGBType.None)
             {
                 RGBType = eRGBType.Gray;
             }
             else
             {
                 RGBType = App.GlobalConfig.RGBImageType;
             }
            OpacityValue = App.GlobalConfig.OpacityValue;
            MinValuePortionint = App.GlobalConfig.MinValuePortion;
            ReconType = App.GlobalConfig.ReconType;
            ReconMeasurTime = App.GlobalConfig.ReconMeasurTime;
            ReconMeasurCount = App.GlobalConfig.ReconMeasurCount;
            S2M = App.GlobalConfig.S2M;
            ReconMaxValue = App.GlobalConfig.ReconMaxValue;
            LabelingCheck = App.GlobalConfig.UseLabelingCheck; //241021
            VisualizationRange = App.GlobalConfig.VisualizationRange;

            realtimeRGB = new BitmapImage();
            LoopTask = Task.Run(Loop);
            codedImgRGB = new BitmapImage();
            comptonImgRGB = new BitmapImage();
            hybridImgRGB = new BitmapImage();

            GridCreat();    //240327

            LahgiApi.StatusUpdate += StatusUpdate;
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
                //if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Status || lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.SlamRadImage)
                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.SlamRadImage && LahgiApi.TimerBoolSlamRadImage)
                {
                    BitmapImage? tmpCode;
                    BitmapImage? tmpCompton;
                    BitmapImage? tmpHybrid;

                    //848 480 90 60    51 90  48 85
                    //231025-1 sbkwon : Recon type
                    //if (ReconSpace == eReconSpace.Pointcloud)
                    //    (tmpCode, tmpCompton, tmpHybrid) = LahgiApi.GetRadation2dImage(timeInMiliSeconds, S2M, Det_W, ResImprov, M2D, MinValuePortion);    //231025-1 sbkwon Posint cloud recon
                    //else
                    //    (tmpCode, tmpCompton, tmpHybrid) = LahgiApi.GetRadation2dImage(timeInMiliSeconds, S2M, Det_W, ResImprov, M2D, 58, 90, ImgSize, MinValuePortion);    //231023 sbkwon : RGB FOV 동일하게

                    //250115 init
                    if (MLEM2DVisibility)
                        MLEM2DVisibility = false;
                    if (MLEM2DRGB)
                        MLEM2DRGB = false;

                    //231222 : SlamedPointCloud Count를 이용하여 영상 공간 설정
                    if (ReconSpaceAuto == true)
                    {
                        int nCount = LahgiApi.GetSlamedPointCloudCount();
                        try
                        {
                            _isAutoAlgorithmUpdate = true;
                            if (nCount > 10000)
                            {
                                if (ReconSpace != eReconSpace.Pointcloud)
                                    ReconSpace = eReconSpace.Pointcloud;
                            }
                            else
                            {
                                if (ReconSpace != eReconSpace.Plane)
                                    ReconSpace = eReconSpace.Plane;
                            }

                            // 자동 모드에서는 실외 옵션을 항상 type1로 유지
                            if (ReconOption != eReconOption.type1)
                                ReconOption = eReconOption.type1;
                        }
                        finally
                        {
                            _isAutoAlgorithmUpdate = false;
                        }
                    }

                    //231100-GUI sbkwon : 누적 카운트 수로 변경
                    if (ReconSpace == eReconSpace.Pointcloud)
                        (tmpCode, tmpCompton, tmpHybrid) = LahgiApi.GetRadation2dImageCount(ReconMeasurCount, S2M, Det_W, ResImprov, M2D, MinValuePortion, ReconMaxValue, ReconMeasurTime, LabelingCheck);    //231100-GUI sbkwon Posint cloud recon
                    else //d455 58 87 //d435 42 69 //d457 55 87
                    {
                        if(ReconOption == eReconOption.type2 || ReconOption == eReconOption.type3)
                            (tmpCode, tmpCompton, tmpHybrid) = LahgiApi.GetRadation2dImageCount(ReconMeasurCount, S2M, Det_W, ResImprov, M2D, 58, 87, ImgSize, MinValuePortion, ReconMaxValue, ReconMeasurTime, true, LabelingCheck);    //231100-GUI sbkwon : RGB FOV 동일하게, 240311
                        else
                            (tmpCode, tmpCompton, tmpHybrid) = LahgiApi.GetRadation2dImageCount(ReconMeasurCount, S2M, Det_W, ResImprov, M2D, 58, 87, ImgSize, MinValuePortion, ReconMaxValue, ReconMeasurTime,false, LabelingCheck);    //231100-GUI sbkwon : RGB FOV 동일하게, 240311
                    }

                    //test sbkwon
                    //RGBCameraStatus = $"{tmpCompton.PixelHeight} : {tmpCompton.PixelWidth}";


                    //tmpCode = LahgiApi.GetTransPoseRadiationImage(timeInMiliSeconds, minValuePortion, 10);
                    if (tmpCode == null || tmpCompton == null || tmpHybrid == null)
                    {
                        StatusUpdateMutex.ReleaseMutex();

                        return;
                    }
                    else
                    {
                        CodedImgRGB = tmpCode;
                        ComptonImgRGB = tmpCompton;
                        HybridImgRGB = tmpHybrid;
                    }
                }
                //250107 2D MLEM : 결과 영상
                else if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.MLEM)
                {
                    //기존 파일 삭제
                    //var fi = new FileInfo("_MLEM2DSelect.png");
                    //if ( fi.Exists )
                    //{
                    //    File.Delete( fi.FullName );
                    //}
                    VisibitityCompton = Visibility.Hidden;
                    VisibitityCoded = Visibility.Hidden;
                    VisibitityHybrid = Visibility.Hidden;

                    LogManager.GetLogger(typeof(ReconstructionImageViewModel)).Info($"정밀영상 2D 결과 표시 : {LahgiApi.MLEMSelectNo}");
                   
                    LahgiApi.Get2DMLEMImage(LahgiApi.MLEMSelectNo);

                    

                    MLEM2DRGB = false;
                    MLEM2DVisibility = true;

                    MLEM2DDisplay();                    
                }
                //250107 2D MLEM : RGB 초기 영상 출력
                else if (/*LahgiApi.MLEMRun == true && */lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading)
                {
                    VisibitityCompton = Visibility.Hidden;
                    VisibitityCoded = Visibility.Hidden;
                    VisibitityHybrid = Visibility.Hidden;

                    LogManager.GetLogger(typeof(ReconstructionImageViewModel)).Info($"정밀영상 Loading");

                    MLEMLoading();

                    MLEM2DVisibility = false;
                    MLEM2DRGB = true;
                }
            }

            StatusUpdateMutex.ReleaseMutex();
        }

        //231025-2 sbkwon : ReconType
        private eReconType _reconType = eReconType.HybridImage;
        public eReconType ReconType
        {
            get { return _reconType; }
            set 
            { 
                _reconType = value;

                SetVisibitity();
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.ReconType = value;

                OnPropertyChanged(nameof(ReconType)); 
            }
        }

        //250107 type 변경
        public void SetVisibitity()
        {
            VisibitityCompton = Visibility.Hidden;
            VisibitityCoded = Visibility.Hidden;
            VisibitityHybrid = Visibility.Hidden;
            switch (ReconType)
            {
                case eReconType.HybridImage:
                    VisibitityHybrid = Visibility.Visible;
                    break;
                case eReconType.ComptonImage:
                    VisibitityCompton = Visibility.Visible;
                    break;
                case eReconType.CodedImage:
                    VisibitityCoded = Visibility.Visible;
                    break;
                default:
                    VisibitityHybrid = Visibility.Visible;
                    break;
            }
        }

        //231100-GUI sbkwon
        private Visibility _visibitityHybrid = Visibility.Visible;
        private Visibility _visibitityCompton = Visibility.Hidden;
        private Visibility _visibitityCoded = Visibility.Hidden;
        public Visibility VisibitityHybrid
        {
            get => _visibitityHybrid;
            set { _visibitityHybrid = value; OnPropertyChanged(nameof(VisibitityHybrid)); }
        }
        public Visibility VisibitityCompton
        {
            get => _visibitityCompton;
            set { _visibitityCompton = value; OnPropertyChanged(nameof(VisibitityCompton)); }
        }

        public Visibility VisibitityCoded
        {
            get => _visibitityCoded;
            set { _visibitityCoded = value; OnPropertyChanged(nameof(VisibitityCoded)); }
        }


        //231109-1 sbkwon
        private AsyncCommand? _selectNoneRecon = null;

        public ICommand SelectNoneRecon
        {
            get { return _selectNoneRecon ?? (_selectNoneRecon = new AsyncCommand(SelReconType1)); }
        }

        private Task SelReconType1() => Task.Run(() =>
        {
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                ReconType = eReconType.None;
            });
        });

        private AsyncCommand? _selectComptonImageRecon = null;

        public ICommand SelectComptonImageRecon
        {
            get { return _selectComptonImageRecon ?? (_selectComptonImageRecon = new AsyncCommand(SelReconType2)); }
        }

        private Task SelReconType2() => Task.Run(() =>
        {
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                ReconType = eReconType.ComptonImage;
            });
        });

        private AsyncCommand? _selectCodedImageRecon = null;

        public ICommand SelectCodedImageRecon
        {
            get { return _selectCodedImageRecon ?? (_selectCodedImageRecon = new AsyncCommand(SelReconType3)); }
        }

        private Task SelReconType3() => Task.Run(() =>
        {
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                ReconType = eReconType.CodedImage;
            });
        });

        private AsyncCommand? _selectHypridImageRecon = null;

        public ICommand SelectHypridImageRecon
        {
            get { return _selectHypridImageRecon ?? (_selectHypridImageRecon = new AsyncCommand(SelReconType4)); }
        }

        private Task SelReconType4() => Task.Run(() =>
        {
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                ReconType = eReconType.HybridImage;
            });
        });

        //231025 sbkwon : ReconSpace
        private eReconSpace _reconSpace = eReconSpace.Plane;
        public eReconSpace ReconSpace
        {
            get { return _reconSpace; }
            set 
            { 
                if (_isUpdatingProperties) return;  // 순환 참조 방지
                
                // 자동 모드일 때는 사용자가 직접 실내/실외 모드를 선택하는 것을 방지
                // 단, 자동 모드 내부 알고리즘 업데이트 시에는 허용
                if (ReconSpaceAuto && !_isAutoAlgorithmUpdate && (value == eReconSpace.Pointcloud || value == eReconSpace.Plane))
                {
                    System.Diagnostics.Debug.WriteLine($"자동 모드에서는 실내/실외 모드를 직접 선택할 수 없습니다.");
                    return; // 선택 차단
                }
                
                _isUpdatingProperties = true;
                _reconSpace = value;

                // 실내/실외 모드가 사용자에 의해 선택되면 자동 모드를 체크 해제하고 수동 모드를 활성화
                // 자동 알고리즘에 의한 변경(_isAutoAlgorithmUpdate)일 때는 해제하지 않음
                if (!_isAutoAlgorithmUpdate && (value == eReconSpace.Pointcloud || value == eReconSpace.Plane))
                {
                    _reconSpaceAuto = false;  // 자동 모드 체크 완전 해제
                    _reconSpaceManual = true;  // 수동 모드 활성화
                    OnPropertyChanged(nameof(ReconSpaceAuto));
                    OnPropertyChanged(nameof(ReconSpaceManual));
                    
                    // UI 업데이트를 위한 속성 변경 알림
                    OnPropertyChanged(nameof(ShowIndoorOutdoorCheckboxes));
                    OnPropertyChanged(nameof(ShowIndoorCheckbox));
                    OnPropertyChanged(nameof(ShowOutdoorCheckbox));
                    OnPropertyChanged(nameof(ReconOptionEnable));
                    OnPropertyChanged(nameof(ShowReconOptions));
                }

                //240326 : 영상정합 옵션 enable 설정
                if (ReconSpaceManual && value == eReconSpace.Plane)
                {
                    ReconOptionEnable = true;

                    // 실외 모드로 전환된 경우 기본값을 type1으로 설정
                    if (_reconOption != eReconOption.type1)
                    {
                        _reconOption = eReconOption.type1;
                        OnPropertyChanged(nameof(ReconOption));
                    }

                    if (ReconOption == eReconOption.type3)
                        VisibitityRGB = Visibility.Hidden;
                    else
                        VisibitityRGB = Visibility.Visible;
                }
                else
                {
                    ReconOptionEnable = false;
                    VisibitityRGB = Visibility.Visible;
                }
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.ReconSpace = value;
                
                OnPropertyChanged(nameof(ReconSpace)); 
                _isUpdatingProperties = false;
            }
        }

        private bool _reconSpaceAuto = false;
        private bool _isUpdatingProperties = false;  // 순환 참조 방지 플래그
        private bool _isAutoAlgorithmUpdate = false; // 자동 알고리즘 업데이트 플래그
        
        public bool ReconSpaceAuto
        {
            get { return _reconSpaceAuto; }
            set 
            { 
                if (_isUpdatingProperties) return;  // 순환 참조 방지
                
                _isUpdatingProperties = true;
                _reconSpaceAuto = value; 
                OnPropertyChanged(nameof(ReconSpaceAuto));
                
                if (value)
                {
                    // 자동 모드가 선택되면 실내/실외 모드 체크 완전 해제
                    _reconSpaceManual = false;  // setter 호출하지 않고 직접 필드 변경
                    OnPropertyChanged(nameof(ReconSpaceManual));
                    
                    // 실내/실외 모드 체크 상태를 명시적으로 해제
                    // 이는 UI에서 실내/실외 체크박스가 체크되지 않도록 하기 위함
                    // _isUpdatingProperties 플래그를 true로 설정하여 ReconSpace setter의 제한을 우회
                    _isUpdatingProperties = true;
                    
                    // 현재 선택된 실내/실외 모드를 초기화하여 체크 상태 해제
                    _reconSpace = eReconSpace.Plane;  // 기본값으로 설정
                    OnPropertyChanged(nameof(ReconSpace));
                    
                    // 자동 모드에서는 실외 모드 1,2,3 비활성화 (실내 모드 또는 실외 모드 1만 가능)
                    // ReconOption을 type1으로 강제 설정하여 실외 모드 1만 사용 가능하도록 함 (즉시 적용 위해 setter 사용)
                    if (ReconOption != eReconOption.type1)
                    {
                        ReconOption = eReconOption.type1;
                    }

                    // 타입3에서 자동으로 전환 시 RGB 숨김 상태를 복구
                    VisibitityRGB = Visibility.Visible;
                    
                    // 자동 모드 알고리즘: SLAM된 포인트 클라우드 개수에 따라 실내/실외 모드 자동 선택
                    try
                    {
                        int nCount = LahgiApi.GetSlamedPointCloudCount();
                        
                        if (nCount > 10000)
                        {
                            // 포인트 클라우드가 10,000개 이상이면 실내 모드 (Pointcloud)
                            _reconSpace = eReconSpace.Pointcloud;
                            OnPropertyChanged(nameof(ReconSpace));
                        }
                        else
                        {
                            // 포인트 클라우드가 10,000개 미만이면 실외 모드 (Plane) - type1만 사용
                            _reconSpace = eReconSpace.Plane;
                            OnPropertyChanged(nameof(ReconSpace));
                        }
                    }
                    catch (Exception ex)
                    {
                        // LahgiApi 호출 실패 시 기본값으로 설정
                        System.Diagnostics.Debug.WriteLine($"자동 모드 알고리즘 실행 실패: {ex.Message}");
                        _reconSpace = eReconSpace.Plane;
                        OnPropertyChanged(nameof(ReconSpace));
                    }
                    
                    // UI 업데이트를 위한 속성 변경 알림
                    OnPropertyChanged(nameof(ShowIndoorOutdoorCheckboxes));
                    OnPropertyChanged(nameof(ShowIndoorCheckbox));
                    OnPropertyChanged(nameof(ShowOutdoorCheckbox));
                    OnPropertyChanged(nameof(ReconOptionEnable));
                    OnPropertyChanged(nameof(ShowReconOptions));
                    
                    // _isUpdatingProperties 플래그를 false로 복원
                    _isUpdatingProperties = false;
                }
                else
                {
                    // 수동 모드 활성화
                    _reconSpaceManual = true;  // setter 호출하지 않고 직접 필드 변경
                    OnPropertyChanged(nameof(ReconSpaceManual));
                    
                    // UI 업데이트를 위한 속성 변경 알림
                    OnPropertyChanged(nameof(ShowIndoorOutdoorCheckboxes));
                    OnPropertyChanged(nameof(ShowIndoorCheckbox));
                    OnPropertyChanged(nameof(ShowOutdoorCheckbox));
                    OnPropertyChanged(nameof(ReconOptionEnable));
                }
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.ReconSpaceAuto = value;
                _isUpdatingProperties = false;
            }
        }

        private bool _reconSpaceManual = false;
        public bool ReconSpaceManual
        {
            get => _reconSpaceManual;
            set 
            { 
                if (_isUpdatingProperties) return;  // 순환 참조 방지
                
                _isUpdatingProperties = true;
                _reconSpaceManual = value;
                
                // 수동 모드가 활성화되면 자동 모드를 비활성화
                if (value)
                {
                    _reconSpaceAuto = false;  // setter 호출하지 않고 직접 필드 변경
                    OnPropertyChanged(nameof(ReconSpaceAuto));
                    
                    // UI 업데이트를 위한 속성 변경 알림
                    OnPropertyChanged(nameof(ShowIndoorOutdoorCheckboxes));
                    OnPropertyChanged(nameof(ShowIndoorCheckbox));
                    OnPropertyChanged(nameof(ShowOutdoorCheckbox));
                    OnPropertyChanged(nameof(ReconOptionEnable));
                }
                
                //240326
                if (value && ReconSpace == eReconSpace.Plane)
                {
                    // 수동 모드에서 실외 모드 선택 시 실외 모드 1,2,3 활성화
                    _reconOptionEnable = true;
                    OnPropertyChanged(nameof(ReconOptionEnable));
                }
                else
                {
                    _reconOptionEnable = false;
                    OnPropertyChanged(nameof(ReconOptionEnable));
                }

                OnPropertyChanged(nameof(ReconSpaceManual));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.ReconSpaceManual = value;
                _isUpdatingProperties = false;
            }
        }

        private int timeInMiliSeconds = 2000;
        public int TimeInMiliSeconds
        {
            get
            {
                return timeInMiliSeconds;
            }
            set
            {
                timeInMiliSeconds = value;
                OnPropertyChanged(nameof(TimeInMiliSeconds));
            }
        }

        private double s2M = 2;
        public double S2M
        {
            get
            {
                return s2M;
            }
            set
            {
                s2M = value;
                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                OnPropertyChanged(nameof(S2M));
            }
        }

        private double m2D = 0.073-0.002;
        public double M2D
        {
            get
            {
                return m2D;
            }
            set
            {
                m2D = value;
                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                OnPropertyChanged(nameof(M2D));
            }
        }

        private double minValuePortion = 0.50;
        public double MinValuePortion
        {
            get { return minValuePortion; }
            set
            {
                minValuePortion = value;
                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                OnPropertyChanged(nameof(MinValuePortion));
            }
        }

        private int _minValuePortionint = 0;
        public int MinValuePortionint
        {
            get => _minValuePortionint;
            set
            {
                _minValuePortionint = value;
                MinValuePortion = Math.Round(value * 0.01, 2); ;
                OnPropertyChanged(nameof(MinValuePortionint));
            }
        }

        private double det_W = 0.312;
        public double Det_W
        {
            get { return det_W; }
            set { det_W = value; OnPropertyChanged(nameof(Det_W)); }
        }

        private double resImprov = 13;
        public double ResImprov
        {
            get
            {
                return resImprov;
            }
            set
            {
                resImprov = value;
                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);

                OnPropertyChanged(nameof(ResImprov));
            }
        }

        private int imgSize = 800;
        public int ImgSize
        {
            get
            {
                return imgSize;
            }
            set
            {
                imgSize = value;
                OnPropertyChanged(nameof(ImgSize));
            }
        }

        private BitmapImage codedImgRGB;
        public BitmapImage CodedImgRGB
        {
            get { return codedImgRGB; }
            set
            {
                codedImgRGB = value;
                OnPropertyChanged(nameof(CodedImgRGB));
            }
        }

        private BitmapImage comptonImgRGB;
        public BitmapImage ComptonImgRGB
        {
            get { return comptonImgRGB; }
            set { comptonImgRGB = value; OnPropertyChanged(nameof(ComptonImgRGB)); }
        }

        private BitmapImage hybridImgRGB;
        public BitmapImage HybridImgRGB
        {
            get { return hybridImgRGB; }
            set { hybridImgRGB = value; OnPropertyChanged(nameof(HybridImgRGB)); }
        }

        private BitmapImage realtimeRGB;
        public BitmapImage RealtimeRGB
        {
            get { return realtimeRGB; }
            set
            {
                realtimeRGB = value;
                OnPropertyChanged(nameof(RealtimeRGB));
            }
        }

        //250107 정밀검사 2D 화면표시 여부
        private bool _MLEM2DVisibility = false;
        public bool MLEM2DVisibility
        {
            get { return _MLEM2DVisibility; }
            set { _MLEM2DVisibility = value; OnPropertyChanged(nameof(MLEM2DVisibility)); }
        }

        //250107 정밀검사 2D 초기 화면(RGB)
        private bool _MLEM2DRGB = false;
        public bool MLEM2DRGB
        {
            get { return _MLEM2DRGB; }
            set { _MLEM2DRGB = value; OnPropertyChanged(nameof(MLEM2DRGB)); }
        }

        //MLEM 2D 화면 표시
        private void MLEM2DDisplay()
        {
            if (LahgiApi.StatusCalMLEM && MLEM2DVisibility)   //MLEM 연산 정상적으로 완료 된 경우
            {
                string imagepath = LahgiApi.MLEMDataPath + "_MLEM2D_" + LahgiApi.MLEMSelectNo.ToString() + ".png";
                //string imagepath = System.IO.Path.Combine( LahgiApi.MLEMDataPath, "_MLEM2D", "_", LahgiApi.MLEMSelectNo.ToString(), ".png");
                //FilePath + "_MLEM2D" + "_" + std::to_string(nNo) + ".png";MLEMSelectNo
                //string imagepath = "_MLEM2DSelect.png";
                Image tempImage = Image.FromFile(imagepath);
                BitmapImage? temp1;

                //LogManager.GetLogger(typeof(ReconstructionImageViewModel)).Info(imagepath);

                using (MemoryStream ms = new MemoryStream())
                {
                    tempImage.Save(ms, System.Drawing.Imaging.ImageFormat.Jpeg);

                    temp1 = new BitmapImage();
                    temp1.BeginInit();
                    ms.Seek(0, SeekOrigin.Begin);
                    temp1.StreamSource = ms;
                    temp1.CacheOption = BitmapCacheOption.OnLoad;
                    temp1.EndInit();
                    temp1.Freeze();
                    //img = bitMapimg;
                }
                RealtimeRGB = temp1;
            }
        }

        private void MLEMLoading()
        {
            if (LahgiApi.MLEMDataLoad && MLEM2DRGB)   //data load 가 완료 된 경우
            {
                string imagepath = LahgiApi.MLEMDataPath + "_rgb.png";
                Image tempImage = Image.FromFile(imagepath);
                BitmapImage? temp1;

                //LogManager.GetLogger(typeof(ReconstructionImageViewModel)).Info(imagepath);

                using (MemoryStream ms = new MemoryStream())
                {
                    tempImage.Save(ms, System.Drawing.Imaging.ImageFormat.Jpeg);

                    temp1 = new BitmapImage();
                    temp1.BeginInit();
                    ms.Seek(0, SeekOrigin.Begin);
                    temp1.StreamSource = ms;
                    temp1.CacheOption = BitmapCacheOption.OnLoad;
                    temp1.EndInit();
                    temp1.Freeze();
                    //img = bitMapimg;
                }
                RealtimeRGB = temp1;
            }
        }

        public void RGBDisplay()
        {
            if (MLEM2DVisibility)
                MLEM2DVisibility = false;
            if (MLEM2DRGB)
                MLEM2DRGB = false;

            BitmapImage? temp;

            //240326
            if ((ReconOption == eReconOption.type2 || ReconOption == eReconOption.type3) && ReconSpace == eReconSpace.Plane)
            {
                // RGBType 값이 반대로 전달되는 문제 수정
                int colorType = RGBType == eRGBType.Color ? 0 : 1;  // Color=0, Gray=1
                temp = LahgiApi.GetRgbImage(colorType, true);
            }
            else
            {
                // RGBType 값이 반대로 전달되는 문제 수정
                int colorType = RGBType == eRGBType.Color ? 0 : 1;  // Color=0, Gray=1
                temp = LahgiApi.GetRgbImage(colorType);
            }

            if (temp != null)
            {
                RealtimeRGB = temp;

                if (RGBCamera == false)
                    RGBCamera = true;

                Thread.Sleep(100);
            }
            else
            {
                if (RGBCamera == true)
                    RGBCamera = false;

                Image tempImage = Image.FromFile("Resource/not_connected.jpg");
                
                // not_connected 이미지를 848x480 비율로 리사이즈
                Bitmap resizedImage = new Bitmap(848, 480);
                using (Graphics g = Graphics.FromImage(resizedImage))
                {
                    g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                    g.DrawImage(tempImage, 0, 0, 848, 480);
                }

                using (MemoryStream ms = new MemoryStream())
                {
                    resizedImage.Save(ms, System.Drawing.Imaging.ImageFormat.Jpeg);

                    temp = new BitmapImage();
                    temp.BeginInit();
                    ms.Seek(0, SeekOrigin.Begin);
                    temp.StreamSource = ms;
                    temp.CacheOption = BitmapCacheOption.OnLoad;
                    temp.EndInit();
                    temp.Freeze();
                    //img = bitMapimg;
                }
                
                // 리소스 정리
                resizedImage.Dispose();

                // RGBType 값이 반대로 처리되는 문제 수정
                if (RGBType == eRGBType.Color)
                {
                    // 컬러 모드일 때는 흑백 변환
                    Bitmap newBitmap = new Bitmap(848, 480);
                    Graphics g = Graphics.FromImage(newBitmap);
                    g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                    
                    ColorMatrix colorMatrix = new ColorMatrix(
                                                               new float[][]
                                                              {
                                             new float[] {.3f, .3f, .3f, 0, 0},
                                             new float[] {.59f, .59f, .59f, 0, 0},
                                             new float[] {.11f, .11f, .11f, 0, 0},
                                             new float[] {0, 0, 0, 1, 0},
                                             new float[] {0, 0, 0, 0, 1}
                                                              });
                    ImageAttributes attributes = new ImageAttributes();
                    attributes.SetColorMatrix(colorMatrix);
                    g.DrawImage(tempImage, new Rectangle(0, 0, 848, 480),
                       0, 0, tempImage.Width, tempImage.Height, GraphicsUnit.Pixel, attributes);
                    g.Dispose();

                    using (MemoryStream ms = new MemoryStream())
                    {
                        newBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Jpeg);

                        temp = new BitmapImage();
                        temp.BeginInit();
                        ms.Seek(0, SeekOrigin.Begin);
                        temp.StreamSource = ms;
                        temp.CacheOption = BitmapCacheOption.OnLoad;
                        temp.EndInit();
                        temp.Freeze();
                    }

                    RealtimeRGB = temp;
                    
                    // 리소스 정리
                    newBitmap.Dispose();
                }
                else
                {
                    // 흑백 모드일 때는 원본 컬러 이미지 사용
                    RealtimeRGB = temp;
                }

                Thread.Sleep(5000);
            }
        }

        private Task LoopTask;
        private bool RunLoop = true;
        private void Loop()
        {
            while (RunLoop)
            {
                //BitmapImage? temp = LahgiApi.GetRgbImage();

                //250107 정밀검사 2D 화면표시
                if (MLEM2DVisibility)
                {
                    MLEM2DDisplay();
                     
                    Thread.Sleep(2000);
                    continue;
                }
                else if(MLEM2DRGB)
                {
                    MLEMLoading();
                       
                   Thread.Sleep(2000);
                    continue;
                }

                RGBDisplay();
            }
        }

        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= StatusUpdate;

            if (LoopTask != null)
            {
                RunLoop = false;
                LoopTask.Wait();
            }

            LogManager.GetLogger(typeof(ReconstructionImageViewModel)).Info("Unhandle");
        }

        private eRGBType _rgbType = eRGBType.Gray;
        public eRGBType RGBType
        {
            get { return _rgbType; }
            set 
            { 
                _rgbType = value; 
                OnPropertyChanged(nameof(RGBType));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.RGBImageType = value;
                
                // RGB 영상 업데이트 트리거
                OnPropertyChanged(nameof(RealtimeRGB));
            }
        }

        private AsyncCommand? _selectRGBColor = null;

        public ICommand SelectRGBColor
        {
            get { return _selectRGBColor ?? (_selectRGBColor = new AsyncCommand(SelRGBColor)); }
        }

        private Task SelRGBColor() => Task.Run(() =>
        {
            var logger = log4net.LogManager.GetLogger(typeof(ReconstructionImageViewModel));
            logger.Info($"SelectRGBColor Command 실행됨. 현재 값: {RGBType}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                RGBType = eRGBType.Color;
                logger.Info($"SelectRGBColor Command 실행 후 값: {RGBType}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.RGBImageType = eRGBType.Color;
            });
        });

        private AsyncCommand? _selectRGBGray = null;

        public ICommand SelectRGBGray
        {
            get { return _selectRGBGray ?? (_selectRGBGray = new AsyncCommand(SelRGBGray)); }
        }

        private Task SelRGBGray() => Task.Run(() =>
        {
            var logger = log4net.LogManager.GetLogger(typeof(ReconstructionImageViewModel));
            logger.Info($"SelectRGBGray Command 실행됨. 현재 값: {RGBType}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                RGBType = eRGBType.Gray;
                logger.Info($"SelectRGBGray Command 실행 후 값: {RGBType}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.RGBImageType = eRGBType.Gray;
            });
        });

        // SelectLabeling Command 추가
        private AsyncCommand? _selectLabeling = null;
        public ICommand SelectLabeling
        {
            get { return _selectLabeling ?? (_selectLabeling = new AsyncCommand(SelLabeling)); }
        }

        private Task SelLabeling()
        {
            var logger = log4net.LogManager.GetLogger(typeof(ReconstructionImageViewModel));
            logger.Info($"SelectLabeling Command 실행됨. 현재 값: {LabelingCheck}");
            
            // UI 스레드에서 직접 속성 변경
            bool oldValue = LabelingCheck;
            LabelingCheck = !oldValue;
            logger.Info($"SelectLabeling Command 실행 후 값: {LabelingCheck}");
            
            return Task.CompletedTask;
        }

        // SelectOpacityIntValue Command 추가
        private AsyncCommand? _selectOpacityIntValue = null;
        public ICommand SelectOpacityIntValue
        {
            get { return _selectOpacityIntValue ?? (_selectOpacityIntValue = new AsyncCommand(SelOpacityIntValue)); }
        }

        private Task SelOpacityIntValue() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectOpacityIntValue Command 실행됨. 현재 값: {OpacityIntValue}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                // 투명도 값이 변경되면 자동으로 App.GlobalConfig에 저장됨 (setter에서 처리)
                System.Diagnostics.Debug.WriteLine($"SelectOpacityIntValue Command 실행 후 값: {OpacityIntValue}");
            });
        });

        // SelectVisualizationRange Command 추가
        private AsyncCommand? _selectVisualizationRange = null;
        public ICommand SelectVisualizationRange
        {
            get { return _selectVisualizationRange ?? (_selectVisualizationRange = new AsyncCommand(SelVisualizationRange)); }
        }

        private Task SelVisualizationRange() => Task.Run(() =>
        {
            System.Diagnostics.Debug.WriteLine($"SelectVisualizationRange Command 실행됨. 현재 값: {VisualizationRange}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                // 가시화 범위 값이 변경되면 자동으로 App.GlobalConfig에 저장됨 (setter에서 처리)
                System.Diagnostics.Debug.WriteLine($"SelectVisualizationRange Command 실행 후 값: {VisualizationRange}");
            });
        });

        // 투명도 증가/감소 Command 추가
        private AsyncCommand? _increaseOpacity = null;
        public ICommand IncreaseOpacity
        {
            get { return _increaseOpacity ?? (_increaseOpacity = new AsyncCommand(IncOpacity)); }
        }

        private Task IncOpacity() => Task.Run(() =>
        {
            Application.Current.Dispatcher.Invoke(() =>
            {
                if (OpacityIntValue < 100)
                {
                    OpacityIntValue += 5;  // 5%씩 증가
                }
            });
        });

        private AsyncCommand? _decreaseOpacity = null;
        public ICommand DecreaseOpacity
        {
            get { return _decreaseOpacity ?? (_decreaseOpacity = new AsyncCommand(DecOpacity)); }
        }

        private Task DecOpacity() => Task.Run(() =>
        {
            Application.Current.Dispatcher.Invoke(() =>
            {
                if (OpacityIntValue > 5)
                {
                    OpacityIntValue -= 5;  // 5%씩 감소
                }
            });
        });

        // 가시화 범위 증가/감소 Command 추가
        private AsyncCommand? _increaseVisualizationRange = null;
        public ICommand IncreaseVisualizationRange
        {
            get { return _increaseVisualizationRange ?? (_increaseVisualizationRange = new AsyncCommand(IncVisualizationRange)); }
        }

        private Task IncVisualizationRange() => Task.Run(() =>
        {
            Application.Current.Dispatcher.Invoke(() =>
            {
                if (MinValuePortionint < 100)
                {
                    MinValuePortionint += 5;  // 5%씩 증가
                }
            });
        });

        private AsyncCommand? _decreaseVisualizationRange = null;
        public ICommand DecreaseVisualizationRange
        {
            get { return _decreaseVisualizationRange ?? (_decreaseVisualizationRange = new AsyncCommand(DecVisualizationRange)); }
        }

        private Task DecVisualizationRange() => Task.Run(() =>
        {
            Application.Current.Dispatcher.Invoke(() =>
            {
                if (MinValuePortionint > 5)
                {
                    MinValuePortionint -= 5;  // 5%씩 감소
                }
            });
        });

        // SelectReconSpaceAuto Command 추가
        private AsyncCommand? _selectReconSpaceAuto = null;
        public ICommand SelectReconSpaceAuto
        {
            get { return _selectReconSpaceAuto ?? (_selectReconSpaceAuto = new AsyncCommand(SelReconSpaceAuto)); }
        }

        private Task SelReconSpaceAuto() => Task.Run(() =>
        {
            var logger = log4net.LogManager.GetLogger(typeof(ReconstructionImageViewModel));
            logger.Info($"SelectReconSpaceAuto Command 실행됨. 현재 값: {ReconSpaceAuto}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                bool oldValue = ReconSpaceAuto;
                ReconSpaceAuto = !oldValue;
                logger.Info($"SelectReconSpaceAuto Command 실행 후 값: {ReconSpaceAuto}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.ReconSpaceAuto = ReconSpaceAuto;
                logger.Info($"App.GlobalConfig.ReconSpaceAuto 저장 완료: {App.GlobalConfig.ReconSpaceAuto}");
            });
        });

        // SelectReconSpacePointcloud Command 추가
        private AsyncCommand? _selectReconSpacePointcloud = null;
        public ICommand SelectReconSpacePointcloud
        {
            get { return _selectReconSpacePointcloud ?? (_selectReconSpacePointcloud = new AsyncCommand(SelReconSpacePointcloud)); }
        }

        private Task SelReconSpacePointcloud() => Task.Run(() =>
        {
            var logger = log4net.LogManager.GetLogger(typeof(ReconstructionImageViewModel));
            logger.Info($"SelectReconSpacePointcloud Command 실행됨. 현재 값: {ReconSpace}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                ReconSpace = eReconSpace.Pointcloud;
                logger.Info($"SelectReconSpacePointcloud Command 실행 후 값: {ReconSpace}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.ReconSpace = eReconSpace.Pointcloud;
                logger.Info($"App.GlobalConfig.ReconSpace 저장 완료: {App.GlobalConfig.ReconSpace}");
            });
        });

        // SelectReconSpacePlane Command 추가
        private AsyncCommand? _selectReconSpacePlane = null;
        public ICommand SelectReconSpacePlane
        {
            get { return _selectReconSpacePlane ?? (_selectReconSpacePlane = new AsyncCommand(SelReconSpacePlane)); }
        }

        private Task SelReconSpacePlane() => Task.Run(() =>
        {
            var logger = log4net.LogManager.GetLogger(typeof(ReconstructionImageViewModel));
            logger.Info($"SelectReconSpacePlane Command 실행됨. 현재 값: {ReconSpace}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                ReconSpace = eReconSpace.Plane;
                logger.Info($"SelectReconSpacePlane Command 실행 후 값: {ReconSpace}");
                
                // App.GlobalConfig에 저장
                App.GlobalConfig.ReconSpace = eReconSpace.Plane;
                logger.Info($"App.GlobalConfig.ReconSpace 저장 완료: {App.GlobalConfig.ReconSpace}");
            });
        });



        private int _reconMeasurTime = 20;
        public int ReconMeasurTime
        {
            get { return _reconMeasurTime; }
            set { _reconMeasurTime = value; OnPropertyChanged(nameof(ReconMeasurTime)); }
        }

        private int _reconMeasurCount = 300;
        public int ReconMeasurCount
        {
            get { return _reconMeasurCount; }
            set { _reconMeasurCount = value; OnPropertyChanged(nameof(ReconMeasurCount)); }
        }

        //240122
        private int _reconMaxValue = 70;
        public int ReconMaxValue
        {
            get { return _reconMaxValue; }
            set { _reconMaxValue = value; OnPropertyChanged(nameof(ReconMaxValue)); }
        }

        private double _opacityValue = 0.7;
        public double OpacityValue
        {
            get { return _opacityValue; }
            set { _opacityValue = value; OnPropertyChanged(nameof(OpacityValue)); }
        }

        private int _opacityIntValue = 70;
        public int OpacityIntValue
        {
            get { return _opacityIntValue; }
            set 
            { 
                _opacityIntValue = value; 
                OpacityValue = Math.Round(value * 0.01, 2); 
                OnPropertyChanged(nameof(OpacityIntValue));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.OpacityValue = value;
            }
        }

        //231208 sbkwon : 레이저 스캐너 상태(realsense)
        private bool _rgbCamera = false;
        public bool RGBCamera
        {
            get => _rgbCamera;
            set
            {
                _rgbCamera = value;
                if (value)
                {
                    RGBCameraStatus = "정상";
                    RGBCameraTextColor = Brushes.Black;
                }
                else
                {
                    RGBCameraStatus = "점검 필요";
                    RGBCameraTextColor = Brushes.Red;
                }
            }
        }

        // D455 카메라 연결 상태 체크 메서드
        public void CheckD455CameraConnection()
        {
            try
            {
                // RTAB-Map이 초기화되었는지 확인 (중복 초기화 시도 없음)
                if (LahgiApi.IsRtabmapInitiate)
                {
                    // GetRgbImage를 호출하여 실제로 이미지를 가져올 수 있는지 테스트
                    var temp = LahgiApi.GetRgbImage();
                    if (temp != null)
                    {
                        RGBCamera = true; // 카메라가 연결되어 있고 이미지 스트림이 정상
                        System.Diagnostics.Debug.WriteLine("D455 카메라 연결 확인됨 - 이미지 스트림 정상");
                    }
                    else
                    {
                        // GetRgbImage가 null을 반환하는 경우, RTAB-Map이 초기화되었지만 이미지 스트림이 없는 상태
                        System.Diagnostics.Debug.WriteLine("RTAB-Map이 초기화되었지만 이미지를 가져올 수 없음. 이미지 스트림 대기 중...");
                        RGBCamera = false; // 이미지 스트림이 없음
                    }
                }
                else
                {
                    // RTAB-Map이 초기화되지 않은 경우, LahgiApi에서 자동 초기화 완료 대기
                    System.Diagnostics.Debug.WriteLine("RTAB-Map이 초기화되지 않음. LahgiApi에서 자동 초기화 완료 대기 중...");
                    RGBCamera = false; // 초기화 대기 중
                }
            }
            catch (Exception ex)
            {
                // 예외 발생 시 카메라 연결 실패로 간주
                RGBCamera = false;
                System.Diagnostics.Debug.WriteLine($"D455 카메라 연결 체크 실패: {ex.Message}");
            }
        }
        private string _rgbCameraStatus = "";
        public string RGBCameraStatus
        {
            get => _rgbCameraStatus;
            set
            {
                _rgbCameraStatus = value;
                OnPropertyChanged(nameof(RGBCameraStatus));
            }
        }

        private Brush _rgbCameraTexttColor = Brushes.Black;
        public Brush RGBCameraTextColor
        {
            get => _rgbCameraTexttColor;
            set
            {
                _rgbCameraTexttColor = value;
                OnPropertyChanged(nameof(RGBCameraTextColor));
            }
        }

        //240228 Scatter - 2채널 모드: 채널 4(Scatter)만 사용
        private System.Windows.Media.Brush[] _ScatterCH1 = new System.Windows.Media.Brush[9];
        public System.Windows.Media.Brush[] ScatterCH1
        {
            get { return _ScatterCH1; }
            set { _ScatterCH1 = value; OnPropertyChanged(); }
        }

        //240228 Absorber - 2채널 모드: 채널 12(Absorber)만 사용
        private System.Windows.Media.Brush[] _AbsorberCH1 = new System.Windows.Media.Brush[9];
        public System.Windows.Media.Brush[] AbsorberCH1
        {
            get { return _AbsorberCH1; }
            set { _AbsorberCH1 = value; OnPropertyChanged(); }
        }

        public void ClearFaultColor()
        {
            System.Windows.Media.Brush[] result = new System.Windows.Media.Brush[9];  //검사 결과 저장
            for (int i = 0; i < 9; i++)
            {
                result[i] = System.Windows.Media.Brushes.White;
            }
            // 2채널 모드: 채널 4(Scatter), 채널 12(Absorber)만 초기화
            ScatterCH1 = result;   // 채널 4
            AbsorberCH1 = result;  // 채널 12
        }

        //240326 : 영상 정합 방법 체크박스 설정 여부 (true : enable, false : diable)
        private bool _reconOptionEnable = false;
        public bool ReconOptionEnable
        {
            get 
            {
                // 자동 모드일 때는 실외 모드 1,2,3 비활성화
                if (ReconSpaceAuto)
                {
                    return false;
                }
                return _reconOptionEnable; 
            }
            set { _reconOptionEnable = value; OnPropertyChanged(nameof(ReconOptionEnable)); }
        }

        // 실내/실외 모드 체크박스 표시 여부 (자동 모드일 때는 숨김)
        public bool ShowIndoorOutdoorCheckboxes
        {
            get { return !ReconSpaceAuto; }
        }

        // 실내 모드 체크박스 표시 여부 (자동 모드일 때는 숨김)
        public bool ShowIndoorCheckbox
        {
            get { return !ReconSpaceAuto; }
        }

        // 실외 모드 체크박스 표시 여부 (자동 모드일 때는 숨김)
        public bool ShowOutdoorCheckbox
        {
            get { return !ReconSpaceAuto; }
        }

        // 실외 모드 1,2,3 옵션 표시 여부 (자동 모드일 때는 숨김)
        public bool ShowReconOptions
        {
            get { return !ReconSpaceAuto; }
        }

        //240326 : 영상 정합 방법
        private eReconOption _reconOption = eReconOption.type1;
        public eReconOption ReconOption
        {
            get { return _reconOption; }
            set 
            { 
                // 자동 모드일 때는 실외 모드 1,2,3 선택 제한 (실내 모드 또는 실외 모드 1만 가능)
                if (ReconSpaceAuto && value != eReconOption.type1)
                {
                    System.Diagnostics.Debug.WriteLine($"자동 모드에서는 실외 모드 1만 선택 가능합니다. 선택된 값: {value}");
                    return; // 선택 차단
                }
                
                _reconOption = value; 
                OnPropertyChanged(nameof(ReconOption));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.ReconOption = value;
            }
        }

        //240326 : check box command 1
        private AsyncCommand? _selectReconOption1 = null;
        public ICommand SelectReconOption1
        {
            get { return _selectReconOption1 ?? (_selectReconOption1 = new AsyncCommand(SelReconOption1)); }
        }

        private Task SelReconOption1() => Task.Run(() =>
        {
            var logger = log4net.LogManager.GetLogger(typeof(ReconstructionImageViewModel));
            logger.Info($"SelReconOption1 Command 실행됨. 현재 값: {ReconOption}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                ReconOption = eReconOption.type1;
                VisibitityRGB = Visibility.Visible;
                logger.Info($"SelReconOption1 Command 실행 후 값: {ReconOption}");
            });
        });

        //240326 : check box command 2
        private AsyncCommand? _selectReconOption2 = null;
        public ICommand SelectReconOption2
        {
            get { return _selectReconOption2 ?? (_selectReconOption2 = new AsyncCommand(SelReconOption2)); }
        }

        private Task SelReconOption2() => Task.Run(() =>
        {
            var logger = log4net.LogManager.GetLogger(typeof(ReconstructionImageViewModel));
            logger.Info($"SelReconOption2 Command 실행됨. 현재 값: {ReconOption}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                ReconOption = eReconOption.type2;
                VisibitityRGB = Visibility.Visible;
                logger.Info($"SelReconOption2 Command 실행 후 값: {ReconOption}");
            });
        });

        //240326 : check box command 3
        private AsyncCommand? _selectReconOption3 = null;
        public ICommand SelectReconOption3
        {
            get { return _selectReconOption3 ?? (_selectReconOption3 = new AsyncCommand(SelReconOption3)); }
        }

        private Task SelReconOption3() => Task.Run(() =>
        {
            var logger = log4net.LogManager.GetLogger(typeof(ReconstructionImageViewModel));
            logger.Info($"SelReconOption3 Command 실행됨. 현재 값: {ReconOption}");
            
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                ReconOption = eReconOption.type3;
                VisibitityRGB = Visibility.Hidden;
                logger.Info($"SelReconOption3 Command 실행 후 값: {ReconOption}");
            });
        });

        //240326 : RGB 화면표시 여부
        private Visibility _visibitityRGB = Visibility.Visible;
        public Visibility VisibitityRGB
        {
            get => _visibitityRGB;
            set { _visibitityRGB = value; VisibitityGrid = value; OnPropertyChanged(nameof(VisibitityRGB)); }
        }

        //240327 grid
        // VisibitityGrid
        private Visibility _visibitityGrid = Visibility.Visible;
        public Visibility VisibitityGrid
        {
            get => _visibitityGrid;
            set 
            {
                if (ReconSpaceManual && ReconSpace == eReconSpace.Plane && (ReconOption == eReconOption.type2 || ReconOption == eReconOption.type3))
                    _visibitityGrid = Visibility.Visible;
                else
                    _visibitityGrid = Visibility.Hidden;

                VisibitityGridBG = value;

                OnPropertyChanged(nameof(VisibitityGrid)); 
            }
        }

        //240422 grid Background
        private Visibility _visibitityGridBG = Visibility.Visible;
        public Visibility VisibitityGridBG
        {
            get => _visibitityGridBG;
            set
            {
                if (ReconSpaceManual && ReconSpace == eReconSpace.Plane && ReconOption == eReconOption.type3)
                    _visibitityGridBG = Visibility.Visible;
                else
                    _visibitityGridBG = Visibility.Hidden;
                OnPropertyChanged(nameof(VisibitityGridBG));
            }
        }

        //240327 Grid
        private BitmapImage _fovGrid;
        public BitmapImage FOVGrid
        {
            get { return _fovGrid; }
            set
            {
                _fovGrid = value;
                OnPropertyChanged(nameof(FOVGrid));
            }
        }

        //240422 Grid Background
        private BitmapImage _fovGridBG;
        public BitmapImage FOVGridBG
        {
            get { return _fovGridBG; }
            set
            {
                _fovGridBG = value;
                OnPropertyChanged(nameof(FOVGridBG));
            }
        }

        //241021 : 핵종 라벨링 유무
        private bool _labelingCheck = false;
        public bool LabelingCheck
        {
            get => _labelingCheck;
            set
            {
                if (_labelingCheck == value)
                    return;
                    
                _labelingCheck = value;
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.UseLabelingCheck = value;
                
                // OnPropertyChanged를 마지막에 호출
                OnPropertyChanged(nameof(LabelingCheck));
            }
        }

        // 가시화 범위 설정
        private double _visualizationRange = 100.0;
        public double VisualizationRange
        {
            get => _visualizationRange;
            set
            {
                _visualizationRange = value;
                OnPropertyChanged(nameof(VisualizationRange));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.VisualizationRange = value;
            }
        }

        //240327 Grid
        private void GridCreat()
        {
            Bitmap tempBitmap;

            // 비트맵 개체 초기화
            const int width = 800;
            const int height = 400;
            tempBitmap = new Bitmap(width, height, System.Drawing.Imaging.PixelFormat.Format24bppRgb);

            // 새 그래픽 만들기
            Graphics graphics = Graphics.FromImage(tempBitmap);

            // 펜 초기화
            const int size = 2;
            System.Drawing.Pen pen = new System.Drawing.Pen(System.Drawing.Color.White, size);    //line 색상

            //배경
            Rectangle rectangle = new Rectangle(0, 0, width, height);
            graphics.FillRectangle(System.Drawing.Brushes.Black, rectangle);
            //test
            //graphics.DrawRectangle(pen, new Rectangle(0, 0, width, height));

            //line 30도 간격
            int nlinelen = 15;
            //width line : 11ea
            for (int i = 1; i <= 11; i++)
            {
                int nX = (int)Math.Floor(i * (width / 360.0) * 30.0);
                graphics.DrawLine(pen, nX, 0, nX, nlinelen);
                graphics.DrawLine(pen, nX, height - nlinelen, nX, height);
            }

            //height line : 5ea
            for (int i = 1; i <= 5; i++)
            {
                int nY = (int)Math.Floor(i * (height / 180.0) * 30.0);
                graphics.DrawLine(pen, 0, nY, nlinelen, nY);
                graphics.DrawLine(pen, width - nlinelen, nY, width, nY);
            }

            graphics.Dispose();

            //tempBitmap.Save(@"G:\temp.png");

            //배경 투명하게
            System.Drawing.Color targetColor = System.Drawing.Color.Black;
            tempBitmap.MakeTransparent(targetColor);

            BitmapImage? img = null;
            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);

                img = new BitmapImage();
                img.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                img.StreamSource = ms;
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.EndInit();
                img.Freeze();
            }

            FOVGrid = img;

            //240422
            Bitmap tempbgBitmap;
            tempbgBitmap = new Bitmap(width, height, System.Drawing.Imaging.PixelFormat.Format24bppRgb);

            Graphics g = Graphics.FromImage(tempbgBitmap);
            g.Clear(System.Drawing.Color.Blue);

            g.DrawImage(tempbgBitmap, 0, 0, width, height);

            g.Dispose();

            BitmapImage? imgBG = null;
            using (MemoryStream ms = new MemoryStream())
            {
                tempbgBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);

                imgBG = new BitmapImage();
                imgBG.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgBG.StreamSource = ms;
                imgBG.CacheOption = BitmapCacheOption.OnLoad;
                imgBG.EndInit();
                imgBG.Freeze();
            }

            FOVGridBG = imgBG;
        }
    }
}
