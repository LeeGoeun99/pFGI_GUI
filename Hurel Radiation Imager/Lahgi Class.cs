using System.Configuration;
using System.Diagnostics;
using System.Drawing.Imaging;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using HUREL.Compton.RadioisotopeAnalysis;
using log4net;
using Newtonsoft.Json.Linq;
using static System.Windows.Forms.AxHost;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;

namespace HUREL.Compton
{
    public record AddListModeDataEchk(double MinE, double MaxE, IsotopeElement element = IsotopeElement.None);  //240123
    public record SelectEchk(IsotopeElement element);   //240123
    public enum eLahgiApiEnvetArgsState
    {
        Loading,
        SlamPoints,
        SlamRadImage,
        Spectrum,
        Massage,
        Status,
        FPGAUpdate,
        MLEM,   //2404 : MLEM
        Reconstruction,  // 방사선 영상 reconstruction 상태
    }
    public enum eEcalState
    {
        Fail,
        Success,
        Unknown
    }
    public class LahgiApiEnvetArgs : EventArgs
    {
        public eLahgiApiEnvetArgsState State { get; private set; }
        public LahgiApiEnvetArgs(eLahgiApiEnvetArgsState state)
        {
            State = state;
        }
    }
    public static class LahgiApi
    {
        private static ILog log = LogManager.GetLogger(typeof(LahgiApi));

        private static CRUXELLLACC fpga;
        private static LahgiWrapper lahgiWrapper;
        private static RtabmapWrapper rtabmapWrapper;
        public static CRUXELLLACC.VariableInfo fpgaVariables;

        public static EventHandler? StatusUpdate;

        public static void StatusUpdateInvoke(object? obj, eLahgiApiEnvetArgsState state)
        {
            Task.Run(() => { StatusUpdate?.Invoke(obj, new LahgiApiEnvetArgs(state)); });

        }

        public static bool TimerBoolSlamPoints { get; set; } = false;
        public static bool TimerBoolSlamRadImage { get; set; } = false;
        public static bool TimerBoolSpectrum { get; set; } = false;
        private static void UpdateTimerInvoker(object? obj, EventArgs args)
        {
            if (TimerBoolSlamPoints)
            {
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.SlamPoints);
            }
            if (TimerBoolSlamRadImage)
            {
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.SlamRadImage);
            }
            if (TimerBoolSpectrum)
            {
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
            }
        }

        private static string statusMsg = "";
        public static string StatusMsg
        {
            get
            {
                return statusMsg;
            }

            set
            {
                log = LogManager.GetLogger("LahgiApi");
                log.Info(value);
                statusMsg = value;
                Trace.WriteLine(value);
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Massage);
            }
        }
        public static bool IsSavingBinary
        {
            get { return fpga.IsSavingBinaryData; }
            set { fpga.IsSavingBinaryData = value; }
        }
        public static bool IsLahgiInitiate { get; private set; }
        public static bool IsRtabmapInitiate { get; private set; }

        public static bool IsInitiate
        {
            get
            {
                return IsLahgiInitiate && IsRtabmapInitiate;
            }
        }


        /// <summary>
        /// Start Stop Counting, Get Spectrum, Get 3D image, Get 2D image, 
        /// Start and stop imaging
        /// </summary>
        static LahgiApi()
        {
            fpga = new CRUXELLLACC();
            fpgaVariables = fpga.Variables;
            fpga.USBChangeHandler += UpdateDeviceList;
            IsFpgaAvailable = false;
            lahgiWrapper = new LahgiWrapper();
            rtabmapWrapper = new RtabmapWrapper();
            StatusMsg = "Wrappers loaded";

            IsSavingBinary = false;
            UpdateDeviceList(null, EventArgs.Empty);

            System.Timers.Timer timer = new System.Timers.Timer();
            timer.Interval = 500;
            timer.Elapsed += UpdateTimerInvoker;
            timer.Start();

            InitialLizeConfigFile();
        }

        private static void InitialLizeConfigFile()
        {
            var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
            var appSetting = configFile.AppSettings.Settings;
            if (appSetting["Test"] == null)
            {
                appSetting.Add("Test", "0");
            }
            if (appSetting[nameof(ref_x)] == null)
            {
                appSetting.Add(nameof(ref_x), "662");
            }
            if (appSetting[nameof(ref_fwhm)] == null)
            {
                appSetting.Add(nameof(ref_fwhm), "50");
            }
            if (appSetting[nameof(ref_at_0)] == null)
            {
                appSetting.Add(nameof(ref_at_0), "10");
            }
            if (appSetting[nameof(min_snr)] == null)
            {
                appSetting.Add(nameof(min_snr), "5");
            }
            if (appSetting[nameof(LastBootUp)] == null)
            {
                appSetting.Add(nameof(LastBootUp), DateTime.Now.Ticks.ToString());
            }
            for (int i = 0; i < 8; i++)
            {
                appSetting.Add(nameof(eEcalStates) + i.ToString(), eEcalState.Unknown.ToString());
            }

            configFile.Save(ConfigurationSaveMode.Modified);
            ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
        }

        private static float ref_x = Convert.ToSingle(ConfigurationManager.AppSettings.Get(nameof(ref_x)));
        public static float Ref_x
        {
            get { return ref_x; }
            set
            {
                ref_x = value;

                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(ref_x)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);

            }
        }

        private static float ref_fwhm = Convert.ToSingle(ConfigurationManager.AppSettings.Get(nameof(ref_fwhm)));
        public static float Ref_fwhm
        {
            get { return ref_fwhm; }
            set
            {
                ref_fwhm = value;

                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(ref_fwhm)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);

                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
            }
        }
        private static float ref_at_0 = Convert.ToSingle(ConfigurationManager.AppSettings.Get(nameof(ref_at_0)));
        public static float Ref_at_0
        {
            get { return ref_at_0; }
            set
            {
                ref_at_0 = value;
                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(ref_at_0)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);

                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
            }
        }
        private static float min_snr = Convert.ToSingle(ConfigurationManager.AppSettings.Get(nameof(min_snr)));
        public static float Min_snr
        {
            get { return min_snr; }
            set
            {
                min_snr = value;
                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(min_snr)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
                //LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
            }
        }
        public static readonly DateTime LastBootUp = new DateTime(Convert.ToInt64(ConfigurationManager.AppSettings.Get(nameof(LastBootUp))));

        /// <summary>
        /// Ecal Scatter 4, Absorber 4
        /// </summary>
        public static List<eEcalState> eEcalStates = new List<eEcalState>();

        public static bool InitiateLaghi()
        {
            StatusMsg = "Initiating LAHGI";
            var tempEchk = new List<AddListModeDataEchk>();
            ////tempEchk.Add(new AddListModeDataEchk(30, 90));
            ////tempEchk.Add(new AddListModeDataEchk(60, 100));
            ////tempEchk.Add(new AddListModeDataEchk(330, 370));
            ////tempEchk.Add(new AddListModeDataEchk(450, 570));
            //tempEchk.Add(new AddListModeDataEchk(600, 720));
            //tempEchk.Add(new AddListModeDataEchk(1594, 1984));
            //tempEchk.Add(new AddListModeDataEchk(0, 3000));
            //tempEchk.Add(new AddListModeDataEchk(1173 - 70, 1173 + 70));
            //tempEchk.Add(new AddListModeDataEchk(1333 - 50, 1333 + 50));
            //Echks = tempEchk;

            if (lahgiWrapper.Initiate(eModuleManagedType.QUAD))
            {
                StatusMsg = "Successfully initiate Lahgi Software";

                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(LastBootUp)].Value = DateTime.Now.Ticks.ToString();

                if (LastBootUp < DateTime.Now.AddDays(-1))
                {
                    eEcalStates.Clear();
                    for (int i = 0; i < 2; i++)
                    {
                        eEcalStates.Add(eEcalState.Unknown);
                    }
                }
                else
                {
                    eEcalStates.Clear();
                    for (int i = 0; i < 2; i++)
                    {
                        eEcalStates.Add((eEcalState)Enum.Parse(typeof(eEcalState), appSetting[nameof(eEcalStates) + i.ToString()].Value));
                    }
                }

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);

                //현수선배카트(295-312줄 전체 주석)
                if (LahgiSerialControl.StartCommunication())
                {
                    StatusMsg = "LahgiSerialControl Open Successfully";
                    LahgiSerialControl.CheckParams();

                    StartFPGA();

                    //240315 : Start Button click 시 진행되던 Start_usb를 프로그램 시작으로 변경
                    string status = "";
                    log.Info($"InitiateLahgi: Start_usb 호출 전, fpga.IsStart={fpga.IsStart}");
                    IsFPGAStart = fpga.Start_usb(out status);
                    log.Info($"InitiateLahgi: Start_usb 호출 후, IsFPGAStart={IsFPGAStart}, status={status}");
                    StatusMsg = status;
                    if (!IsFPGAStart)
                    {
                        StatusMsg = "LahgiSerialControl Thread Start Fail..";
                    }
                    else
                        StatusMsg = "LahgiSerialControl Thread Start Successfully";
                }
                else
                    StatusMsg = "LahgiSerialControl Open Fail";

                IsLahgiInitiate = true;
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Lahgi";
                IsLahgiInitiate = false;
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                return false;
            }
        }

        public static bool IsFPGAStart { get; set; } = false;

        public static bool InititateRtabmap()
        {
            StatusMsg = "Initiating RTABAMP";

            // Lahgi 초기화 상태와 관계없이 RTAB-Map 초기화 시도
            // D455 카메라만 연결되어 있어도 RTAB-Map은 작동할 수 있음
            if (rtabmapWrapper.InitiateRtabmap())
            {
                StatusMsg = "Successfully initiate Rtabmap";
                IsRtabmapInitiate = true;
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.FPGAUpdate);
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Rtabmap";
                IsRtabmapInitiate = false;
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                return false;
            }
        }

        public static void StopAll()
        {
            StopSlam();
            rtabmapWrapper.StopVideoStream();
        }

        public static void StopUpdateInvoker()
        {
            TimerBoolSlamPoints = false;
            TimerBoolSlamRadImage = false;
            TimerBoolSpectrum = false;
        }

        //231113-1 sbkwon
        public static void InitRadiationImage()
        {
            Task.Run(() =>
            {
                lahgiWrapper.InitRadiationImage();
            });
        }

        public static Mutex GetResponseImageMutex = new Mutex();
        public static BitmapImage? GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter)
        {
            BitmapImage? img = null;
            int width = 1;
            int height = 1;
            int stride = 1;
            GetResponseImageMutex.WaitOne();
            IntPtr data = IntPtr.Zero;
            var outData = lahgiWrapper.GetResponseImage(imgSize, pixelCount, timeInSeconds, isScatter);
            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            data = outData.ptr;

            if (data == IntPtr.Zero)
            {
                GetResponseImageMutex.ReleaseMutex();
                return img;
            }

            width = outData.width;
            height = outData.height;
            stride = outData.stride;
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, data);

            if (tempBitmap.Width == 1)
            {
                GetResponseImageMutex.ReleaseMutex();
                return img;
            }

            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                img = new BitmapImage();
                img.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                img.StreamSource = ms;
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.EndInit();
                img.Freeze();
            }
            GetResponseImageMutex.ReleaseMutex();
            return img;
        }

        public static Mutex GetRadation2dImageMutex = new Mutex();

        public static (BitmapImage?, BitmapImage?, BitmapImage?) GetRadation2dImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion)
        {
            BitmapImage? imgCoded = null;
            BitmapImage? imgCompton = null;
            BitmapImage? imgHybrid = null;

            if (!GetRadation2dImageMutex.WaitOne())
            {
                return (imgCoded, imgCompton, imgHybrid);
            }
            var outData = lahgiWrapper.Get2dRadationImage(timeInMiliSeconds, s2M, det_W, resImprov, m2D, hFov, wFov, imgSize, minValuePortion);
            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            IntPtr dataCoded = outData.Item1.ptr;
            IntPtr dataCompton = outData.Item2.ptr;
            IntPtr dataHybrid = outData.Item3.ptr;

            if (dataCompton == IntPtr.Zero || outData.Item1.width == 1)
            {
                GetRadation2dImageMutex.ReleaseMutex();
                return (imgCoded, imgCompton, imgHybrid);
            }

            int width = outData.Item1.width;
            int height = outData.Item1.height;
            int stride = outData.Item1.stride;
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCoded);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCoded = new BitmapImage();
                imgCoded.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCoded.StreamSource = ms;
                imgCoded.CacheOption = BitmapCacheOption.OnLoad;
                imgCoded.EndInit();
                imgCoded.Freeze();
            }


            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCompton);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCompton = new BitmapImage();
                imgCompton.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCompton.StreamSource = ms;
                imgCompton.CacheOption = BitmapCacheOption.OnLoad;
                imgCompton.EndInit();
                imgCompton.Freeze();
            }


            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataHybrid);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgHybrid = new BitmapImage();
                imgHybrid.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgHybrid.StreamSource = ms;
                imgHybrid.CacheOption = BitmapCacheOption.OnLoad;
                imgHybrid.EndInit();
                imgHybrid.Freeze();
            }

            GetRadation2dImageMutex.ReleaseMutex();


            return (imgCoded, imgCompton, imgHybrid);
        }

        //231025-1 sbkwon : Point Cloud
        public static (BitmapImage?, BitmapImage?, BitmapImage?) GetRadation2dImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double minValuePortion)
        {
            BitmapImage? imgCoded = null;
            BitmapImage? imgCompton = null;
            BitmapImage? imgHybrid = null;

            if (!GetRadation2dImageMutex.WaitOne())
            {
                return (imgCoded, imgCompton, imgHybrid);
            }
            var outData = lahgiWrapper.Get2dRadationImage(timeInMiliSeconds, s2M, det_W, resImprov, m2D, minValuePortion);
            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            IntPtr dataCoded = outData.Item1.ptr;
            IntPtr dataCompton = outData.Item2.ptr;
            IntPtr dataHybrid = outData.Item3.ptr;

            if (dataCompton == IntPtr.Zero || outData.Item1.width == 1)
            {
                GetRadation2dImageMutex.ReleaseMutex();
                return (imgCoded, imgCompton, imgHybrid);
            }

            int width = outData.Item1.width;
            int height = outData.Item1.height;
            int stride = outData.Item1.stride;
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCoded);

            //231115
            width = outData.Item2.width;
            height = outData.Item2.height;
            stride = outData.Item2.stride;
            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCoded = new BitmapImage();
                imgCoded.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCoded.StreamSource = ms;
                imgCoded.CacheOption = BitmapCacheOption.OnLoad;
                imgCoded.EndInit();
                imgCoded.Freeze();
            }


            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCompton);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCompton = new BitmapImage();
                imgCompton.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCompton.StreamSource = ms;
                imgCompton.CacheOption = BitmapCacheOption.OnLoad;
                imgCompton.EndInit();
                imgCompton.Freeze();
            }


            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataHybrid);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgHybrid = new BitmapImage();
                imgHybrid.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgHybrid.StreamSource = ms;
                imgHybrid.CacheOption = BitmapCacheOption.OnLoad;
                imgHybrid.EndInit();
                imgHybrid.Freeze();
            }

            GetRadation2dImageMutex.ReleaseMutex();


            return (imgCoded, imgCompton, imgHybrid);
        }

        //231100-GUI sbkwon : Plane - List Mode Data : count
        //240326 fullrange = true ( Detector FOV : 360, 180도 //real )
        //241021 sbkwon : 라벨링 사용 유무 추가
        public static (BitmapImage?, BitmapImage?, BitmapImage?) GetRadation2dImageCount(int count, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion, int maxValue, int time = 0, bool fullrange = false, bool labeling = false)
        {
            BitmapImage? imgCoded = null;
            BitmapImage? imgCompton = null;
            BitmapImage? imgHybrid = null;


            //test
            //StatusMsg = $"실외 (labeling = {labeling})";


            if (!GetRadation2dImageMutex.WaitOne())
            {
                return (imgCoded, imgCompton, imgHybrid);
            }
            var outData = lahgiWrapper.GetRadation2dImageCount(count, s2M, det_W, resImprov, m2D, hFov, wFov, imgSize, minValuePortion, time, maxValue, fullrange, labeling);
            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            IntPtr dataCoded = outData.Item1.ptr;
            IntPtr dataCompton = outData.Item2.ptr;
            IntPtr dataHybrid = outData.Item3.ptr;

            if (dataCompton == IntPtr.Zero || outData.Item1.width == 1)
            {
                GetRadation2dImageMutex.ReleaseMutex();
                return (imgCoded, imgCompton, imgHybrid);
            }

            int width = outData.Item1.width;
            int height = outData.Item1.height;
            int stride = outData.Item1.stride;
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCoded);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCoded = new BitmapImage();
                imgCoded.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCoded.StreamSource = ms;
                imgCoded.CacheOption = BitmapCacheOption.OnLoad;
                imgCoded.EndInit();
                imgCoded.Freeze();
            }

            //231214
            width = outData.Item2.width;
            height = outData.Item2.height;
            stride = outData.Item2.stride;
            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCompton);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCompton = new BitmapImage();
                imgCompton.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCompton.StreamSource = ms;
                imgCompton.CacheOption = BitmapCacheOption.OnLoad;
                imgCompton.EndInit();
                imgCompton.Freeze();
            }


            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataHybrid);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgHybrid = new BitmapImage();
                imgHybrid.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgHybrid.StreamSource = ms;
                imgHybrid.CacheOption = BitmapCacheOption.OnLoad;
                imgHybrid.EndInit();
                imgHybrid.Freeze();
            }

            GetRadation2dImageMutex.ReleaseMutex();


            return (imgCoded, imgCompton, imgHybrid);
        }

        //231100-GUI sbkwon : Pointcloud - List Mode Data : count
        //240122 sbkwon : Recon Max Value insert
        //241021 sbkwon : 라벨링 사용 유무 추가
        public static (BitmapImage?, BitmapImage?, BitmapImage?) GetRadation2dImageCount(int conut, double s2M, double det_W, double resImprov, double m2D, double minValuePortion, int maxValue, int time = 0, bool labeling = false)
        {
            BitmapImage? imgCoded = null;
            BitmapImage? imgCompton = null;
            BitmapImage? imgHybrid = null;


            //test
            //StatusMsg = $"Pointcloud (labeling = {labeling})";

            if (!GetRadation2dImageMutex.WaitOne())
            {
                return (imgCoded, imgCompton, imgHybrid);
            }
            var outData = lahgiWrapper.GetRadation2dImageCount(conut, s2M, det_W, resImprov, m2D, minValuePortion, time, maxValue, labeling);
            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            IntPtr dataCoded = outData.Item1.ptr;
            IntPtr dataCompton = outData.Item2.ptr;
            IntPtr dataHybrid = outData.Item3.ptr;

            if (dataCompton == IntPtr.Zero || outData.Item1.width == 1 ||
                dataCoded == IntPtr.Zero)
            {
                //StatusMsg = $"Image : {outData.Item1.width}, {outData.Item1.height}, {outData.Item2.width}, {outData.Item2.height}, {outData.Item3.width}, {outData.Item3.height}";
                GetRadation2dImageMutex.ReleaseMutex();
                return (imgCoded, imgCompton, imgHybrid);
            }

            int width = outData.Item1.width;
            int height = outData.Item1.height;
            int stride = outData.Item1.stride;
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCoded);

            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCoded = new BitmapImage();
                imgCoded.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCoded.StreamSource = ms;
                imgCoded.CacheOption = BitmapCacheOption.OnLoad;
                imgCoded.EndInit();
                imgCoded.Freeze();
            }

            //231115
            width = outData.Item2.width;
            height = outData.Item2.height;
            stride = outData.Item2.stride;
            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCompton);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCompton = new BitmapImage();
                imgCompton.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCompton.StreamSource = ms;
                imgCompton.CacheOption = BitmapCacheOption.OnLoad;
                imgCompton.EndInit();
                imgCompton.Freeze();
            }


            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataHybrid);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgHybrid = new BitmapImage();
                imgHybrid.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgHybrid.StreamSource = ms;
                imgHybrid.CacheOption = BitmapCacheOption.OnLoad;
                imgHybrid.EndInit();
                imgHybrid.Freeze();
            }

            GetRadation2dImageMutex.ReleaseMutex();


            return (imgCoded, imgCompton, imgHybrid);
        }

        //250107 2D MLEM
        public static bool Get2DMLEMImage(int nNo = 0)
        {
            bool bRet = lahgiWrapper.Get2DMLEMData(MLEMDataPath, nNo);           

            return bRet;
        }

        public static Mutex GetTransPoseRadiationImageMutex = new Mutex();


        public static BitmapImage? GetTransPoseRadiationImage(int timeInMiliSecond, double minValuePortion, double resolution = 10)
        {
            BitmapImage? img = null;

            if (!GetTransPoseRadiationImageMutex.WaitOne())
            {
                return img;
            }
            var outData = lahgiWrapper.GetTransPoseRadiationImage(timeInMiliSecond, minValuePortion, resolution);
            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            IntPtr data = outData.ptr;

            if (data == IntPtr.Zero || outData.width == 1)
            {
                GetRadation2dImageMutex.ReleaseMutex();
                return img;
            }

            int width = outData.width;
            int height = outData.height;
            int stride = outData.stride;
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, data);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                img = new BitmapImage();
                img.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                img.StreamSource = ms;
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.EndInit();
                img.Freeze();
            }

            GetTransPoseRadiationImageMutex.ReleaseMutex();


            return img;
        }

        //231100-GUI sbkwon : type 0:Color, 1:Gray
        //240105 sbkwon : 측정 중일 경우에는 listmote Data에서 생성하는  RGB를 이용하고 그이외의 경우에는 실시간 획득
        //240315 : point cloud => RadiationImage에서 사용한 RGB 사용
        public static BitmapImage? GetRgbImage(int colorType = 0, bool Type2 = false)
        {

            if (!IsRtabmapInitiate)
            {
                return null;
            }
            BitmapImage? img = null;
            int width = 1;
            int height = 1;
            int stride = 1;

            IntPtr data = IntPtr.Zero;

            //240105 : timerBoolSpectrum - ListModeData 획득 직전에 On, SessionStopwatch는 스펙트럼 전체 카운트가 일정이상 일경우에 restart
            //// 원래 코드 복원: 측정 중에는 ListModeData RGB 사용, 그 외에는 실시간 RGB 사용
            //// 단, MainWindowViewModel의 D455 카메라 영상 업데이트를 위해 측정 중에도 실시간 RGB를 사용하도록 수정
            //// 원래: if (TimerBoolSpectrum && SessionStopwatch.ElapsedMilliseconds > 1000)
            ////     rtabmapWrapper.GetRealTimeRGB(ref width, ref height, ref stride, ref data, false);  // ListModeData RGB
            //// else
            ////     rtabmapWrapper.GetRealTimeRGB(ref width, ref height, ref stride, ref data, true);   // 실시간 RGB
            //// 수정: 항상 실시간 RGB 사용 (측정 중에도 실시간 영상 표시)
            rtabmapWrapper.GetRealTimeRGB(ref width, ref height, ref stride, ref data, true);
            ////
            ////
            //rtabmapWrapper.GetRealTimeRGB(ref width, ref height, ref stride, ref data, true);
            ////

            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            if (data == IntPtr.Zero)
            {
                return img;
            }

            Bitmap tempBitmap;

            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format24bppRgb, data);

            if (colorType == 1)//gray
            {
                //create a blank bitmap the same size as original
                Bitmap newBitmap = new Bitmap(width, height);
                //get a graphics object from the new image
                Graphics g = Graphics.FromImage(newBitmap);
                //create the grayscale ColorMatrix
                ColorMatrix colorMatrix = new ColorMatrix(new float[][]
                {
                    new float[] {.3f, .3f, .3f, 0, 0},
                    new float[] {.59f, .59f, .59f, 0, 0},
                    new float[] {.11f, .11f, .11f, 0, 0},
                    new float[] {0, 0, 0, 1, 0},
                    new float[] {0, 0, 0, 0, 1}
                });
                //create some image attributes
                ImageAttributes attributes = new ImageAttributes();
                //set the color matrix attribute
                attributes.SetColorMatrix(colorMatrix);
                //draw the original image on the new image
                //using the grayscale color matrix
                g.DrawImage(tempBitmap, new Rectangle(0, 0, tempBitmap.Width, tempBitmap.Height),
                   0, 0, tempBitmap.Width, tempBitmap.Height, GraphicsUnit.Pixel, attributes);
                //dispose the Graphics object
                g.Dispose();

                tempBitmap = newBitmap;
            }

            if (tempBitmap.Width == 1)
            {
                return img;
            }

            //240326
            if (Type2)
            {
                //FOV 영역 좌측 상단을 0,0으로 하고 우측 하단을 360,180으로 하였을 경우 0,0에 해당하는 위치
                const int zeroFOVX = 90;
                const int zeroFOVY = 90;

                const int RGBFOVH = 58; //세로 d435 42 65 58
                const int RGBFOVW = 87; //가로 d435 69 90 87

                const int ReconFOVH = 180;  //세로
                const int ReconFOVW = 360;  //가로

                const int ReconImageH = 452;
                const int ReconImageW = 800;

                const double FOVStartX = zeroFOVX - (RGBFOVW / 2.0);    //55.5
                const double FOVStartY = zeroFOVY - (RGBFOVH / 2.0);    //69
                int offsetX = 10;   //카메라 위치 보정 X
                int offsetY = 4;   //카메라 위치 보정 Y

                //double FovtoPixelRateX = width / ReconFOVW;     //2.3555
                //double FovtoPixelRateY = height / ReconFOVH;    //1.71428
                double FovtoPixelRateX = 800 / ReconFOVW;     //2.3555
                double FovtoPixelRateY = 400 / ReconFOVH;    //1.71428


                int startX = (int)Math.Floor(FOVStartX * FovtoPixelRateX);
                int startY = (int)Math.Floor(FOVStartY * FovtoPixelRateY);
                //int newWidth = (int)Math.Floor(width * (RGBFOVW / (double)ReconFOVW));
                //int newHeight = (int)Math.Floor(height * (RGBFOVH / (double)ReconFOVH));
                //Bitmap newBitmap = new Bitmap(width, height);//848, 480


                int newWidth = (int)Math.Floor(800 * (RGBFOVW / (double)ReconFOVW));
                int newHeight = (int)Math.Floor(400 * (RGBFOVH / (double)ReconFOVH));
                Bitmap newBitmap = new Bitmap(800, 400);//848, 480
                Graphics g = Graphics.FromImage(newBitmap);
                g.Clear(Color.Blue);

                g.DrawImage(tempBitmap, startX + offsetX, startY - offsetY, newWidth, newHeight); //tempBitmap RGB

                g.Dispose();

                tempBitmap = newBitmap;
            }

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
                //img = bitMapimg;
            }


            //tempBitmap.Dispose();

            return img;
        }

        public static List<CRUXELLLACC.DeviceInfo> DeviceInfos = new List<CRUXELLLACC.DeviceInfo>();

        private static CRUXELLLACC.DeviceInfo? selectDevice;
        public static CRUXELLLACC.DeviceInfo? SelectDevice
        {
            get
            {
                return selectDevice;
            }
            set
            {
                fpga.SelectedDevice = value;
                selectDevice = value;
            }
        }
        public static bool IsFpgaAvailable { get; private set; }
        private static void UpdateDeviceList(object? sender, EventArgs e)
        {
            StatusMsg = "FPGA device list update";
            DeviceInfos = new List<CRUXELLLACC.DeviceInfo>(fpga.DeviceList);
            SelectDevice = fpga.SelectedDevice;
            if (DeviceInfos.Count > 0)
            {
                StatusMsg = "FPGA usb is connected";
                IsFpgaAvailable = true;
            }
            else
            {
                StatusMsg = "FPGA usb is unconnected";
                IsFpgaAvailable = false;
            }
            StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
        }

        static public Stopwatch SessionStopwatch = new Stopwatch();
        private static bool isSessionStart = false;
        public static bool IsSessionStart
        {
            get { return isSessionStart; }
            private set { StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status); isSessionStart = value; }
        }
        private static bool isSessionStarting = false;

        public static bool IsSessionStarting
        {
            get { return isSessionStarting; }
            set { StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status); isSessionStarting = value; }
        }

        //231016 sbkwon : 경과 시간
        private static uint _elapsedTime = 0;
        public static uint ElapsedTime
        {
            get => _elapsedTime;
            set => _elapsedTime = value;
        }

        public static async Task StartSessionAsync(string fileName, CancellationTokenSource tokenSource)
        {
            log.Info($"StartSessionAsync 호출됨: fileName={fileName}, IsSessionStart={IsSessionStart}, IsFPGAStart={IsFPGAStart}");
            //SessionStopwatch.Reset();//231017 sbkwon : time 초기화
            lahgiWrapper.SetUseFD(false);
            IsSessionStarting = true;
            if (!IsInitiate)
            {
                StatusMsg = "LAHGI is not initiated";
                log.Warn("StartSessionAsync: IsInitiate=false, LAHGI is not initiated");
            }
            if (!IsSessionStart)
            {
                if (!fpga.SetVaribles(fpgaVariables))
                {
                    StatusMsg = "Please configure FPGA.";
                    IsSessionStarting = false;
                    return;
                }
                else
                {
                    IsSessionStart = true;
                    StatusMsg = "FPGA setting Start";

                    string status = "";
                    fpga.Variables.FileName = fileName;

                    //bool isFPGAStart = await Task.Run(() => fpga.Start_usb(out status)).ConfigureAwait(false);//240315

                    StatusMsg = status;
                    
                    // IsFPGAStart 상태 확인 로그
                    log.Info($"StartSessionAsync: IsFPGAStart={IsFPGAStart}, fpga.IsStart={fpga.IsStart}");

                    if (IsFPGAStart)//240315 : isFPGAStart 지역변수 대체
                    {
                        log.Info($"StartSessionAsync: IsFPGAStart=true, fpga.IsStart={fpga.IsStart}");
                        fpga.init_file_save_bin();  //240422 Start_usb()를 프로그램 시작시 1회만 진행으로 변경하여 측정 폴더 경로 생성 추가

                        IsSessionStart = true;
                        StartSlam();

                        lahgiWrapper.ResetListmodeData();   //240122

                        StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);

                        log.Info($"StartSessionAsync: StartMeasurement 설정 전, 현재 값={fpga.StartMeasurement}");
                        fpga.StartMeasurement = true;   //242315
                        log.Info($"StartSessionAsync: StartMeasurement 설정 후, 설정된 값={fpga.StartMeasurement}, DataInQueue.Count={fpga.DataInQueue.Count}, ParsedQueue.Count={fpga.ParsedQueue.Count}, ShortArrayQueue.Count={fpga.ShortArrayQueue.Count}");
                        // StartMeasurement 설정 확인을 위한 추가 로그
                        Thread.Sleep(10); // 다른 스레드가 값을 읽을 수 있도록 짧은 대기
                        log.Info($"StartSessionAsync: StartMeasurement 재확인, 값={fpga.StartMeasurement}");

                        isSessionStarting = false;
                        TimerBoolSpectrum = true;
                        TimerBoolSlamRadImage = true;
                        //SessionStopwatch.Restart();
                        await Task.Run(() => AddListModeData(tokenSource)).ConfigureAwait(false);
                        SessionStopwatch.Stop();

                        fpga.StartMeasurement = false;   //240315

                        TimerBoolSpectrum = false;
                        TimerBoolSlamRadImage = false;
                        IsSessionStarting = true;
                        StatusMsg = "Stopping usb";

                        //StatusMsg = await fpga.Stop_usb();//240315
                        IsSessionStarting = true;
                        StatusMsg = "Saving CSV and ply file";

                        string saveFileName = Path.GetDirectoryName(fpga.FileMainPath) + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + fileName;   //230912 sbkwon : 시간, 날짜 정보 추가
                        rtabmapWrapper.SavePlyFile(saveFileName); //240621 sbkwon : 저장 파일 명 수정 

                        lahgiWrapper.SaveListModeData(saveFileName);
                        StatusMsg = "Done saving CSV and ply file";

                        SaveSumSpectrum(saveFileName + "_Spectrum.csv");    //230911 sbkwon : 스펙트럼 데이터 저장 (X, Y)                       

                        //(BitmapImage? codedsave, BitmapImage? comptonsave, BitmapImage? hybridsave) = GetRadation2dImageCount();
                        await Task.Run(() => StopSlam());


                        IsSessionStarting = false;
                        IsSessionStart = false;
                    }
                    else
                    {
                        IsSessionStart = false;
                        StatusMsg = "Something wrong with FPGA";
                        IsSessionStarting = false;

                        return;
                    }

                }
            }
            else
            {
                // IsSessionStart가 이미 true인 경우 (이전 측정 후 재시작)
                log.Info($"StartSessionAsync: IsSessionStart=true, 기존 세션 재사용, StartMeasurement 설정");
                if (IsFPGAStart)
                {
                    lahgiWrapper.ResetListmodeData();   //240122
                    StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                    
                    log.Info($"StartSessionAsync: StartMeasurement 설정 전, 현재 값={fpga.StartMeasurement}");
                    fpga.StartMeasurement = true;   //242315
                    log.Info($"StartSessionAsync: StartMeasurement 설정 후, 설정된 값={fpga.StartMeasurement}");
                    
                    isSessionStarting = false;
                    TimerBoolSpectrum = true;
                    TimerBoolSlamRadImage = true;
                    await Task.Run(() => AddListModeData(tokenSource)).ConfigureAwait(false);
                    SessionStopwatch.Stop();
                    
                    fpga.StartMeasurement = false;   //240315
                    
                    TimerBoolSpectrum = false;
                    TimerBoolSlamRadImage = false;
                    IsSessionStarting = true;
                    StatusMsg = "Stopping usb";
                    
                    IsSessionStarting = true;
                    StatusMsg = "Saving CSV and ply file";
                    
                    string saveFileName = Path.GetDirectoryName(fpga.FileMainPath) + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + fileName;
                    rtabmapWrapper.SavePlyFile(saveFileName);
                    lahgiWrapper.SaveListModeData(saveFileName);
                    StatusMsg = "Done saving CSV and ply file";
                    SaveSumSpectrum(saveFileName + "_Spectrum.csv");
                    await Task.Run(() => StopSlam());
                    
                    IsSessionStarting = false;
                    IsSessionStart = false;
                }
                else
                {
                    StatusMsg = "Session is already started but FPGA is not started";
                    IsSessionStarting = false;
                    return;
                }
            }

            StatusMsg = "Session is done";
            IsSessionStarting = false;

            return;
        }

        /// <summary>
        /// 240228 고장 검사용
        /// </summary>
        /// <param name="fileName"></param>
        /// <param name="tokenSource"></param>
        /// <returns></returns>
        public static async Task StartSessionAsyncFD(string fileName, CancellationTokenSource tokenSource)
        {
            StatusMsg = "고장검사 측정 시작";

            //SessionStopwatch.Reset();//231017 sbkwon : time 초기화
            IsSessionStarting = true;
            if (!IsInitiate)
            {
                StatusMsg = "LAHGI is not initiated";
            }
            if (!IsSessionStart)
            {
                lahgiWrapper.SetUseFD(true);

                if (!fpga.SetVaribles(fpgaVariables))
                {
                    StatusMsg = "Please configure FPGA.";
                    IsSessionStarting = false;
                    return;
                }
                else
                {
                    IsSessionStart = true;
                    StatusMsg = "FPGA setting Start";

                    string status = "";
                    fpga.Variables.FileName = fileName;

                    //bool isFPGAStart = await Task.Run(() => fpga.Start_usb(out status)).ConfigureAwait(false);//240315

                    StatusMsg = status;

                    if (IsFPGAStart)//240315 : isFPGAStart 지역변수 대체
                    {
                        IsSessionStart = true;
                        StartSlam();

                        lahgiWrapper.ResetListmodeData();   //240122

                        StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);

                        fpga.StartMeasurement = true;   //240315

                        isSessionStarting = false;
                        TimerBoolSpectrum = true;
                        TimerBoolSlamRadImage = false;
                        //SessionStopwatch.Restart();
                        await Task.Run(() => AddListModeData(tokenSource)).ConfigureAwait(false);
                        SessionStopwatch.Stop();

                        fpga.StartMeasurement = false;   //240315

                        TimerBoolSpectrum = false;
                        TimerBoolSlamRadImage = false;
                        IsSessionStarting = true;
                        StatusMsg = "Stopping usb";

                        //StatusMsg = await fpga.Stop_usb();//240315
                        IsSessionStarting = true;
                        StatusMsg = "Saving CSV and ply file";

                        //string saveFileName = Path.GetDirectoryName(fpga.FileMainPath) + "\\" + DateTime.Now.ToString("yyyyMMddHHmmss") + "_" + fileName;   //230912 sbkwon : 시간, 날짜 정보 추가
                        //rtabmapWrapper.SavePlyFile(saveFileName + "_SlamData.ply");

                        //lahgiWrapper.SaveListModeData(saveFileName);
                        //StatusMsg = "Done saving CSV and ply file";

                        //SaveSumSpectrum(saveFileName + "_Spectrum.csv");    //230911 sbkwon : 스펙트럼 데이터 저장 (X, Y)                       

                        await Task.Run(() => StopSlam());


                        IsSessionStarting = false;
                        IsSessionStart = false;
                    }
                    else
                    {
                        IsSessionStart = false;
                        StatusMsg = "Something wrong with FPGA";
                        IsSessionStarting = false;

                        return;
                    }

                }
            }
            else
            {
                StatusMsg = "Session is already started";
                IsSessionStarting = false;

                return;
            }

            StatusMsg = "고장검사 측정 종료";
            IsSessionStarting = false;
            lahgiWrapper.SetUseFD(false);

            return;
        }


        private static List<AddListModeDataEchk> echks = new List<AddListModeDataEchk>();
        public static List<AddListModeDataEchk> Echks
        {
            get
            {
                return echks;
            }
            set
            {

                List<double[]> UnmanagedEcks = new List<double[]>();
                List<int> UnmanagedElements = new List<int>();

                UnmanagedEcks.Clear();
                UnmanagedElements.Clear();

                foreach (var eck in value)
                {
                    double[] eckUnmanaged = new double[] { eck.MinE, eck.MaxE };
                    UnmanagedEcks.Add(eckUnmanaged);

                    UnmanagedElements.Add((int)eck.element);
                }
                lahgiWrapper.SetEchks(UnmanagedEcks, UnmanagedElements);

                echks = value;
            }
        }

        //240123 : list mode data(MLPE) 에 사용될 echk(모든 찾은 핵종, k-40 제외)
        private static List<SelectEchk> _selectEchks = new List<SelectEchk>();
        public static List<SelectEchk> SelectEchks
        {
            get { return _selectEchks; }
            set
            {
                List<int> UnmanagedElements = new List<int>();
                UnmanagedElements.Clear();

                foreach (var eck in value)
                {
                    UnmanagedElements.Add((int)eck.element);
                    //StatusMsg = $"lahgi class SelectEchk Add : {eck.element}";
                }

                lahgiWrapper.SelectEchks(UnmanagedElements);
                _selectEchks = value;

                //StatusMsg = $"lahgi class SelectEchk count : {UnmanagedElements.Count}";
            }
        }

        private static void AddListModeData(CancellationTokenSource tokenSource)
        {
            //lahgiWrapper.ResetListmodeData(); //240624 중복 제거
            //for (uint i = 0; i < 16; ++i)   //230907 sbkwon : 윗줄 lahgiWrapper.ResetListmodeData(); 내부에서 ResetSpectrum 호출함
            //{
            //    lahgiWrapper.ResetSpectrum(i);
            //}

            //Trace.WriteLine(DateTime.Now.ToString("yyyy-MM-dd hh:mm:ss.fff"));
            StatusMsg = "Add List Mode Data loop start";


            ////test 
            UInt64 counttemp = 0;

            bool checkfirst = true;
            while (true)
            {
                ushort[] item;
                while (fpga.ShortArrayQueue.TryTake(out item!))
                {
                    if (fpga.ShortArrayQueue.Count > 0 & checkfirst == true)
                    {
                        StatusMsg = "First Queue data arrived";
                        checkfirst = false;
                    }

                    //counttemp++;
                    //if (counttemp % 100000 == 0)
                    //    StatusMsg = $"queue count : {fpga.ShortArrayQueue.Count}";
                    //continue;

                    lahgiWrapper.AddListModeDataWraper(item);//fpga에서 원시 데이터 획득 : 144ea, listup

                    if (tokenSource.IsCancellationRequested)
                    {
                        break;
                    }
                }
                if (tokenSource.IsCancellationRequested)
                {
                    break;
                }
                //Thread.Sleep(0);
            }
            IsSessionStarting = true;

            while (fpga.ShortArrayQueue.Count > 0)
            {
                ushort[] item;
                fpga.ShortArrayQueue.TryTake(out item);
            }
            StatusMsg = "Add List Mode Data loop ended";
        }

        public static void TestAddingListModeData(int count)
        {
            StatusMsg = $"TestAddingListModeData starts (count = {count})";

            Random rnd = new Random();
            for (int c = 0; c < count; ++c)
            {
                ushort[] shortArray = new ushort[144];

                for (int j = 0; j < 144; ++j)
                {
                    shortArray[j] = 0;
                }
                //Channel 4 to 8
                for (int i = 4; i < 5; ++i)
                {
                    for (int j = 0; j < 9; ++j)
                    {
                        shortArray[i * 9 + j] = (ushort)rnd.Next(300);
                    }
                }

                //Channel 12 to 16
                for (int i = 12; i < 13; ++i)
                {
                    for (int j = 0; j < 9; ++j)
                    {
                        shortArray[i * 9 + j] = (ushort)rnd.Next(300);
                    }
                }
                fpga.ShortArrayQueue.Add(shortArray);
            }
            lahgiWrapper.ResetListmodeData();
            for (uint i = 0; i < 2; ++i)
            {
                lahgiWrapper.ResetSpectrum(i);
            }

            StatusMsg = $"TestAddingListModeData loop starts (count = {count})";
            Stopwatch sw = Stopwatch.StartNew();
            ushort[] item;
            while (fpga.ShortArrayQueue.TryTake(out item!))
            {
                lahgiWrapper.AddListModeDataWraper(item);
                //Thread.Sleep(0);
            }
            while (lahgiWrapper.GetListedListModeDataSize() != count)
            {

            }

            sw.Stop();
            Thread.Sleep(0);

            GetResponseImage(500, 200, 0, true);

            StatusMsg = $"TestAddingListModeData took {sw.ElapsedMilliseconds} ms for {count} counts";
        }

        public static void StartSlam()
        {
            rtabmapWrapper.StartSLAM();
            TimerBoolSlamPoints = true;
            StatusMsg = "Slam Started";

        }
        public static void StopSlam()
        {
            TimerBoolSlamPoints = false;
            rtabmapWrapper.StopSLAM();

        }
        private static Matrix3D currentSystemTranformation = Matrix3D.Identity;
        public static Matrix3D CurrentSystemTranformation
        {
            get
            {
                double[] marix3DElement = new double[0];
                rtabmapWrapper.GetPoseFrame(ref marix3DElement);
                if (marix3DElement != null)
                {

                    currentSystemTranformation = new Matrix3D(marix3DElement[0], marix3DElement[1], marix3DElement[2], marix3DElement[3],
                                            marix3DElement[4], marix3DElement[5], marix3DElement[6], marix3DElement[7],
                                            marix3DElement[8], marix3DElement[9], marix3DElement[10], marix3DElement[11],
                                            marix3DElement[12], marix3DElement[13], marix3DElement[14], marix3DElement[15]);
                }
                else
                {
                    currentSystemTranformation = new Matrix3D(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
                }
                SystemPoseX = currentSystemTranformation.OffsetX;
                SystemPoseY = currentSystemTranformation.OffsetY;
                SystemPoseZ = currentSystemTranformation.OffsetZ;
                return currentSystemTranformation;
            }
            private set
            {
                currentSystemTranformation = value;

            }
        }
        public static double SystemPoseX { get; private set; }
        public static double SystemPoseY { get; private set; }
        public static double SystemPoseZ { get; private set; }
        public static bool GetSLAMPointCloud(ref List<double[]> poseVect, ref List<double[]> colorVect)
        {

            rtabmapWrapper.GetSLAMPointCloud(ref poseVect, ref colorVect);
            if (poseVect.Count == 0 || colorVect.Count == 0)
            {
                return false;
            }
            return true;
        }

        //231121-1 sbkwon
        public static bool GetSLAMOccupancyGrid(ref List<double[]> poseVect, ref List<double[]> colorVect)
        {

            rtabmapWrapper.GetSLAMOccupancyGrid(ref poseVect, ref colorVect);
            if (poseVect.Count == 0 || colorVect.Count == 0)
            {
                return false;
            }
            return true;
        }

        public static (double, double, double) GetOdomentryPos()
        {
            double x = 0.0, y = 0.0, z = 0.0;

            rtabmapWrapper.GetOdomentryPos(ref x, ref y, ref z);
            return (x, y, z);
        }

        public static bool GetReconSLAMPointCloud(double time, eReconManaged reconType, ref List<double[]> poseVect, ref List<double[]> colorVect, double voxelSize, bool isLoading)
        {

            rtabmapWrapper.GetReconSLAMPointCloud(time, reconType, ref poseVect, ref colorVect, voxelSize, isLoading);
            if (poseVect.Count == 0 || colorVect.Count == 0)
            {
                return false;
            }
            return true;
        }

        public static bool GetOptimizedPoses(ref List<Matrix3D> matrixes)
        {
            matrixes = new List<Matrix3D>();
            List<double[]> posesMat = new List<double[]>();

            rtabmapWrapper.GetOptimizePoses(ref posesMat);
            if (posesMat.Count == 0)
            {
                return false;
            }

            foreach (var marix3DElement in posesMat)
            {
                Matrix3D pose = new Matrix3D(marix3DElement[0], marix3DElement[1], marix3DElement[2], marix3DElement[3],
                                            marix3DElement[4], marix3DElement[5], marix3DElement[6], marix3DElement[7],
                                            marix3DElement[8], marix3DElement[9], marix3DElement[10], marix3DElement[11],
                                            marix3DElement[12], marix3DElement[13], marix3DElement[14], marix3DElement[15]);
                matrixes.Add(pose);
            }
            return true;
        }
        public static SpectrumEnergyNasa GetSpectrumEnergy(int channelNumber)
        {
            if (!IsLahgiInitiate)
            {
                return new SpectrumEnergyNasa(5, 3000);
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetSpectrum((uint)channelNumber, ref eCount);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }
        public static SpectrumEnergyNasa GetSumSpectrumEnergy()
        {
            if (!IsLahgiInitiate)
            {
                return new SpectrumEnergyNasa(5, 3000);
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetSumSpectrum(ref eCount);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }

        //231100-GUI sbkwon : time
        public static SpectrumEnergyNasa GetSumSpectrumEnergyByTime(uint time)
        {
            if (!IsLahgiInitiate)
            {
                statusMsg = "GetSumSpectrumEnergyByTime : IsLahgiInitiate = false ";
                return new SpectrumEnergyNasa(5, 3000);
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetSumSpectrumByTime(ref eCount, time);
            log.Info($"GetSumSpectrumEnergyByTime: time={time}, eCount.Count={eCount.Count}");
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            int totalCount = 0;
            int nonZeroCount = 0;
            for (int i = 0; i < eCount.Count; i++)
            {
                int count = Convert.ToInt32(eCount[i][1]);
                totalCount += count;
                if (count > 0)
                {
                    nonZeroCount++;
                }
                histoEnergy.Add(new HistoEnergy(eCount[i][0], count));
            }
            log.Info($"GetSumSpectrumEnergyByTime: totalCount={totalCount}, nonZeroCount={nonZeroCount}, histoEnergy.Count={histoEnergy.Count}");
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }

        public static SpectrumEnergyNasa GetAbsorberSumSpectrum()
        {
            if (!IsLahgiInitiate)
            {
                return new SpectrumEnergyNasa(10, 3000);
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetAbsorberSumSpectrum(ref eCount);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }
        public static SpectrumEnergyNasa GetScatterSumSpectrum()
        {
            if (!IsLahgiInitiate)
            {
                return new SpectrumEnergyNasa(10, 3000);
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetScatterSumSpectrum(ref eCount);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }
        public static SpectrumEnergyNasa GetScatterSumSpectrumByTime(uint time)
        {
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetScatterSumSpectrumByTime(ref eCount, time);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }

        //250410 sbkwon
        public static SpectrumEnergyNasa GetSpectrumData(uint type, uint ch)
        {
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetSpectrumData(ref eCount, type, ch);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }


        //231123 sbkwon
        public static SpectrumEnergyNasa GetSpectrumByTime(int chNo, uint time)
        {
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetSpectrumByTime((uint)chNo, ref eCount, time);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }

        public static SpectrumEnergyNasa GetAbsorberSumSpectrumByTime(uint time)
        {
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetAbsorberSumSpectrumByTime(ref eCount, time);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }

        //240228 : 고장검사 Data Copy
        public static void CopyPMTData()
        {
            lahgiWrapper.CopyPMTData();
        }

        //240228 고장 검사 채널별 게인 획득
        public static List<double> GetGainref(int chNo)
        {
            //0, 1번 채널만 허용
            if (chNo < 0 || chNo > 1)
            {
                throw new ArgumentException("Invalid channel number. Use 0 (Scatter) or 1 (Absorber)");
            }

            List<double> gainref = new List<double>();

            lahgiWrapper.GetGainref((uint)chNo, ref gainref);

            return gainref;
        }

        //240228 : 고장검사용, Gain[9] 적용된 PMT 데이터 획득
        public static SpectrumEnergyNasa GetPMTEnergyData(int chNo)
        {
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetPMTEnergyData((uint)chNo, ref eCount);

            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();

            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }

        //240228 : 고장검사용, CorrMatIn 적용된 PMT 데이터 획득
        public static SpectrumEnergyNasa GetPMTEnergyData(int chNo, List<double> CorrMatIn)
        {
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetPMTEnergyData((uint)chNo, CorrMatIn, ref eCount);

            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();

            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }

        //240228 고장 검사 : usedPeak range를 이용하여 gain 계산
        public static List<double> GetPMTCorrMatIn(int chNo, int[] usedPeak, double[] range_bkg)
        {
            //0, 1번 채널만 허용
            if (chNo < 0 || chNo > 1)
            {
                throw new ArgumentException("Invalid channel number. Use 0 (Scatter) or 1 (Absorber)");
            }

            List<double> CorrMatIn = new List<double>();

            List<int> listused = new List<int> { usedPeak[0], usedPeak[1], usedPeak[2], usedPeak[3] };
            List<double> listRange = new List<double> { range_bkg[0], range_bkg[1], range_bkg[2], range_bkg[3], range_bkg[4], range_bkg[5], range_bkg[6], range_bkg[7] };

            lahgiWrapper.GetPMTCorrMatIn((uint)chNo, listused, listRange, ref CorrMatIn);

            return CorrMatIn;
        }

        //240315
        public static List<double> GetPMTCorrMatInBeforGain(int chNo, int[] usedPeak, double[] range_bkg, List<double> CorrMatIn)
        {
            List<double> CorrMatOut = new List<double>();

            List<int> listused = new List<int> { usedPeak[0], usedPeak[1], usedPeak[2], usedPeak[3] };
            List<double> listRange = new List<double> { range_bkg[0], range_bkg[1], range_bkg[2], range_bkg[3], range_bkg[4], range_bkg[5], range_bkg[6], range_bkg[7] };

            lahgiWrapper.GetPMTCorrMatInBeforGain((uint)chNo, listused, listRange, CorrMatIn, ref CorrMatOut);

            return CorrMatOut;
        }

        public static bool Ecal609keV = false;

        public static void CheckEcalState(double min1461E = 1360, double max1461E = 1560)
        {
            StatusMsg = "Ecal Start";
            for (int i = 0; i < 2; ++i)
            {
                if (eEcalStates[i] == eEcalState.Success)
                {
                    if (i == 1)
                    {
                        return;
                    }
                }
                else
                {
                    break;
                }
            }
            for (int i = 0; i < 1; ++i)
            {
                //Scatter

                StatusMsg = "Ecal " + i + 4 + " Channel";

                var peaks = GetSpectrumEnergy(i + 4).FindPeaks(ref_x, ref_fwhm, ref_at_0, min_snr);
                bool isEcalSuccessFlag = false;
                foreach (var e in peaks)
                {
                    if (Ecal609keV)
                    {

                    }
                    else
                    {
                        if (e >= min1461E && e <= max1461E)
                        {
                            double ecalA = 0;
                            double ecalB = 0;
                            double ecalC = 0;
                            lahgiWrapper.GetEcal(Convert.ToUInt32(i + 4), ref ecalA, ref ecalB, ref ecalC);
                            //Only linearity affect
                            ecalA *= 1461 * 1461 / e / e;
                            ecalB *= 1461 / e;

                            lahgiWrapper.SetEcal(Convert.ToUInt32(i + 4), ecalA, ecalB, ecalC);

                            eEcalStates[i] = eEcalState.Success;
                            isEcalSuccessFlag = true;
                        }

                    }
                    if (!isEcalSuccessFlag)
                    {
                        eEcalStates[i] = eEcalState.Fail;
                    }
                }
            }
            for (int i = 0; i < 1; ++i)
            {
                //Absorber
                StatusMsg = "Ecal " + i + 12 + " Channel";

                var peaks = GetSpectrumEnergy(i + 12).FindPeaks(ref_x, ref_fwhm, ref_at_0, min_snr);
                bool isEcalSuccessFlag = false;

                foreach (var e in peaks)
                {
                    if (Ecal609keV)
                    {

                    }
                    else
                    {
                        if (e >= min1461E && e <= max1461E)
                        {
                            double ecalA = 0;
                            double ecalB = 0;
                            double ecalC = 0;
                            lahgiWrapper.GetEcal(Convert.ToUInt32(i + 12), ref ecalA, ref ecalB, ref ecalC);
                            //Only linearity affect
                            ecalA *= 1461 * 1461 / e / e;
                            ecalB *= 1461 / e;

                            lahgiWrapper.SetEcal(Convert.ToUInt32(i + 12), ecalA, ecalB, ecalC);

                            eEcalStates[i + 4] = eEcalState.Success;
                            isEcalSuccessFlag = true;
                        }
                    }
                    if (!isEcalSuccessFlag)
                    {
                        eEcalStates[i + 4] = eEcalState.Fail;
                    }
                }

            }
            StatusMsg = "Ecal Done";



            var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
            var appSetting = configFile.AppSettings.Settings;

            for (int i = 0; i < 8; i++)
            {
                appSetting[nameof(eEcalStates) + i.ToString()].Value = eEcalState.Unknown.ToString();
            }
            configFile.Save(ConfigurationSaveMode.Modified);
            ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
            LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);


            StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
        }

        public static bool LoadListModeData(string filePath)
        {
            lahgiWrapper.ResetListmodeData();
            for (uint i = 0; i < 2; ++i)
            {
                lahgiWrapper.ResetSpectrum(i);
            }
            StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Loading);
            return lahgiWrapper.LoadListModeData(filePath);
        }

        public static bool LoadPlyFile(string filePath)
        {
            if (Path.GetExtension(filePath) != ".ply")
            {
                log.Error("LoadPyFile fail due to extenstion is not ply");
                return false;
            }
            log.Info($"Loading ply file: {filePath}");
            rtabmapWrapper.ResetSLAM();
            StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Loading);

            if (rtabmapWrapper.LoadPlyFile(filePath))
            {
                log.Info("Loading seccess");
                return true;
            }
            else
            {
                log.Info("Loading fail");
                return false;
            }
        }

        public static void GetLoadedPointCloud(ref List<double[]> poseVect, ref List<double[]> colorVect)
        {
            rtabmapWrapper.GetLoadedPointCloud(ref poseVect, ref colorVect);
        }

        //230911 sbkwon : spectrum 데이터(X, Y) 저장
        public static void SaveSumSpectrum(string filepath)
        {
            if (!IsLahgiInitiate)
            {
                return;
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetSumSpectrum(ref eCount);

            using (StreamWriter file = new StreamWriter(filepath))
            {
                file.WriteLine("Energy,Count");

                for (int i = 0; i < eCount.Count; i++)
                {
                    file.WriteLine($"{eCount[i][0]},{eCount[i][1]}");
                }
            }
        }

        //231019 sbkwon : File Name
        public static string GetFileSavePath()
        {
            if (string.IsNullOrEmpty(fpga.FileMainPath))
                return DateTime.Now.ToString("yyyyMMddHHmmss");
            else
                return fpga.FileMainPath;
        }

        //231109-2 sbkwon
        public static void GetEcal(uint chanelNo, ref double ecalA, ref double ecalB, ref double ecalC)
        {
            lahgiWrapper.GetEcal(chanelNo, ref ecalA, ref ecalB, ref ecalC);
        }

        //231109-2 sbkwon
        public static void SetEcal(uint channelNo, double ecalA, double ecalB, double ecalC)
        {
            lahgiWrapper.SetEcal(channelNo, ecalA, ecalB, ecalC);
        }

        public static void StartFPGA()
        {
            if (LahgiSerialControl.IsFPGAOn == false)
            {
                LahgiSerialControl.SetFPGA(true);
                StatusMsg = "FPGA Command on";
            }
            else
                StatusMsg = "FPGA is already on";

            if (LahgiSerialControl.IsFPGAOn)
            {
                if (LahgiSerialControl.HvModuleVoltage < 600)   //600
                {
                    LahgiSerialControl.SetHvMoudle(true);
                    StatusMsg = "FPGA HV Module On";
                }

                fpga.ResetFPGA();
            }
            else
                StatusMsg = "FPGA On Fail";
        }

        public static void StopFPGA()
        {
            if (LahgiSerialControl.IsFPGAOn)
            {
                StatusMsg = "FPGA HV Module off Command";

                LahgiSerialControl.SetHvMoudle(false);

                //while (LahgiSerialControl.HvModuleVoltage > 50)
                //{
                //    LahgiSerialControl.CheckParams();
                //    Thread.Sleep(100);
                //}

                LahgiSerialControl.SetFPGA(false);

                LahgiSerialControl.StopCommunication();

                StatusMsg = "FPGA HV Module off seccess";
            }
            else
                StatusMsg = "FPGA is already off";
        }

        //240315 : stop_usb()
        public static async Task StopUSBAsync()
        {
            StatusMsg = "Stop_usb()";
            StatusMsg = await fpga.Stop_usb();
        }

        public static void FPGACheck()
        {
            LahgiSerialControl.CheckParams();
        }

        public static int GetSlamedPointCloudCount()
        {
            return lahgiWrapper.GetSlamedPointCloudCount();
        }


        //250115 정밀 영상 데이터 로드 성공 여부
        private static bool _MLEMDataLoad = false;
        public static bool MLEMDataLoad
        {
            get { return _MLEMDataLoad; }
            set { _MLEMDataLoad = value; }
        }

        //250115 정밀 영상 데이터 로드 성공 경로
        private static string _MLEMDataPath = "";
        public static string MLEMDataPath
        {
            get { return _MLEMDataPath; }
            set { _MLEMDataPath = value; }
        }

        //2404 : MLEM
        /// <summary>
        ///240429 : MLEM 
        /// </summary>
        /// <param name="PLYPath">point cloud file then bLoad == true or 측정 결과 저장 경로 then bLoad == false</param>
        /// <param name="LMDPath">list mode data file path</param>
        /// <param name="bLoad">true : file load, false : 측정 자료 사용</param>
        /// <param name="nSize">정밀영상 연속 개수</param>
        /// <returns></returns>
        public static bool LoadMLEMData(string PLYPath, string LMDPath, bool bLoad, int nSize = 1)
        {
            bool bRet = lahgiWrapper.LoadMLEMData(PLYPath, LMDPath, bLoad, nSize);

            MLEMDataLoad = bRet;
            MLEMDataPath = PLYPath;

            if(bRet)
            {
                //StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Loading);    //250324
            }
            else
            {
                StatusMsg = "MLEM Data Load Fail";
            }

            return bRet;
        }

        //250107 : faile 결과 전달
        private static bool statusCalMLEM = false;
        public static bool StatusCalMLEM
        {
            get { return statusCalMLEM; }
            set { statusCalMLEM = value; }
        }

        //240429 : MLEM List
        public static async Task CalMLEM(string systemMPath, List<double> energy, List<double> EgateMin, 
            List<double> EgateMax, double minValuePer = 0.7, bool bShow = true)
        {
            StatusCalMLEM = false;
            StatusCalMLEM = await Task.Run(() => lahgiWrapper.CalMLEMList(systemMPath, energy, EgateMin, EgateMax, minValuePer));
            if (StatusCalMLEM)
            {
                //정상 동작
                if(bShow)
                    StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.MLEM);
               // LahgiApi.UIMessageUpdateInvoke(null, "정밀 영상 계산 완료");
            }
            else
            {
                StatusMsg = "CalMLEM Fail";
            }
        }

        //2404 : MLEM
        public static async Task CalMLEM(string systemMPath, double energy, double EgateMin, double EgateMax, double minValuePer = 0.7)
        {
            bool breturn = await Task.Run(() => lahgiWrapper.CalMLEM(systemMPath, energy, EgateMin, EgateMax, minValuePer));
            if (breturn)
            {
                //정상 동작
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.MLEM);
            }
            else
            {
                StatusMsg = "CalMLEM Fail";
            }
        }

        //2404 : MLEM - MLEM 결과 포인트 클라우드 획득
        public static bool GetLoadedPointCloudMLEM(ref List<double[]> poseVect, ref List<double[]> colorVect, int nNo = 0)
        {
            lahgiWrapper.GetMLEMPointCloud(ref poseVect, ref colorVect, nNo);

            //UIMessageUpdateInvoke(null, "정밀 영상 계산 완료");
            
            if (poseVect.Count == 0 || colorVect.Count == 0)
            {
                return false;
            }

            return true;
        }

        //240429 : 핵종 리스트에서 선택된 EM
        public static int MLEMSelectNo { get; set; } = 0;
        //240429 : 현재 정밀영상 수행중인지 판단, Start 버튼 클릭시 false, 정밀영상 버튼 클릭시 true
        public static bool MLEMRun { get; set; } = false;

        //2404 : MLEM - UI message 표시
        public static EventHandler? UIMessageUpdate;

        public static string UIMessage { get; private set; }

        public class LahgiApiUIMessageEventArgs : EventArgs
        {
            public LahgiApiUIMessageEventArgs(string message)
            {
                UIMessage = message;
            }
        }

        public static void UIMessageUpdateInvoke(object? obj, string msg)
        {
            Task.Run(() => { UIMessageUpdate?.Invoke(obj, new LahgiApiUIMessageEventArgs(msg)); });
        }
    }
}

