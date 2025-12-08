using System;
using System.IO;
using System.Text.Json;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using HUREL.Compton;
using HUREL_Imager_GUI.ViewModel;
using log4net;
using log4net.Appender;
using System.Diagnostics;
using System.Windows.Threading;

namespace HUREL_Imager_GUI
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application        
    {
        internal const string ConfFileName = "Config.json";

        //internal static MainWindowViewModel? MainVM { get; private set; }   //230921 sbkwon : global variable

        public static Config GlobalConfig { get; private set; } = new();

        // MainWindow 참조 저장
        public static MainWindow? CurrentMainWindow { get; set; }

        //public static SynchronizationContext SyncCtt { get; set; }

        public static Dispatcher? mainDispatcher { get; set; }

        private static readonly ILog logger = LogManager.GetLogger(typeof(App));
        public App()
        {
            // COM 초기화 제거 - WPF가 자동으로 처리
            logger.Info("App 생성자 시작");

            if (File.Exists(ConfFileName) is true)
            {
                using FileStream confFile = File.OpenRead(ConfFileName);
                GlobalConfig = JsonSerializer.Deserialize<Config>(confFile) ?? throw new NullReferenceException("");
            }

            ShutdownMode = ShutdownMode.OnLastWindowClose;
            //NativeMethods.AllocConsole();
            logger.Info("Start application");
            
            logger.Info("Console loaded");
            Syncfusion.Licensing.SyncfusionLicenseProvider.RegisterLicense("NTcxMjAyQDMxMzkyZTM0MmUzMEw2eUs1OURYTGswSnNaZ3p5WjlIcWdPQTcrM2UxWEdSbWd6TW9iUnRlcjA9");
            logger.Info("Config Setting");
        }
        protected override void OnStartup(StartupEventArgs e)
        {
            System.Diagnostics.Debug.WriteLine("=== App.OnStartup 시작 ===");
            
            //240126 : 중복 실행 방지
            System.Diagnostics.Process[] processes = null;
            string strCurrentProcess = System.Diagnostics.Process.GetCurrentProcess().ProcessName.ToLower();
            processes = System.Diagnostics.Process.GetProcessesByName(strCurrentProcess);
            
            if(processes.Length > 1 )
            {
                MessageBox.Show($"'{System.Diagnostics.Process.GetCurrentProcess().ProcessName}' 프로그램이 이미 실행 중입니다..");
                Application.Current.Shutdown();
                return;
            }
            System.Diagnostics.Debug.WriteLine("중복 실행 체크 완료");

            // 기본 API만 초기화 (MainWindow 표시에 필요한 것만)
            System.Diagnostics.Debug.WriteLine("기본 API 초기화 시작...");
            
            LahgiApi.InitRadiationImage();//231113-1 sbkwon
            System.Diagnostics.Debug.WriteLine("InitRadiationImage 완료");
            
            LahgiApi.InitiateLaghi();
            System.Diagnostics.Debug.WriteLine("InitiateLaghi 완료");
            
            // RTAB-Map 초기화는 MainWindow 생성 후 별도로 처리
            // GUI가 먼저 표시되도록 함

            // MainWindow 수동 생성 (RTAB-Map 초기화 전)
            logger.Info("MainWindow 수동 생성 시작...");
            try
            {
                CurrentMainWindow = new MainWindow();
                logger.Info("MainWindow 생성 성공!");
                
                // MainWindow 표시
                CurrentMainWindow.Show();
                logger.Info("MainWindow 표시 완료!");
            }
            catch (Exception ex)
            {
                logger.Error($"MainWindow 생성 실패: {ex.Message}");
                logger.Error($"스택 트레이스: {ex.StackTrace}");
            }

            base.OnStartup(e);
            System.Diagnostics.Debug.WriteLine("=== App.OnStartup 완료 ===");
        }

        protected override void OnExit(ExitEventArgs e)
        {
            // COM 정리 제거 - WPF가 자동으로 처리
            logger.Info("App 종료 시작");

            using var confFile = File.Open(ConfFileName, FileMode.Create);
            JsonSerializer.Serialize<Config>(confFile, GlobalConfig);
            logger.Info("Exit");

            base.OnExit(e);
        }
    }
    static class NativeMethods
    {
        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool AllocConsole();
    }

}
