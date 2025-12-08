using Syncfusion.Windows.Shared;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Syncfusion.Windows.Tools.Controls;


namespace HUREL_Imager_GUI
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private bool _isClosing = false; // 중복 실행 방지 플래그
        public MainWindow()
        {
            try
            {
                var logger = log4net.LogManager.GetLogger(typeof(MainWindow));
                logger.Info("=== MainWindow 생성 시작 ===");
                logger.Info($"현재 스레드: {System.Threading.Thread.CurrentThread.ManagedThreadId}");
                
                App.mainDispatcher = System.Windows.Threading.Dispatcher.CurrentDispatcher;
                logger.Info("Dispatcher 설정 완료");

                logger.Info("InitializeComponent 시작...");
                InitializeComponent();
                logger.Info("InitializeComponent 완료");
                
                // 윈도우 표시 설정 강제 적용
                this.Visibility = Visibility.Visible;
                this.ShowInTaskbar = true;
                this.Topmost = false;
                logger.Info($"윈도우 표시 설정: Visibility={this.Visibility}, ShowInTaskbar={this.ShowInTaskbar}, Topmost={this.Topmost}");
                
                // App에 MainWindow 참조 저장
                App.CurrentMainWindow = this;
                logger.Info("App 참조 저장 완료");
                
                // DataContext 설정
                logger.Info("DataContext 설정 시작...");
                try
                {
                    logger.Info("MainWindowViewModel 인스턴스 생성 시작...");
                    var mainVM = new ViewModel.MainWindowViewModel();
                    logger.Info("MainWindowViewModel 인스턴스 생성 완료");
                    
                    logger.Info("DataContext에 MainWindowViewModel 할당 시작...");
                    this.DataContext = mainVM;
                    logger.Info("DataContext에 MainWindowViewModel 할당 완료");
                    
                    logger.Info("DataContext 설정 완료");
                }
                catch (Exception ex)
                {
                    logger.Error($"DataContext 설정 실패: {ex.Message}");
                    logger.Error($"예외 타입: {ex.GetType().Name}");
                    logger.Error($"스택 트레이스: {ex.StackTrace}");
                    
                    // 예외 발생 시에도 기본 DataContext 설정 시도
                    try
                    {
                        logger.Info("기본 DataContext 설정 시도...");
                        this.DataContext = new object(); // 임시 객체
                        logger.Info("기본 DataContext 설정 완료");
                    }
                    catch (Exception fallbackEx)
                    {
                        logger.Error($"기본 DataContext 설정도 실패: {fallbackEx.Message}");
                    }
                }
                
                // 디버깅 정보 추가
                logger.Info("MainWindow 생성 완료");
                logger.Info($"MainWindow 크기: {this.Width}x{this.Height}");
                logger.Info($"MainWindow 상태: {this.WindowState}");
                logger.Info($"MainWindow 가시성: {this.Visibility}");
                
                //HomeItem.IsSelected = true;

                this.Closing += MainWindow_Closing;
                logger.Info("Closing 이벤트 등록 완료");
                
                // Loaded 이벤트 추가
                this.Loaded += MainWindow_Loaded;
                logger.Info("Loaded 이벤트 등록 완료");
                
                logger.Info("=== MainWindow 생성 완료 ===");
                
                // RTAB-Map 지연된 자동 초기화 (GUI 표시 후 5초 뒤)
                _ = Task.Run(async () =>
                {
                    try
                    {
                        logger.Info("RTAB-Map 지연된 자동 초기화 시작...");
                        await Task.Delay(5000); // GUI가 완전히 표시될 때까지 5초 대기
                        
                        // 이미 초기화된 경우 중복 초기화 방지
                        if (HUREL.Compton.LahgiApi.IsRtabmapInitiate)
                        {
                            logger.Info("RTAB-Map이 이미 초기화되어 있음 - 중복 초기화 건너뜀");
                            return;
                        }
                        
                        logger.Info("RTAB-Map 초기화 실행...");
                        bool rtabmapResult = HUREL.Compton.LahgiApi.InititateRtabmap();
                        if (rtabmapResult)
                        {
                            logger.Info("RTAB-Map 지연된 자동 초기화 성공");
                        }
                        else
                        {
                            logger.Warn($"RTAB-Map 지연된 자동 초기화 실패: {HUREL.Compton.LahgiApi.StatusMsg}");
                        }
                    }
                    catch (Exception rtabmapEx)
                    {
                        logger.Error($"RTAB-Map 지연된 자동 초기화 중 예외: {rtabmapEx.Message}");
                    }
                });
            }
            catch (Exception ex)
            {
                var logger = log4net.LogManager.GetLogger(typeof(MainWindow));
                logger.Error($"MainWindow 생성 중 오류 발생: {ex.Message}");
                logger.Error($"스택 트레이스: {ex.StackTrace}");
                throw;
            }
        }
        
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            var logger = log4net.LogManager.GetLogger(typeof(MainWindow));
            logger.Info("MainWindow Loaded 이벤트 발생");
            
            logger.Info($"MainWindow 실제 크기: {this.ActualWidth}x{this.ActualHeight}");
            logger.Info($"MainWindow 위치: {this.Left}, {this.Top}");
            logger.Info($"MainWindow 가시성: {this.Visibility}");
            logger.Info($"MainWindow 윈도우 상태: {this.WindowState}");
            
            // IsotopeTable에 SpectrumViewModel 연결
            try
            {
                if (this.DataContext is ViewModel.MainWindowViewModel mainVM)
                {
                    logger.Info("IsotopeTable에 SpectrumViewModel 연결 시작...");
                    IsotopeTableControl.SetSpectrumViewModel(mainVM.SpectrumVM);
                    logger.Info("IsotopeTable에 SpectrumViewModel 연결 완료");
                }
                else
                {
                    logger.Warn("DataContext가 MainWindowViewModel이 아님");
                }
            }
            catch (Exception ex)
            {
                logger.Error($"IsotopeTable 연결 중 오류: {ex.Message}");
            }
        }

        private async void MainWindow_Closing(object? sender, System.ComponentModel.CancelEventArgs e)
        {
            // 중복 실행 방지
            if (_isClosing)
            {
                e.Cancel = true;
                return;
            }
            
            var logger = log4net.LogManager.GetLogger(typeof(MainWindow));
            logger.Info("MainWindow_Closing 시작");
            
            _isClosing = true;
            
            // 창 닫기를 취소하여 비동기 작업이 완료될 때까지 대기
            e.Cancel = true;
            
            try
            {
                // DataContext에서 Closing 메서드를 직접 호출하여 완료 대기
                if (this.DataContext is ViewModel.MainWindowViewModel mainVM)
                {
                    logger.Info("Closing 메서드 실행 시작");
                    await mainVM.Closing().ConfigureAwait(false);
                    logger.Info("Closing 메서드 실행 완료");
                }
                
                // 추가 대기 시간 제거 (이미 각 작업에서 타임아웃 처리)
                // await Task.Delay(500).ConfigureAwait(false);
                
                logger.Info("Application.Current.Shutdown() 호출");
                // UI 스레드에서 Shutdown 호출
                Application.Current.Dispatcher.Invoke(() =>
                {
                    Application.Current.Shutdown();
                });
            }
            catch (Exception ex)
            {
                logger.Error($"MainWindow_Closing 중 예외 발생: {ex.Message}");
                logger.Error($"스택 트레이스: {ex.StackTrace}");
                // 예외가 발생해도 종료 시도
                Application.Current.Dispatcher.Invoke(() =>
                {
                    Application.Current.Shutdown();
                });
            }
        }



        private void SpectrumView_Loaded(object sender, RoutedEventArgs e)
        {

        }
    }
}
