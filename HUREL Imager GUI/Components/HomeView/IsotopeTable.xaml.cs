using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;
using HUREL_Imager_GUI.ViewModel;
using System;

namespace HUREL_Imager_GUI.Components
{
    /// <summary>
    /// IsotopeTable.xaml에 대한 상호 작용 논리
    /// </summary>
    public partial class IsotopeTable : UserControl
    {
        private Dictionary<string, Button> _isotopeButtons;
        private SpectrumViewModel _spectrumVM;

        public IsotopeTable()
        {
            InitializeComponent();
            this.Loaded += IsotopeTable_Loaded;
        }
        
        private void IsotopeTable_Loaded(object sender, RoutedEventArgs e)
        {
            InitializeIsotopeButtons();
        }

        private void InitializeIsotopeButtons()
        {
            _isotopeButtons = new Dictionary<string, Button>();
            
            // 첫 번째 줄
            _isotopeButtons["Co-58"] = FindButtonInGrid(0, 0);
            _isotopeButtons["Co-60"] = FindButtonInGrid(0, 1);
            _isotopeButtons["Cs-137"] = FindButtonInGrid(0, 2);
            _isotopeButtons["Eu-152"] = FindButtonInGrid(0, 3);
            _isotopeButtons["Cs-134"] = FindButtonInGrid(0, 4);
            _isotopeButtons["I-131"] = FindButtonInGrid(0, 5);
            _isotopeButtons["Pu-238"] = FindButtonInGrid(0, 6);
            _isotopeButtons["Pu-239"] = FindButtonInGrid(0, 7);
            _isotopeButtons["Pu-240"] = FindButtonInGrid(0, 8);
            _isotopeButtons["Pu-241"] = FindButtonInGrid(0, 9);

            // 두 번째 줄
            _isotopeButtons["Ir-192"] = FindButtonInGrid(1, 0);
            _isotopeButtons["Se-75"] = FindButtonInGrid(1, 1);
            _isotopeButtons["U-235"] = FindButtonInGrid(1, 2);
            _isotopeButtons["U-238"] = FindButtonInGrid(1, 3);
            _isotopeButtons["Am-241"] = FindButtonInGrid(1, 4);
            _isotopeButtons["Ba-133"] = FindButtonInGrid(1, 5);
            _isotopeButtons["Na-22"] = FindButtonInGrid(1, 6);
            _isotopeButtons["Cd-109"] = FindButtonInGrid(1, 7);
            _isotopeButtons["Tc-99m"] = FindButtonInGrid(1, 8);
            _isotopeButtons["Annihilation"] = FindButtonInGrid(1, 9);

            // 세 번째 줄
            _isotopeButtons["K-40"] = FindButtonInGrid(2, 0);
            _isotopeButtons["Tl-208"] = FindButtonInGrid(2, 1);
            _isotopeButtons["Bi-214"] = FindButtonInGrid(2, 2);
            _isotopeButtons["Pb-212"] = FindButtonInGrid(2, 3);
            _isotopeButtons["F-18"] = FindButtonInGrid(2, 4);

            // 모든 핵종 버튼에 클릭 이벤트 추가
            foreach (var button in _isotopeButtons.Values)
            {
                if (button != null)
                {
                    button.Click += IsotopeButton_Click;
                }
            }
        }

        private Button FindButtonInGrid(int row, int column)
        {
            var grid = IsotopeButtonsGrid as Grid; // 핵종 버튼들이 있는 Grid
            if (grid != null && grid.Children.Count > row)
            {
                var rowGrid = grid.Children[row] as Grid;
                if (rowGrid != null && rowGrid.Children.Count > column)
                {
                    return rowGrid.Children[column] as Button;
                }
            }
            return null;
        }

        public void SetSpectrumViewModel(SpectrumViewModel spectrumVM)
        {
            _spectrumVM = spectrumVM;
            if (_spectrumVM != null)
            {
                _spectrumVM.PropertyChanged += SpectrumViewModel_PropertyChanged;
                SubscribeToIsotopeInfos();
                UpdateIsotopeDisplay();
            }
        }

        private void SpectrumViewModel_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(SpectrumViewModel.IsotopeInfos))
            {
                SubscribeToIsotopeInfos();
                UpdateIsotopeDisplay();
            }
        }

        private void SubscribeToIsotopeInfos()
        {
            if (_spectrumVM?.IsotopeInfos == null) return;

            // 각 IsotopeInfo의 PropertyChanged 이벤트 구독
            foreach (var isotopeInfo in _spectrumVM.IsotopeInfos)
            {
                isotopeInfo.PropertyChanged -= IsotopeInfo_PropertyChanged;
                isotopeInfo.PropertyChanged += IsotopeInfo_PropertyChanged;
            }
        }

        private void IsotopeInfo_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            // IsSelected, IsManualSelection, IsDetected가 변경되면 버튼 색상 업데이트
            if (e.PropertyName == nameof(IsotopeInfo.IsSelected) || 
                e.PropertyName == nameof(IsotopeInfo.IsManualSelection) || 
                e.PropertyName == nameof(IsotopeInfo.IsDetected))
            {
                UpdateIsotopeDisplay();
            }
        }

        private void UpdateIsotopeDisplay()
        {
            if (_spectrumVM?.IsotopeInfos == null) return;
            
            // _isotopeButtons가 초기화되지 않았으면 초기화
            if (_isotopeButtons == null)
            {
                InitializeIsotopeButtons();
            }
            
            // _isotopeButtons가 여전히 null이면 반환
            if (_isotopeButtons == null) return;

            // UI 스레드가 아닌 경우에만 Dispatcher 사용
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.Invoke(() => UpdateIsotopeDisplay());
                return;
            }

            // 모든 핵종 버튼을 기본 상태로 초기화 (탐지되지 않은 핵종)
            foreach (var button in _isotopeButtons.Values)
            {
                if (button != null)
                {
                    button.Foreground = Brushes.Gray;
                    button.BorderBrush = Brushes.Gray;
                    button.Background = Brushes.White;
                }
            }

            // IsotopeInfos에 있는 핵종들에 대해 색상 설정
            foreach (var isotopeInfo in _spectrumVM.IsotopeInfos)
            {
                if (_isotopeButtons.ContainsKey(isotopeInfo.Name))
                {
                    var button = _isotopeButtons[isotopeInfo.Name];
                    if (button != null)
                    {
                        if (isotopeInfo.IsDetected)
                        {
                            // 탐지된 핵종: 테두리/글자 검정색, 배경 흰색
                            button.Foreground = Brushes.Black;
                            button.BorderBrush = Brushes.Black;
                            button.Background = Brushes.White;

                            // 탐지된 핵종이 영상화를 위해 선택되면: 배경 파스텔 빨간색, 테두리/글자 검정 유지
                            if (isotopeInfo.IsSelected == Visibility.Visible)
                            {
                                button.Background = new SolidColorBrush(Color.FromRgb(255, 182, 193)); // 파스텔 빨강 (Light Pink)
                                button.Foreground = Brushes.Black;
                                button.BorderBrush = Brushes.Black;
                            }
                        }
                        else
                        {
                            // 탐지되지 않은 핵종: 기본 회색
                            button.Foreground = Brushes.Gray;
                            button.BorderBrush = Brushes.Gray;
                            button.Background = Brushes.White;

                            // 탐지되지 않은 핵종이 영상화를 위해 선택되면: 배경 파스텔 초록색, 테두리/글자 회색 유지
                            if (isotopeInfo.IsSelected == Visibility.Visible)
                            {
                                button.Background = new SolidColorBrush(Color.FromRgb(144, 238, 144)); // 파스텔 초록 (Light Green)
                                button.Foreground = Brushes.Gray;
                                button.BorderBrush = Brushes.Gray;
                            }
                        }
                    }
                }
            }
        }

        public void UpdateIsotopeDetection(List<string> detectedIsotopes)
        {
            // UI 스레드가 아닌 경우에만 Dispatcher 사용
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.Invoke(() => UpdateIsotopeDetection(detectedIsotopes));
                return;
            }

            // _isotopeButtons가 초기화되지 않았으면 초기화
            if (_isotopeButtons == null)
            {
                InitializeIsotopeButtons();
            }
            
            // _isotopeButtons가 여전히 null이면 반환
            if (_isotopeButtons == null) return;
            
            // 모든 핵종 버튼을 기본 회색으로 초기화
            foreach (var button in _isotopeButtons.Values)
            {
                if (button != null)
                {
                    button.Foreground = Brushes.Gray;
                    button.BorderBrush = Brushes.Gray;
                }
            }

            // 탐지된 핵종들을 검정색으로 표시
            foreach (var isotopeName in detectedIsotopes)
            {
                if (_isotopeButtons.ContainsKey(isotopeName))
                {
                    var button = _isotopeButtons[isotopeName];
                    if (button != null)
                    {
                        button.Foreground = Brushes.Black;
                        button.BorderBrush = Brushes.Black;
                    }
                }
            }
        }
        
        // 테스트용: 특정 핵종을 탐지된 것으로 표시
        public void SimulateIsotopeDetection(string isotopeName)
        {
            // _isotopeButtons가 초기화되지 않았으면 초기화
            if (_isotopeButtons == null)
            {
                InitializeIsotopeButtons();
            }
            
            if (_isotopeButtons != null && _isotopeButtons.ContainsKey(isotopeName))
            {
                var button = _isotopeButtons[isotopeName];
                if (button != null)
                {
                    button.Foreground = Brushes.Black;
                    button.BorderBrush = Brushes.Black;
                }
            }
        }
        
        // 테스트용: 모든 핵종을 기본 상태로 초기화
        public void ResetAllIsotopes()
        {
            // _isotopeButtons가 초기화되지 않았으면 초기화
            if (_isotopeButtons == null)
            {
                InitializeIsotopeButtons();
            }
            
            // _isotopeButtons가 여전히 null이면 반환
            if (_isotopeButtons == null) return;
            
            foreach (var button in _isotopeButtons.Values)
            {
                if (button != null)
                {
                    button.Foreground = Brushes.Gray;
                    button.BorderBrush = Brushes.Gray;
                }
            }
        }

        // 핵종 버튼 클릭 이벤트 핸들러
        private void IsotopeButton_Click(object sender, RoutedEventArgs e)
        {
            // 이벤트가 중복 발생하는 것을 방지하기 위해 Handled 체크
            if (e.Handled)
                return;

            if (sender is Button button && button.Content is string isotopeName)
            {
                // 빈 버튼은 무시
                if (string.IsNullOrEmpty(isotopeName))
                    return;

                System.Diagnostics.Debug.WriteLine($"IsotopeButton_Click: isotopeName={isotopeName}, _spectrumVM={_spectrumVM != null}");

                // IsotopeInfos에서 현재 선택 상태 확인
                bool isCurrentlySelected = false;
                if (_spectrumVM?.IsotopeInfos != null)
                {
                    foreach (var isotopeInfo in _spectrumVM.IsotopeInfos)
                    {
                        if (isotopeInfo.Name == isotopeName && isotopeInfo.IsSelected == Visibility.Visible)
                        {
                            isCurrentlySelected = true;
                            break;
                        }
                    }
                }
                
                if (isCurrentlySelected)
                {
                    // 이미 선택된 상태면 해제
                    // SpectrumViewModel에 핵종 해제 알림
                    System.Diagnostics.Debug.WriteLine($"핵종 해제 호출: {isotopeName}");
                    _spectrumVM?.OnIsotopeManualDeselection(isotopeName);
                }
                else
                {
                    // 선택되지 않은 상태면 선택
                    // SpectrumViewModel에 핵종 선택 알림
                    System.Diagnostics.Debug.WriteLine($"핵종 선택 호출: {isotopeName}");
                    _spectrumVM?.OnIsotopeManualSelection(isotopeName);
                }
                
                // 이벤트 처리 완료 표시
                e.Handled = true;
            }
            else
            {
                System.Diagnostics.Debug.WriteLine($"IsotopeButton_Click: sender가 Button이 아니거나 Content가 string이 아님. sender={sender?.GetType()}, Content={((sender as Button)?.Content)?.GetType()}");
            }
        }
    }
}
