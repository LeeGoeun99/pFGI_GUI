using System;
using System.Configuration;

namespace HUREL_Imager_GUI
{
    /// <summary>
    /// 테스트 모드 설정을 관리하는 클래스
    /// 검출기와 신호처리 시스템이 연결되지 않아도 내부 코드를 테스트할 수 있도록 합니다.
    /// </summary>
    public static class TestModeConfig
    {
        private static bool? _isTestMode = null;
        
        /// <summary>
        /// 테스트 모드 활성화 여부
        /// App.config의 "TestMode" 설정값을 읽어옵니다.
        /// </summary>
        public static bool IsTestMode
        {
            get
            {
                if (_isTestMode == null)
                {
                    try
                    {
                        var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                        var appSetting = configFile.AppSettings.Settings;
                        
                        if (appSetting["TestMode"] == null)
                        {
                            appSetting.Add("TestMode", "false");
                            configFile.Save(ConfigurationSaveMode.Modified);
                            ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
                            _isTestMode = false;
                        }
                        else
                        {
                            _isTestMode = appSetting["TestMode"].Value.ToLower() == "true" || 
                                         appSetting["TestMode"].Value == "1";
                        }
                    }
                    catch (Exception)
                    {
                        _isTestMode = false;
                    }
                }
                return _isTestMode.Value;
            }
            set
            {
                try
                {
                    var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                    var appSetting = configFile.AppSettings.Settings;
                    
                    if (appSetting["TestMode"] == null)
                    {
                        appSetting.Add("TestMode", value.ToString());
                    }
                    else
                    {
                        appSetting["TestMode"].Value = value.ToString();
                    }
                    
                    configFile.Save(ConfigurationSaveMode.Modified);
                    ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
                    _isTestMode = value;
                }
                catch (Exception)
                {
                    // 설정 저장 실패 시 메모리 값만 업데이트
                    _isTestMode = value;
                }
            }
        }
        
        /// <summary>
        /// 테스트 모드 상태를 리셋합니다 (다시 App.config에서 읽어옵니다)
        /// </summary>
        public static void Reset()
        {
            _isTestMode = null;
        }
    }
}

