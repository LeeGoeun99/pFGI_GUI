using System;
using System.Configuration;

namespace HUREL_Imager_GUI
{
    /// <summary>
    /// App.config (또는 exe.config)의 appSettings 섹션에서 TestMode 값을 읽어
    /// 프로그램 전역에서 테스트 모드 여부를 확인하기 위한 헬퍼 클래스
    /// </summary>
    public static class TestModeConfig
    {
        /// <summary>
        /// 테스트 모드 여부
        /// - "true" / "1" (대소문자 무관) 이면 true
        /// - 그 외 또는 미설정/null 이면 false
        /// </summary>
        public static bool IsTestMode
        {
            get
            {
                string? value = ConfigurationManager.AppSettings["TestMode"];
                if (string.IsNullOrWhiteSpace(value))
                {
                    return false;
                }

                value = value.Trim();
                return string.Equals(value, "true", StringComparison.OrdinalIgnoreCase)
                    || value == "1";
            }
        }
    }
}


