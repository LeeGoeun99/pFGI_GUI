using System;
using System.Globalization;
using System.Windows.Data;

namespace HUREL_Imager_GUI.Converters
{
    public class FontSizeConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value is double windowWidth)
            {
                // 기본 창 너비 1422px에서 폰트 크기 16px 기준
                // 창이 커지면 폰트도 비례적으로 커짐
                double baseWidth = 1422.0;
                double baseFontSize = 16.0;
                
                // 최소 폰트 크기 12px, 최대 폰트 크기 24px로 제한
                double calculatedFontSize = (windowWidth / baseWidth) * baseFontSize;
                return Math.Max(12.0, Math.Min(24.0, calculatedFontSize));
            }
            
            return 16.0; // 기본값
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
