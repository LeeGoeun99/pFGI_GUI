using System;
using System.Globalization;
using System.Windows.Data;

namespace HUREL_Imager_GUI.Converters
{
    public class ButtonSizeConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value is double windowWidth)
            {
                // 기본 창 너비 1422px에서 버튼 크기 120px 기준
                // 창이 커지면 버튼도 비례적으로 커짐
                double baseWidth = 1422.0;
                double baseButtonSize = 120.0;
                
                // 최소 버튼 크기 80px, 최대 버튼 크기 200px로 제한
                double calculatedButtonSize = (windowWidth / baseWidth) * baseButtonSize;
                return Math.Max(80.0, Math.Min(200.0, calculatedButtonSize));
            }
            
            return 120.0; // 기본값
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
