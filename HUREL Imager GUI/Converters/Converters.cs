using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;
using System.Windows;
using HUREL.Compton;

namespace HUREL_Imager_GUI.Converters
{
    public class BooleanToReverseConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
         => !(bool?)value ?? true;

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
         => !(value as bool?);
    }
    public class RTReconModeToBooleanConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            string parameterString = parameter as string;
            if (parameterString == null)
            {
                return DependencyProperty.UnsetValue;
            }

            if (Enum.IsDefined(value.GetType(), value) == false)
            {
                return DependencyProperty.UnsetValue;
            }

            object parameterValue = Enum.Parse(value.GetType(), parameterString);

            return parameterValue.Equals(value);
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            string parameterString = parameter as string;

            if (parameterString == null)
            {
                return DependencyProperty.UnsetValue;
            }

            return Enum.Parse(targetType, parameterString);
        }
    }
    public class TimespaneToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var timeSpan = (TimeSpan)value;
            return timeSpan.ToString();
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value == null || !(value is string strValue))
            {
                return DependencyProperty.UnsetValue;
            }

            if (double.TryParse(strValue, NumberStyles.Any, CultureInfo.InvariantCulture, out double seconds))
            {
                return TimeSpan.FromSeconds(seconds);
            }

            return DependencyProperty.UnsetValue;
        }
    }

    //231109-1 sbkwon
    public class eReconTypeConverter : IValueConverter
    {
        public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
            => value is HUREL_Imager_GUI.ViewModel.eReconType type && parameter is HUREL_Imager_GUI.ViewModel.eReconType mask ? type == mask : false;

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
            => value is bool isChecked && parameter is HUREL_Imager_GUI.ViewModel.eReconType mask ? mask : DependencyProperty.UnsetValue;
    }

    //231109-1 sbkwon
    public class eReconSpaceConverter : IValueConverter
    {
        public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
            => value is HUREL_Imager_GUI.ViewModel.eReconSpace type && parameter is HUREL_Imager_GUI.ViewModel.eReconSpace mask ? type == mask : false;

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
            => value is bool isChecked && parameter is HUREL_Imager_GUI.ViewModel.eReconSpace mask ? mask : DependencyProperty.UnsetValue;
    }

    //240326
    public class eReconOptionConverter : IValueConverter
    {
        public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
            => value is HUREL_Imager_GUI.ViewModel.eReconOption type && parameter is HUREL_Imager_GUI.ViewModel.eReconOption mask ? type == mask : false;

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
            => value is bool isChecked && parameter is HUREL_Imager_GUI.ViewModel.eReconOption mask ? mask : DependencyProperty.UnsetValue;
    }



    //240110
    public class e3DViewTypeConverter : IValueConverter
    {
        public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
            => value is HUREL_Imager_GUI.ViewModel.e3DViewType type && parameter is HUREL_Imager_GUI.ViewModel.e3DViewType mask ? type == mask : false;

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
            => value is bool isChecked && parameter is HUREL_Imager_GUI.ViewModel.e3DViewType mask ? mask : DependencyProperty.UnsetValue;
    }

    //231100-GUI sbkwon
    public class eMeasuremetTypeConverter : IValueConverter
    {
        public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var result = value is HUREL_Imager_GUI.ViewModel.eMeasuremetType type && parameter is HUREL_Imager_GUI.ViewModel.eMeasuremetType mask ? type == mask : false;
            
            // 로그 추가
            System.Diagnostics.Debug.WriteLine($"eMeasuremetTypeConverter.Convert - value: {value}, parameter: {parameter}, result: {result}");
            
            return result;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var result = value is bool isChecked && parameter is HUREL_Imager_GUI.ViewModel.eMeasuremetType mask ? mask : DependencyProperty.UnsetValue;
            
            // 로그 추가
            System.Diagnostics.Debug.WriteLine($"eMeasuremetTypeConverter.ConvertBack - value: {value}, parameter: {parameter}, result: {result}");
            
            return result;
        }
    }

    // 측정 모드 Converter
    public class eMeasurementModeConverter : IValueConverter
    {
        public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            // parameter가 string인 경우 enum으로 변환
            if (parameter is string paramString)
            {
                if (Enum.TryParse<HUREL_Imager_GUI.ViewModel.eMeasurementMode>(paramString, out var paramMode))
                {
                    var result = value is HUREL_Imager_GUI.ViewModel.eMeasurementMode type && type == paramMode;
                    System.Diagnostics.Debug.WriteLine($"eMeasurementModeConverter.Convert - value: {value}, parameter: {parameter}, result: {result}");
                    return result;
                }
            }
            
            var result2 = value is HUREL_Imager_GUI.ViewModel.eMeasurementMode type2 && parameter is HUREL_Imager_GUI.ViewModel.eMeasurementMode mask ? type2 == mask : false;
            System.Diagnostics.Debug.WriteLine($"eMeasurementModeConverter.Convert - value: {value}, parameter: {parameter}, result: {result2}");
            return result2;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            // parameter가 string인 경우 enum으로 변환
            if (parameter is string paramString)
            {
                if (Enum.TryParse<HUREL_Imager_GUI.ViewModel.eMeasurementMode>(paramString, out var paramMode))
                {
                    var result = value is bool isChecked && isChecked ? paramMode : DependencyProperty.UnsetValue;
                    System.Diagnostics.Debug.WriteLine($"eMeasurementModeConverter.ConvertBack - value: {value}, parameter: {parameter}, result: {result}");
                    return result;
                }
            }
            
            var result2 = value is bool isChecked2 && parameter is HUREL_Imager_GUI.ViewModel.eMeasurementMode mask ? mask : DependencyProperty.UnsetValue;
            System.Diagnostics.Debug.WriteLine($"eMeasurementModeConverter.ConvertBack - value: {value}, parameter: {parameter}, result: {result2}");
            return result2;
        }
    }

    // 고장 검사 타입 Converter
    public class eFaultCheckTypeConverter : IValueConverter
    {
        public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var result = value is HUREL_Imager_GUI.ViewModel.eFaultCheckType type && parameter is HUREL_Imager_GUI.ViewModel.eFaultCheckType mask ? type == mask : false;
            
            // 로그 추가
            System.Diagnostics.Debug.WriteLine($"eFaultCheckTypeConverter.Convert - value: {value}, parameter: {parameter}, result: {result}");
            
            return result;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var result = value is bool isChecked && parameter is HUREL_Imager_GUI.ViewModel.eFaultCheckType mask ? mask : DependencyProperty.UnsetValue;
            
            // 로그 추가
            System.Diagnostics.Debug.WriteLine($"eFaultCheckTypeConverter.ConvertBack - value: {value}, parameter: {parameter}, result: {result}");
            
            return result;
        }
    }

    // RGB 타입 Converter
    public class eRGBTypeConverter : IValueConverter
    {
        public object? Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var result = value is HUREL_Imager_GUI.ViewModel.eRGBType type && parameter is HUREL_Imager_GUI.ViewModel.eRGBType mask ? type == mask : false;
            
            // 로그 추가
            System.Diagnostics.Debug.WriteLine($"eRGBTypeConverter.Convert - value: {value}, parameter: {parameter}, result: {result}");
            
            return result;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var result = value is bool isChecked && parameter is HUREL_Imager_GUI.ViewModel.eRGBType mask ? mask : DependencyProperty.UnsetValue;
            
            // 로그 추가
            System.Diagnostics.Debug.WriteLine($"eRGBTypeConverter.ConvertBack - value: {value}, parameter: {parameter}, result: {result}");
            
            return result;
        }
    }

    public class EspectModeConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            string parameterString = parameter as string;
            if (parameterString == null)
            {
                return DependencyProperty.UnsetValue;
            }

            if (Enum.IsDefined(value.GetType(), value) == false)
            {
                return DependencyProperty.UnsetValue;
            }

            object parameterValue = Enum.Parse(value.GetType(), parameterString);

            return parameterValue.Equals(value);
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            string parameterString = parameter as string;

            if (parameterString == null)
            {
                return DependencyProperty.UnsetValue;
            }

            return Enum.Parse(targetType, parameterString);
        }
    }
    public class ObservalbeDoubleToChannelNameConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value == null)
            {
                List<string> non = new List<string>();
                non.Add("PMT CorrMat");
                return non;
            }
            var channelGain = (ObservableCollection<double>)value;
            ObservableCollection<string> CorrMatNameAndGain = new ObservableCollection<string>();
            int i = 0;
            CorrMatNameAndGain.Add("PMT CorrMat");
            foreach (double d in channelGain)
            {
                CorrMatNameAndGain.Add("CorrMat[" + i + "]: " + d.ToString("F2"));
                i++;
            }
            return CorrMatNameAndGain;
        }
        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    [ValueConversion(typeof(bool), typeof(bool))]
    public class InverseBooleanConverter : IValueConverter
    {
        #region IValueConverter Members

        public object Convert(object value, Type targetType, object parameter,
            System.Globalization.CultureInfo culture)
        {
            if (targetType != typeof(bool))
                throw new InvalidOperationException("The target must be a boolean");

            return !(bool)value;
        }

        public object ConvertBack(object value, Type targetType, object parameter,
            System.Globalization.CultureInfo culture)
        {
            throw new NotSupportedException();
        }

        #endregion
    }

    public class BoolToBrushConverter : BoolToValueConverter<String> { }
    public class BoolToStringConverter : BoolToValueConverter<String> { }
    public class BoolToVisibilityConverter : BoolToValueConverter<Visibility> { }
    public class BoolToValueConverter<T> : IValueConverter
    {
        public T FalseValue { get; set; }
        public T TrueValue { get; set; }

        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if (value == null)
                return FalseValue;
            else
                return (bool)value ? TrueValue : FalseValue;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return value != null ? value.Equals(TrueValue) : false;
        }
    }


}
