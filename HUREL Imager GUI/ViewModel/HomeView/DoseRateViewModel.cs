using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using log4net;
using log4net.Core;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Claims;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Controls;

namespace HUREL_Imager_GUI.ViewModel
{
    //230920 sbkwon : DoseRate Alarm Level
    public enum enAlarm
    {
        enAlarmNone,
        enAlarm3,
        enAlarm4,
        enAlarm5,
        enPeakToValleyWarning,  //240228-PeakToValley 문제
        enFaultDiagnosisWarning, //240228 고장검사 측정
        enFaultDiagnosisError,  //240228 고장 검사 PMT 문제
    }

    //240206-Gain :
    public enum enModuleType
    {
        enScatter,
        enAbsorber,
    }

    //230920 sbkwon : DoseRate Alarm Event
    public class AlarmEventArgs : EventArgs
    {
        public enAlarm AlarmStatus { get; private set; }
        public AlarmEventArgs(enAlarm alarm)
        {
            AlarmStatus = alarm;
        }
    }

    public class DoseRateViewModel : ViewModelBase
    {
        public static EventHandler? AlarmUpdate;     //230920 sbkwon : DoseRate Alarm EventHandler

        public TopButtonViewModel TopButtonVM { get; set; } //240311

        //230920 sbkwon : DoseRate Alarm Invoke
        public static void AlarmUpdateInvoke(object? obj, enAlarm state)
        {
            Task.Run(() => { AlarmUpdate?.Invoke(obj, new AlarmEventArgs(state)); });
        }

        private bool _isInitialized = false;

        public DoseRateViewModel()
        {
            // 초기화 완료 후 이벤트 등록
            _isInitialized = true;
            LahgiApi.StatusUpdate += StatusUpdate;

        }

        private bool bDoseRateCal = false;

        Mutex StatusUpdateMutex = new Mutex();
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {
            // 초기화가 완료되지 않았으면 처리하지 않음
            if (!_isInitialized) return;

            if (!StatusUpdateMutex.WaitOne(0))
            {
                return;
            }
            try
            {

                if (eventArgs is LahgiApiEnvetArgs)
                {
                    LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                    if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading)
                    {
                        var espect = LahgiApi.GetScatterSumSpectrumByTime(0);
                        if (espect != null)
                        {
                            (double dose, double std) = espect.GetAmbientDose(DoseCalcTime);
                            if (dose > 0.01)
                            {
                                DoseLogScale = Math.Log10(dose * 1000);
                            }
                            else
                            {
                                DoseLogScale = 0;
                            }
                        }
                    }

                    if (/*bDoseRateCal && */lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum && LahgiApi.TimerBoolSpectrum && TopButtonVM?.FaultDiagnosis == false)
                    {
                        var espect = LahgiApi.GetScatterSumSpectrumByTime(DoseCalcTime);
                        if (espect != null)
                        {
                            //(double dose, double std) = espect.GetAmbientDose(LahgiApi.ElapsedTime > DoseCalcTime ? DoseCalcTime : LahgiApi.ElapsedTime);    //231017 : sbkwon - if DoseCalcTime < 측정시간 then 측정시간 대입

                            if (LahgiApi.ElapsedTime >= 5) //240311 : 단위 초
                            {
                                (double dose, double std) = espect.GetAmbientDose(LahgiApi.ElapsedTime > DoseCalcTime ? DoseCalcTime : LahgiApi.ElapsedTime);
                                //Trace.WriteLine($"선량률 : {dose}");
                                if (dose > 0.01)
                                {
                                    DoseLogScale = dose;//Math.Log10(dose * 1000);
                                }
                                else
                                {
                                    DoseLogScale = 0;
                                }
                            }
                            //240311 : 선량률 5초 이후부터 표시되게
                            else
                            {
                            //    (double dose, double std) = espect.GetAmbientDose(1);
                            //    //Trace.WriteLine($"선량률 : {dose}");
                            //    if (dose > 0.01)
                            //    {
                            //        DoseLogScale = dose;//Math.Log10(dose * 1000);
                            //    }
                            //    else
                            //    {
                                    DoseLogScale = 0;
                            //    }
                            }
                        }
                    }

                    bDoseRateCal = !bDoseRateCal;
                }
            }
            finally
            {
                StatusUpdateMutex.ReleaseMutex();
            }
        }

        private double doseLogScale = Math.Log10(30 * 1000);
        public double DoseLogScale
        {
            get { return doseLogScale; }
            set
            {

                //230921 sbkwon : doseRate Alarm 구분
                enAlarm enLevel = enAlarm.enAlarmNone;
                /*
                if (value >= 1 && value < 2.5) { enLevel = enAlarm.enAlarm3; }
                else if (value >= 2.5 && value < 3.5) { enLevel = enAlarm.enAlarm4; }
                else if (value >= 3.5) { enLevel = enAlarm.enAlarm5; }
                */
                if (value >= 0.25 && value < 5) { enLevel = enAlarm.enAlarm3; } //yellow zone
                //else if (value >= 3.5 && value < 4) { enLevel = enAlarm.enAlarm5; } //orange zone
                else if (value >= 5) { enLevel = enAlarm.enAlarm5; } //red zone

                //240206-Gain : PeakToValley 우선순위 높음.
                if (oldAlarmLeve != enLevel && enLevel <= enAlarm.enAlarm5 && oldAlarmLeve <= enAlarm.enAlarm5)
                {
                    AlarmUpdateInvoke(null, enLevel);

                    oldAlarmLeve = enLevel;
                }

                doseLogScale = value;
                DoseScale = value;
                DoseScaleText = value.ToString("0.00") + " μSv/hr";
                OnPropertyChanged(nameof(DoseLogScale));
            }
        }

        /// <summary>
        /// 240206-Gain : PeakToValley 측정 결과 문제 있을 경우 호출
        /// </summary>
        /// <param name="module"> enScatter or enAbsorber</param>
        /// <param name="ch">channel number(0 - 3)</param>
        public void SetPeakToValleyWarning()
        {
            if (oldAlarmLeve != enAlarm.enPeakToValleyWarning)
                AlarmUpdateInvoke(null, enAlarm.enPeakToValleyWarning);

            oldAlarmLeve = enAlarm.enPeakToValleyWarning;
        }

        //240206-Gain : alarm clear
        public void ClearPeakToValleyWarning()
        {
            if (oldAlarmLeve == enAlarm.enPeakToValleyWarning)
            {
                AlarmUpdateInvoke(null, enAlarm.enAlarmNone);

                oldAlarmLeve = enAlarm.enAlarmNone;
            }
        }

        //240228 고장 검사 : 정상적인 검사가 진행안된 경우 - 측정을 더 오래하도록
        public void SetFaultDiagnosisWarning()
        {
            if (oldAlarmLeve != enAlarm.enFaultDiagnosisWarning)
                AlarmUpdateInvoke(null, enAlarm.enFaultDiagnosisWarning);

            oldAlarmLeve = enAlarm.enFaultDiagnosisWarning;
        }

        //240228 고장 검사 : PMT 문제 발생할 경우
        public void SetFaultDiagnosisError()
        {
            if (oldAlarmLeve != enAlarm.enFaultDiagnosisError)
                AlarmUpdateInvoke(null, enAlarm.enFaultDiagnosisError);

            oldAlarmLeve = enAlarm.enFaultDiagnosisError;
        }

        public void ClearFaultDiagnosis()
        {
            if (oldAlarmLeve == enAlarm.enFaultDiagnosisWarning || oldAlarmLeve == enAlarm.enFaultDiagnosisError)
            {
                AlarmUpdateInvoke(null, enAlarm.enAlarmNone);

                oldAlarmLeve = enAlarm.enAlarmNone;
            }
        }

        //240228 DoseRate clear
        public void ClearDoseRateAlarm()
        {
            AlarmUpdateInvoke(null, enAlarm.enAlarmNone);

            oldAlarmLeve = enAlarm.enAlarmNone;
        }

        //231100-GUI 
        private string _doseScaleText = "";
        public string DoseScaleText
        {
            get => _doseScaleText;
            set
            {
                _doseScaleText = value;
                OnPropertyChanged(nameof(DoseScaleText));
            }
        }

        //231100-GUI sbkwon : UI Range 변환
        private double _doseScale = 0;
        public double DoseScale
        {
            get => _doseScale;
            set
            {
                double reValue;
                if (value <= 0.25)
                    reValue = 3 * value / 0.25;
                else if (value <= 5)
                    reValue = 4 * (value - 0.25) / 4.75 + 3;
                else
                    reValue = 3 * (value - 5) / 5 + 7;   //max 10

                _doseScale = reValue;
                OnPropertyChanged(nameof(DoseScale));
            }
        }

        private uint doseCalcTime = 60; //dose rate 누적시간
        public uint DoseCalcTime
        {
            get
            {
                return doseCalcTime;
            }
            set
            {
                doseCalcTime = value;
                OnPropertyChanged(nameof(DoseCalcTime));
            }
        }

        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= StatusUpdate;

            LogManager.GetLogger(typeof(DoseRateViewModel)).Info("Unhandle");
        }

        private enAlarm oldAlarmLeve = enAlarm.enAlarmNone;
    }
}