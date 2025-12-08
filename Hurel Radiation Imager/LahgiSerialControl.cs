using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Diagnostics;
using log4net;

namespace HUREL.Compton
{
    public static class LahgiSerialControl
    {

   
        private static readonly ILog logger = LogManager.GetLogger(nameof(LahgiSerialControl));

        static SerialPort Serial = new SerialPort();

        public static List<string> PortsName = new List<string>();

        static LahgiSerialControl()
        {
            PortsName = new List<string>(SerialPort.GetPortNames());
            if (PortsName.Count == 0)
            {
                logger.Warn("There is no COM port");
                SelectedPortName = null; // 명시적으로 null 설정
            }
            else
            {
                SelectedPortName = PortsName[0]; //COM3은 PortName 0, COM4는 PortName 2
            }
        }   
        public static void UpdatePortsName()
        {
            PortsName = new List<string>(SerialPort.GetPortNames());
        }

        public static int Baudrate = 9600;
        public static string? SelectedPortName;
        public static bool StartCommunication()
        {
            // COM 포트가 없으면 false 반환
            if (PortsName.Count == 0 || SelectedPortName == null)
            {
                logger.Warn("No COM port available for communication");
                logger.Error("시리얼 포트 연결 실패: 사용 가능한 COM 포트가 없습니다");
                return false;
            }
            
            logger.Info($"COM 포트 검색 시작 (Baudrate: {Baudrate})");
            //240315
            bool result = CheckPortStart();
            
            if (result)
            {
                logger.Info($"=== 시리얼 포트 연결 성공: {SelectedPortName} ===");
            }
            else
            {
                logger.Error("=== 시리얼 포트 연결 실패: 모든 포트에서 연결에 실패했습니다 ===");
            }
            
            return result;

            if (Serial.PortName != SelectedPortName)
            {
                Serial.Close();
            }
            if (Serial.IsOpen == true)
            {
                return true;
            }

            Serial.PortName = SelectedPortName!;
            Serial.BaudRate = Baudrate;
            Serial.Parity = Parity.None;
            Serial.ReadTimeout = 2000;
            try
            {
                Serial.Open();
            }
            catch (Exception e)
            {
                logger.Error(e.Message);
                
                return false;
            }
            

            return true;
        }

        //240315 : 모든 포트 확인하여 연결
        private static bool CheckPortStart()
        {
            bool bfind = false;
            logger.Info($"총 {PortsName.Count}개의 포트를 확인합니다");
            
            foreach (var port in PortsName)
            {
                logger.Info($"포트 {port} 연결 시도 중...");
                SelectedPortName = port;

                if (Serial.PortName != SelectedPortName)
                {
                    Serial.Close();
                }
                if (Serial.IsOpen == true)
                {
                    logger.Info($"포트 {port}는 이미 열려있습니다");
                    return true;
                }

                Serial.PortName = SelectedPortName!;
                Serial.BaudRate = Baudrate;
                Serial.Parity = Parity.None;
                Serial.WriteTimeout = 2000;
                Serial.ReadTimeout = 2000;
                try
                {
                    Serial.Open();
                    logger.Info($"포트 {port} 열기 성공");

                    if (CheckParams())
                    {
                        logger.Info($"포트 {port} 파라미터 확인 성공");
                        bfind = true;
                        break;
                    }
                    else
                    {
                        logger.Warn($"포트 {port} 파라미터 확인 실패 - 다음 포트 시도");
                        Serial.Close();
                    }
                }
                catch (Exception e)
                {
                    logger.Error($"포트 {port} 연결 실패: {e.Message}");
                    try
                    {
                        if (Serial.IsOpen)
                        {
                            Serial.Close();
                        }
                    }
                    catch { }
                }
            }

            if (bfind)
            {
                logger.Info($"최종 연결된 포트: {SelectedPortName}");
                return true;
            }
            else
            {
                logger.Error("모든 포트에서 연결에 실패했습니다");
                return false;
            }
        }
        public static void StopCommunication()
        {
            if(Serial.IsOpen == false)
            {
                logger.Info("시리얼 포트가 이미 닫혀있습니다");
                return;
            }
            
            logger.Info($"시리얼 포트 연결 종료: {Serial.PortName}");
            Serial.Close();
            logger.Info("시리얼 포트 연결 종료 완료");
        }

        //240315 : CheckParams() 결과 반환
        public static bool CheckParams()
        {
            if (Serial.IsOpen)
            {
                try
                {
                    logger.Debug($"포트 {Serial.PortName} 파라미터 확인 요청 전송");
                    Serial.WriteLine("check");
                    if (Serial.IsOpen)
                    {
                        try
                        {
                            string response = Serial.ReadLine();
                            logger.Debug($"포트 {Serial.PortName} 응답 수신: {response}");
                            ReadCheck(response);
                            logger.Info($"포트 {Serial.PortName} 파라미터 확인 성공");
                            return true;
                        }
                        catch(Exception e)
                        {
                            logger.Error($"포트 {Serial.PortName} Readline 실패: {e.ToString()}");
                            return false;
                        }
                    }
                    else
                    {
                        logger.Warn($"포트 {Serial.PortName}가 열려있지 않습니다");
                        return false;
                    }
                }
                catch (Exception e)
                {
                    logger.Error($"포트 {Serial.PortName} 파라미터 확인 중 오류: {e.Message}");
                    return false;
                }
            }
            else
            {
                logger.Warn("시리얼 포트가 열려있지 않아 파라미터 확인을 수행할 수 없습니다");
                return false;
            }
        }
        public static void SetFPGA(bool on)
        {
            if (Serial.IsOpen)
            {
                try
                {
                    if (on)
                    {
                        Serial.WriteLine("setfpga:on");
                    }
                    else
                    {
                        Serial.WriteLine("setfpga:off");
                    }
                    ReadCheck(Serial.ReadLine());
                }
                catch (System.TimeoutException)
                {
                    // 프로그램 종료 시 시리얼 포트 타임아웃은 무시
                }
                catch (System.InvalidOperationException)
                {
                    // 시리얼 포트가 이미 닫혔으면 무시
                }
            }
        }

        public static void SetHvMoudle(bool on)
        {
            if (Serial.IsOpen)
            {
                try
                {
                    if (on)
                    {
                        Serial.WriteLine("sethv:on");
                    }
                    else
                    {
                        Serial.WriteLine("sethv:off");

                    }
                    
                    // 최대 반복 횟수 제한 (무한 루프 방지)
                    int maxIterations = 7;
                    int iteration = 0;
                    string s = Serial.ReadLine();
                    
                    while(s != "done\r" && iteration < maxIterations)
                    {
                        iteration++;
                        // 안전하게 파싱 - 숫자가 아닌 경우 기본값 사용
                        if (double.TryParse(s, System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out double voltage))
                        {
                            HvModuleVoltage = voltage;
                        }
                        try
                        {
                            s = Serial.ReadLine();
                        }
                        catch (System.TimeoutException)
                        {
                            // 타임아웃 발생 시 루프 종료
                            logger.Warn($"SetHvMoudle 타임아웃 발생 (반복 {iteration}회)");
                            break;
                        }
                        catch (System.InvalidOperationException)
                        {
                            // 시리얼 포트가 닫혔으면 루프 종료
                            logger.Warn($"SetHvMoudle 중 시리얼 포트 닫힘 (반복 {iteration}회)");
                            break;
                        }
                    }
                    
                    if (iteration >= maxIterations)
                    {
                        logger.Warn($"SetHvMoudle 최대 반복 횟수 도달 ({maxIterations}회)");
                    }

                    try
                    {
                        ReadCheck(Serial.ReadLine());
                    }
                    catch (System.TimeoutException)
                    {
                        // 타임아웃 발생 시 무시
                        logger.Debug("SetHvMoudle ReadCheck 타임아웃 (무시)");
                    }
                    catch (System.InvalidOperationException)
                    {
                        // 시리얼 포트가 닫혔으면 무시
                        logger.Debug("SetHvMoudle ReadCheck 중 시리얼 포트 닫힘 (무시)");
                    }
                }
                catch (System.TimeoutException)
                {
                    // 프로그램 종료 시 시리얼 포트 타임아웃은 무시
                    logger.Debug("SetHvMoudle 타임아웃 (무시)");
                }
                catch (System.InvalidOperationException)
                {
                    // 시리얼 포트가 이미 닫혔으면 무시
                    logger.Debug("SetHvMoudle 중 시리얼 포트 닫힘 (무시)");
                }
            }
        }

        public static void SetSwitch(int num, bool on)
        {
            if (Serial.IsOpen)
            {
                try
                {
                    string s;
                    if (on)
                    {
                        s = "on";
                    }
                    else
                    {
                        s = "off";
                    }

                    Serial.WriteLine($"setswitch:{num}:{s}");
                    ReadCheck(Serial.ReadLine());
                }
                catch (System.TimeoutException)
                {
                    // 프로그램 종료 시 시리얼 포트 타임아웃은 무시
                }
                catch (System.InvalidOperationException)
                {
                    // 시리얼 포트가 이미 닫혔으면 무시
                }
            }

        }
        private static void ReadCheck(string s)
        {
            string[] parameters = s.Split(',');

            if (parameters[0].Split(':')[0] != "hvvolt")
            {
                return;
            }

            HvModuleVoltage = Convert.ToDouble(parameters[0].Split(':')[1]);
            HvModuleCurrent = Convert.ToDouble(parameters[1].Split(':')[1]);
            BatteryVoltage = Convert.ToDouble(parameters[2].Split(':')[1]);
            if (parameters[3].Split(':')[1] == "on")
            {
                IsFPGAOn = true;
            }
            else
            {
                IsFPGAOn = false;
            }

            for (int i = 0; i < 6; ++i)
            {
                if (parameters[4 + i].Split(':')[1] == "on")
                {
                    IsSwitchOn[i] = true;
                }
                else
                {
                    IsSwitchOn[i] = false;
                }
            }


        }

        static public bool IsFPGAOn;
        static public bool[] IsSwitchOn = new bool[6];
        static public double HvModuleVoltage;
        static public double HvModuleCurrent;
        static public double BatteryVoltage;
    }
}
