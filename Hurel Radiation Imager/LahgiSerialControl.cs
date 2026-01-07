using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Diagnostics;
using log4net;
using System.Configuration;

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
            // 테스트 모드에서는 실제 시리얼 포트 연결을 수행하지 않고
            // 성공한 것처럼 동작시켜 내부 로직만 테스트할 수 있게 한다.
            string? testModeValue = ConfigurationManager.AppSettings["TestMode"];
            bool isTestMode =
                !string.IsNullOrWhiteSpace(testModeValue) &&
                (string.Equals(testModeValue.Trim(), "true", StringComparison.OrdinalIgnoreCase)
                 || testModeValue.Trim() == "1");

            if (isTestMode)
            {
                logger.Info("StartCommunication() called in Test Mode - skipping serial port initialization.");
                return true;
            }

            logger.Info("=== StartCommunication() 함수 시작 ===");
            logger.Info($"현재 포트 목록 개수: {PortsName.Count}, SelectedPortName: {SelectedPortName ?? "null"}");
            
            // COM 포트가 없으면 false 반환
            if (PortsName.Count == 0 || SelectedPortName == null)
            {
                logger.Warn("No COM port available for communication");
                logger.Error("시리얼 포트 연결 실패: 사용 가능한 COM 포트가 없습니다");
                return false;
            }
            
            logger.Info($"COM 포트 검색 시작 (Baudrate: {Baudrate})");
            logger.Info("CheckPortStart() 함수 호출 전");
            
            //240315
            bool result = false;
            try
            {
                result = CheckPortStart();
                logger.Info($"CheckPortStart() 함수 호출 완료, 결과: {result}");
            }
            catch (Exception ex)
            {
                logger.Error($"CheckPortStart() 함수 실행 중 예외 발생: {ex.GetType().Name} - {ex.Message}");
                logger.Error($"스택 트레이스: {ex.StackTrace}");
                result = false;
            }
            
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
            logger.Info("=== CheckPortStart() 함수 진입 ===");
            bool bfind = false;
            logger.Info($"총 {PortsName.Count}개의 포트를 확인합니다");
            
            if (PortsName.Count == 0)
            {
                logger.Warn("CheckPortStart: 포트 목록이 비어있습니다");
                return false;
            }
            
            logger.Info($"확인할 포트 목록: {string.Join(", ", PortsName)}");
            
            foreach (var port in PortsName)
            {
                logger.Info($"포트 {port} 연결 시도 중...");
                SelectedPortName = port;

                // 이전 포트 연결 정리
                try
                {
                    if (Serial.IsOpen)
                    {
                        if (Serial.PortName != SelectedPortName)
                        {
                            logger.Debug($"이전 포트 {Serial.PortName} 닫기");
                            Serial.Close();
                            System.Threading.Thread.Sleep(200); // 포트 닫기 후 대기
                        }
                        else
                        {
                            logger.Info($"포트 {port}는 이미 열려있습니다");
                            // 이미 열려있어도 파라미터 확인 필요
                            if (CheckParams())
                            {
                                logger.Info($"포트 {port} 파라미터 확인 성공");
                                return true;
                            }
                            else
                            {
                                logger.Warn($"포트 {port}는 열려있지만 파라미터 확인 실패 - 재연결 시도");
                                Serial.Close();
                                System.Threading.Thread.Sleep(500); // 재연결 전 대기
                            }
                        }
                    }
                }
                catch (Exception ex)
                {
                    logger.Warn($"포트 정리 중 오류 (무시): {ex.Message}");
                    try
                    {
                        if (Serial.IsOpen)
                        {
                            Serial.Close();
                        }
                    }
                    catch { }
                }

                // 포트 열기 재시도 로직 (블루투스 모드 지원)
                bool portOpened = false;
                int maxOpenRetries = 3;
                for (int openRetry = 0; openRetry < maxOpenRetries; openRetry++)
                {
                    if (openRetry > 0)
                    {
                        logger.Info($"포트 {port} 열기 재시도 {openRetry}/{maxOpenRetries - 1}");
                        System.Threading.Thread.Sleep(1000); // 재시도 전 대기 (블루투스 연결 안정화)
                    }

                    try
                    {
                        // 포트 설정
                        Serial.PortName = SelectedPortName!;
                        Serial.BaudRate = Baudrate;
                        Serial.Parity = Parity.None;
                        Serial.DataBits = 8;
                        Serial.StopBits = StopBits.One;
                        Serial.Handshake = Handshake.None;
                        // 블루투스 모드 지원: 더 긴 타임아웃 설정
                        Serial.WriteTimeout = 5000;
                        Serial.ReadTimeout = 5000;

                        // 포트 열기 시도
                        Serial.Open();
                        logger.Info($"포트 {port} 열기 성공 (시도 {openRetry + 1}/{maxOpenRetries})");
                        portOpened = true;
                        break;
                    }
                    catch (System.UnauthorizedAccessException ex)
                    {
                        logger.Error($"포트 {port} 접근 거부 (다른 프로그램이 사용 중일 수 있음): {ex.Message}");
                        if (openRetry < maxOpenRetries - 1)
                        {
                            System.Threading.Thread.Sleep(1000);
                            continue;
                        }
                    }
                    catch (System.ArgumentException ex)
                    {
                        logger.Error($"포트 {port} 설정 오류: {ex.Message}");
                        break; // 잘못된 포트명이면 재시도 불필요
                    }
                    catch (System.IO.IOException ex)
                    {
                        logger.Warn($"포트 {port} I/O 오류 (시도 {openRetry + 1}/{maxOpenRetries}): {ex.Message}");
                        if (openRetry < maxOpenRetries - 1)
                        {
                            System.Threading.Thread.Sleep(1000);
                            continue;
                        }
                    }
                    catch (InvalidOperationException ex)
                    {
                        logger.Warn($"포트 {port} 상태 오류 (시도 {openRetry + 1}/{maxOpenRetries}): {ex.Message}");
                        try
                        {
                            if (Serial.IsOpen)
                            {
                                Serial.Close();
                            }
                        }
                        catch { }
                        if (openRetry < maxOpenRetries - 1)
                        {
                            System.Threading.Thread.Sleep(1000);
                            continue;
                        }
                    }
                    catch (Exception ex)
                    {
                        logger.Error($"포트 {port} 열기 실패 (시도 {openRetry + 1}/{maxOpenRetries}): {ex.GetType().Name} - {ex.Message}");
                        if (openRetry < maxOpenRetries - 1)
                        {
                            System.Threading.Thread.Sleep(1000);
                            continue;
                        }
                    }
                }

                if (!portOpened)
                {
                    logger.Warn($"포트 {port} 열기 실패 ({maxOpenRetries}회 시도) - 다음 포트 시도");
                    continue; // 다음 포트로
                }

                try
                {
                    // 블루투스 모드 지원: 포트를 연 후 초기화 대기 시간 추가
                    // 블루투스 모듈이 준비될 때까지 대기 (일반적으로 500ms~1초 필요)
                    System.Threading.Thread.Sleep(1000);
                    logger.Debug($"포트 {port} 초기화 대기 완료 (블루투스 모드 지원)");

                    // 재시도 로직 추가 (블루투스 모드에서 안정성 향상)
                    bool paramsChecked = false;
                    int maxRetries = 3;
                    for (int retry = 0; retry < maxRetries; retry++)
                    {
                        if (retry > 0)
                        {
                            logger.Info($"포트 {port} 파라미터 확인 재시도 {retry}/{maxRetries - 1}");
                            System.Threading.Thread.Sleep(500); // 재시도 전 대기
                        }

                        // 포트가 여전히 열려있는지 확인
                        if (!Serial.IsOpen)
                        {
                            logger.Warn($"포트 {port}가 예기치 않게 닫혔습니다");
                            break;
                        }

                        if (CheckParams())
                        {
                            logger.Info($"포트 {port} 파라미터 확인 성공");
                            paramsChecked = true;
                            bfind = true;
                            break;
                        }
                    }

                    if (!paramsChecked)
                    {
                        logger.Warn($"포트 {port} 파라미터 확인 실패 ({maxRetries}회 시도) - 다음 포트 시도");
                        try
                        {
                            Serial.Close();
                        }
                        catch { }
                    }
                    else
                    {
                        break; // 성공한 경우 루프 종료
                    }
                }
                catch (Exception e)
                {
                    logger.Error($"포트 {port} 통신 중 오류: {e.Message}");
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

        // 포트가 열려있는지 확인하는 public 메서드 추가
        public static bool IsPortOpen()
        {
            return Serial.IsOpen;
        }

        //240315 : CheckParams() 결과 반환
        public static bool CheckParams()
        {
            if (!Serial.IsOpen)
            {
                logger.Warn("시리얼 포트가 열려있지 않아 파라미터 확인을 수행할 수 없습니다");
                return false;
            }

            try
            {
                // 포트가 열려있는지 다시 한 번 확인 (블루투스 모드에서 포트가 갑자기 닫힐 수 있음)
                if (!Serial.IsOpen)
                {
                    logger.Warn($"포트 {Serial.PortName}가 열려있지 않습니다");
                    return false;
                }

                logger.Debug($"포트 {Serial.PortName} 파라미터 확인 요청 전송");
                
                // 버퍼 비우기 (블루투스 모드에서 이전 데이터 잔여 가능)
                Serial.DiscardInBuffer();
                Serial.DiscardOutBuffer();
                
                Serial.WriteLine("check");
                
                // 포트 상태 재확인
                if (!Serial.IsOpen)
                {
                    logger.Warn($"포트 {Serial.PortName}가 전송 후 닫혔습니다");
                    return false;
                }

                try
                {
                    string response = Serial.ReadLine();
                    logger.Debug($"포트 {Serial.PortName} 응답 수신: {response}");
                    ReadCheck(response);
                    logger.Info($"포트 {Serial.PortName} 파라미터 확인 성공");
                    return true;
                }
                catch (System.TimeoutException)
                {
                    logger.Warn($"포트 {Serial.PortName} Readline 타임아웃 (블루투스 모드일 수 있음)");
                    return false;
                }
                catch (Exception e)
                {
                    logger.Error($"포트 {Serial.PortName} Readline 실패: {e.ToString()}");
                    return false;
                }
            }
            catch (System.InvalidOperationException e)
            {
                logger.Error($"포트 {Serial.PortName} 파라미터 확인 중 포트 상태 오류: {e.Message}");
                return false;
            }
            catch (Exception e)
            {
                logger.Error($"포트 {Serial.PortName} 파라미터 확인 중 오류: {e.Message}");
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
