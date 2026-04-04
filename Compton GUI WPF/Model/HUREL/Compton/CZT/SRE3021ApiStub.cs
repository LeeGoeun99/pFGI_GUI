// SRE3021 CZT DLL이 없는 환경에서 빌드·디자이너가 깨지지 않도록 최소 스텁을 둡니다.
// 실제 장비용 DLL을 쓰려면 Compton GUI WPF.csproj에 SRE3021 API 참조를 추가하고 이 파일을 제외하세요.

namespace HUREL.Compton.CZT
{
    public sealed class SRE3021ImageData
    {
        public int[,] AnodeTiming { get; } = new int[11, 11];
        public int[,] AnodeValue { get; } = new int[11, 11];
    }

    public static class SRE3021API
    {
        public static bool IsTCPOpen => false;
        public static bool IsUDPOpen => false;

        public static int[,] AnodeValueBaseline { get; } = new int[11, 11];
        public static int[,] AnodeTimingBaseline { get; } = new int[11, 11];

        public static event System.Action<SRE3021ImageData>? IMGDataEventRecieved;

        public static void OpenUDPPort() { }
        public static void OpenTCPPort() { }
        public static void CheckAPI() { }
        public static void Close() { }
        public static void CheckBaseline() { }
        public static void StartAcqusition() { }
        public static void StopAcqusition() { }
        public static void SetHighVoltage(int hvValue, int a, int b) { }
    }
}
