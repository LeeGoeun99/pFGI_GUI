using System;
using System.Threading;

namespace HUREL.Compton
{
    /// <summary>
    /// 측정 시작 시점을 저장하고, 모든 타임스탬프를 상대 시간(ms, 0부터 시작)으로 변환하는 클래스
    /// RGB/Depth 이미지와 동일한 형식으로 통일하기 위함
    /// </summary>
    public static class MeasurementTimestampManager
    {
        private static DateTime? _measurementStartTime = null;
        private static readonly object _lockObject = new object();

        /// <summary>
        /// 측정 시작 시점 기록 (측정 시작 시 호출)
        /// </summary>
        public static void SetMeasurementStartTime(DateTime startTime)
        {
            lock (_lockObject)
            {
                _measurementStartTime = startTime;
            }
        }

        /// <summary>
        /// 측정 시작 시점 초기화 (측정 종료 시 호출)
        /// </summary>
        public static void ResetMeasurementStartTime()
        {
            lock (_lockObject)
            {
                _measurementStartTime = null;
            }
        }

        /// <summary>
        /// DateTime을 측정 시작 시점 기준 상대 시간(ms, 0부터 시작)으로 변환
        /// </summary>
        /// <param name="timestamp">변환할 DateTime</param>
        /// <returns>측정 시작 시점으로부터의 경과 시간(ms), 측정 시작 시점이 없으면 0</returns>
        public static long ToRelativeMilliseconds(DateTime timestamp)
        {
            lock (_lockObject)
            {
                if (_measurementStartTime == null)
                {
                    // 측정 시작 시점이 없으면 0을 반환
                    return 0;
                }

                var elapsed = timestamp - _measurementStartTime.Value;
                return (long)elapsed.TotalMilliseconds;
            }
        }

        /// <summary>
        /// 측정 시작 시점 반환
        /// </summary>
        /// <returns>측정 시작 시점, 설정되지 않았으면 null</returns>
        public static DateTime? GetMeasurementStartTime()
        {
            lock (_lockObject)
            {
                return _measurementStartTime;
            }
        }

        /// <summary>
        /// 측정 시작 시점이 설정되었는지 확인
        /// </summary>
        public static bool IsMeasurementStarted()
        {
            lock (_lockObject)
            {
                return _measurementStartTime != null;
            }
        }
    }
}

