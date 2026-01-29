using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using HUREL_Imager_GUI.ViewModel.ObjectDetection.Models;
using HUREL.Compton;
using log4net;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection
{
    /// <summary>
    /// ID별 bounding box 좌표 누적 기록 및 저장을 관리하는 클래스
    /// </summary>
    public class DetectionDataManager
    {
        private static readonly ILog logger = LogManager.GetLogger(typeof(DetectionDataManager));

        private readonly Dictionary<int, PersonTrajectory> _trajectories;

        /// <summary>
        /// 생성자
        /// </summary>
        public DetectionDataManager()
        {
            _trajectories = new Dictionary<int, PersonTrajectory>();
        }

        /// <summary>
        /// 탐지 결과 기록
        /// </summary>
        /// <param name="personId">사람 ID</param>
        /// <param name="box">Bounding box</param>
        /// <param name="timestamp">탐지 시각</param>
        /// <param name="confidence">신뢰도</param>
        public void RecordDetection(int personId, BoundingBox box, DateTime timestamp, float confidence)
        {
            // BoundingBox가 null이면 기록하지 않음
            if (box == null)
            {
                logger.Warn($"PersonId {personId}의 BoundingBox가 null입니다. 기록을 건너뜁니다.");
                return;
            }

            if (!_trajectories.ContainsKey(personId))
            {
                _trajectories[personId] = new PersonTrajectory(personId);
            }

            var record = new BoundingBoxRecord(box, timestamp, confidence);
            _trajectories[personId].Records.Add(record);
        }

        /// <summary>
        /// 특정 ID의 이동 궤적 반환
        /// </summary>
        /// <param name="personId">사람 ID</param>
        /// <returns>이동 궤적</returns>
        public PersonTrajectory? GetPersonTrajectory(int personId)
        {
            return _trajectories.ContainsKey(personId) ? _trajectories[personId] : null;
        }

        /// <summary>
        /// 모든 이동 궤적 반환
        /// </summary>
        /// <returns>모든 이동 궤적 리스트</returns>
        public List<PersonTrajectory> GetAllTrajectories()
        {
            return _trajectories.Values.ToList();
        }

        /// <summary>
        /// 데이터를 CSV 파일로 저장
        /// </summary>
        /// <param name="filePath">저장할 파일 경로</param>
        /// <returns>저장 성공 여부</returns>
        public bool SaveToFile(string filePath)
        {
            try
            {
                var directory = Path.GetDirectoryName(filePath);
                if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                {
                    Directory.CreateDirectory(directory);
                }

                using (var writer = new StreamWriter(filePath, false, System.Text.Encoding.UTF8))
                {
                    // CSV 헤더 작성 (타임스탬프를 ms 단위로 변경)
                    writer.WriteLine("PersonId,Timestamp_ms,X,Y,Width,Height,Confidence");

                    // 데이터 작성
                    foreach (var trajectory in _trajectories.Values.Where(t => t != null).OrderBy(t => t.PersonId))
                    {
                        if (trajectory == null || trajectory.Records == null)
                        {
                            logger.Warn($"PersonId {trajectory?.PersonId}의 trajectory 또는 Records가 null입니다. 건너뜁니다.");
                            continue;
                        }

                        foreach (var record in trajectory.Records.Where(r => r != null).OrderBy(r => r.Timestamp))
                        {
                            if (record == null || record.Box == null)
                            {
                                logger.Warn($"PersonId {trajectory.PersonId}의 record 또는 Box가 null입니다. 건너뜁니다.");
                                continue;
                            }

                            // 측정 시작 시점 기준 상대 시간(ms, 0부터 시작)으로 변환
                            long relativeTimeMs = MeasurementTimestampManager.ToRelativeMilliseconds(record.Timestamp);
                            
                            writer.WriteLine($"{trajectory.PersonId}," +
                                           $"{relativeTimeMs}," +
                                           $"{record.Box.X:F2}," +
                                           $"{record.Box.Y:F2}," +
                                           $"{record.Box.Width:F2}," +
                                           $"{record.Box.Height:F2}," +
                                           $"{record.Confidence:F4}");
                        }
                    }
                }

                logger.Info($"객체탐지 데이터 저장 완료: {filePath} (총 {_trajectories.Count}명의 사람, {_trajectories.Values.Sum(t => t.Records.Count)}개 기록)");
                return true;
            }
            catch (Exception ex)
            {
                logger.Error($"객체탐지 데이터 저장 실패: {ex.Message}", ex);
                return false;
            }
        }

        /// <summary>
        /// 모든 데이터 초기화
        /// </summary>
        public void Clear()
        {
            _trajectories.Clear();
            logger.Debug("DetectionDataManager 데이터 초기화됨");
        }

        /// <summary>
        /// 기록된 총 사람 수 반환
        /// </summary>
        public int GetPersonCount()
        {
            return _trajectories.Count;
        }

        /// <summary>
        /// 기록된 총 탐지 수 반환
        /// </summary>
        public int GetTotalRecordCount()
        {
            return _trajectories.Values.Sum(t => t.Records.Count);
        }
    }
}

