using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using HUREL_Imager_GUI.ViewModel.ObjectDetection.Models;
using log4net;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection
{
    /// <summary>
    /// 사람 추적을 수행하는 클래스 (IoU 기반 간단한 추적 알고리즘)
    /// </summary>
    public class PersonTracker
    {
        private static readonly ILog logger = LogManager.GetLogger(typeof(PersonTracker));

        private readonly Dictionary<int, TrackedPerson> _activeTracks;
        private readonly Dictionary<int, int> _trackLossCount;  // 추적 실패 프레임 수
        private int _nextId;
        private readonly float _iouThreshold;
        private readonly int _maxLossFrames;  // 이 프레임 수만큼 추적 실패 시 ID 제거
        private readonly object _lockObject = new object();  // Thread-safety를 위한 lock 객체

        /// <summary>
        /// 생성자
        /// </summary>
        /// <param name="iouThreshold">IoU 임계값 (기본값: 0.3)</param>
        /// <param name="maxLossFrames">최대 추적 실패 프레임 수 (기본값: 5)</param>
        public PersonTracker(float iouThreshold = 0.3f, int maxLossFrames = 5)
        {
            _activeTracks = new Dictionary<int, TrackedPerson>();
            _trackLossCount = new Dictionary<int, int>();
            _nextId = 1;
            _iouThreshold = iouThreshold;
            _maxLossFrames = maxLossFrames;
        }

        /// <summary>
        /// 새로운 탐지 결과로 추적 업데이트
        /// </summary>
        /// <param name="detections">현재 프레임의 탐지 결과</param>
        /// <param name="imageTimestamp">이미지 캡처 시점의 타임스탬프 (동기화를 위해 사용)</param>
        /// <returns>추적된 사람들의 리스트</returns>
        public List<TrackedPerson> Update(List<Detection> detections, DateTime imageTimestamp)
        {
            // Thread-safety를 위해 lock 사용
            lock (_lockObject)
            {
                var timestamp = imageTimestamp;  // 이미지 캡처 시점의 타임스탬프 사용
                var trackedPersons = new List<TrackedPerson>();

                try
                {
                // null 체크 추가
                if (detections == null)
                {
                    logger.Warn("Update: detections가 null입니다.");
                    // 탐지가 null이면 모든 트랙의 실패 카운트 증가
                    IncreaseLossCount();
                    RemoveLostTracks();
                    return trackedPersons;
                }

                if (detections.Count == 0)
                {
                    // 탐지가 없으면 모든 트랙의 실패 카운트 증가
                    IncreaseLossCount();
                    RemoveLostTracks();
                    return trackedPersons;
                }

                // 현재 프레임의 탐지 결과를 BoundingBox로 변환 (null 체크 포함)
                var currentBoxes = new List<BoundingBox>();
                try
                {
                    currentBoxes = detections
                        .Where(d => d != null)
                        .Select(d => 
                        {
                            try
                            {
                                return new BoundingBox(d.X, d.Y, d.Width, d.Height);
                            }
                            catch (Exception ex)
                            {
                                logger.Warn($"BoundingBox 생성 실패: {ex.Message}");
                                return null;
                            }
                        })
                        .Where(b => b != null)
                        .ToList();
                }
                catch (Exception ex)
                {
                    logger.Error($"BoundingBox 변환 중 오류 발생: {ex.Message}", ex);
                    return trackedPersons;
                }

                // 기존 트랙과 새로운 탐지를 매칭
                List<(int trackId, int detectionIndex)> matchedPairs;
                try
                {
                    matchedPairs = MatchTracks(currentBoxes, detections);
                }
                catch (Exception ex)
                {
                    logger.Error($"트랙 매칭 중 오류 발생: {ex.Message}", ex);
                    return trackedPersons;
                }

                // 매칭된 트랙 업데이트
                foreach (var (trackId, detectionIndex) in matchedPairs)
                {
                    try
                    {
                        if (detectionIndex < 0 || detectionIndex >= detections.Count)
                        {
                            logger.Warn($"유효하지 않은 detectionIndex: {detectionIndex} (detections.Count: {detections.Count})");
                            continue;
                        }

                        var detection = detections[detectionIndex];
                        if (detection == null)
                        {
                            logger.Warn($"detection이 null입니다. detectionIndex: {detectionIndex}");
                            continue;
                        }

                        var boundingBox = new BoundingBox(detection.X, detection.Y, detection.Width, detection.Height);

                        if (_activeTracks.ContainsKey(trackId) && _activeTracks[trackId] != null)
                        {
                            _activeTracks[trackId].BoundingBox = boundingBox;
                            _activeTracks[trackId].Timestamp = timestamp;
                            _activeTracks[trackId].Confidence = detection.Confidence;
                            if (!_trackLossCount.ContainsKey(trackId))
                            {
                                _trackLossCount[trackId] = 0;
                            }
                            _trackLossCount[trackId] = 0;  // 추적 성공 시 실패 카운트 리셋
                        }
                        else if (_activeTracks.ContainsKey(trackId))
                        {
                            logger.Warn($"트랙 {trackId}의 Value가 null입니다. 건너뜁니다.");
                        }
                    }
                    catch (Exception ex)
                    {
                        logger.Error($"트랙 업데이트 중 오류 발생 (trackId: {trackId}, detectionIndex: {detectionIndex}): {ex.Message}", ex);
                    }
                }

                // 매칭되지 않은 탐지에 새 ID 할당
                var matchedDetectionIndices = new HashSet<int>(matchedPairs.Select(p => p.Item2));
                for (int i = 0; i < detections.Count; i++)
                {
                    try
                    {
                        if (!matchedDetectionIndices.Contains(i))
                        {
                            var detection = detections[i];
                            if (detection == null)
                            {
                                logger.Warn($"detection이 null입니다. 인덱스: {i}");
                                continue;
                            }

                            var newId = _nextId++;
                            var boundingBox = new BoundingBox(detection.X, detection.Y, detection.Width, detection.Height);
                            var trackedPerson = new TrackedPerson(newId, boundingBox, timestamp, detection.Confidence);
                            _activeTracks[newId] = trackedPerson;
                            _trackLossCount[newId] = 0;
                        }
                    }
                    catch (Exception ex)
                    {
                        logger.Error($"새 트랙 생성 중 오류 발생 (인덱스: {i}): {ex.Message}", ex);
                    }
                }

                // 매칭되지 않은 트랙의 실패 카운트 증가
                var matchedTrackIds = new HashSet<int>(matchedPairs.Select(p => p.Item1));
                foreach (var trackId in _activeTracks.Keys.ToList())
                {
                    try
                    {
                        if (!matchedTrackIds.Contains(trackId))
                        {
                            if (!_trackLossCount.ContainsKey(trackId))
                            {
                                _trackLossCount[trackId] = 1;
                            }
                            else
                            {
                                _trackLossCount[trackId]++;
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        logger.Error($"트랙 실패 카운트 증가 중 오류 발생 (trackId: {trackId}): {ex.Message}", ex);
                    }
                }

                // 오래된 트랙 제거
                RemoveLostTracks();

                // 활성 트랙을 결과에 추가 (null 제외)
                try
                {
                    if (_activeTracks != null && _activeTracks.Values != null)
                    {
                        trackedPersons.AddRange(_activeTracks.Values.Where(t => t != null));
                    }
                }
                catch (Exception ex)
                {
                    logger.Error($"트랙 결과 수집 중 오류 발생: {ex.Message}", ex);
                }
            }
                catch (Exception ex)
                {
                    logger.Error($"Update 메서드에서 예외 발생: {ex.Message}", ex);
                    logger.Error($"Stack trace: {ex.StackTrace}");
                }

                return trackedPersons;
            }
        }

        /// <summary>
        /// 기존 트랙과 새로운 탐지를 IoU 기반으로 매칭
        /// </summary>
        private List<(int trackId, int detectionIndex)> MatchTracks(List<BoundingBox> currentBoxes, List<Detection> detections)
        {
            var matchedPairs = new List<(int, int)>();
            var usedDetections = new HashSet<int>();
            var usedTracks = new HashSet<int>();

            try
            {
                // null 체크 추가
                if (currentBoxes == null || detections == null)
                {
                    logger.Warn("MatchTracks: currentBoxes 또는 detections가 null입니다.");
                    return new List<(int, int)>();
                }

                if (_activeTracks == null)
                {
                    logger.Warn("MatchTracks: _activeTracks가 null입니다.");
                    return new List<(int, int)>();
                }

                // 모든 트랙-탐지 쌍의 IoU 계산
                var iouMatrix = new List<(int trackId, int detectionIndex, float iou)>();
                foreach (var track in _activeTracks)
                {
                    try
                    {
                        // track.Value가 null인 경우 체크
                        if (track.Value == null)
                        {
                            logger.Warn($"트랙 {track.Key}의 Value가 null입니다. 매칭에서 제외합니다.");
                            continue;
                        }

                        // BoundingBox가 null인 트랙은 건너뛰기
                        if (track.Value.BoundingBox == null)
                        {
                            logger.Debug($"트랙 {track.Key}의 BoundingBox가 null입니다. 매칭에서 제외합니다.");
                            continue;
                        }

                        for (int i = 0; i < currentBoxes.Count; i++)
                        {
                            try
                            {
                                // currentBoxes[i]도 null 체크
                                if (currentBoxes[i] == null)
                                {
                                    continue;
                                }

                                var iou = CalculateIoU(track.Value.BoundingBox, currentBoxes[i]);
                                if (iou > _iouThreshold)
                                {
                                    iouMatrix.Add((track.Key, i, iou));
                                }
                            }
                            catch (Exception ex)
                            {
                                logger.Warn($"IoU 계산 중 오류 발생 (trackId: {track.Key}, boxIndex: {i}): {ex.Message}");
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        logger.Warn($"트랙 처리 중 오류 발생 (trackId: {track.Key}): {ex.Message}");
                    }
                }

                // IoU가 높은 순서로 정렬
                var sortedIoU = iouMatrix.OrderByDescending(x => x.iou).ToList();

                // 그리디 방식으로 매칭 (최고 IoU부터 매칭)
                foreach (var (trackId, detectionIndex, iou) in sortedIoU)
                {
                    try
                    {
                        if (!usedTracks.Contains(trackId) && !usedDetections.Contains(detectionIndex))
                        {
                            matchedPairs.Add((trackId, detectionIndex));
                            usedTracks.Add(trackId);
                            usedDetections.Add(detectionIndex);
                        }
                    }
                    catch (Exception ex)
                    {
                        logger.Warn($"매칭 추가 중 오류 발생 (trackId: {trackId}, detectionIndex: {detectionIndex}): {ex.Message}");
                    }
                }
            }
            catch (Exception ex)
            {
                logger.Error($"MatchTracks 메서드에서 예외 발생: {ex.Message}", ex);
            }

            return matchedPairs;
        }

        /// <summary>
        /// IoU (Intersection over Union) 계산
        /// </summary>
        private float CalculateIoU(BoundingBox box1, BoundingBox box2)
        {
            // null 체크 추가
            if (box1 == null || box2 == null)
            {
                logger.Warn("CalculateIoU: box1 또는 box2가 null입니다.");
                return 0.0f;
            }

            var x1 = Math.Max(box1.X, box2.X);
            var y1 = Math.Max(box1.Y, box2.Y);
            var x2 = Math.Min(box1.X + box1.Width, box2.X + box2.Width);
            var y2 = Math.Min(box1.Y + box1.Height, box2.Y + box2.Height);

            if (x2 <= x1 || y2 <= y1)
                return 0.0f;

            var intersection = (x2 - x1) * (y2 - y1);
            var area1 = box1.Width * box1.Height;
            var area2 = box2.Width * box2.Height;
            var union = area1 + area2 - intersection;

            if (union <= 0)
                return 0.0f;

            return intersection / union;
        }

        /// <summary>
        /// 모든 트랙의 실패 카운트 증가 (탐지가 없을 때 호출)
        /// </summary>
        private void IncreaseLossCount()
        {
            // Update 메서드 내부에서 호출되므로 이미 lock 안에 있음
            var trackIds = _trackLossCount.Keys.ToList();
            foreach (var trackId in trackIds)
            {
                _trackLossCount[trackId]++;
            }
        }

        /// <summary>
        /// 오래된 트랙 제거
        /// </summary>
        private void RemoveLostTracks()
        {
            // Update 메서드 내부에서 호출되므로 이미 lock 안에 있음
            var tracksToRemove = _trackLossCount
                .Where(kvp => kvp.Value >= _maxLossFrames)
                .Select(kvp => kvp.Key)
                .ToList();

            foreach (var trackId in tracksToRemove)
            {
                _activeTracks.Remove(trackId);
                _trackLossCount.Remove(trackId);
                logger.Debug($"트랙 {trackId} 제거됨 (추적 실패 프레임: {_maxLossFrames})");
            }
        }

        /// <summary>
        /// 모든 트랙 초기화
        /// </summary>
        public void Reset()
        {
            lock (_lockObject)
            {
                _activeTracks.Clear();
                _trackLossCount.Clear();
                _nextId = 1;
                logger.Debug("PersonTracker 초기화됨");
            }
        }

        /// <summary>
        /// 현재 활성 트랙 수 반환
        /// </summary>
        public int GetActiveTrackCount()
        {
            lock (_lockObject)
            {
                return _activeTracks.Count;
            }
        }
    }
}

