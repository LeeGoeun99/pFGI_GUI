using System;
using System.Collections.Generic;
using OpenCvSharp;
using HUREL_Imager_GUI.ViewModel.ObjectDetection.Models;
using log4net;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection
{
    /// <summary>
    /// 객체탐지 서비스를 관리하는 클래스
    /// </summary>
    public class ObjectDetectionService : IDisposable
    {
        private static readonly ILog logger = LogManager.GetLogger(typeof(ObjectDetectionService));

        private YOLOv11Model? _model;
        private PersonTracker? _tracker;
        private DetectionDataManager? _dataManager;

        private readonly string _modelPath;
        private readonly float _confidenceThreshold;
        private readonly float _nmsThreshold;
        private bool _isInitialized;

        /// <summary>
        /// 초기화 여부
        /// </summary>
        public bool IsInitialized => _isInitialized;

        private readonly bool _useCpuOnly;
        
        /// <summary>
        /// 생성자
        /// </summary>
        /// <param name="modelPath">YOLOv11 ONNX 모델 파일 경로</param>
        /// <param name="confidenceThreshold">신뢰도 임계값 (기본값: 0.5)</param>
        /// <param name="nmsThreshold">NMS 임계값 (기본값: 0.4)</param>
        /// <param name="useCpuOnly">CPU 전용 모드 사용 여부 (기본값: false, GPU 사용 시도)</param>
        public ObjectDetectionService(string modelPath, float confidenceThreshold = 0.5f, float nmsThreshold = 0.4f, bool useCpuOnly = false)
        {
            _modelPath = modelPath;
            _confidenceThreshold = confidenceThreshold;
            _nmsThreshold = nmsThreshold;
            _useCpuOnly = useCpuOnly;
            _isInitialized = false;
        }

        /// <summary>
        /// 서비스 초기화
        /// </summary>
        /// <returns>초기화 성공 여부</returns>
        public bool Initialize()
        {
            try
            {
                logger.Info($"ObjectDetectionService 초기화 시작: 모델 경로={_modelPath}");

                // 모델 로드 (CPU 전용 모드 설정 전달)
                _model = new YOLOv11Model(_modelPath, _confidenceThreshold, _nmsThreshold, _useCpuOnly);
                if (!_model.LoadModel())
                {
                    logger.Error("YOLOv11 모델 로드 실패");
                    return false;
                }

                // 추적기 초기화
                _tracker = new PersonTracker(iouThreshold: 0.3f, maxLossFrames: 5);

                // 데이터 관리자 초기화
                _dataManager = new DetectionDataManager();

                _isInitialized = true;
                logger.Info("ObjectDetectionService 초기화 완료");
                return true;
            }
            catch (Exception ex)
            {
                logger.Error($"ObjectDetectionService 초기화 실패: {ex.Message}", ex);
                _isInitialized = false;
                return false;
            }
        }

        /// <summary>
        /// 프레임 처리 (탐지 및 추적)
        /// </summary>
        /// <param name="frame">입력 프레임 (OpenCvSharp Mat)</param>
        /// <param name="imageTimestamp">이미지 캡처 시점의 타임스탬프 (동기화를 위해 사용)</param>
        /// <returns>추적된 사람들의 리스트</returns>
        public List<TrackedPerson> ProcessFrame(Mat frame, DateTime imageTimestamp)
        {
            if (!_isInitialized || _model == null || _tracker == null || _dataManager == null)
            {
                logger.Warn("ObjectDetectionService가 초기화되지 않았습니다.");
                return new List<TrackedPerson>();
            }

            if (frame.Empty())
            {
                return new List<TrackedPerson>();
            }

            try
            {
                logger.Debug($"ProcessFrame 시작: 프레임 크기={frame.Width}x{frame.Height}, 타임스탬프={imageTimestamp:yyyy-MM-dd HH:mm:ss.fff}");
                
                // 1. 객체 탐지
                if (_model == null)
                {
                    logger.Error("모델이 null입니다.");
                    return new List<TrackedPerson>();
                }
                
                var detections = _model.Predict(frame);
                logger.Debug($"모델 예측 완료: 탐지된 객체 수={detections?.Count ?? 0}");

                // 2. 추적 업데이트 (이미지 캡처 시점의 타임스탬프 전달)
                if (_tracker == null)
                {
                    logger.Error("추적기가 null입니다.");
                    return new List<TrackedPerson>();
                }
                
                List<TrackedPerson> trackedPersons;
                try
                {
                    trackedPersons = _tracker.Update(detections ?? new List<Detection>(), imageTimestamp);
                    logger.Debug($"추적 업데이트 완료: 추적된 사람 수={trackedPersons?.Count ?? 0}");
                }
                catch (Exception trackerEx)
                {
                    logger.Error($"추적 업데이트 중 오류 발생: {trackerEx.Message}", trackerEx);
                    logger.Error($"Stack trace: {trackerEx.StackTrace}");
                    return new List<TrackedPerson>();
                }

                // 3. 데이터 기록 (이미지 캡처 시점의 타임스탬프 사용)
                if (_dataManager == null)
                {
                    logger.Error("데이터 관리자가 null입니다.");
                    return trackedPersons ?? new List<TrackedPerson>();
                }
                
                if (trackedPersons != null)
                {
                    foreach (var trackedPerson in trackedPersons)
                    {
                        if (trackedPerson == null)
                        {
                            logger.Warn("TrackedPerson이 null입니다. 건너뜁니다.");
                            continue;
                        }
                        
                        // BoundingBox가 null이 아닌 경우에만 기록
                        if (trackedPerson.BoundingBox != null)
                        {
                            try
                            {
                                _dataManager.RecordDetection(
                                    trackedPerson.Id,
                                    trackedPerson.BoundingBox,
                                    imageTimestamp,  // 이미지 캡처 시점의 타임스탬프 사용
                                    trackedPerson.Confidence
                                );
                            }
                            catch (Exception recordEx)
                            {
                                logger.Error($"데이터 기록 중 오류 발생 (PersonId: {trackedPerson.Id}): {recordEx.Message}", recordEx);
                            }
                        }
                        else
                        {
                            logger.Warn($"TrackedPerson ID {trackedPerson.Id}의 BoundingBox가 null입니다. 기록을 건너뜁니다.");
                        }
                    }
                }

                return trackedPersons ?? new List<TrackedPerson>();
            }
            catch (Exception ex)
            {
                logger.Error($"프레임 처리 중 오류 발생: {ex.Message}", ex);
                logger.Error($"Stack trace: {ex.StackTrace}");
                if (ex.InnerException != null)
                {
                    logger.Error($"Inner exception: {ex.InnerException.Message}", ex.InnerException);
                }
                return new List<TrackedPerson>();
            }
        }

        /// <summary>
        /// 데이터를 파일로 저장
        /// </summary>
        /// <param name="filePath">저장할 파일 경로</param>
        /// <returns>저장 성공 여부</returns>
        public bool SaveData(string filePath)
        {
            if (_dataManager == null)
            {
                logger.Warn("DetectionDataManager가 초기화되지 않았습니다.");
                return false;
            }

            return _dataManager.SaveToFile(filePath);
        }

        /// <summary>
        /// 모든 데이터 초기화
        /// </summary>
        public void ClearData()
        {
            _tracker?.Reset();
            _dataManager?.Clear();
            logger.Debug("ObjectDetectionService 데이터 초기화됨");
        }

        /// <summary>
        /// 현재 활성 추적 수 반환
        /// </summary>
        public int GetActiveTrackCount()
        {
            return _tracker?.GetActiveTrackCount() ?? 0;
        }

        /// <summary>
        /// 기록된 총 사람 수 반환
        /// </summary>
        public int GetPersonCount()
        {
            return _dataManager?.GetPersonCount() ?? 0;
        }

        /// <summary>
        /// 기록된 총 탐지 수 반환
        /// </summary>
        public int GetTotalRecordCount()
        {
            return _dataManager?.GetTotalRecordCount() ?? 0;
        }

        public void Dispose()
        {
            _model?.Dispose();
            _tracker = null;
            _dataManager = null;
            _isInitialized = false;
            logger.Debug("ObjectDetectionService 리소스 해제됨");
        }
    }
}

