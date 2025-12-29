using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;
using OpenCvSharp;
using HUREL_Imager_GUI.ViewModel.ObjectDetection.Models;
using log4net;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection
{
    /// <summary>
    /// YOLOv11 모델을 사용하여 객체탐지를 수행하는 클래스
    /// </summary>
    public class YOLOv11Model : IDisposable
    {
        private static readonly ILog logger = LogManager.GetLogger(typeof(YOLOv11Model));

        private InferenceSession? _session;
        private readonly string _modelPath;
        private readonly float _confidenceThreshold;
        private readonly float _nmsThreshold;
        private readonly int _inputWidth;
        private readonly int _inputHeight;

        // YOLO COCO 클래스 ID (0 = person)
        private const int PersonClassId = 0;

        /// <summary>
        /// 모델이 로드되었는지 여부
        /// </summary>
        public bool IsLoaded => _session != null;

        /// <summary>
        /// 생성자
        /// </summary>
        /// <param name="modelPath">ONNX 모델 파일 경로</param>
        /// <param name="confidenceThreshold">신뢰도 임계값 (기본값: 0.5)</param>
        /// <param name="nmsThreshold">NMS 임계값 (기본값: 0.4)</param>
        public YOLOv11Model(string modelPath, float confidenceThreshold = 0.5f, float nmsThreshold = 0.4f)
        {
            _modelPath = modelPath;
            _confidenceThreshold = confidenceThreshold;
            _nmsThreshold = nmsThreshold;
            _inputWidth = 640;  // YOLOv11 기본 입력 크기
            _inputHeight = 640;
        }

        /// <summary>
        /// ONNX 모델 로드
        /// </summary>
        public bool LoadModel()
        {
            try
            {
                if (string.IsNullOrEmpty(_modelPath) || !System.IO.File.Exists(_modelPath))
                {
                    logger.Error($"모델 파일을 찾을 수 없습니다: {_modelPath}");
                    return false;
                }

                var options = new SessionOptions();
                // CPU 실행 (필요시 GPU로 변경 가능)
                // options.AppendExecutionProvider_CUDA();
                
                _session = new InferenceSession(_modelPath, options);
                logger.Info($"YOLOv11 모델 로드 완료: {_modelPath}");
                return true;
            }
            catch (Exception ex)
            {
                logger.Error($"모델 로드 실패: {ex.Message}", ex);
                return false;
            }
        }

        /// <summary>
        /// 이미지에서 사람을 탐지
        /// </summary>
        /// <param name="frame">입력 이미지 (OpenCvSharp Mat)</param>
        /// <returns>탐지된 사람들의 리스트</returns>
        public List<Detection> Predict(Mat frame)
        {
            if (_session == null)
            {
                logger.Warn("모델이 로드되지 않았습니다.");
                return new List<Detection>();
            }

            if (frame.Empty())
            {
                logger.Warn("입력 프레임이 비어있습니다.");
                return new List<Detection>();
            }

            try
            {
                logger.Info($"Predict 시작: 입력 프레임 크기={frame.Width}x{frame.Height}");
                
                // 1. 이미지 전처리
                var preprocessed = PreprocessImage(frame);
                logger.Info("이미지 전처리 완료");

                // 2. 모델 추론
                var detections = RunInference(preprocessed);
                logger.Info($"모델 추론 완료: 원시 탐지 수={detections.Count}");

                // 3. 좌표를 원본 이미지 크기로 변환
                var scaledDetections = ScaleDetections(detections, frame.Width, frame.Height);
                logger.Info($"좌표 스케일링 완료: 탐지 수={scaledDetections.Count}");

                // 4. NMS (Non-Maximum Suppression) 적용
                var filteredDetections = ApplyNMS(scaledDetections);
                logger.Info($"NMS 적용 완료: 필터링된 탐지 수={filteredDetections.Count}");

                // 5. 사람 클래스만 필터링
                var personDetections = filteredDetections.Where(d => d.ClassId == PersonClassId).ToList();
                logger.Info($"사람 클래스 필터링 완료: 최종 탐지 수={personDetections.Count}");

                return personDetections;
            }
            catch (Exception ex)
            {
                logger.Error($"탐지 중 오류 발생: {ex.Message}", ex);
                return new List<Detection>();
            }
        }

        /// <summary>
        /// 이미지 전처리 (리사이즈, 정규화, 텐서 변환)
        /// </summary>
        private Tensor<float> PreprocessImage(Mat frame)
        {
            // RGB로 변환 (BGR -> RGB)
            Mat rgb = new Mat();
            Cv2.CvtColor(frame, rgb, ColorConversionCodes.BGR2RGB);

            // 리사이즈
            Mat resized = new Mat();
            Cv2.Resize(rgb, resized, new Size(_inputWidth, _inputHeight), 0, 0, InterpolationFlags.Linear);

            // 정규화 (0-255 -> 0-1)
            Mat normalized = new Mat();
            resized.ConvertTo(normalized, MatType.CV_32F, 1.0 / 255.0);

            // 텐서 생성 [1, 3, 640, 640]
            var tensorData = new float[1 * 3 * _inputHeight * _inputWidth];
            var index = 0;

            // CHW 형식으로 변환
            for (int c = 0; c < 3; c++)
            {
                for (int y = 0; y < _inputHeight; y++)
                {
                    for (int x = 0; x < _inputWidth; x++)
                    {
                        var pixel = normalized.At<Vec3f>(y, x);
                        tensorData[index++] = pixel[c];
                    }
                }
            }

            rgb.Dispose();
            resized.Dispose();
            normalized.Dispose();

            var tensor = new DenseTensor<float>(tensorData, new[] { 1, 3, _inputHeight, _inputWidth });
            return tensor;
        }

        /// <summary>
        /// 모델 추론 실행
        /// </summary>
        private List<Detection> RunInference(Tensor<float> inputTensor)
        {
            // 입력 텐서 이름 확인 (일반적으로 "images" 또는 "input")
            var inputNames = _session!.InputMetadata.Keys.ToList();
            logger.Info($"사용 가능한 입력 텐서 이름: [{string.Join(", ", inputNames)}]");
            
            var inputName = inputNames.FirstOrDefault() ?? "images";
            logger.Info($"사용할 입력 텐서 이름: {inputName}");

            var inputs = new List<NamedOnnxValue>
            {
                NamedOnnxValue.CreateFromTensor(inputName, inputTensor)
            };

            logger.Info("모델 추론 실행 중...");
            using var results = _session!.Run(inputs);
            
            // 출력 텐서 이름 확인
            var outputNames = results.Select(r => r.Name).ToList();
            logger.Info($"출력 텐서 이름: [{string.Join(", ", outputNames)}]");
            
            var output = results.FirstOrDefault();

            if (output == null)
            {
                logger.Warn("모델 출력이 없습니다.");
                return new List<Detection>();
            }

            var outputTensor = output.Value as Tensor<float>;
            if (outputTensor == null)
            {
                logger.Warn("모델 출력 텐서가 null입니다.");
                return new List<Detection>();
            }

            // 출력 텐서 정보 로깅
            var dims = outputTensor.Dimensions.ToArray();
            logger.Info($"출력 텐서 형식: [{string.Join(", ", dims)}]");

            // YOLO 출력 형식: [1, num_detections, 6] (x, y, w, h, confidence, class)
            // 또는 [1, 8400, 84] 형식일 수 있음 (COCO 80 클래스 + 4 좌표)
            return ParseOutput(outputTensor);
        }

        /// <summary>
        /// 모델 출력 파싱
        /// YOLOv11 ONNX 출력 형식: [1, num_detections, 84]
        /// 각 detection: [x_center, y_center, width, height, objectness, class_prob_0, ..., class_prob_79]
        /// 좌표는 정규화된 값 (0-1) 또는 픽셀 값일 수 있음
        /// </summary>
        private List<Detection> ParseOutput(Tensor<float> outputTensor)
        {
            var detections = new List<Detection>();
            var dimensions = outputTensor.Dimensions.ToArray();

            logger.Info($"출력 텐서 차원: [{string.Join(", ", dimensions)}]");

            // 출력 형식에 따라 파싱
            // YOLOv11 ONNX 출력은 보통 [1, num_detections, 84] 형식
            // 또는 [1, num_detections, 6] 형식일 수 있음
            if (dimensions.Length == 3)
            {
                var batchSize = dimensions[0];
                var numDetections = dimensions[1];
                var numValues = dimensions[2];

                logger.Info($"파싱 시작: batchSize={batchSize}, numDetections={numDetections}, numValues={numValues}");

                int validDetections = 0;
                for (int i = 0; i < numDetections; i++)
                {
                    // YOLOv11 출력 형식: [x_center, y_center, width, height, objectness, class_prob_0, ..., class_prob_79]
                    // 좌표는 정규화된 값 (0-1 범위)일 수 있음
                    var xCenter = outputTensor[0, i, 0];
                    var yCenter = outputTensor[0, i, 1];
                    var width = outputTensor[0, i, 2];
                    var height = outputTensor[0, i, 3];
                    var objectness = outputTensor[0, i, 4]; // 객체 존재 확률

                    // objectness가 낮으면 건너뛰기
                    if (objectness < _confidenceThreshold)
                        continue;

                    // 클래스 확률 찾기 (5번째 인덱스부터 시작)
                    int classId = PersonClassId;
                    float maxClassProb = 0.0f;
                    
                    if (numValues > 5)
                    {
                        // 클래스 확률 중 최대값 찾기 (person = class 0)
                        // YOLOv11은 보통 [x, y, w, h, objectness, class_0, class_1, ..., class_79] 형식
                        for (int c = 5; c < numValues && c < 85; c++)
                        {
                            var classProb = outputTensor[0, i, c];
                            if (classProb > maxClassProb)
                            {
                                maxClassProb = classProb;
                                classId = c - 5; // 클래스 ID는 0부터 시작
                            }
                        }
                    }
                    else
                    {
                        // numValues <= 5인 경우, objectness를 클래스 확률로 사용
                        maxClassProb = objectness;
                        classId = PersonClassId; // 기본값
                    }

                    // 최종 신뢰도 = objectness * class_probability
                    float confidence = objectness * maxClassProb;
                    
                    // 디버깅: 처음 몇 개만 로깅
                    if (validDetections < 5)
                    {
                        logger.Info($"Detection {i}: xCenter={xCenter:F3}, yCenter={yCenter:F3}, w={width:F3}, h={height:F3}, " +
                                   $"objectness={objectness:F3}, maxClassProb={maxClassProb:F3}, classId={classId}, confidence={confidence:F3}");
                    }

                    // 신뢰도 임계값 체크
                    if (confidence < _confidenceThreshold)
                        continue;

                    // 사람 클래스만 필터링
                    if (classId != PersonClassId)
                        continue;

                    // 좌표가 정규화된 값인지 확인 (0-1 범위면 정규화된 값)
                    // YOLOv11은 보통 정규화된 좌표를 출력하므로 640을 곱해야 함
                    float x, y, w, h;
                    
                    // 정규화 여부 판단: 대부분의 값이 1.0 이하이면 정규화된 값으로 간주
                    bool isNormalized = (xCenter <= 1.0f && yCenter <= 1.0f && width <= 1.0f && height <= 1.0f);
                    
                    if (isNormalized)
                    {
                        // 정규화된 좌표 -> 픽셀 좌표로 변환
                        // YOLO 형식: center_x, center_y, width, height (모두 0-1 범위)
                        x = (xCenter - width / 2.0f) * _inputWidth;
                        y = (yCenter - height / 2.0f) * _inputHeight;
                        w = width * _inputWidth;
                        h = height * _inputHeight;
                    }
                    else
                    {
                        // 이미 픽셀 좌표 (center_x, center_y 형식)
                        x = xCenter - width / 2.0f;
                        y = yCenter - height / 2.0f;
                        w = width;
                        h = height;
                    }
                    
                    // 좌표 유효성 검사
                    if (w <= 0 || h <= 0 || x < -w || y < -h)
                    {
                        if (validDetections < 3)
                        {
                            logger.Debug($"유효하지 않은 좌표 건너뜀: x={x:F1}, y={y:F1}, w={w:F1}, h={h:F1}");
                        }
                        continue;
                    }

                    detections.Add(new Detection(x, y, w, h, confidence, classId));
                    validDetections++;
                }

                logger.Info($"유효한 탐지 수: {validDetections} / {numDetections}");
            }
            else
            {
                logger.Warn($"예상하지 못한 출력 형식: {dimensions.Length}차원");
            }

            return detections;
        }

        /// <summary>
        /// 탐지 결과를 원본 이미지 크기로 스케일링
        /// </summary>
        private List<Detection> ScaleDetections(List<Detection> detections, int originalWidth, int originalHeight)
        {
            var scaleX = (float)originalWidth / _inputWidth;
            var scaleY = (float)originalHeight / _inputHeight;

            return detections.Select(d => new Detection(
                d.X * scaleX,
                d.Y * scaleY,
                d.Width * scaleX,
                d.Height * scaleY,
                d.Confidence,
                d.ClassId
            )).ToList();
        }

        /// <summary>
        /// NMS (Non-Maximum Suppression) 적용
        /// </summary>
        private List<Detection> ApplyNMS(List<Detection> detections)
        {
            if (detections.Count == 0)
                return detections;

            // 신뢰도 기준으로 정렬
            var sorted = detections.OrderByDescending(d => d.Confidence).ToList();
            var result = new List<Detection>();

            while (sorted.Count > 0)
            {
                // 가장 높은 신뢰도의 탐지를 결과에 추가
                var best = sorted[0];
                result.Add(best);
                sorted.RemoveAt(0);

                // IoU가 임계값보다 높은 탐지 제거
                sorted.RemoveAll(d => CalculateIoU(best, d) > _nmsThreshold);
            }

            return result;
        }

        /// <summary>
        /// IoU (Intersection over Union) 계산
        /// </summary>
        private float CalculateIoU(Detection box1, Detection box2)
        {
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

        public void Dispose()
        {
            _session?.Dispose();
            _session = null;
        }
    }
}

