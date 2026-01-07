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
        private readonly bool _useCpuOnly;

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
        /// <param name="useCpuOnly">CPU 전용 모드 사용 여부 (기본값: false, GPU 사용 시도)</param>
        public YOLOv11Model(string modelPath, float confidenceThreshold = 0.5f, float nmsThreshold = 0.4f, bool useCpuOnly = false)
        {
            _modelPath = modelPath;
            _confidenceThreshold = confidenceThreshold;
            _nmsThreshold = nmsThreshold;
            _inputWidth = 640;  // YOLOv11 기본 입력 크기
            _inputHeight = 640;
            _useCpuOnly = useCpuOnly;
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
                
                // CPU 전용 모드가 아니면 GPU 사용 시도 (우선순위: DirectML > CUDA > CPU)
                if (!_useCpuOnly)
                {
                    bool gpuInitialized = false;
                    
                    // Windows에서 DirectML 사용 (AMD/NVIDIA/Intel GPU 모두 지원)
                    try
                    {
                        options.AppendExecutionProvider_DML();
                        logger.Info("DirectML GPU 실행 제공자 추가 성공");
                        gpuInitialized = true;
                    }
                    catch (Exception ex)
                    {
                        logger.Warn($"DirectML GPU 실행 제공자 추가 실패: {ex.Message}. CUDA 시도...");
                        
                        // NVIDIA GPU용 CUDA 사용
                        try
                        {
                            options.AppendExecutionProvider_CUDA();
                            logger.Info("CUDA GPU 실행 제공자 추가 성공");
                            gpuInitialized = true;
                        }
                        catch (Exception ex2)
                        {
                            logger.Warn($"CUDA GPU 실행 제공자 추가 실패: {ex2.Message}. CPU로 실행합니다.");
                        }
                    }
                    
                    if (!gpuInitialized)
                    {
                        logger.Info("GPU 초기화 실패. CPU 모드로 실행합니다.");
                    }
                }
                else
                {
                    logger.Info("CPU 전용 모드로 실행합니다.");
                }
                
                _session = new InferenceSession(_modelPath, options);
                logger.Info($"YOLOv11 모델 로드 완료: {_modelPath}");
                
                // 입력 메타데이터 확인 및 로깅
                foreach (var inputMeta in _session.InputMetadata)
                {
                    logger.Info($"입력: 이름={inputMeta.Key}, 형식={inputMeta.Value.ElementType}, 차원=[{string.Join(", ", inputMeta.Value.Dimensions)}]");
                }
                
                // 출력 메타데이터 확인 및 로깅
                foreach (var outputMeta in _session.OutputMetadata)
                {
                    logger.Info($"출력: 이름={outputMeta.Key}, 형식={outputMeta.Value.ElementType}, 차원=[{string.Join(", ", outputMeta.Value.Dimensions)}]");
                }
                
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
            Mat rgb = null;
            Mat resized = null;
            Mat normalized = null;
            
            try
            {
                // RGB로 변환 (BGR -> RGB)
                rgb = new Mat();
                Cv2.CvtColor(frame, rgb, ColorConversionCodes.BGR2RGB);

                // 리사이즈
                resized = new Mat();
                Cv2.Resize(rgb, resized, new Size(_inputWidth, _inputHeight), 0, 0, InterpolationFlags.Linear);

                // 정규화 (0-255 -> 0-1)
                normalized = new Mat();
                resized.ConvertTo(normalized, MatType.CV_32F, 1.0 / 255.0);

                // 텐서 생성 [1, 3, 640, 640]
                var tensorSize = 1 * 3 * _inputHeight * _inputWidth;
                var tensorData = new float[tensorSize];
                var index = 0;

                // CHW 형식으로 변환 (Channel, Height, Width)
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

                // 텐서 생성 및 반환 (메모리 복사)
                var tensor = new DenseTensor<float>(tensorData, new[] { 1, 3, _inputHeight, _inputWidth });
                logger.Debug($"텐서 생성 완료: 크기=[1, 3, {_inputHeight}, {_inputWidth}], 데이터 크기={tensorData.Length}");
                return tensor;
            }
            finally
            {
                // 리소스 정리
                rgb?.Dispose();
                resized?.Dispose();
                normalized?.Dispose();
            }
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

            // 입력 메타데이터 확인 및 검증
            if (_session.InputMetadata.ContainsKey(inputName))
            {
                var inputMeta = _session.InputMetadata[inputName];
                logger.Info($"입력 메타데이터: 이름={inputName}, 형식={inputMeta.ElementType}, 차원=[{string.Join(", ", inputMeta.Dimensions)}]");
                
                // 텐서 차원 검증
                var expectedDims = inputMeta.Dimensions.ToArray();
                var actualDims = inputTensor.Dimensions.ToArray();
                
                if (expectedDims.Length != actualDims.Length)
                {
                    logger.Error($"입력 텐서 차원 불일치: 예상={expectedDims.Length}, 실제={actualDims.Length}");
                    return new List<Detection>();
                }
                
                for (int i = 0; i < expectedDims.Length; i++)
                {
                    // -1은 동적 차원이므로 건너뜀
                    if (expectedDims[i] != -1 && expectedDims[i] != actualDims[i])
                    {
                        logger.Error($"입력 텐서 차원[{i}] 불일치: 예상={expectedDims[i]}, 실제={actualDims[i]}");
                        return new List<Detection>();
                    }
                }
            }

            // 텐서 데이터 검증
            if (inputTensor.Length == 0)
            {
                logger.Error("입력 텐서가 비어있습니다.");
                return new List<Detection>();
            }

            try
            {
                var inputs = new List<NamedOnnxValue>
                {
                    NamedOnnxValue.CreateFromTensor(inputName, inputTensor)
                };

                logger.Info($"모델 추론 실행 중... (입력 텐서 크기: {inputTensor.Length})");
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
            catch (AccessViolationException ex)
            {
                logger.Error($"액세스 위반 오류 발생: {ex.Message}", ex);
                logger.Error("입력 텐서 형식이나 모델과의 호환성을 확인하세요.");
                return new List<Detection>();
            }
            catch (Exception ex)
            {
                logger.Error($"모델 추론 중 오류 발생: {ex.Message}", ex);
                return new List<Detection>();
            }
        }

        /// <summary>
        /// 모델 출력 파싱
        /// YOLOv11 ONNX 출력 형식: [1, 84, 8400] (batch, features, detections)
        /// Python 코드 참고: outputs[0].transpose(1, 0) -> (8400, 84)
        /// 각 detection: [cx, cy, w, h, class_0, class_1, ..., class_79]
        /// 좌표는 입력 이미지 크기(640x640) 기준 픽셀 값
        /// </summary>
        private List<Detection> ParseOutput(Tensor<float> outputTensor)
        {
            var detections = new List<Detection>();
            var dimensions = outputTensor.Dimensions.ToArray();

            logger.Info($"출력 텐서 차원: [{string.Join(", ", dimensions)}]");

            if (dimensions.Length == 3)
            {
                var batchSize = dimensions[0];
                var dim1 = dimensions[1];
                var dim2 = dimensions[2];

                // 형식 판단: [1, 84, 8400] vs [1, 8400, 84]
                bool isFeatureFirst = (dim1 < dim2); // features가 detections보다 작으면 [1, features, detections]
                int numDetections = isFeatureFirst ? dim2 : dim1;
                int numFeatures = isFeatureFirst ? dim1 : dim2;

                logger.Info($"파싱 시작: batchSize={batchSize}, 형식={(isFeatureFirst ? "[1, features, detections]" : "[1, detections, features]")}, numDetections={numDetections}, numFeatures={numFeatures}");

                int validDetections = 0;
                for (int i = 0; i < numDetections; i++)
                {
                    // YOLOv11 출력 형식: [cx, cy, w, h, class_0, class_1, ..., class_79]
                    // 84 = 4 (좌표) + 80 (클래스)
                    // 좌표는 입력 이미지 크기(640x640) 기준 픽셀 값
                    // 클래스 점수는 이미 sigmoid가 적용된 값 (0-1 범위)
                    float cx, cy, w, h;
                    
                    if (isFeatureFirst)
                    {
                        // [1, 84, 8400] 형식: outputTensor[0, feature_index, detection_index]
                        cx = outputTensor[0, 0, i];
                        cy = outputTensor[0, 1, i];
                        w = outputTensor[0, 2, i];
                        h = outputTensor[0, 3, i];
                    }
                    else
                    {
                        // [1, 8400, 84] 형식: outputTensor[0, detection_index, feature_index]
                        cx = outputTensor[0, i, 0];
                        cy = outputTensor[0, i, 1];
                        w = outputTensor[0, i, 2];
                        h = outputTensor[0, i, 3];
                    }

                    // Python 코드 참고: cls_id = np.argmax(cls_scores), confidence = cls_scores[cls_id]
                    // 모든 클래스 점수 중 최대값을 찾고, 그 값이 임계값보다 크면 탐지
                    // 하지만 우리는 person만 찾으므로 person 클래스 점수만 확인
                    float personClassScore;
                    float maxClassScore = float.MinValue;
                    int maxClassId = -1;
                    
                    // 모든 클래스 점수 확인 (인덱스 4~83 = 클래스 0~79)
                    for (int c = 4; c < numFeatures && c < 84; c++)
                    {
                        float classScore;
                        if (isFeatureFirst)
                        {
                            classScore = outputTensor[0, c, i];
                        }
                        else
                        {
                            classScore = outputTensor[0, i, c];
                        }
                        
                        if (classScore > maxClassScore)
                        {
                            maxClassScore = classScore;
                            maxClassId = c - 4; // 클래스 ID는 0부터 시작
                        }
                    }
                    
                    // person 클래스 점수 확인 (인덱스 4 = 클래스 0)
                    if (isFeatureFirst)
                    {
                        personClassScore = outputTensor[0, 4, i]; // person = class 0
                    }
                    else
                    {
                        personClassScore = outputTensor[0, i, 4]; // person = class 0
                    }

                    // Python 코드 참고: if max(o[4:] > 0.5) - 클래스 점수 중 최대값이 0.5보다 크면
                    // 그리고 if confidence > conf_threshold - 최종 confidence가 임계값보다 크면
                    // person 클래스만 찾으므로 person 클래스 점수를 confidence로 사용
                    float confidence = personClassScore;
                    
                    // Python 코드: if max(o[4:] > 0.5) - 최대 클래스 점수가 0.5보다 크면
                    // 그리고 person 클래스 점수가 임계값보다 크면 탐지
                    if (maxClassScore < 0.5f || confidence < _confidenceThreshold)
                        continue;
                    
                    // person 클래스가 아니면 건너뛰기
                    if (maxClassId != PersonClassId)
                        continue;
                    
                    // 디버깅: 처음 몇 개만 로깅
                    if (validDetections < 5)
                    {
                        logger.Info($"Detection {i}: cx={cx:F3}, cy={cy:F3}, w={w:F3}, h={h:F3}, " +
                                   $"personScore={personClassScore:F3}, confidence={confidence:F3}");
                    }

                    // Python 코드 참고: x1 = (cx - w/2), y1 = (cy - h/2)
                    // 좌표는 이미 입력 이미지 크기(640x640) 기준 픽셀 값
                    float x1 = cx - w / 2.0f;
                    float y1 = cy - h / 2.0f;
                    float x2 = cx + w / 2.0f;
                    float y2 = cy + h / 2.0f;
                    
                    // Python 코드 참고: 좌표가 이미지 경계를 벗어나지 않도록 클램핑
                    x1 = Math.Max(0, Math.Min(x1, _inputWidth));
                    y1 = Math.Max(0, Math.Min(y1, _inputHeight));
                    x2 = Math.Max(0, Math.Min(x2, _inputWidth));
                    y2 = Math.Max(0, Math.Min(y2, _inputHeight));
                    
                    // 좌상단 좌표와 크기로 변환
                    float x = x1;
                    float y = y1;
                    float width = x2 - x1;
                    float height = y2 - y1;
                    
                    // 좌표 유효성 검사
                    if (width <= 0 || height <= 0 || width < 10 || height < 10)
                    {
                        continue;
                    }

                    detections.Add(new Detection(x, y, width, height, confidence, PersonClassId));
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

