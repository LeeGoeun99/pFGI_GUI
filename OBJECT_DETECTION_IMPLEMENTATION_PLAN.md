# 객체탐지 모드 구현 계획

## 1. 개요
객체탐지 모드 선택 시 YOLOv11 모델을 사용하여 RGB 영상에서 사람을 탐지하고, 탐지된 사람을 ID로 구분하여 bounding box 좌표를 누적 기록하는 기능을 구현합니다.

## 2. 요구사항 분석
- **모델**: YOLOv11 (사람 클래스만 탐지)
- **입력**: RealSense 카메라의 RGB 영상 (실시간 스트림)
- **출력**: 
  - 탐지된 사람의 bounding box 좌표
  - 각 사람의 고유 ID
  - ID별 bounding box 좌표 누적 기록

## 3. 기술 스택 및 통합 방식 선택

### 3.1 현재 코드베이스 구조 분석
프로젝트는 두 가지 wrapper 패턴을 사용 중:
- **C++ 코드**: C++/CLI wrapper (`SLAM wrapper`)를 통해 C#에서 사용
- **Python 코드**: Python.NET (`pythonnet_netstandard_py38_win`)을 통해 사용 (예: `Energy Spectrum.cs`에서 nasagamma 모듈 호출)

### 3.2 YOLOv11 통합 방식 선택

#### 옵션 1: ONNX Runtime 사용 (권장) ⭐
- **방식**: C#에서 직접 ONNX 모델 실행 (Python 없이)
- **패키지**: `Microsoft.ML.OnnxRuntime`
- **장점**:
  - ✅ **성능 우수**: Python 인터프리터 오버헤드 없음
  - ✅ **배포 간단**: Python 환경 불필요
  - ✅ **메모리 효율**: 더 적은 메모리 사용
  - ✅ **속도 빠름**: 네이티브 C# 실행
  - ✅ **안정성**: Python GIL 문제 없음
- **단점**:
  - 모델을 ONNX 형식으로 변환 필요

#### 옵션 2: Python.NET 사용 (기존 패턴과 일치)
- **방식**: Python.NET을 통해 ultralytics YOLOv11 호출
- **패키지**: `pythonnet_netstandard_py38_win` (이미 사용 중)
- **장점**:
  - ✅ 기존 Python 호출 패턴과 일치 (Energy Spectrum.cs 참고)
  - ✅ YOLOv11 공식 Python API 그대로 사용 가능
- **단점**:
  - ❌ **성능 오버헤드**: Python 인터프리터 오버헤드
  - ❌ **배포 복잡**: Python 환경 필요 (PyTorch, ultralytics 등)
  - ❌ **메모리 사용량 증가**: Python 런타임 메모리
  - ❌ **속도 느림**: Python → C# 마샬링 비용
  - ❌ **GIL 문제**: 멀티스레드 환경에서 제약

### 3.3 권장 방식: ONNX Runtime ⭐

**최종 기술 스택 (ONNX Runtime 방식)**:
- **프레임워크**: .NET 6.0 (WPF)
- **영상 처리**: OpenCvSharp4 (이미 사용 중)
- **딥러닝 모델**: YOLOv11 (ONNX 형식)
- **ONNX 런타임**: `Microsoft.ML.OnnxRuntime` (NuGet 패키지)
- **추적 알고리즘**: IoU 기반 또는 ByteTrack (사람 ID 유지용)

**모델 변환 방법**:
```python
# Python에서 YOLOv11을 ONNX로 변환 (한 번만 실행)
from ultralytics import YOLO
model = YOLO('yolov11n.pt')  # 또는 yolov11s.pt, yolov11m.pt 등
model.export(format='onnx')  # yolov11n.onnx 생성
```

### 3.4 Python.NET 방식 선택 시 (비권장)
만약 기존 Python 패턴을 유지하려면:
- `pythonnet_netstandard_py38_win` 패키지 사용 (이미 있음)
- ultralytics YOLOv11 Python 패키지 설치 필요
- `Energy Spectrum.cs`의 Python 호출 패턴 참고하여 구현
- 성능 및 배포 복잡도 증가

## 4. 아키텍처 설계

### 4.1 전체 구조
```
TopButtonViewModel (MeasurementMode 관리)
    ↓
ObjectDetectionService (객체탐지 서비스)
    ├── YOLOv11Model (모델 로드 및 추론)
    ├── PersonTracker (사람 추적 및 ID 관리)
    └── DetectionDataManager (Bounding box 좌표 누적 기록)
    ↓
ReconstructionImageViewModel (RGB 영상 처리 루프)
    └── Loop() 메서드에서 객체탐지 모드일 때 처리
```

### 4.2 주요 컴포넌트

#### 4.2.1 ObjectDetectionService 클래스
- **역할**: 객체탐지 및 추적의 전체적인 흐름 관리
- **위치**: `HUREL Imager GUI/ViewModel/ObjectDetection/ObjectDetectionService.cs`
- **주요 메서드**:
  - `Initialize()`: YOLOv11 모델 및 추적기 초기화
  - `ProcessFrame(Mat frame)`: 프레임 단위 객체탐지 수행
  - `Dispose()`: 리소스 해제

#### 4.2.2 YOLOv11Model 클래스
- **역할**: YOLOv11 모델 로드 및 추론
- **위치**: `HUREL Imager GUI/ViewModel/ObjectDetection/YOLOv11Model.cs`
- **주요 메서드**:
  - `LoadModel(string modelPath)`: ONNX 모델 로드
  - `Predict(Mat frame)`: 입력 영상에서 사람 탐지
  - **출력**: List<Detection> (x, y, width, height, confidence, classId=0 (person))

#### 4.2.3 PersonTracker 클래스
- **역할**: 탐지된 사람을 추적하여 ID 할당 및 유지
- **위치**: `HUREL Imager GUI/ViewModel/ObjectDetection/PersonTracker.cs`
- **주요 메서드**:
  - `Update(List<Detection> detections)`: 새로운 탐지 결과로 추적 업데이트
  - **출력**: List<TrackedPerson> (id, boundingBox, timestamp)

#### 4.2.4 DetectionDataManager 클래스
- **역할**: ID별 bounding box 좌표 누적 기록 및 저장
- **위치**: `HUREL Imager GUI/ViewModel/ObjectDetection/DetectionDataManager.cs`
- **주요 메서드**:
  - `RecordDetection(int personId, BoundingBox box, DateTime timestamp)`: 좌표 기록
  - `GetPersonTrajectory(int personId)`: 특정 ID의 누적 좌표 반환
  - `SaveToFile(string filePath)`: 데이터를 파일로 저장 (CSV 또는 JSON)
  - `Clear()`: 측정 종료 시 데이터 초기화

#### 4.2.5 데이터 모델
```csharp
public class Detection
{
    public float X { get; set; }        // Bounding box 좌상단 X
    public float Y { get; set; }        // Bounding box 좌상단 Y
    public float Width { get; set; }    // Bounding box 너비
    public float Height { get; set; }   // Bounding box 높이
    public float Confidence { get; set; } // 탐지 신뢰도
    public int ClassId { get; set; }    // 클래스 ID (0 = person)
}

public class TrackedPerson
{
    public int Id { get; set; }         // 고유 ID
    public BoundingBox BoundingBox { get; set; }
    public DateTime Timestamp { get; set; }
}

public class BoundingBox
{
    public float X { get; set; }
    public float Y { get; set; }
    public float Width { get; set; }
    public float Height { get; set; }
}

public class PersonTrajectory
{
    public int PersonId { get; set; }
    public List<BoundingBoxRecord> Records { get; set; }
}

public class BoundingBoxRecord
{
    public BoundingBox Box { get; set; }
    public DateTime Timestamp { get; set; }
}
```

## 5. 구현 단계

### 5.1 Phase 1: 인프라 구축 및 모델 준비

**ONNX Runtime 방식 (권장)** ⭐

1. **NuGet 패키지 추가**
   - `Microsoft.ML.OnnxRuntime` (ONNX 런타임)
   - 필요 시 `System.Text.Json` (JSON 저장용)

2. **YOLOv11 모델 준비 및 변환**
   ```python
   # Python 환경에서 실행 (한 번만)
   from ultralytics import YOLO
   model = YOLO('yolov11n.pt')  # nano 버전 권장 (속도 우선)
   model.export(format='onnx')  # yolov11n.onnx 생성
   ```
   - YOLOv11 모델 다운로드 (COCO 데이터셋 기반, 사람 클래스 포함)
   - ONNX 형식으로 변환
   - 모델 파일을 `HUREL Imager GUI/Models/` 폴더에 배치
   - 또는 설정 파일에서 모델 경로 지정 가능하도록 구현

**Python.NET 방식 (비권장, 기존 패턴 유지 시)**

1. **Python 환경 설정**
   - Python 3.8+ 설치 확인
   - ultralytics 패키지 설치: `pip install ultralytics`
   - PyTorch 설치 (CPU 또는 GPU 버전)

2. **C# 코드에서 Python 호출**
   - `pythonnet_netstandard_py38_win` 패키지 사용 (이미 있음)
   - `Energy Spectrum.cs`의 Python 호출 패턴 참고

3. **폴더 구조 생성**
   ```
   HUREL Imager GUI/
   ├── ViewModel/
   │   └── ObjectDetection/
   │       ├── ObjectDetectionService.cs
   │       ├── YOLOv11Model.cs
   │       ├── PersonTracker.cs
   │       ├── DetectionDataManager.cs
   │       └── Models/
   │           ├── Detection.cs
   │           ├── TrackedPerson.cs
   │           └── PersonTrajectory.cs
   ├── Models/
   │   └── yolov11n.pt 또는 yolov11n.onnx
   ```

### 5.2 Phase 2: YOLOv11 모델 통합

**ONNX Runtime 방식 (권장)** ⭐

1. **YOLOv11Model 클래스 구현**
   ```csharp
   using Microsoft.ML.OnnxRuntime;
   using Microsoft.ML.OnnxRuntime.Tensors;
   using OpenCvSharp;
   ```
   - `InferenceSession`을 사용하여 ONNX 모델 로드
   - 입력 이미지 전처리 (리사이즈 640x640, 정규화)
   - `Tensor<float>` 생성 및 모델 추론 실행
   - 출력 후처리 (NMS, 사람 클래스(classId=0)만 필터링)
   - Confidence threshold 설정 (예: 0.5)

2. **테스트**
   - 정적 이미지로 테스트
   - 탐지 결과 확인

**Python.NET 방식 (비권장)**

1. **YOLOv11Model 클래스 구현**
   ```csharp
   using Python.Runtime;  // pythonnet 사용
   ```
   - PythonEngine 초기화 (기존 Energy Spectrum.cs 패턴 참고)
   - ultralytics YOLO 모델 로드
   - OpenCvSharp Mat를 Python numpy array로 변환
   - Python 모델.predict() 호출
   - 결과를 C# 객체로 변환

2. **주의사항**
   - GIL (Global Interpreter Lock) 문제 고려
   - 메모리 누수 방지를 위한 적절한 해제 필요
   - 성능 오버헤드 존재

### 5.3 Phase 3: 사람 추적 구현
1. **PersonTracker 클래스 구현**
   - 추적 알고리즘 선택:
     - **옵션 1**: 간단한 IoU 기반 추적 (구현 간단)
     - **옵션 2**: DeepSORT (더 정확하지만 복잡)
     - **옵션 3**: ByteTrack (YOLO와 잘 맞음, 권장)
   - ID 할당 및 유지 로직
   - 프레임 간 매칭 알고리즘

2. **테스트**
   - 동영상으로 테스트
   - ID 유지 여부 확인

### 5.4 Phase 4: 데이터 기록 및 관리
1. **DetectionDataManager 클래스 구현**
   - 인메모리 데이터 구조 (Dictionary<int, PersonTrajectory>)
   - 좌표 기록 메서드
   - 파일 저장 기능 (CSV 형식 권장)
     - 형식: `PersonId, Timestamp, X, Y, Width, Height`

2. **측정 종료 시 저장**
   - `TopButtonViewModel.StopSession()`에서 호출
   - 저장 경로: 측정 데이터 폴더 내 `{FileName}_object_detection.csv`

### 5.5 Phase 5: ReconstructionImageViewModel 통합
1. **Loop() 메서드 수정**
   - `MeasurementMode == eMeasurementMode.ObjectDetection` 체크
   - RGB 영상 획득 후 `ObjectDetectionService.ProcessFrame()` 호출
   - 탐지 결과를 영상에 그리기 (선택사항)

2. **UI 표시 (선택사항)**
   - 탐지된 사람 수 표시
   - 실시간 bounding box 오버레이

### 5.6 Phase 6: TopButtonViewModel 통합
1. **ObjectDetectionService 인스턴스 관리**
   - `TopButtonViewModel`에 `ObjectDetectionService` 속성 추가
   - 측정 시작 시 초기화
   - 측정 종료 시 정리 및 데이터 저장

2. **MeasurementMode 조건부 로직**
   - `StartSession()`에서 객체탐지 모드 확인
   - 필요한 경우 별도 초기화 로직 추가

## 6. 세부 구현 고려사항

### 6.1 성능 최적화
- **비동기 처리**: 객체탐지를 별도 Task로 실행하여 UI 블로킹 방지
- **프레임 스킵**: 필요시 일정 간격으로만 탐지 수행 (예: 3프레임마다)
- **모델 크기**: YOLOv11n (nano) 권장 (속도 우선)

### 6.2 추적 알고리즘 간단 구현 (IoU 기반)
```csharp
// 간단한 추적 알고리즘 (참고)
// 1. 현재 프레임의 탐지 결과와 이전 프레임의 추적 결과를 IoU로 매칭
// 2. IoU > 0.3 이상인 경우 같은 ID로 매칭
// 3. 매칭되지 않은 새로운 탐지는 새 ID 할당
// 4. 일정 프레임 동안 추적되지 않은 ID는 제거
```

### 6.3 데이터 저장 형식
**CSV 형식 예시:**
```csv
PersonId,Timestamp,X,Y,Width,Height,Confidence
1,2024-01-01 10:00:00.123,100.5,200.3,150.0,300.0,0.95
1,2024-01-01 10:00:00.233,102.1,201.0,148.5,298.5,0.94
2,2024-01-01 10:00:00.123,500.0,300.0,120.0,250.0,0.88
```

### 6.4 에러 처리
- 모델 로드 실패 시 처리
- 추론 중 예외 처리
- 메모리 부족 시 처리

## 7. 테스트 계획
1. **단위 테스트**
   - YOLOv11Model: 정적 이미지 테스트
   - PersonTracker: 추적 정확도 테스트
   - DetectionDataManager: 데이터 저장/로드 테스트

2. **통합 테스트**
   - 객체탐지 모드에서 전체 플로우 테스트
   - 여러 사람이 동시에 있을 때 테스트
   - 사람이 프레임을 벗어났다가 다시 들어올 때 테스트

3. **성능 테스트**
   - FPS 측정 (목표: 최소 10 FPS)
   - 메모리 사용량 모니터링

## 8. 추후 개선 사항
- 실시간 UI에 bounding box 오버레이
- 탐지 결과 통계 표시 (최대 동시 탐지 인원 수 등)
- 데이터 분석 기능 (이동 경로 시각화)
- YOLOv11 외 다른 모델 지원 (설정에서 선택)
- GPU 가속 지원 (ONNX Runtime GPU 버전)

## 9. 참고 자료

### ONNX Runtime 방식
- YOLOv11: https://github.com/ultralytics/ultralytics
- ONNX Runtime C#: https://onnxruntime.ai/docs/get-started/with-csharp.html
- ONNX Runtime NuGet: https://www.nuget.org/packages/Microsoft.ML.OnnxRuntime
- OpenCvSharp: https://github.com/shimat/opencvsharp
- ByteTrack: https://github.com/ifzhang/ByteTrack (추적 알고리즘 참고)
- YOLO ONNX 변환: https://docs.ultralytics.com/modes/export/#onnx

### Python.NET 방식 (참고용)
- Python.NET: https://github.com/pythonnet/pythonnet
- 기존 코드 참고: `Hurel Radiation Imager/Energy Spectrum.cs` (nasagamma 모델 호출 패턴)

## 10. 최종 권장사항

**ONNX Runtime 방식 강력 권장** ⭐

이유:
1. **성능**: 실시간 영상 처리에서 Python 오버헤드는 치명적
2. **배포**: Python 환경 없이 독립 실행 가능
3. **안정성**: GIL 문제 등 Python의 한계 없음
4. **유지보수**: C# 코드베이스와 일관성 유지

단, 모델 변환은 Python 환경이 필요하지만, **개발 시 한 번만** 실행하면 됩니다.

