# YOLOv11 모델 파일 안내

## 모델 파일 위치

이 폴더에 다음 파일 중 하나를 넣어주세요:

- **yolo11n.onnx** (권장) - ONNX 형식 모델 파일
- **yolo11n.pt** - PyTorch 형식 모델 파일 (변환 필요)

## .pt 파일을 .onnx로 변환하기

만약 `yolo11n.pt` 파일만 있다면:

1. `yolo11n.pt` 파일을 이 폴더(`Models`)에 넣어주세요
2. Python 환경에서 다음 명령 실행:
   ```bash
   cd "HUREL Imager GUI\Models"
   python convert_pt_to_onnx.py
   ```
3. 변환된 `yolo11n.onnx` 파일이 생성됩니다

## 필요한 Python 패키지

```bash
pip install ultralytics
```

## 모델 파일 다운로드

YOLOv11 모델이 없다면:

```python
from ultralytics import YOLO

# 모델 다운로드
model = YOLO('yolo11n.pt')  # nano 버전 (가장 빠름)

# ONNX로 변환
model.export(format='onnx')  # yolo11n.onnx 생성
```

## 파일 경로 우선순위

프로그램은 다음 순서로 모델 파일을 찾습니다:

1. `{실행파일경로}/Models/yolo11n.onnx`
2. `{실행파일경로}/yolo11n.onnx`

