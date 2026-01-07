"""
YOLOv11 .pt 파일을 .onnx 파일로 변환하는 스크립트

사용 방법:
1. 이 스크립트가 있는 폴더에 yolo11n.pt 파일을 넣어주세요
2. Python 환경에서 실행:
   python convert_pt_to_onnx.py
3. 변환된 yolo11n.onnx 파일이 생성됩니다
"""

from ultralytics import YOLO
import os

def convert_pt_to_onnx():
    # 현재 스크립트가 있는 폴더 경로
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # .pt 파일 경로
    pt_file = os.path.join(script_dir, "yolo11n.pt")
    
    if not os.path.exists(pt_file):
        print(f"오류: {pt_file} 파일을 찾을 수 없습니다.")
        print("yolo11n.pt 파일을 이 스크립트와 같은 폴더에 넣어주세요.")
        return False
    
    try:
        print(f"모델 로드 중: {pt_file}")
        model = YOLO(pt_file)
        
        print("ONNX 형식으로 변환 중...")
        # ONNX 형식으로 변환 (yolo11n.onnx 파일 생성)
        model.export(format='onnx')
        
        onnx_file = os.path.join(script_dir, "yolo11n.onnx")
        if os.path.exists(onnx_file):
            print(f"변환 완료: {onnx_file}")
            print(f"파일 크기: {os.path.getsize(onnx_file) / (1024*1024):.2f} MB")
            return True
        else:
            print("오류: 변환된 .onnx 파일을 찾을 수 없습니다.")
            return False
            
    except Exception as e:
        print(f"변환 중 오류 발생: {e}")
        print("\n필요한 패키지 설치:")
        print("  pip install ultralytics")
        return False

if __name__ == "__main__":
    convert_pt_to_onnx()

