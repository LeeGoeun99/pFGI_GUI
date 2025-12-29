import os
import shutil
from pathlib import Path

def update_files_from_github():
    """GitHub 리포지토리의 파일들을 현재 디렉토리로 복사합니다."""
    current_dir = Path.cwd()
    github_dir = current_dir.parent / "pFGI_GUI_temp"
    
    # 차이점이 있는 파일 목록
    different_files = [
        "Compton GUI WPF/Model/HUREL/LACC.cs",
        "HUREL Imager GUI/Components/HomeView/IsotopeTable.xaml.cs",
        "HUREL Imager GUI/Components/HomeView/SpectrumView.xaml",
        "HUREL Imager GUI/Components/HomeView/TopButtonsView.xaml",
        "HUREL Imager GUI/Config/Config.cs",
        "HUREL Imager GUI/Converters/Converters.cs",
        "HUREL Imager GUI/ViewModel/HomeView/DoseRateViewModel.cs",
        "HUREL Imager GUI/ViewModel/HomeView/HomeViewModel.cs",
        "HUREL Imager GUI/ViewModel/HomeView/ReconstructionImageViewModel.cs",
        "HUREL Imager GUI/ViewModel/HomeView/SpectrumViewModel.cs",
        "HUREL Imager GUI/ViewModel/HomeView/TopButtonViewModel.cs",
        "HUREL Imager GUI/ViewModel/InteractionPointViewModel.cs",
        "HUREL Imager GUI/ViewModel/MainWindowViewModel.cs",
        "HUREL Imager GUI/Views/MainWindow.xaml",
        "HUREL Imager GUI/Views/MainWindow.xaml.cs",
        "HUREL Imager GUI/Views/SettingWindow.xaml",
        "Hurel Radiation Imager/CRUXELL Impletement/CRUXELLLACC.cs",
        "Hurel Radiation Imager/CRUXELL Impletement/Datapipelining.cs",
        "Hurel Radiation Imager/Energy Spectrum.cs",
        "Hurel Radiation Imager/Lahgi Class.cs",
        "Hurel Radiation Imager/LahgiSerialControl.cs",
        "Image reconstruction/LahgiControl.cpp",
        "Image reconstruction/RadiationImage.cpp",
        "Image reconstruction/RadiationImage.h",
        "Image reconstruction/ReconPointCloud.cpp",
        "Rtabmap SLAM Control Library/RtabmapSlamControl.cpp",
        "Rtabmap SLAM Control Library/RtabmapSlamControl.h",
        "SLAM wrapper/CppWrapper.cpp",
        "SLAM wrapper/CppWrapper.h",
        "SLAM wrapper/RtabmapWrapper.cpp",
        "SLAM wrapper/RtabmapWrapper.h",
    ]
    
    updated_count = 0
    failed_files = []
    
    for file_path in different_files:
        github_file = github_dir / file_path
        current_file = current_dir / file_path
        
        if github_file.exists():
            try:
                # 디렉토리가 없으면 생성
                current_file.parent.mkdir(parents=True, exist_ok=True)
                
                # 파일 복사
                shutil.copy2(github_file, current_file)
                print(f"업데이트됨: {file_path}")
                updated_count += 1
            except Exception as e:
                print(f"오류 ({file_path}): {e}")
                failed_files.append(file_path)
        else:
            print(f"파일을 찾을 수 없음: {file_path}")
            failed_files.append(file_path)
    
    print(f"\n총 {updated_count}개 파일이 업데이트되었습니다.")
    if failed_files:
        print(f"\n업데이트 실패한 파일 ({len(failed_files)}개):")
        for f in failed_files:
            print(f"  - {f}")

if __name__ == "__main__":
    update_files_from_github()
