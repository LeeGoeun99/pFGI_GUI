import os
import filecmp
import difflib
from pathlib import Path

# 소스 파일 확장자
SOURCE_EXTENSIONS = {'.cs', '.cpp', '.h', '.xaml', '.xaml.cs', '.hpp', '.c'}

def find_source_files(root_dir):
    """주어진 디렉토리에서 모든 소스 파일을 찾습니다."""
    source_files = []
    for root, dirs, files in os.walk(root_dir):
        # 특정 디렉토리 제외
        dirs[:] = [d for d in dirs if d not in ['bin', 'obj', 'Debug', 'Release', 'x64', '.git', 'node_modules']]
        
        for file in files:
            file_path = Path(root) / file
            if file_path.suffix in SOURCE_EXTENSIONS or file_path.suffixes[-2:] == ['.xaml', '.cs']:
                rel_path = file_path.relative_to(root_dir)
                source_files.append(rel_path)
    return sorted(source_files)

def compare_files(file1, file2):
    """두 파일의 내용을 비교합니다."""
    try:
        with open(file1, 'r', encoding='utf-8', errors='ignore') as f1:
            lines1 = f1.readlines()
        with open(file2, 'r', encoding='utf-8', errors='ignore') as f2:
            lines2 = f2.readlines()
        
        if lines1 == lines2:
            return None, "identical"
        
        diff = list(difflib.unified_diff(lines1, lines2, fromfile=str(file1), tofile=str(file2), lineterm=''))
        return diff, "different"
    except Exception as e:
        return None, f"error: {str(e)}"

def main():
    current_dir = Path.cwd()
    github_dir = current_dir.parent / "pFGI_GUI_temp"
    
    if not github_dir.exists():
        print(f"GitHub 디렉토리를 찾을 수 없습니다: {github_dir}")
        return
    
    print("소스 파일 검색 중...")
    current_files = set(find_source_files(current_dir))
    github_files = set(find_source_files(github_dir))
    
    # 양쪽에 모두 있는 파일
    common_files = current_files & github_files
    # GitHub에만 있는 파일
    github_only = github_files - current_files
    # 현재에만 있는 파일
    current_only = current_files - github_files
    
    print(f"\n비교 결과:")
    print(f"공통 파일: {len(common_files)}개")
    print(f"GitHub에만 있는 파일: {len(github_only)}개")
    print(f"현재에만 있는 파일: {len(current_only)}개")
    
    # 차이점이 있는 파일 찾기
    different_files = []
    print("\n파일 비교 중...")
    for file in sorted(common_files):
        current_file = current_dir / file
        github_file = github_dir / file
        
        if current_file.exists() and github_file.exists():
            diff, status = compare_files(current_file, github_file)
            if status == "different":
                different_files.append(str(file))
                print(f"차이: {file}")
    
    # 결과를 파일로 저장
    output_file = current_dir / "file_differences.txt"
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("=== 차이점이 있는 파일 목록 ===\n\n")
        for file in different_files:
            f.write(f"{file}\n")
        
        f.write(f"\n=== GitHub에만 있는 파일 ===\n\n")
        for file in sorted(github_only):
            f.write(f"{file}\n")
    
    print(f"\n비교 결과가 {output_file}에 저장되었습니다.")
    print(f"\n총 {len(different_files)}개의 파일이 다릅니다.")
    
    return different_files, github_only

if __name__ == "__main__":
    main()
