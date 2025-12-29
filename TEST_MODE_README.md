# 테스트 모드 사용 가이드

## 개요
테스트 모드는 검출기와 신호처리 시스템이 연결되지 않아도 내부 코드를 테스트할 수 있도록 하는 기능입니다.

## 활성화 방법

### 1. App.config 파일 수정

**위치**: `HUREL Imager GUI\App.config` 파일을 열고 `<appSettings>` 섹션을 확인합니다.

이미 파일이 생성되어 있으므로, `TestMode` 값을 수정하면 됩니다:

```xml
<configuration>
  <appSettings>
    <!-- 테스트 모드 활성화하려면 value="true"로 변경 -->
    <add key="TestMode" value="false" />
  </appSettings>
</configuration>
```

**주의사항**: 
- 프로젝트를 빌드하면 `bin\Debug\net6.0-windows\` 또는 `bin\Release\net6.0-windows\` 폴더에 `HUREL Imager GUI.exe.config` 파일이 생성됩니다.
- 빌드 후에는 실행 파일과 같은 디렉토리의 `.exe.config` 파일을 직접 수정해도 됩니다.

### 2. 테스트 모드 활성화/비활성화

- **활성화**: `value="true"` 또는 `value="1"`
- **비활성화**: `value="false"` 또는 `value="0"` 또는 설정 제거

## 테스트 모드에서 건너뛰는 기능

테스트 모드가 활성화되면 다음 하드웨어 초기화가 건너뜁니다:

1. **FPGA (CRUXELLLACC) USB 연결**
   - `CRUXELLLACC.Start_usb()` 메서드가 하드웨어 연결 없이 성공으로 반환됩니다.

2. **시리얼 통신 (LahgiSerialControl)**
   - `LahgiSerialControl.StartCommunication()` 메서드가 실제 COM 포트 연결 없이 성공으로 반환됩니다.

3. **LahgiWrapper 초기화**
   - `LahgiApi.InitiateLaghi()` 메서드가 하드웨어 초기화를 건너뛰고 성공으로 반환됩니다.

4. **RTAB-Map (RealSense 카메라) 초기화**
   - `LahgiApi.InititateRtabmap()` 메서드가 카메라 연결 없이 성공으로 반환됩니다.

## 코드 수정 내역

### 수정된 파일들:

1. `HUREL Imager GUI/Config/TestModeConfig.cs` (신규)
   - 테스트 모드 설정을 관리하는 클래스

2. `Hurel Radiation Imager/Lahgi Class.cs`
   - `InitiateLaghi()` 메서드에 테스트 모드 체크 추가
   - `InititateRtabmap()` 메서드에 테스트 모드 체크 추가

3. `Hurel Radiation Imager/LahgiSerialControl.cs`
   - `StartCommunication()` 메서드에 테스트 모드 체크 추가

4. `Hurel Radiation Imager/CRUXELL Impletement/CRUXELLLACC.cs`
   - `Start_usb()` 메서드에 테스트 모드 체크 추가

## 주의사항

- 테스트 모드에서는 실제 하드웨어 데이터를 받을 수 없습니다.
- 테스트 모드에서는 실제 측정 데이터가 생성되지 않습니다.
- 테스트 모드는 내부 코드 로직을 테스트하는 용도로만 사용하세요.
- 실제 측정을 위해서는 반드시 테스트 모드를 비활성화하고 하드웨어를 연결해야 합니다.

## 사용 예시

```csharp
// TestModeConfig를 사용하여 프로그램에서 테스트 모드 확인
if (HUREL_Imager_GUI.TestModeConfig.IsTestMode)
{
    Console.WriteLine("테스트 모드가 활성화되어 있습니다.");
    // 테스트 모드 전용 로직 실행
}
```

## 문제 해결

### 테스트 모드가 작동하지 않는 경우

1. `App.config` 파일이 올바른 위치에 있는지 확인하세요.
2. `TestMode` 키의 값이 올바른지 확인하세요 (대소문자 구분 없음).
3. 애플리케이션을 재시작하세요 (설정 변경 후 반영을 위해 필요할 수 있습니다).

