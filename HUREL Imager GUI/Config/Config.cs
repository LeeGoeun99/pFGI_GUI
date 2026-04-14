using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using HUREL_Imager_GUI.ViewModel;

namespace HUREL_Imager_GUI
{
    public class Config
    {
        #region 에너지
        /// <summary>
        /// ECal 실행 여부
        /// </summary>
        public bool IsECalUse { get; set; } = false;

        /// <summary>
        /// ECal 실행 inverval
        /// </summary>
        public int ECalIntervalTime { get; set; } = 60000;   //231113-1 sbkwon : 단위 분(60000), 저장 값은 ms로 변환하여 저장

        /// <summary>
        /// 누적 시간(초)
        /// </summary>
        public uint SpectrumEffectTime { get; set; } = 10;

        /// <summary>
        /// spectrum에 분석 그래프 추가 여부
        /// </summary>
        public bool isSpectrumAnalysisShow { get; set; } = false;

        /// <summary>
        /// spectrum 선택 위치
        /// 0 : Scatter
        /// 1 : Absorber
        /// 2 : All
        /// 3 : ByChannel
        /// </summary>
        public eSpectrumCases SpectrumCases { get; set; } = eSpectrumCases.All;

        /// <summary>
        /// Spectrum byChannel value
        /// </summary>
        public int FpgaChannelNumber { get; set; } = 0;

        /// <summary>
        /// spectrum 표시 방법
        /// 0 : Linear
        /// 1 : Log
        /// </summary>
        public eSpectrumType SpectrumType { get; set; } = eSpectrumType.Log;

        public float Ref_x { get; set; } = 2650;
        public float Ref_fwhm { get; set; } = 40;
        public float Ref_at_0 { get; set; } = 0;
        public float Min_snr { get; set; } = 5;
        #endregion

        #region 위치영상
        /// <summary>
        /// 이미지 모드 자동 선택 여부
        /// </summary>
        public bool ReconSpaceAuto { get; set; } = false;
        /// <summary>
        /// 이미지 모드 수동 선택 여부
        /// </summary>
        public bool ReconSpaceManual { get; set; } = false;
        /// <summary>
        /// 이미징 모드
        /// </summary>
        public eReconSpace ReconSpace { get; set; } = eReconSpace.Plane;
        /// <summary>
        /// 가시화 설정 : RGB Image
        /// 0 : color
        /// 1 : gray
        /// </summary>
        public eRGBType RGBImageType { get; set; } = eRGBType.Color;

        /// <summary>
        /// 방사선 영상 투명도
        /// </summary>
        public int OpacityValue { get; set; } = 70;

        /// <summary>
        /// 가시화 범위
        /// </summary>
        public int MinValuePortion { get; set; } = 70;

        /// <summary>
        /// 이미징 기법
        /// </summary>
        public eReconType ReconType { get; set; } = eReconType.HybridImage;

        /// <summary>
        /// 영상 정합 방법
        /// </summary>
        public eReconOption ReconOption { get; set; } = eReconOption.type1;

        /// <summary>
        /// 누적 시간
        /// </summary>
        public int ReconMeasurTime { get; set; } = 20;
        /// <summary>
        /// 누적 카운트 수
        /// </summary>
        public int ReconMeasurCount { get; set; } = 300;

        /// <summary>
        /// 영상 공간 거리
        /// </summary>
        public double S2M { get; set; } = 2;
        /// <summary>
        /// 최소 영상값 : 이하일 경우 영상을 0으로 채움
        /// </summary>
        public int ReconMaxValue { get; set; } = 70;
        #endregion

        #region 일반
        /// <summary>
        /// 저장 폴더명
        /// </summary>
        public string SaveFileName { get; set; } = string.Empty;

        /// <summary>
        /// 계측 시간
        /// false : 무한
        /// true : 설정(초)
        /// </summary>
        public eMeasuremetType MeasurementType { get; set; } = eMeasuremetType.Infinite;

        /// <summary>
        /// 측정 시간(초)
        /// </summary>
        public int MeasurementTime { get; set; } = 0;

        /// <summary>
        /// 측정 모드
        /// Moving : 이동모드
        /// Static : 정지모드
        /// ObjectDetection : 객체탐지 모드
        /// </summary>
        public eMeasurementMode MeasurementMode { get; set; } = eMeasurementMode.Moving;

        /// <summary>
        /// 고장 검사
        /// 240206 - 실시간 검사 실행 여부
        /// </summary>
        public bool UseRealTimeCheck { get; set; } = false;

        /// <summary>
        /// 핵종 라벨링 사용 유무
        /// </summary>
        public bool UseLabelingCheck { get; set; } = false;

        /// <summary>
        /// 가시화 범위 설정
        /// </summary>
        public double VisualizationRange { get; set; } = 100.0;

        /// <summary>
        /// 240206 : 실시간 검사 주기(분)
        /// </summary>
        public int RealTimeCycleTime { get; set; } = 3;

        /// <summary>
        /// 고장검사
        /// 240206 - 정밀검사
        /// </summary>
        public bool UseFaultDiagnosis { get; set; } = false;

        /// <summary>
        /// 240206 : 정밀검사 측정 시간(분)
        /// </summary>
        public int FaultDiagnosisMeasurementTime { get; set; } = 20;

        /// <summary>
        /// 3D View 화면 표시 선택
        /// </summary>
        public e3DViewType Type3DView { get; set; } = e3DViewType.eSlamOccupancyGrid;
        #endregion

        #region Capture
        public int SpectrumX { get; set; } = 15;
        public int SpectrumY { get; set; } = 170;
        public int SpectrumWidth { get; set; } = 760;
        public int SpectrumHeight { get; set; } = 850;
        public int ReconX { get; set; } = 865;
        public int ReconY { get; set; } = 75;
        public int ReconWidth { get; set; } = 1195;
        public int ReconHeight { get; set; } = 540;
        #endregion
    }
}
