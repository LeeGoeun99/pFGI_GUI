using System;
using System.Collections.Generic;
using System.Windows.Media.Imaging;
using OpenCvSharp;
using HUREL_Imager_GUI.ViewModel.ObjectDetection.Models;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection.Models
{
    /// <summary>
    /// 사람별 방사선 영상 데이터를 저장하는 클래스
    /// </summary>
    public class PersonRadiationData
    {
        /// <summary>
        /// 사람의 Track ID
        /// </summary>
        public int TrackId { get; set; }

        /// <summary>
        /// 클래스 ID (0 = person)
        /// </summary>
        public int ClassId { get; set; }

        /// <summary>
        /// 현재 프레임의 CA (Coded Aperture) 영상 (누적 전)
        /// </summary>
        public Mat? CurrentCAImage { get; set; }

        /// <summary>
        /// 현재 프레임의 CC (Compton) 영상 (누적 전)
        /// </summary>
        public Mat? CurrentCCImage { get; set; }

        /// <summary>
        /// 현재 프레임의 Hybrid 영상 (누적 전)
        /// </summary>
        public Mat? CurrentHybridImage { get; set; }

        /// <summary>
        /// 누적된 CA 영상
        /// </summary>
        public Mat? CumulatedCAImage { get; set; }

        /// <summary>
        /// 누적된 CC 영상
        /// </summary>
        public Mat? CumulatedCCImage { get; set; }

        /// <summary>
        /// 누적된 Hybrid 영상
        /// </summary>
        public Mat? CumulatedHybridImage { get; set; }

        /// <summary>
        /// RGB UV 좌표로 변환된 CA 영상
        /// </summary>
        public Mat? ResampledCAImage { get; set; }

        /// <summary>
        /// RGB UV 좌표로 변환된 CC 영상
        /// </summary>
        public Mat? ResampledCCImage { get; set; }

        /// <summary>
        /// RGB UV 좌표로 변환된 Hybrid 영상
        /// </summary>
        public Mat? ResampledHybridImage { get; set; }

        /// <summary>
        /// CA 이벤트 수
        /// </summary>
        public int CACount { get; set; }

        /// <summary>
        /// CC 이벤트 수
        /// </summary>
        public int CCCount { get; set; }

        /// <summary>
        /// 선원 위치(SP) [m] — Depth ROI median. 재구성 파이프라인(GetRadation2dImageCount 등)에 전달.
        /// </summary>
        public double? SourcePositionM { get; set; }

        /// <summary>
        /// 이전 프레임의 Bounding Box 중심 위치 (shift 계산용)
        /// </summary>
        public System.Drawing.PointF PreviousBoxPosition { get; set; }

        /// <summary>
        /// Criminal 판단 결과 (0: 없음, 1: 확실, 2: 의심)
        /// </summary>
        public int CriminalStatus { get; set; }

        /// <summary>
        /// GUI에서 선택되었는지 여부
        /// </summary>
        public bool IsSelected { get; set; }

        /// <summary>
        /// 마지막 업데이트 시간
        /// </summary>
        public DateTime LastUpdateTime { get; set; }

        /// <summary>
        /// 생성자
        /// </summary>
        public PersonRadiationData(int trackId, int classId)
        {
            TrackId = trackId;
            ClassId = classId;
            CACount = 0;
            CCCount = 0;
            CriminalStatus = 0;
            IsSelected = false;
            PreviousBoxPosition = new System.Drawing.PointF(0, 0);
            LastUpdateTime = DateTime.Now;
        }

        /// <summary>
        /// 리소스 정리
        /// </summary>
        public void Dispose()
        {
            CurrentCAImage?.Dispose();
            CurrentCCImage?.Dispose();
            CurrentHybridImage?.Dispose();
            CumulatedCAImage?.Dispose();
            CumulatedCCImage?.Dispose();
            CumulatedHybridImage?.Dispose();
            ResampledCAImage?.Dispose();
            ResampledCCImage?.Dispose();
            ResampledHybridImage?.Dispose();
        }
    }
}

