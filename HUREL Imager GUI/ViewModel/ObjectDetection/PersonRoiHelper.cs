using System;
using System.Collections.Generic;
using System.Linq;
using HUREL_Imager_GUI.ViewModel.ObjectDetection.Models;
using OpenCvSharp;
using HUREL.Compton;
using log4net;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection
{
    /// <summary>
    /// Bounding box로 사람별 RGB/Depth ROI 추출 및 선원 위치(SP) 계산
    /// </summary>
    public static class PersonRoiHelper
    {
        private static readonly ILog logger = LogManager.GetLogger(typeof(PersonRoiHelper));

        // 오프셋 보정(T265_TO_LAHGI)은 Step 3 RGB UV 변환 및 정합 시 적용. 여기서는 투영만 수행.

        /// <summary>
        /// Bounding box로 해당 사람의 RGB 픽셀(ROI) 추출
        /// </summary>
        /// <param name="frame">RGB 프레임 (OpenCvSharp Mat)</param>
        /// <param name="box">픽셀 좌표 기준 Bounding box (X, Y, Width, Height)</param>
        /// <returns>ROI 영역 Mat (Clone). 유효하지 않으면 null</returns>
        public static Mat? ExtractRgbRoi(Mat frame, BoundingBox box)
        {
            if (frame == null || frame.Empty() || box == null)
                return null;

            int x = (int)Math.Max(0, Math.Floor(box.X));
            int y = (int)Math.Max(0, Math.Floor(box.Y));
            int w = (int)Math.Ceiling(box.Width);
            int h = (int)Math.Ceiling(box.Height);

            if (w <= 0 || h <= 0)
                return null;

            // 경계 클램핑
            if (x + w > frame.Width) w = frame.Width - x;
            if (y + h > frame.Height) h = frame.Height - y;
            if (w <= 0 || h <= 0)
                return null;

            try
            {
                var rect = new OpenCvSharp.Rect(x, y, w, h);
                return frame.Clone(rect);
            }
            catch (Exception ex)
            {
                logger.Warn($"ExtractRgbRoi 실패: {ex.Message}");
                return null;
            }
        }

        /// <summary>
        /// SLAM 포인트 클라우드에서 Bounding box 내 포인트들의 Z(깊이) median을 선원 위치(SP)로 계산.
        /// RtabmapSlamControl.generatePointCloud와 동일 좌표계: u = cx - x*fx/z, v = cy - y*fy/z.
        /// 오프셋(T265_TO_LAHGI) 보정은 Step 3 RGB UV 변환 및 정합 시 적용.
        /// </summary>
        /// <param name="box">픽셀 좌표 기준 Bounding box</param>
        /// <param name="imageWidth">RGB 이미지 너비</param>
        /// <param name="imageHeight">RGB 이미지 높이</param>
        /// <returns>Depth ROI median [m], 또는 유효한 포인트가 없으면 null</returns>
        public static double? GetDepthRoiMedian(BoundingBox box, int imageWidth, int imageHeight)
        {
            if (box == null || imageWidth <= 0 || imageHeight <= 0)
                return null;

            var poseVect = new List<double[]>();
            var colorVect = new List<double[]>();

            try
            {
                bool hasPoints = LahgiApi.GetSLAMPointCloud(ref poseVect, ref colorVect) && poseVect != null && poseVect.Count > 0;
                if (!hasPoints)
                {
                    poseVect = new List<double[]>();
                    colorVect = new List<double[]>();
                    if (LahgiApi.GetRealTimePointCloud(ref poseVect, ref colorVect) && poseVect != null && poseVect.Count > 0)
                    {
                        logger.Info("GetDepthRoiMedian: SLAM 비어있음 → 실시간 포인트클라우드 사용, 포인트 수=" + poseVect.Count);
                        hasPoints = true;
                    }
                }
                if (!hasPoints)
                {
                    logger.Info("GetDepthRoiMedian: SLAM/실시간 포인트클라우드 모두 비어있음. 비디오 스트림 켜짐 여부 확인.");
                    return null;
                }
            }
            catch (Exception ex)
            {
                logger.Warn($"GetDepthRoiMedian 포인트클라우드 조회 실패: {ex.Message}");
                return null;
            }

            // RtabmapSlamControl에서 카메라 내부 파라미터 추출 (generatePointCloud와 동일 소스)
            float fxVal = 0, fyVal = 0, cxVal = 0, cyVal = 0;
            bool gotIntrinsics = LahgiApi.GetCameraIntrinsics(ref fxVal, ref fyVal, ref cxVal, ref cyVal);
            double fx, fy, cx, cy;
            if (gotIntrinsics && fxVal > 0 && fyVal > 0)
            {
                fx = fxVal;
                fy = fyVal;
                cx = cxVal;
                cy = cyVal;
            }
            else
            {
                logger.Warn("GetCameraIntrinsics 실패 또는 비디오 스트림 미동작 - 대략값 사용 (fx,fy,cx,cy). RtabmapSlamControl 비디오 스트림 동작 후 정확한 값 적용됨.");
                fx = imageWidth * 0.75;
                fy = imageHeight * 0.75;
                cx = imageWidth * 0.5;
                cy = imageHeight * 0.5;
            }

            int x1 = (int)Math.Floor(box.X);
            int y1 = (int)Math.Floor(box.Y);
            int x2 = (int)Math.Ceiling(box.X + box.Width);
            int y2 = (int)Math.Ceiling(box.Y + box.Height);
            x1 = Math.Max(0, x1);
            y1 = Math.Max(0, y1);
            x2 = Math.Min(imageWidth, x2);
            y2 = Math.Min(imageHeight, y2);

            var depthsInRoi = new List<double>();
            foreach (var pose in poseVect)
            {
                if (pose == null || pose.Length < 3)
                    continue;

                double x = pose[0], y = pose[1], z = pose[2];
                if (z <= 0.01) // 전방만, 0 제거
                    continue;

                // RtabmapSlamControl.generatePointCloud: point.x = -(n-cx)*z/fx => u = cx - x*fx/z, v = cy - y*fy/z
                double u = cx - x * fx / z;
                double v = cy - y * fy / z;
                if (u >= x1 && u < x2 && v >= y1 && v < y2)
                    depthsInRoi.Add(z);
            }

            if (depthsInRoi.Count == 0)
            {
                foreach (var pose in poseVect)
                {
                    if (pose == null || pose.Length < 3) continue;
                    double x = pose[0], y = pose[1], z = pose[2];
                    if (z <= 0.01) continue;
                    double u = cx - x * fx / z;
                    double v = cy - y * fy / z;
                    logger.Info($"GetDepthRoiMedian: 포인트클라우드 {poseVect.Count}개 중 bbox 내 0개. bbox=[{x1},{y1},{x2},{y2}] 이미지=[{imageWidth}x{imageHeight}] 샘플(u,v,z)=({u:F1},{v:F1},{z:F3})");
                    break;
                }
                return null;
            }

            depthsInRoi.Sort();
            int mid = depthsInRoi.Count / 2;
            double median = depthsInRoi.Count % 2 != 0
                ? depthsInRoi[mid]
                : (depthsInRoi[mid - 1] + depthsInRoi[mid]) * 0.5;

            return median;
        }
    }
}
