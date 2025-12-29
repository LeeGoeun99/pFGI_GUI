namespace HUREL_Imager_GUI.ViewModel.ObjectDetection.Models
{
    /// <summary>
    /// 객체탐지 결과를 나타내는 클래스
    /// </summary>
    public class Detection
    {
        /// <summary>
        /// Bounding box 좌상단 X 좌표
        /// </summary>
        public float X { get; set; }

        /// <summary>
        /// Bounding box 좌상단 Y 좌표
        /// </summary>
        public float Y { get; set; }

        /// <summary>
        /// Bounding box 너비
        /// </summary>
        public float Width { get; set; }

        /// <summary>
        /// Bounding box 높이
        /// </summary>
        public float Height { get; set; }

        /// <summary>
        /// 탐지 신뢰도 (0.0 ~ 1.0)
        /// </summary>
        public float Confidence { get; set; }

        /// <summary>
        /// 클래스 ID (0 = person)
        /// </summary>
        public int ClassId { get; set; }

        public Detection()
        {
        }

        public Detection(float x, float y, float width, float height, float confidence, int classId)
        {
            X = x;
            Y = y;
            Width = width;
            Height = height;
            Confidence = confidence;
            ClassId = classId;
        }
    }
}

