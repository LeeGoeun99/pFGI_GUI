using System;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection.Models
{
    /// <summary>
    /// 추적된 사람 정보를 나타내는 클래스
    /// </summary>
    public class TrackedPerson
    {
        /// <summary>
        /// 사람의 고유 ID
        /// </summary>
        public int Id { get; set; }

        /// <summary>
        /// Bounding box 정보
        /// </summary>
        public BoundingBox BoundingBox { get; set; }

        /// <summary>
        /// 탐지 시각
        /// </summary>
        public DateTime Timestamp { get; set; }

        /// <summary>
        /// 탐지 신뢰도
        /// </summary>
        public float Confidence { get; set; }

        public TrackedPerson()
        {
            BoundingBox = new BoundingBox();
        }

        public TrackedPerson(int id, BoundingBox boundingBox, DateTime timestamp, float confidence)
        {
            Id = id;
            BoundingBox = boundingBox;
            Timestamp = timestamp;
            Confidence = confidence;
        }
    }
}

