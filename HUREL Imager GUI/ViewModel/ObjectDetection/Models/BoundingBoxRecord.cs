using System;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection.Models
{
    /// <summary>
    /// Bounding box 기록을 나타내는 클래스
    /// </summary>
    public class BoundingBoxRecord
    {
        public BoundingBox Box { get; set; }
        public DateTime Timestamp { get; set; }
        public float Confidence { get; set; }

        public BoundingBoxRecord()
        {
            Box = new BoundingBox();
        }

        public BoundingBoxRecord(BoundingBox box, DateTime timestamp, float confidence)
        {
            Box = box;
            Timestamp = timestamp;
            Confidence = confidence;
        }
    }
}

