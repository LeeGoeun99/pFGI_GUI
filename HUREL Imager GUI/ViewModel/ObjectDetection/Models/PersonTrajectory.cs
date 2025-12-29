using System.Collections.Generic;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection.Models
{
    /// <summary>
    /// 사람의 이동 궤적을 나타내는 클래스
    /// </summary>
    public class PersonTrajectory
    {
        /// <summary>
        /// 사람의 고유 ID
        /// </summary>
        public int PersonId { get; set; }

        /// <summary>
        /// Bounding box 기록 리스트
        /// </summary>
        public List<BoundingBoxRecord> Records { get; set; }

        public PersonTrajectory()
        {
            Records = new List<BoundingBoxRecord>();
        }

        public PersonTrajectory(int personId)
        {
            PersonId = personId;
            Records = new List<BoundingBoxRecord>();
        }
    }
}

