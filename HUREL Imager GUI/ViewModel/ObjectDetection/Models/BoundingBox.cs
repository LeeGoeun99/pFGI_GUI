namespace HUREL_Imager_GUI.ViewModel.ObjectDetection.Models
{
    /// <summary>
    /// Bounding box를 나타내는 클래스
    /// </summary>
    public class BoundingBox
    {
        public float X { get; set; }
        public float Y { get; set; }
        public float Width { get; set; }
        public float Height { get; set; }

        public BoundingBox()
        {
        }

        public BoundingBox(float x, float y, float width, float height)
        {
            X = x;
            Y = y;
            Width = width;
            Height = height;
        }
    }
}

