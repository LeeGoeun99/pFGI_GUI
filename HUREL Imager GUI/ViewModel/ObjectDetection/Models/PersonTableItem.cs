using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace HUREL_Imager_GUI.ViewModel.ObjectDetection.Models
{
    /// <summary>
    /// GUI Table에 표시할 사람 정보 항목
    /// </summary>
    public class PersonTableItem : INotifyPropertyChanged
    {
        private int _trackId;
        private int _classId;
        private string _sourceCarrier = "";
        private bool _isSelected;

        /// <summary>
        /// Track ID
        /// </summary>
        public int TrackId
        {
            get => _trackId;
            set
            {
                _trackId = value;
                OnPropertyChanged();
            }
        }

        /// <summary>
        /// Class ID (0 = person)
        /// </summary>
        public int ClassId
        {
            get => _classId;
            set
            {
                _classId = value;
                OnPropertyChanged();
            }
        }

        /// <summary>
        /// Source carrier 표시 (O, X, 또는 빈 문자열)
        /// </summary>
        public string SourceCarrier
        {
            get => _sourceCarrier;
            set
            {
                _sourceCarrier = value;
                OnPropertyChanged();
            }
        }

        /// <summary>
        /// 선택 여부 (체크박스)
        /// </summary>
        public bool IsSelected
        {
            get => _isSelected;
            set
            {
                _isSelected = value;
                OnPropertyChanged();
            }
        }

        public event PropertyChangedEventHandler? PropertyChanged;

        protected virtual void OnPropertyChanged([CallerMemberName] string? propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}

