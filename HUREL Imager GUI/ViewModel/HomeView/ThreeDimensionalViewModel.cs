using AsyncAwaitBestPractices.MVVM;
using HelixToolkit.Wpf.SharpDX;
using HUREL.Compton;
using log4net;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;

namespace HUREL_Imager_GUI.ViewModel
{
    public enum e3DViewType
    {
        eSlamPointCloud,
        eSlamOccupancyGrid
    };
    public class ThreeDimensionalViewModel : ViewModelBase
    {
        public ThreeDimensionalViewModel()
        {           
            LahgiApi.StatusUpdate += updateView;

            Type3DView = App.GlobalConfig.Type3DView;

            var vc = new Vector3Collection();
            var cc = new Color4Collection();

            for (int i = 0; i < 10; ++i)
            {
                var pointx = new Vector3(0.1f * i, 0, 0);
                var pointy = new Vector3(0, 0.1f * i, 0);
                var pointz = new Vector3(0, 0, 0.1f * i);
                vc.Add(new Vector3(Convert.ToSingle(pointx.X), Convert.ToSingle(pointx.Y), Convert.ToSingle(pointx.Z)));
                cc.Add(new Color4(1f, 0f, 0f, 1f));
                vc.Add(new Vector3(Convert.ToSingle(pointy.X), Convert.ToSingle(pointy.Y), Convert.ToSingle(pointy.Z)));
                cc.Add(new Color4(0f, 1f, 0f, 1f));
                vc.Add(new Vector3(Convert.ToSingle(pointz.X), Convert.ToSingle(pointz.Z), Convert.ToSingle(pointz.Z)));
                cc.Add(new Color4(0f, 0f, 1f, 1f));
            }     

            SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };

            //Text3d = new BillboardText3D();
            //TextInfoExt tinfo = new TextInfoExt();
            //tinfo.Text = ("Cs-137");
            //tinfo.Origin = new Vector3(-1, 0.2f, 2.5f);
            //tinfo.Scale = 1f;
            //Text3d.TextInfo.Add(tinfo);
        }

        private BillboardText3D text3d = new BillboardText3D();
        public BillboardText3D Text3d
        {
            get { return text3d; }
            set
            {
                text3d = value;
                OnPropertyChanged(nameof(Text3d));
            }
        }

        private void updateView(object? obj, EventArgs args)
        {
            //Trace.WriteLine("Update");

            if (args is LahgiApiEnvetArgs)
            { 
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)args;

                if(lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading)
                {
                    UpdateLoadedSlamPointCloud();
                }             
                else if(lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.SlamPoints)
                {
                    if(Type3DView == e3DViewType.eSlamPointCloud)
                        UpdateRealtimeSlamPointCloud();
                    else
                        UpdateRealtimeSlamOccupancyGrid();  //231121-1 sbkwon
                }
                else if(lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.SlamRadImage)
                {
                    UpateRealtimeReconSlamPointCloud();
                }
                else if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.MLEM)    //2404 : MLEM
                {
                    UpdateLoadedMLEM();
                }
            }
        }

        private static Mutex updateLoadDataMutex = new Mutex();

        //2404 : MLEM
        private void UpdateLoadedMLEM()
        {
            //LogManager.GetLogger(typeof(ThreeDimensionalViewModel)).Info("3D MLEM : 결과 영상");
            Task.Run(() =>
            {
                updateLoadDataMutex.WaitOne();
                var vc = new Vector3Collection();
                var cc = new Color4Collection();
                var tempposeVect = new List<double[]>();
                var tempColorVect = new List<double[]>();
                List<float[]> uvs = new List<float[]>();
                var ret = LahgiApi.GetLoadedPointCloudMLEM(ref tempposeVect, ref tempColorVect, LahgiApi.MLEMSelectNo);
                if (ret)    //250310sbkwon
                {
                    for (int i = 0; i < tempposeVect.Count; i++)
                    {
                        vc.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                        cc.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
                    }

                    SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };

                    var vc2 = new Vector3Collection();
                    var cc2 = new Color4Collection();
                    LahgiApi.GetReconSLAMPointCloud(0, eReconManaged.COMPTON, ref tempposeVect, ref tempColorVect, 0.01, true);
                    for (int i = 0; i < tempposeVect.Count; i++)
                    {
                        vc2.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                        cc2.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
                    }

                    SLAMReconPointCloud = new PointGeometry3D() { Positions = vc2, Colors = cc2 };
                }
                updateLoadDataMutex.ReleaseMutex();
            });
        }

        private void UpdateLoadedSlamPointCloud()
        {            
            Task.Run(() =>
            {
                updateLoadDataMutex.WaitOne();
                var vc = new Vector3Collection();
                var cc = new Color4Collection();
                var tempposeVect = new List<double[]>();
                var tempColorVect = new List<double[]>();
                List<float[]> uvs = new List<float[]>();
                LahgiApi.GetLoadedPointCloud(ref tempposeVect, ref tempColorVect);
                for (int i = 0; i < tempposeVect.Count; i++)
                {
                    vc.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                    cc.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
                }

                SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };

                var vc2 = new Vector3Collection();
                var cc2 = new Color4Collection();
                LahgiApi.GetReconSLAMPointCloud(0, eReconManaged.COMPTON, ref tempposeVect, ref tempColorVect, 0.01, true);
                for (int i = 0; i < tempposeVect.Count; i++)
                {
                    vc2.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                    cc2.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
                }
                
                SLAMReconPointCloud = new PointGeometry3D() { Positions = vc2, Colors = cc2 };
                
                updateLoadDataMutex.ReleaseMutex();
            });
        }

        private void UpdateRealtimeSlamPointCloud()
        {
            if(!updateLoadDataMutex.WaitOne(100))
            {
                return;
            };
            var vc = new Vector3Collection();
            var cc = new Color4Collection();
            var tempposeVect = new List<double[]>();
            var tempColorVect = new List<double[]>();
            List<float[]> uvs = new List<float[]>();
            LahgiApi.GetSLAMPointCloud(ref tempposeVect, ref tempColorVect);
            for (int i = 0; i < tempposeVect.Count; i++)
            {
                vc.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                cc.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
            }

            SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };

            UpdateRealtimeSlamPoseGraph();

            ////Position
            //(double x, double y, double z) = LahgiApi.GetOdomentryPos();

            //Point3D pos = new Point3D(x, y, z);
            //Position = pos;

            updateLoadDataMutex.ReleaseMutex();
        }

        //231121-1 sbkwon
        private void UpdateRealtimeSlamOccupancyGrid()
        {
            if (!updateLoadDataMutex.WaitOne(100))
            {
                return;
            };
            try
            {
                var vc = new Vector3Collection();
                var cc = new Color4Collection();
                var tempposeVect = new List<double[]>();
                var tempColorVect = new List<double[]>();
                List<float[]> uvs = new List<float[]>();
                LahgiApi.GetSLAMOccupancyGrid(ref tempposeVect, ref tempColorVect);
                for (int i = 0; i < tempposeVect.Count; i++)
                {
                    vc.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                    cc.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
                }

                //SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };
                SlamOccupancyGrid = new PointGeometry3D() { Positions = vc, Colors = cc };

                //Trace.WriteLine($"Occu - temp : {tempposeVect.Count}:{tempColorVect.Count}, Point : {vc.Count}, Color : {cc.Count}");

                UpdateRealtimeSlamPoseGraph();

                //Position
                (double x, double y, double z) = LahgiApi.GetOdomentryPos();

                //Trace.WriteLine($"x : {x}, y : {y}, z : {z}");

                y = 10;
                Point3D pos = new Point3D(x, y, z);
                Position = pos;
            }
            finally 
            { 
                updateLoadDataMutex.ReleaseMutex(); 
            }
        }

        private void UpdateRealtimeSlamPoseGraph()
        {
            var line = new LineBuilder();
            List<Matrix3D> poses = new List<Matrix3D>();
            if (LahgiApi.GetOptimizedPoses(ref poses))
            {
                //LogManager.GetLogger(typeof(ThreeDimensionalViewModel)).Info($"{(float)poses[0].OffsetX} : {(float)poses[0].OffsetY} : {(float)poses[0].OffsetZ}");

                line.AddLine(new Vector3(0, 0, 0), new Vector3((float)poses[0].OffsetX, (float)poses[0].OffsetY, (float)poses[0].OffsetZ));
                for (int i = 1; i < poses.Count; i++)
                {
                    line.AddLine(
                        new Vector3((float)poses[i - 1].OffsetX, (float)poses[i - 1].OffsetY, (float)poses[i - 1].OffsetZ),
                        new Vector3((float)poses[i].OffsetX, (float)poses[i].OffsetY, (float)poses[i].OffsetZ));
                }
                SLAMPoseInfo = line.ToLineGeometry3D();

                //LogManager.GetLogger(typeof(ThreeDimensionalViewModel)).Info($"{(float)poses[0].OffsetX} : {(float)poses[0].OffsetY} : {(float)poses[0].OffsetZ}");
            }
        }

        private static Mutex updateReconSlamPointMutex = new Mutex();
        private void UpateRealtimeReconSlamPointCloud()
        {
            if(!updateReconSlamPointMutex.WaitOne(100))
            {
                return;
            }
            var vc2 = new Vector3Collection();
            var cc2 = new Color4Collection();
            var tempposeVect = new List<double[]>();
            var tempColorVect = new List<double[]>();
            LahgiApi.GetReconSLAMPointCloud(0, eReconManaged.COMPTON, ref tempposeVect, ref tempColorVect, 0.01, false);
            for (int i = 0; i < tempposeVect.Count; i++)
            {
                vc2.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                cc2.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
            }

            SLAMReconPointCloud = new PointGeometry3D() { Positions = vc2, Colors = cc2 };
            updateReconSlamPointMutex.ReleaseMutex();
            Thread.Sleep(1000);
        }

        private PointGeometry3D slamPointCloud = new PointGeometry3D();
        public PointGeometry3D SLAMPointCloud
        {
            get { return slamPointCloud; }
            set
            {
                slamPointCloud = value;
                OnPropertyChanged(nameof(SLAMPointCloud));
            }
        }

        //231213
        private Point3D _Position = new Point3D(0, 10, 0);
        public Point3D Position
        {
            get => _Position;
            set
            {
                _Position = value;
                OnPropertyChanged(nameof(Position));
            }
        }

        //231121-1 sbkwon
        private PointGeometry3D slamOccupancyGrid = new PointGeometry3D();
        public PointGeometry3D SlamOccupancyGrid
        {
            get { return slamOccupancyGrid; }
            set
            {
                slamOccupancyGrid = value;
                OnPropertyChanged(nameof(SlamOccupancyGrid));
            }
        }

        private PointGeometry3D slamReconPointCloud = new PointGeometry3D();
        public PointGeometry3D SLAMReconPointCloud
        {
            get { return slamReconPointCloud; }
            set
            {
                slamReconPointCloud = value;
                OnPropertyChanged(nameof(SLAMReconPointCloud));
            }
        }

        private LineGeometry3D slamPoseInfo;
        public LineGeometry3D SLAMPoseInfo
        {
            get { return slamPoseInfo; }
            set { slamPoseInfo = value; OnPropertyChanged(nameof(SLAMPoseInfo)); }
        }

        private AsyncCommand? startSlamCommand = null;
        public ICommand StartSlamCommand
        {
            get { return startSlamCommand ?? (startSlamCommand = new AsyncCommand(StartSlam)); }
        }

        private async Task StartSlam()
        {
            await Task.Run(() =>
            {
                LahgiApi.StartSlam();
            });
        }

        //240110
        private e3DViewType _Type3DView = e3DViewType.eSlamOccupancyGrid;
        public e3DViewType Type3DView
        {
            get => _Type3DView;
            set
            {
                _Type3DView = value;
                Set3DVisibitity();
                OnPropertyChanged(nameof(Type3DView));
                
                // App.GlobalConfig에 자동 저장
                App.GlobalConfig.Type3DView = value;
            }
        }

        private void Set3DVisibitity()
        {
            VisibilityPointCloud = Visibility.Hidden;
            VisibilityOccupancyGrid = Visibility.Hidden;
            switch (Type3DView)
            {
                case e3DViewType.eSlamPointCloud:
                    VisibilityPointCloud = Visibility.Visible;
                    break;
                case e3DViewType.eSlamOccupancyGrid:
                    VisibilityOccupancyGrid = Visibility.Visible;
                    break;
                default:
                    VisibilityOccupancyGrid = Visibility.Visible;
                    break;
            }
        }

        private Visibility _visibitityPointCloud = Visibility.Hidden;
        private Visibility _visibilityOccupancyGrid = Visibility.Visible;
        public Visibility VisibilityPointCloud
        {
            get => _visibitityPointCloud;
            set { _visibitityPointCloud = value; OnPropertyChanged(nameof(VisibilityPointCloud)); }
        }

        public Visibility VisibilityOccupancyGrid
        {
            get => _visibilityOccupancyGrid;
            set { _visibilityOccupancyGrid = value; OnPropertyChanged(nameof(VisibilityOccupancyGrid)); }
        }

        private AsyncCommand? _select3DTypePointCloud;
        public ICommand Select3DTypePointCloud
        {
            get { return _select3DTypePointCloud ?? (_select3DTypePointCloud = new AsyncCommand(Sel3DTypePointCloud)); }
        }

        private Task Sel3DTypePointCloud() => Task.Run(() =>
        {
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                Type3DView = e3DViewType.eSlamPointCloud;
            });
        });

        private AsyncCommand? _select3DTypeOccupancyGrid;
        public ICommand Select3DTypeOccupancyGrid
        {
            get { return _select3DTypeOccupancyGrid ?? (_select3DTypeOccupancyGrid = new AsyncCommand(Sel3DTypeOccupancyGrid)); }
        }

        private Task Sel3DTypeOccupancyGrid() => Task.Run(() =>
        {
            // UI 스레드에서 속성 변경
            Application.Current.Dispatcher.Invoke(() =>
            {
                Type3DView = e3DViewType.eSlamOccupancyGrid;
            });
        });

        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= updateView;
            LogManager.GetLogger(typeof(ThreeDimensionalViewModel)).Info("Unhandle");
        }
    }
}
