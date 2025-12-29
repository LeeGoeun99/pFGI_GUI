using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using AsyncAwaitBestPractices;
using AsyncAwaitBestPractices.MVVM;
using Compton_GUI_WPF.View;
using GalaSoft.MvvmLight.Command;
using GalaSoft.MvvmLight.Messaging;
using HUREL.Compton;
using HUREL.Compton.CZT;
using HUREL.Compton.LACC;
using MathNet.Numerics;
using MathNet.Numerics.Statistics;


namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel : INotifyPropertyChanged
    {

        public static LACC_Control LACC_Control_Static;
        public static LahgiWrapper LahgiWrapper_Static;

        /// <summary>
        /// Contructor
        /// </summary>
        public MainViewModel()
        {
            FPGAControl = new CRUXELLLACC();
            FPGAVariable = FPGAControl.Variables;

            IsSessionAvailable = false;

            FPGAControl.USBChangeHandler += UpdateDeviceList;
            FPGAControl.USBChange();            
           

            //RTPointCloudTask =Task.Run(() => GetRealTimePointCloud());

            ModuleInfoViewModels = new ObservableCollection<ModuleInfoViewModel>();
            ModuleEnergySpectrums = new List<ObservableCollection<HistoEnergy>>();

            // Mono 모드: Scatter 1개, Absorber 1개 (총 2개)
            for (int i = 0; i < 2; i++)
            {
                ModuleEnergySpectrums.Add(new ObservableCollection<HistoEnergy>());
                ModuleInfoViewModels.Add(new ModuleInfoViewModel());
            }

            if (SRE3021API.IsTCPOpen && SRE3021API.IsUDPOpen)
            {
                Trace.WriteLine("SRE3021 Loading Successs");
            }

            InitiateCZTAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
            InitiateRealsenseAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
            InitiateLACCAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));          
            TestFunction("").SafeFireAndForget(onException: ex => Debug.WriteLine(ex));

        }

        private ModuleInfo selectedModuleInfo = ModuleInfo.QuadSingleHead;
        public ModuleInfo SelecteModuleInfo
        {
            get { return selectedModuleInfo; }
            set
            {
                selectedModuleInfo = value;
            }
        }


        private readonly string CurrentPath = Directory.GetCurrentDirectory();

        private AsyncCommand mianWindowCloseCommand;
        public ICommand MianWindowCloseCommand
        {
            get { return (this.mianWindowCloseCommand) ?? (this.mianWindowCloseCommand = new AsyncCommand(CloseMainWindow)); }
        }
        private async Task CloseMainWindow()
        {
          
            await StopRealsensePipeline().ConfigureAwait(false);
            await StopSLAM().ConfigureAwait(false);
            FPGAControl.SetVaribles(FPGAVariable);
            await FPGAControl.Dispose().ConfigureAwait(false);
            SRE3021API.Close();
            // rsControl.Dispose();
        }

        #region Draw Graph

        private void DataUpdate()
        {
            LACC_Control.debugCountAbsorber = new int[1] { 0 };
            LACC_Control.debugCountScatter = new int[1] { 0 };
            while (IsSessionStart)
            {
                DrawSpectrum();
                if (RecordTimeSpan.TotalSeconds > 10 && IsMLPEOn)
                {
                    //ResetSpectrumCommand.Execute(null);
                }                
                Thread.Sleep(1000);
            }
            Debug.WriteLine("DataUpdate End");

            // 2개 채널만 출력
            Debug.WriteLine($"||| {LACC_Control.debugCountScatter[0]} ||| \n" +
                $"||| {LACC_Control.debugCountScatter[1]} ||| \n" +
                $"||| {LACC_Control.debugCountAbsorber[0]} ||| \n" +
                $"||| {LACC_Control.debugCountAbsorber[1]} ||| \n");
        }
        public void DrawMLPEPositions()
        {
            if(IsMLPEOn == false)
            {
                return;
            }

            List<double[]> scatterXYZE = new List<double[]>();

            List<double[]> absorberXYZE = new List<double[]>();
            LahgiWrapper_Static.GetRelativeListModeData(ref scatterXYZE, ref absorberXYZE);


            Stopwatch sw = new Stopwatch();
            sw.Start();
            var temp1 = new List<MlpePositionInfo>();
            foreach (var lmdata in absorberXYZE)
            {
                if (lmdata == null)
                {
                    continue;
                }
                temp1.Add(new MlpePositionInfo(lmdata[0], lmdata[1]));
                
            }
            AbsorberPositionData = temp1;
            //ScatterPositionData.Clear();
            var temp2 = new List<MlpePositionInfo>();

            foreach (var lmdata in scatterXYZE)
            {
                if (lmdata == null)
                {
                    continue;
                }
                temp2.Add(new MlpePositionInfo(lmdata[0], lmdata[1]));

            }
            
            ScatterPositionData = temp2;
            sw.Stop();
            Debug.WriteLine("DrawSpectrums elapsed time is " + sw.ElapsedMilliseconds + " ms");
        }


        public record MlpePositionInfo(double X, double Y);

        private List<MlpePositionInfo> absorberPositionData = new List<MlpePositionInfo>();
        public List<MlpePositionInfo> AbsorberPositionData
        {
            get { return absorberPositionData; }
            set { absorberPositionData = value; OnPropertyChanged(nameof(AbsorberPositionData)); }
        }

        private List<MlpePositionInfo> scatterPositionData = new List<MlpePositionInfo>();
        public List<MlpePositionInfo> ScatterPositionData
        {
            get { return scatterPositionData; }
            set { scatterPositionData = value; OnPropertyChanged(nameof(ScatterPositionData)); }
        }

        private AsyncCommand resetSepctrumCommand;
        public ICommand ResetSpectrumCommand
        {
            get { return (this.resetSepctrumCommand) ?? (this.resetSepctrumCommand = new AsyncCommand(ResetSpectrum)); }
        }
        private async Task ResetSpectrum()
        {
            for (uint i = 0; i < 2; ++i)
            {
                await Task.Run(() =>{
                    LahgiWrapper_Static.ResetSpectrum(i);
                    CZTSpectrumEnergy.Reset();
                }) ;

            }
            
        }

        public List<ObservableCollection<HistoEnergy>> ModuleEnergySpectrums { get; set; }
        public ObservableCollection<HistoEnergy> SumEnergySpectrums { get; set; }
        public ObservableCollection<HistoEnergy> ScatterEnergySpectrums { get; set; }
        public ObservableCollection<HistoEnergy> AbsorberEnergySpectrums { get; set; }
        public void DrawSpectrum()
        {

            Stopwatch sw = new Stopwatch();
            sw.Start();

            if (IsLACCModuleInitiate == false)
            {
                return;
            }
            List<double[]> eCounts = new List<double[]>();
            List<HistoEnergy> histoEnergys = new List<HistoEnergy>();

            for (uint i = 0; i < 2; ++i)
            {
                eCounts = new List<double[]>();
                LahgiWrapper_Static.GetSpectrum(i, ref eCounts);

                histoEnergys = new List<HistoEnergy>();

                foreach(var eData in eCounts)
                {
                    histoEnergys.Add(new HistoEnergy(eData[0], (int)eData[1]));
                }    

                ModuleEnergySpectrums[(int)i] = new ObservableCollection<HistoEnergy>(histoEnergys);
                OnPropertyChanged($"ModuleEnergySpectrums[{i}]");
            }
            
            // 나머지 14개 채널은 빈 데이터로 설정
            for (uint i = 2; i < 16; ++i)
            {
                ModuleEnergySpectrums[(int)i] = new ObservableCollection<HistoEnergy>();
                OnPropertyChanged($"ModuleEnergySpectrums[{i}]");
            }

            eCounts = new List<double[]>();
            LahgiWrapper_Static.GetSumSpectrum(ref eCounts);
            histoEnergys = new List<HistoEnergy>();
            foreach (var eData in eCounts)
            {
                histoEnergys.Add(new HistoEnergy(eData[0], (int)eData[1]));
            }
            SumEnergySpectrums = new ObservableCollection<HistoEnergy>(histoEnergys);
            OnPropertyChanged(nameof(SumEnergySpectrums));

            eCounts = new List<double[]>();
            LahgiWrapper_Static.GetAbsorberSumSpectrum(ref eCounts);
            histoEnergys = new List<HistoEnergy>();
            foreach (var eData in eCounts)
            {
                histoEnergys.Add(new HistoEnergy(eData[0], (int)eData[1]));
            }
            AbsorberEnergySpectrums = new ObservableCollection<HistoEnergy>(histoEnergys);
            OnPropertyChanged(nameof(AbsorberEnergySpectrums));

            eCounts = new List<double[]>();
            LahgiWrapper_Static.GetScatterSumSpectrum(ref eCounts);
            histoEnergys = new List<HistoEnergy>();
            foreach (var eData in eCounts)
            {
                histoEnergys.Add(new HistoEnergy(eData[0], (int)eData[1]));
            }
            ScatterEnergySpectrums = new ObservableCollection<HistoEnergy>(histoEnergys);
            OnPropertyChanged(nameof(ScatterEnergySpectrums));


            OnPropertyChanged(nameof(ModuleEnergySpectrums));
            sw.Stop();
        }

        

        #endregion

        #region LACC Module Setting

        private bool isLACCModuleInitiate = false;
        public bool IsLACCModuleInitiate
        {
            get { return isLACCModuleInitiate; }
            set
            {
                isLACCModuleInitiate = value;
                OnPropertyChanged(nameof(IsLACCModuleInitiate));
                OnPropertyChanged(nameof(IsSessionAvailable));
                Application.Current.Dispatcher.Invoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(() => {
                        ((AsyncCommand)StartorStopSessionCommand).RaiseCanExecuteChanged();
                    }));
            }
        }

        private string LUTFolderDirectory = Path.Combine(Directory.GetCurrentDirectory(), "LUT Files");

        #region Mono
        public void InitiateMonoType()
        {
            if (ModuleInfoViewModels[0].IsModuleSet && ModuleInfoViewModels[1].IsModuleSet)
            {
                LACC_Control_Static = new LACC_Control(ModuleInfoViewModels[0].Module, ModuleInfoViewModels[1].Module);
                IsLACCModuleInitiate = true;
                initiating = false;
                return;
            }

            var pmtOrderInfo = new LACC_Module.ModulePMTOrderInfo { IsOrderChange = true, Order = new int[] { 0, 18, 1, 19, 2, 20, 11, 29, 10, 28, 9, 27, 3, 21, 4, 22, 5, 23, 14, 32, 13, 31, 12, 30, 6, 24, 7, 25, 8, 26, 17, 35, 16, 34, 15, 33 } };

            double[] scatterGain = new double[] {
                                                    0.743689349,
                                                    0.485597352,
                                                    0.733393991,
                                                    0.453444903,
                                                    0.433348959,
                                                    0.754890154,
                                                    0.5538563  ,
                                                    -0.430917509
                                                    };

            double[] absorberGain = new double[] {
                                                    0.743689349,
                                                    0.485597352,
                                                    0.733393991,
                                                    0.453444903,
                                                    0.433348959,
                                                    0.754890154,
                                                    0.5538563  ,
                                                    -0.430917509
                                                    };

            Debug.WriteLine("Making Scatter Module");
            VMStatus = "Making Scatter Module";
            ModuleInfoViewModels[0] = new ModuleInfoViewModel(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffset { x = -T265ToLACCOffset.X, y = -T265ToLACCOffset.Y, z = -T265ToLACCOffset.Z },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        scatterGain,
                                                        scatterGain,
                                                        pmtOrderInfo,
                                                        Path.Combine(LUTFolderDirectory, "MonoScatterLUT.csv"));

            Debug.WriteLine("Making Abosrober Module");
            VMStatus = "Making Absorber Module";
            ModuleInfoViewModels[1] = new ModuleInfoViewModel(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffset { x = -T265ToLACCOffset.X, y = -T265ToLACCOffset.Y, z = -T265ToLACCOffset.Z - 0.25 },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        absorberGain,
                                                        absorberGain,
                                                        pmtOrderInfo,
                                                        Path.Combine(LUTFolderDirectory, "MonoAbsorberLUT.csv"));

            if (!ModuleInfoViewModels[0].IsModuleSet || !ModuleInfoViewModels[1].IsModuleSet)
            {
                LACC_Control_Static = new LACC_Control(ModuleInfoViewModels[0].Module, ModuleInfoViewModels[1].Module);
                IsLACCModuleInitiate = false;
                initiating = false;
                VMStatus = "Mono-Type Module Setting Failed";
                return;
            }

            LACC_Control_Static = new LACC_Control(ModuleInfoViewModels[0].Module, ModuleInfoViewModels[1].Module);
            IsLACCModuleInitiate = true;
            initiating = false;

            VMStatus = "Initiate LACC Done";
        }
        #endregion
        public void InitiateSingleHeadQuadType()
        {
            try
            {
                IsLACCModuleInitiate = false;

                var pmtOrderInfo = new LACC_Module.ModulePMTOrderInfo { IsOrderChange = false, Order = new int[] { 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

                double offset = 0.083;
                // 2개 채널만 사용: Scatter 1개, Absorber 1개
                double[] xOffset = new double[] { 0, 0 };  // 4개 → 2개로 변경
                double[] yOffset = new double[] { 0, 0 };  // 4개 → 2개로 변경
                double offsetZ = -(0.251 + (31.5 - 21.5) / 1000);
                
                Debug.WriteLine("Making Scatter Module");
                VMStatus = "Making Scatter Module";
                // Scatter 모듈 1개만 생성
                var scatterGain = LACC_Module.LoadGain(Path.Combine(LUTFolderDirectory, "GainCorrectionMatrix_447278_scatter_1.csv"));
                var scatterMlpeGain = LACC_Module.LoadGain(Path.Combine(LUTFolderDirectory, "GainCorrectionMatrix_447278_scatter_1.csv"));
                ModuleInfoViewModels[0] = new ModuleInfoViewModel(ModuleInfo.QuadSingleHead,
                                            new LACC_Module.ModuleOffset { x = T265ToLACCOffset.X + xOffset[0], y = T265ToLACCOffset.Y + yOffset[0], z = T265ToLACCOffset.Z },
                                            new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                            scatterGain,
                                            scatterMlpeGain,
                                            pmtOrderInfo,
                                            Path.Combine(LUTFolderDirectory, "LUT9chEXP_447278_1_20210610_1mm_step2.csv"));

                Debug.WriteLine("Making Abosrober Module");
                VMStatus = "Making Absorber Module";
                // Absorber 모듈 1개만 생성
                var absorberGain = LACC_Module.LoadGain(Path.Combine(LUTFolderDirectory, "EnergyGainCorrectionMatrix_absorber_1.csv"));
                ModuleInfoViewModels[1] = new ModuleInfoViewModel(ModuleInfo.QuadSingleHead,
                                                        new LACC_Module.ModuleOffset { x = T265ToLACCOffset.X + xOffset[1], y = T265ToLACCOffset.Y + yOffset[1], z = T265ToLACCOffset.Z + offsetZ },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        absorberGain,
                                                        absorberGain,
                                                        pmtOrderInfo,
                                                        Path.Combine(LUTFolderDirectory, "LUT9chEXP_447279_1_20210329_1mm_step2.csv"));
                
                // 2개 모듈만 사용하여 LACC_Control 생성
                LACC_Control_Static = new LACC_Control(new LACC_Module[] { ModuleInfoViewModels[0].Module},
                                                        new LACC_Module[] { ModuleInfoViewModels[1].Module });
                IsLACCModuleInitiate = true;
                initiating = false;

                VMStatus = "Initiate LAHGI Quad Single head Success! (2 Channels)";
            }
            catch
            {
                VMStatus = "Initiate LAHGI Quad Single head Failed";
            }
        }
        public void InitiateDualHeadQuadType()
        {
            IsLACCModuleInitiate = false;
        }

        public ObservableCollection<ModuleInfoViewModel> ModuleInfoViewModels { get; set; } //16 Channels

        private AsyncCommand initiateLACCommand;
        public IAsyncCommand InitiateLACCommand
        {
            get { return (this.initiateLACCommand) ?? (this.initiateLACCommand = new AsyncCommand(InitiateLACCAsync)); }
        }
        private bool initiating;
        private async Task InitiateLACCAsync()
        {
            if (initiating == true)
                return;
            VMStatus = "Initiating LAHGI";
            initiating = true;
            await Task.Run(()=>
            { 
                LahgiWrapper_Static = new LahgiWrapper(eModuleManagedType.QUAD);         // 내부적으로 2개 채널만 활성화하도록 수정 필요
            });

            VMStatus = "Initiate LACC Done";
            initiating = false;
            IsLACCModuleInitiate = true;
            return;
        }

        private AsyncCommand<string> ecalInfoChangedCommand;
        public ICommand EcalInfoChangedCommand
        {
            get { return (this.ecalInfoChangedCommand) ?? (this.ecalInfoChangedCommand = new AsyncCommand<string>(EcalInfoChanged)); }
        }
        private async Task EcalInfoChanged(string a)
        {
            Debug.WriteLine("Channel num is " + a);
            try
            {
                int channelNum = Convert.ToInt32(a);
                // Mono 모드: 0 (Scatter), 1 (Absorber)만 유효
                if (channelNum >= 0 && channelNum < 2)
                {
                    await Task.Run(()=>ModuleInfoViewModels[channelNum].Fitting()).ConfigureAwait(false);
                }
                else
                {
                    Debug.WriteLine("Invalid channel number for Mono mode. Use 0 (Scatter) or 1 (Absorber)");
                }
            }
            catch
            {
                Debug.WriteLine("EcalInfoChanged parameter is wrong");
            }

        }

        #endregion

        #region Test Functions
        private AsyncCommand<object> testCommand;
        public ICommand TestCommand
        {
            get { return (this.testCommand) ?? (this.testCommand = new AsyncCommand<object>(TestFunction)); }
        }

        private async Task TestFunction(object t)
        {
            for (int i = 0; i < 144; ++i)
            await Task.Run(() =>
            {
                while (false)
                {
                //    if (!initiating)
                //    {
                //        //LahgiWrapper_Static.AddListModeData()
                //    }
                    //Trace.WriteLine("DataInQueue count: " + FPGAControl.DataInQueue.Count);
                    //Trace.WriteLine("ParsedQueue count: " + FPGAControl.ParsedQueue.Count);                    
                    //Trace.WriteLine("ShortArrayQueue count: " + FPGAControl.ShortArrayQueue.Count);
                    //Thread.Sleep(1000);
                }
                
            });
            // Write Function to be used     
        }

        private string vmStatus;
        public string VMStatus
        {
            get { return vmStatus; }
            set { vmStatus = value; OnPropertyChanged(nameof(VMStatus)); }
        }
        #endregion

        private RelayCommand<TextCompositionEventArgs> tbPrivewTextInputOnlyNumericCommand;
        public ICommand TBPrivewTextInputOnlyNumericCommand
        {
            get
            {
                return this.tbPrivewTextInputOnlyNumericCommand ??
                    (this.tbPrivewTextInputOnlyNumericCommand = new RelayCommand<TextCompositionEventArgs>(TBPrivewTextInputOnlyNumeric));
            }
        }

        public void TBPrivewTextInputOnlyNumeric(TextCompositionEventArgs e)
        {
            Regex regex = new Regex("[^0-9]+");
            e.Handled = regex.IsMatch(e.Text);
            Debug.WriteLine(e.Text);
        }

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }
    }







}
