#pragma once
#include "LogWrapperCaller.h"
#include "EnergySpectrumData.h"

#include <tuple>

#include <map>

namespace HUREL {
	namespace Compton {
		enum class eModuleCppWrapper
		{
			MONO,
			QUAD,
			QUAD_DUAL
		};
		enum class eReconCppWrapper
		{
			CODED,
			COMPTON,
			HYBRID,
		};

		struct ListModeDataCppWrapper
		{
			double ScatterRelativeInteractionPointX;
			double ScatterRelativeInteractionPointY;
			double ScatterRelativeInteractionPointZ;
			double ScatterInteractionEnergy;
			double AbsorberRelativeInteractionPointX;
			double AbsorberRelativeInteractionPointY;
			double AbsorberRelativeInteractionPointZ;
			double AbsorberInteractionEnergy;
		};



		struct ReconPointCppWrapper
		{
			double pointX;
			double pointY;
			double pointZ;

			double colorR;
			double colorG;
			double colorB;
			double colorA;

			double reconValue;
		};

		typedef struct BitmapUnmanaged
		{
			uint8_t* ptr;
			int width;
			int height;
			int step;
			int channelSize;
		}sBitMapUnmanged;


		class LahgiCppWrapper
		{
		private:
			//240930 sbkwon : Isotope enum ���� : Energy Spectrum�� �����ؾ���.
			enum IsotopeElement
			{
				None = 0,
				Co58,
				Co60,
				Cs137,
				Eu152,
				//Cs134,
				//I131,
				//Te129m,
				//Ag110m,
				Pu238,
				//Pu239,
				//Pu240,
				//Pu241,
				//Ir192,
				//Se75,
				//U235,
				//U238,
				Am241,
				Ba133,
				Na22,
				//Eu152,
				//Co57,
				Cd109,
				//I125,
				//Tc99m,
				Annihilation,
				F18,
				K40,
				Tl208,
				Bi214,
				Pb212
			};

			LahgiCppWrapper() 
			{
				//240930 sbkwon : Isotope �̸� ����
				isotopeList[Co58] = "Co-58";
				isotopeList[Co60] = "Co-60";
				isotopeList[Cs137] = "Cs-137";
				isotopeList[Eu152] = "Eu-152";
				isotopeList[Pu238] = "Pu-238";
				isotopeList[Am241] = "Am-241";
				isotopeList[Ba133] = "Ba-133";
				isotopeList[Na22] = "Na-22";
				isotopeList[Cd109] = "Cd-109";
				isotopeList[Annihilation] = "Annihilation";
				isotopeList[F18] = "F-18";
				isotopeList[K40] = "K-40";
				isotopeList[Tl208] = "Tl-208";
				isotopeList[Bi214] = "Bi-214";
				isotopeList[Pb212] = "Pb-212";
			};
		public:

			bool SetType(eModuleCppWrapper type);

			void SetEchks(std::vector<std::vector<double>>  echks, std::vector<int> elements);	//240123
			void SelectEchks(std::vector<int> elements);	//240123
			void AddListModeDataWithTransformation(const unsigned short* byteData);

			std::vector< ListModeDataCppWrapper> GetRelativeListModeData();
			void ResetListedListModeData();
			void RestEnergySpectrum(int channelNumber);

			std::tuple<double, double, double> GetEcalValue(unsigned  int fpgaChannelNumber);
			void SetEcalValue(int fpgaChannelNumber, std::tuple<double, double, double> ecal);

			size_t GetListedListModeDataSize();

			std::vector<BinningEnergy> GetSpectrum(int channelNumber);
			std::vector<BinningEnergy> GetSumSpectrum();
			std::vector<BinningEnergy> GetAbsorberSumSpectrum();
			std::vector<BinningEnergy> GetScatterSumSpectrum();
			std::vector<BinningEnergy> GetScatterSumSpectrum(int time);
			std::vector<BinningEnergy> GetAbsorberSumSpectrum(int time);
			std::vector<BinningEnergy> GetSumSpectrum(int time);	//231100-GUI sbkwon
			std::vector<BinningEnergy> GetSpectrumData(int type, int ch);//250410

			void SetUseFD(bool set);	//240311
			void CopyPMTData();	//240228 PMT Data copy
			std::vector<double> GetGainref(unsigned int channelNumer);//240228 : ����˻�
			std::vector<BinningEnergy> GetPMTEnergyData(unsigned int channelNumer);//240228 : ����˻�
			std::vector<BinningEnergy> GetPMTEnergyData(unsigned int channelNumer, std::vector<double> corrMatIn);//240228 : ����˻�
			std::vector<double> GetPMTCorrMatIn(unsigned int channelNumer, std::vector<int> usedPeak, std::vector<double> range_bkg);//240228 : ����˻�
			std::vector<double> GetPMTCorrMatInBeforGain(unsigned int channelNumer, std::vector<int> usedPeak, std::vector<double> range_bkg, std::vector<double> corrMatIn);//240315

			std::vector<BinningEnergy> GetSpectrum(int channelNumber, int time);	//231123 sbkwon

			bool SaveListedListModeData(std::string filePath);
			bool LoadListedListModeData(std::string filePath);

			/// <summary>
			/// uint8_t* outImgPtr, int outWidth, int outHeight, int outStep, int outChannelSize
			/// </summary>
			/// <returns>uint8_t* outImgPtr, int outWidth, int outHeight, int outStep, int outChannelSize</returns>
			sBitMapUnmanged GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter);

			/// <summary>
			/// 
			/// </summary>
			/// <returns>Coded Compton Hybrid</returns>
			std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  GetRadiation2dImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion);
			std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  GetRadiation2dImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double minValuePortion);//231025-1 sbkwon
			std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  GetRadation2dImageCount(int count, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion, int time, int maxValue, bool fullrange);//231212, 240311
			std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  GetRadation2dImageCount(int count, double s2M, double det_W, double resImprov, double m2D, double minValuePortion, int time, int maxValue);//231100-GUI, 240122 sbkwon

			std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  GetRadation2dImageCountLabel(int count, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion, int time, int maxValue, bool fullrange);//240930 sbkwon
			std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  GetRadation2dImageCountLabel(int count, double s2M, double det_W, double resImprov, double m2D, double minValuePortion, int time, int maxValue);//240930 sbkwon
			std::map<int, std::string> isotopeList;	//240930 sbkwon


			void InitRadiationImage(); //231113-1 sbkwon

			sBitMapUnmanged GetTransPoseRadiationImage(int timeInMiliSeconds, double minValuePortion, double resolution);

			int GetSlamedPointCloudCount();

			//2404 : MLEM
			bool LoadMLEMData(std::string PLYPath, std::string LMDPath, bool bLoad, int nSize);
			bool CalMLEMList(std::string systemMPath, std::vector<double> energy, std::vector<double> EgateMin, std::vector<double> EgateMax, double minValuePer);
			bool CalMLEM(std::string systemMPath, double energy, double EgateMin, double EgateMax, double minValuePer);
			std::vector<ReconPointCppWrapper> GetMLEMPointCloud(int nNo);

			bool Get2DMLEMData(std::string PLYPath, int nNo); //250107 2D MLEM : rgb + 2D

			static LahgiCppWrapper& instance();
		};

		class RtabmapCppWrapper
		{
		private:
			RtabmapCppWrapper() {};
			uint8_t* mColorImg = nullptr;
		public:

			bool GetIsSlamPipeOn();
			bool GetIsVideoStreamOn();


			bool Initiate();

			std::vector<ReconPointCppWrapper> GetRTPointCloud();
			std::vector<ReconPointCppWrapper> GetRTPointCloudTransposed();

			bool GetCurrentVideoFrame(uint8_t** outImgPtr, int* outWidth, int* outHeight, int* outStep, int* outChannelSize, bool bRealTime);	//240105 bRealTime = true : Realtime, , bool bRealTime = false : LMData


			bool GetCurrentVideoFrame1(uint8_t** outImgPtr, int* outWidth, int* outHeight, int* outStep, int* outChannelSize, bool bRealTime);	//240105
			bool GetLMDataVideoFrame(uint8_t** outImgPtr, int* outWidth, int* outHeight, int* outStep, int* outChannelSize, bool bRealTime);	//240105



			void StartVideoStream();
			void StopVideoStream();

			bool StartSlamPipe();
			void StopSlamPipe();
			void ResetSlam();

			bool LoadPlyFile(std::string filePath);
			void SavePlyFile(std::string filePath);

			std::tuple<double, double, double> GetOdomentryPos();

			std::vector<ReconPointCppWrapper> GetSlamPointCloud();
			std::vector<ReconPointCppWrapper> GetSLAMOccupancyGrid();	//231121-1 sbkwon

			std::vector<ReconPointCppWrapper> GetLoadedPointCloud();

			std::vector<double> getMatrix3DOneLineFromPoseData();

			std::vector<ReconPointCppWrapper> GetReconSLAMPointCloud(double time, eReconCppWrapper reconType, double voxelSize, bool useLoaded);
			std::vector<std::vector<double>> GetOptimizedPoses();
			static RtabmapCppWrapper& instance();
		};


	};
};
