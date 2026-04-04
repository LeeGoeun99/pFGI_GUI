#pragma once

#include <vector>

//#include "RealsenseControl.h"

#include "RtabmapSlamControl.h"

#include "Module.h"
#include "ListModeData.h"
#include "ReconPointCloud.h"
#include "Logger.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <tbb/concurrent_vector.h>
#include <tbb/concurrent_queue.h>
#include <concurrent_vector.h>
#include <concurrent_queue.h>
#include <thread>
#include <future>
#include <array>

#define ACTIVE_AREA_LENGTH 0.

//#define Test_Log

namespace HUREL {
	namespace Compton {
		class RadiationImage;
		class ReconPointCloud;

		//230911 sbkwon : ListModeData.h ?? ??? ???
		/*struct sEnergyCheck
		{
			double minE;
			double maxE;
		};*/

		//240228 PMT Data
		struct PMTData {
			int InteractionChannel;
			Eigen::Array<float, 1, 9> data;
		};

		class LahgiControl
		{
		private:

			Module** mScatterModules;  //./Module information/MONOScatter1/Gain.csv, LUT.csv ...
			Module** mAbsorberModules;	//./Module information/QUADScatter1/Gain.csv, LUT.csv ...
			eMouduleType mModuleType;
			tbb::concurrent_vector <ListModeData> mListedListModeData;
			tbb::concurrent_vector <EnergyTimeData> mListedEnergyTimeData;

			tbb::concurrent_vector <PMTData> mListedPMTGainData;		//240228 : ???? 
			tbb::concurrent_vector <PMTData> mListedPMTGainDataCopy;	//240228 : ????


			LahgiControl();
			inline static ListModeData MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double& scatterEnergy, double& absorberEnergy, Eigen::Matrix4d& transformation, std::chrono::milliseconds& timeInMili, sEnergyCheck echk);//230911 sbkwon : Energy check ??? - ???? ???? ???
			inline static ListModeData MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double& scatterEnergy, double& absorberEnergy, Eigen::Matrix4d& transformation);
			//CodeMaks Setting
			double mMaskThickness = 0.006;

			tbb::concurrent_queue<std::array<unsigned short, 144>> mShortByteDatas;
			std::future<void> ListModeDataListeningThread;
			std::mutex eChksMutex;
			std::mutex eSelectChksMutex;	//240123
			bool mIsListModeDataListeningThreadStart = false;
			bool mIsListModeDataListeningThreadRun = false;

			void ListModeDataListening();

			double mListModeImgInterval;

			std::vector<sEnergyCheck> eChk;
			std::vector<int> eSelectChk;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				static LahgiControl& instance();

			void SetEchk(std::vector<sEnergyCheck> eChksInput);
			void SelectEchk(std::vector<int> eChksInput);	//240123

			bool SetType(eMouduleType type);

			~LahgiControl();
			void AddListModeData(const unsigned short(byteData)[144], Eigen::Matrix4d deviceTransformation);
			void AddListModeDataEigen(const unsigned short(byteData)[144], Eigen::Matrix4d deviceTransformation);
			void AddListModeDataWithTransformation(const unsigned short byteData[144]);
			void AddListModeDataWithTransformationVerification(const unsigned short byteData[]);
			void AddListModeDataWithTransformationLoop(std::array<unsigned short, 144> byteData, std::chrono::milliseconds& timeInMili, Eigen::Matrix4d& deviceTransformation);

			eMouduleType GetDetectorType();

			const std::vector<ListModeData> GetListedListModeData() const;
			std::vector<ListModeData> GetListedListModeData();
			const std::vector<ListModeData> GetListedListModeData(long long timeInMililseconds) const;
			std::vector<ListModeData> GetListedListModeData(long long timeInMililseconds);
			std::vector<ListModeData> GetListedListModeData(sEnergyCheck echk);//230911 sbkwon : Energy Check ????
			std::vector<ListModeData> GetListedListModeData(long long timeInMililseconds, sEnergyCheck echk);	//230911 sbkwon : Energy Check ????
			std::vector<ListModeData> GetEfectListedListModeData(int nEfectCount, long long time);//231121-1 sbkwon : ???????
			std::vector<ListModeData> GetEfectListedListModeData(int nEfectCount);//231121-1 sbkwon : ???????

			std::vector<ListModeData> GetEfectListedListModeData(const int& nElement, const int& nEfectCount, const long long& time);//240930 sbkwon : ??? ?????? ????
			std::vector<int> GetSelectEchek(); //240930 sbkwon : 

			std::vector<EnergyTimeData> GetListedEnergyTimeData();
			const std::vector<EnergyTimeData> GetListedEnergyTimeData(long long timeInMililseconds) const;
			std::vector<EnergyTimeData> GetListedEnergyTimeData(long long timeInMililseconds);
			std::vector<EnergyTimeData> GetListedEnergyTimeData(int fpgaChannelNumber, long long timeInMililseconds);	//231123 sbkwon



			size_t GetListedListModeDataSize();

			// Last listed list-mode event: Scatter/Absorber interaction energy (keV). Returns false if empty.
			bool TryGetLastListedListModeEnergies(double& scatterInteractionEnergyKeV, double& absorberInteractionEnergyKeV);

			void ResetListedListModeData();
			void SaveListedListModeData(std::string filePath);
			bool LoadListedListModeData(std::string filePath);
			bool LoadListedEnergyTimeData(std::string filePath);

			EnergySpectrum& GetEnergySpectrum(int fpgaChannelNumber);
			EnergySpectrum GetSumEnergySpectrum();
			EnergySpectrum GetAbsorberSumEnergySpectrum();
			EnergySpectrum GetScatterSumEnergySpectrum();

			std::tuple<double, double, double> GetEcalValue(int fpgaChannelNumber);
			void SetEcalValue(int fpgaChannelNumber, std::tuple<double, double, double> ecal);
			void ResetEnergySpectrum();
			void ResetEnergySpectrum(int fpgaChannelNumber);

			ReconPointCloud GetReconRealtimePointCloudComptonUntransformed(open3d::geometry::PointCloud& pc, double time);
			ReconPointCloud GetReconRealtimePointCloudCompton(open3d::geometry::PointCloud& pc, double time);

			ReconPointCloud GetReconOverlayPointCloudCoded(open3d::geometry::PointCloud& pc, double time);
			ReconPointCloud GetReconOverlayPointCloudCompton(open3d::geometry::PointCloud& pc, double time);
			ReconPointCloud GetReconOverlayPointCloudHybrid(open3d::geometry::PointCloud& pc, double time);

			cv::Mat GetResponseImage(int imgSize, int pixelCount = 80, double timeInSeconds = 0, bool isScatter = true);

			Eigen::Matrix4d t265toLACCPosTransform;
			Eigen::Matrix4d t265toLACCPosTransformInv;
			Eigen::Matrix4d t265toLACCPosTranslate;
			Eigen::Matrix4d t265toLACCPosTransCalc;	//231012 sbkwon : ?????? ???? ????(???? ??? ??????), t265toLACCPosTransform * t265toLACCPosTransformInv * t265toLACCPosTranslate;

			//240228 - ??????
			void AddListModeDataWithTransformationLoopFD(std::array<unsigned short, 144> byteData, std::chrono::milliseconds& timeInMili, Eigen::Matrix4d& deviceTransformation);
			bool bUseFaultDiagnosis;	//?????? ?????
			void SetUseFD(bool set) { bUseFaultDiagnosis = set; }
			void CopyListedPMTEnergyData();	//240228 : ?????? PMT Data?? ????
			std::vector<double> GetGainref(int fpgaChannelNumber);	//240228 : ??? Gain?? ???
			std::vector<double> GetListedPMTEnergyData(int fpgaChannelNumber);	//240228 : ??? PTM ?????? ?????? ???
			std::vector<double> GetListedPMTEnergyData(int fpgaChannelNumber, std::vector<double> dCorrMatIn);	//240228 : ??? PTM ?????? ?????? ???
			std::vector<double> GetPMTCorrMatIn(int fpgaChannelNumber, std::vector<int> usedPeak, std::vector<double> range_bkg);	//240228 : ??? usedPeak range?? ?????? Gain?? ???
			std::vector<double> GetPMTCorrMatInBeforGain(int fpgaChannelNumber, std::vector<int> usedPeak, std::vector<double> range_bkg, std::vector<double> dCorrMatIn);	//240315

			//2404 : MLEM
			bool CalMLEM(const std::string& systemMPath, const double& energy, const double& EgateMin, const double& EgateMax, const double& minValuePer);
			bool CalMLEMList(const std::string& systemMPath, const std::vector<double>& energy, const std::vector<double>& EgateMin,
				const std::vector<double>& EgateMax, const double& minValuePer);
			int GetSystemMatrixIndex(double eE);
			bool LoadMLEMData(const std::string& FilePath, const std::string& LMDPath, bool bLoad, int nSize);
			open3d::geometry::PointCloud Reconstruct3dPostProcessing(std::vector<ListModeData>& ComptonlmDataEff, std::vector<ListModeData>& CodedlmDataEff, Eigen::MatrixXd& Egate,
				Eigen::MatrixXd& systemmatrix, open3d::geometry::PointCloud& pc, HUREL::Compton::ReconPointCloud* outReconPC);
			open3d::geometry::PointCloud mMLEMPointcloud = open3d::geometry::PointCloud();	//MLEM ????? ???? ????? ?????
			open3d::geometry::PointCloud mCalMLEMPointcloud = open3d::geometry::PointCloud();	//MLEM ??? ???? ????
			open3d::geometry::PointCloud GetMLEMPointCloud(const int& nNo);	//MLEM ??? point cloud ???

			//240429 MLEM ??? Data
			std::vector<open3d::geometry::PointCloud> m_vMLEMPC;

			//2D MLEM
			void Reconstruct2dPostProcessing(std::vector<ListModeData>& ComptonlmDataEff, std::vector<ListModeData>& CodedlmDataEff, Eigen::MatrixXd& Egate,
				Eigen::MatrixXd& systemmatrix, open3d::geometry::PointCloud& pc, HUREL::Compton::ReconPointCloud* outReconPC);

			open3d::geometry::PointCloud mStillPointCloud = open3d::geometry::PointCloud();	//240621 rgb, depth ?? ????? pcl
			std::vector<cv::Mat> m_2DMLEM;
			cv::Mat m_rgb;
			std::string savepath = "";

			cv::Mat m_Cal2D;	//250107 
			cv::Mat GetRGB() { return m_rgb; }	//250107 
			bool GetMLEM2D(const std::string& FilePath, const int& nNo);	//250107
		};
	}
}

