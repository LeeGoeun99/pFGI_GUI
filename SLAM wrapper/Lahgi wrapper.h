#pragma once
#pragma managed(push, off)
#include "CppWrapper.h"
#pragma managed(pop)

using namespace System;
using namespace System::Collections::Generic;
using namespace System::Runtime::InteropServices;


namespace HUREL {
	namespace Compton {



		public enum class eModuleManagedType
		{
			MONO,
			QUAD,
			QUAD_DUAL
		};
		public enum class eReconType
		{
			CODED,
			COMPTON,
			HYBRID,
		};

		public ref struct sBitmapWrapper
		{
			IntPtr ptr;
			int width;
			int height;
			int stride;
			sBitmapWrapper(IntPtr _ptr, int _width, int _height, int _stride)
			{
				ptr = _ptr;
				width = _width;
				height = _height;
				stride = _stride;
			}
			sBitmapWrapper(sBitMapUnmanged data)
			{
				ptr = IntPtr(data.ptr);
				width = data.width;
				height = data.height;
				stride = data.step;
			}
		};

		public ref class LahgiWrapper
		{
		private:
		public:
			LahgiWrapper();
			bool Initiate(eModuleManagedType type);
			void AddListModeDataWraper(array<unsigned short>^ adcData);

			void SetEchks(List<array<double>^>^ echks, List<int>^ elements);	//240123
			void SelectEchks(List<int>^ elements);	//240123

			void GetRelativeListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE);
			Int64 GetListedListModeDataSize();

			void ResetListmodeData();

			void SaveListModeData(System::String^ fileName);

			bool LoadListModeData(System::String^ filePath);

			void GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount);

			void GetEcal(unsigned int channelNumer, double% ecalA, double% ecalB, double% ecalC);
			void SetEcal(unsigned int channelNumer, double ecalA, double ecalB, double ecalC);



			void GetSumSpectrum(List<array<double>^>^% energyCount);
			void GetSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time);	//231100-GUI sbkwon
			void GetAbsorberSumSpectrum(List<array<double>^>^% energyCount);
			void GetScatterSumSpectrum(List<array<double>^>^% energyCount);
			void GetScatterSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time);
			void GetAbsorberSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time);
			void GetSpectrumData(List<array<double>^>^% energyCount, unsigned int type, unsigned int ch);	//250410

			void GetSpectrumByTime(unsigned int channelNumer, List<array<double>^>^% energyCount, unsigned int time);	//231123 sbkwon

			//240228 고장 검사
			void SetUseFD(bool set);	//240311
			void CopyPMTData();
			void GetGainref(unsigned int channelNumer, List<double>^% corrMatRef);//채널벌 게인 획득
			void GetPMTEnergyData(unsigned int channelNumer, List<array<double>^>^% energyCount);	//240228 고장검사
			void GetPMTEnergyData(unsigned int channelNumer, List<double>^ corrMatIn, List<array<double>^>^% energyCount);	//240228 고장검사
			void GetPMTCorrMatIn(unsigned int channelNumer, List<int>^ usedPeak, List<double>^ range_bkg, List<double>^% corrMatIn);	//240228 usedPeak 를 이용 CorrMatIn 구하기
			void GetPMTCorrMatInBeforGain(unsigned int channelNumer, List<int>^ usedPeak, List<double>^ range_bkg, List<double>^ corrMatIn, List<double>^% corrMatOut);	//240315

			void ResetSpectrum(unsigned int channelNumber);


			sBitmapWrapper^ GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter);

			void InitRadiationImage();//231113-1 sbkwon
			Tuple< sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>^ Get2dRadationImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion);
			Tuple< sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>^ Get2dRadationImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double minValuePortion);//231025-1 sbkwon
			Tuple< sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>^ GetRadation2dImageCount(int count, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion, int time, int maxValue, bool fullrange, bool labeling);//231212, 240311, 240326, 241021 labeling
			Tuple< sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>^ GetRadation2dImageCount(int count, double s2M, double det_W, double resImprov, double m2D, double minValuePortion, int time, int maxValue, bool labeling);//231100-GUI, 240122 sbkwon , 241021 labeling

			sBitmapWrapper^ GetTransPoseRadiationImage(int timeInMiliSeconds, double minValuePortion, double resolution);

			void GetRealTimeReconImage(double time, eReconType reconType, int% width, int% height, int% stride, IntPtr% data);

			int GetSlamedPointCloudCount();

			//2404 : MLEM
			bool LoadMLEMData(System::String^ PLYPath, System::String^ LMDPath, bool bLoad, int nSize);
			bool CalMLEMList(System::String^ systemMPath, List<double>^ energy, List<double>^ EgateMin, List<double>^ EgateMax, double minValuePer);
			bool CalMLEM(System::String^ systemMPath, double energy, double EgateMin, double EgateMax, double minValuePer);
			void GetMLEMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, int nNo);

			bool Get2DMLEMData(System::String^ Path, int nNo); //250107 2D MLEM : rgb + 2D

			static void Logging(std::string className, std::string msg);
		};



	};
}
