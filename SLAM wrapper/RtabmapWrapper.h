#pragma once
#pragma managed(push, off)
#include "CppWrapper.h"
#pragma managed(pop)

using namespace System;
using namespace System::IO;
using namespace System::Diagnostics;
using namespace System::Drawing;
using namespace System::Windows;
using namespace System::Threading;
using namespace System::Collections;
using namespace System::Collections::Generic;
using namespace System::Runtime::InteropServices;


namespace HUREL {
	namespace Compton {

		public enum class eReconManaged
		{			
			CODED,
			COMPTON,
			HYBRID
		};
			public ref class RtabmapWrapper:IDisposable
			{
			private:
				Boolean mIsInitiated = false;				
			public:				
				Boolean InitiateRtabmap();

				void GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);
				void GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors);

				void GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data, bool bRealTime);	//240105 bRealTime = true : Realtime, , bool bRealTime = false : LMData


				void GetRealTimeRGB1(int% width, int% height, int% stride, IntPtr% data);	//240105
				void GetLMDataRGB(int% width, int% height, int% stride, IntPtr% data);	//240105



				void GetReconSLAMPointCloud(double time, eReconManaged reconType, List<array<double>^>^% vectors, List<array<double>^>^% colors, double voxelSize, bool useLoaded);
				
				Boolean StartSLAM();
				void StopSLAM();
				void ResetSLAM();

				void StopVideoStream();

				void GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);
				void GetSLAMOccupancyGrid(List<array<double>^>^% vectors, List<array<double>^>^% colors);	//231121-1 sbkwon

				void GetPoseFrame(array<double>^% mat);

				bool LoadPlyFile(System::String^ filePath);
				void GetLoadedPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);
				void GetOptimizePoses(List<array<double>^>^% poses);

				void SavePlyFile(System::String^ filePath);

				void GetOdomentryPos(double% x, double% y, double% z);

				/// <summary>카메라 내부 파라미터(fx,fy,cx,cy) 반환. 성공 시 true.</summary>
				bool GetCameraIntrinsics(float% fx, float% fy, float% cx, float% cy);

				void SetMeasurementFolderPath(System::String^ folderPath);
				void SetMeasurementFileName(System::String^ fileName);

				// RGBD 이미지 저장 여부 설정 (true: 저장, false: 저장 안 함)
				void SetSaveRgbdFrame(bool enable);

				// LM 측정 시작 시점 알림 (프레임 타임스탬프 기준 초기화)
				void BeginMeasurement();

				// RGBD 이미지 저장 시간 간격 설정 (초 단위, 0이면 매 프레임마다 저장)
				void SetRgbdFrameSaveInterval(double intervalSeconds);


				RtabmapWrapper();
				~RtabmapWrapper();
				!RtabmapWrapper();
			};

		
	}
}
