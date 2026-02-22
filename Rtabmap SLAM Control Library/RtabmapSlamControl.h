#pragma once

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/OccupancyGrid.h"

#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/CameraThread.h"

#include <iostream>

#include <rtabmap/core/Odometry.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/io/PointCloudIO.h>
#include <librealsense2/rs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

#include "Logger.h"

//20231109 lge added
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PointIndices.h>
#include <vector>

#include <pcl/point_types.h>
#include <boost/make_shared.hpp>
#include <mutex>


//#define T265_TO_LAHGI_OFFSET_X (0.005)
//#define T265_TO_LAHGI_OFFSET_Y (-0.270) //origin -0.275
//#define T265_TO_LAHGI_OFFSET_Z (-0.12) //front

//pFGI
//#define T265_TO_LAHGI_OFFSET_X (0.008)
//#define T265_TO_LAHGI_OFFSET_Y (-0.165) //origin -0.275
//#define T265_TO_LAHGI_OFFSET_Z (-0.005) //front

#define T265_TO_LAHGI_OFFSET_X (-0.005) //origin -.010
#define T265_TO_LAHGI_OFFSET_Y (-0.10) //origin -197
#define T265_TO_LAHGI_OFFSET_Z (0.005) //front

#define T265_To_Mask_OFFSET_X (T265_TO_LAHGI_OFFSET_X)
#define T265_To_Mask_OFFSET_Y (T265_TO_LAHGI_OFFSET_Y)
#define T265_To_Mask_OFFSET_Z (0.00)

namespace HUREL
{
	namespace Compton
	{

		class RtabmapSlamControl
		{

		private:

			Eigen::Matrix4d mInitOdo = Eigen::Matrix4d::Identity();

			rtabmap::CameraRealSense2* mCamera = nullptr;
			rtabmap::CameraThread* mCameraThread = nullptr;
			rtabmap::Odometry* mOdo = nullptr;// = rtabmap::Odometry::create();;

			cv::Mat mCurrentVideoFrame = cv::Mat();
			cv::Mat mCurrentDepthFrame = cv::Mat();
			cv::Mat mCurrentVideoFrameCopy = cv::Mat();
			cv::Mat mCurrentDepthFrameCopy = cv::Mat();

			void VideoStream();

			pcl::PointCloud<pcl::PointXYZRGB> mRealtimePointCloud = pcl::PointCloud<pcl::PointXYZRGB>();
			pcl::PointCloud<pcl::PointXYZRGB> mSlamedPointCloud = pcl::PointCloud<pcl::PointXYZRGB>();
			//Eigen::Matrix4d mCurrentOdometry = Eigen::Matrix4d::Identity();
			void SlamPipe();

			RtabmapSlamControl();

			open3d::geometry::PointCloud mLoadedPointcloud = open3d::geometry::PointCloud();
			std::vector < Eigen::Matrix4d> mPoses = std::vector<Eigen::Matrix4d>();
			void LockVideoFrame();

			void UnlockVideoFrame();
			void LockDepthFrame();

			void UnlockDepthFrame();

			float m_fxValue;
			float m_fyValue ;
			float m_cxValue;
			float m_cyValue;

			//231121-1 sbkwon
			open3d::geometry::PointCloud mOccupancyPCLGrid;
			double mgridWith;
			double mgridHeight;
			double mminX;
			double mminZ;
			void CalOccupancySize(float res, double* outWidth, double* outHeight, double* outMinX, double* outMinZ);
			open3d::geometry::PointCloud createOccupancyPCL(float res);

			int nSlamedPointCloudCount = 0;

			open3d::geometry::PointCloud pointcloudBackup = open3d::geometry::PointCloud(); //250214 자동 종료시 저장

			std::string mMeasurementFolderPath = ""; // 측정 데이터 저장 폴더 경로
			std::mutex mMeasurementFolderPathMutex; // 경로 접근 보호용 mutex

			std::string mMeasurementFileName = ""; // 측정 데이터 파일명 앞부분 (예: 20251208120913_test8)
			std::mutex mMeasurementFileNameMutex; // 파일명 접근 보호용 mutex

			// RGB/D 프레임 타임스탬프 관리용 (RealSense/RTAB-Map stamp 기준)
			double mFirstFrameStamp = 0.0;      // 측정 시작 후 첫 프레임의 stamp
			bool mHasFirstFrameStamp = false;   // 첫 프레임 stamp 초기화 여부
			std::mutex mFrameStampMutex;        // stamp 접근 보호용 mutex
			
			// 시간 동기화를 위한 첫 프레임의 실제 DateTime 저장
			std::chrono::system_clock::time_point mFirstFrameDateTime;  // 첫 프레임의 실제 DateTime
			bool mHasFirstFrameDateTime = false;  // 첫 프레임 DateTime 초기화 여부
			std::mutex mFirstFrameDateTimeMutex;  // DateTime 접근 보호용 mutex
			
			// 첫 측정 시점의 epoch time (밀리초) - LMData 타임스탬프 동기화를 위해 사용
			std::chrono::milliseconds mFirstMeasurementEpochTimeMs = std::chrono::milliseconds(0);
			bool mHasFirstMeasurementEpochTime = false;  // 첫 측정 시점 epoch time 초기화 여부
			std::mutex mFirstMeasurementEpochTimeMutex;  // epoch time 접근 보호용 mutex

			// RGBD 이미지를 저장할지 여부 (true: 저장, false: 저장 안 함)
			bool mSaveRgbdFrame = false;

			// RGBD 이미지 저장 시간 간격 (초 단위, 0이면 매 프레임마다 저장)
			double mRgbdFrameSaveInterval = 0.0;
			double mLastRgbdFrameSaveTime = 0.0;  // 마지막 저장 시각 (stamp 기준, 초)
			std::mutex mRgbdFrameSaveIntervalMutex;  // 시간 간격 설정 접근 보호용

		public:
			bool mIsInitiate = false;
			bool mIsVideoStreamOn = false;
			bool mIsSlamPipeOn = false;
			bool mOdoInit = false;

			bool Initiate();

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				Eigen::Matrix4d GetOdomentry();

			std::vector< Eigen::Matrix4d> GetOptimizedPoses();

			void	SetCurrentFrame();		//240104 - RGB set 
			void	SetCurrentFrame1();		//240104 - RGB set 
			void	SetCurrentVideoFrame();		//240104 - RGB set 
			cv::Mat GetCurrentVideoFrame();
			cv::Mat GetCurrentVideoFrame1(bool bCopy = false);	//240104 - SetCurrentVideoframe???? ?????? ???? ??? //240312 B : bCopy = true(???? ????)
			cv::Mat GetCurrentVideoFrame2();	//240312 B - Pointcloud Radiationimage ???? ?? ?????? ???? ???
			void	SetCurrentDepthFrame();		//240104 - Depth set
			cv::Mat GetCurrentDepthFrame();
			cv::Mat GetCurrentDepthFrame1();	//240104 - SetCurrentDepthFrame???? ?????? ???? ???
			cv::Mat GetCurrentPointsFrame(double res);

			open3d::geometry::PointCloud GetRTPointCloud();
			open3d::geometry::PointCloud GetRTPointCloudTransposed();

			void StartVideoStream();
			void StopVideoStream();

			void StartSlamPipe();
			void StopSlamPipe();

			void ResetSlam();

			void IntrinsicParamters();

			/// <summary>카메라 내부 파라미터 (fx, fy, cx, cy) 반환. 비디오 스트림 동작 중일 때 유효.</summary>
			bool GetCameraIntrinsics(float& fx, float& fy, float& cx, float& cy);

			open3d::geometry::PointCloud GetSlamPointCloud();
			open3d::geometry::PointCloud GetSlamPointCloudBackup() {
				return pointcloudBackup;
			}	//250214

			std::vector<double> getMatrix3DOneLineFromPoseData();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud();	//231025-1 sbkwon
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud(cv::Mat& depth, cv::Mat& rgb);	//231031-1 sbkwon
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointClouddowin(cv::Mat &depth, cv::Mat &rgb, int down = 2);	//240621 sbkwon
			open3d::geometry::PointCloud PclToOpen3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp);	//231025-1 sbkwon
			open3d::geometry::PointCloud RTPointCloudTransposed(open3d::geometry::PointCloud& initialPC, Eigen::Matrix4d transMatrix);	//231106-2 sbkwon

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inputCloud, float voxelSize, pcl::PointIndices& indices);

			bool LoadPlyFile(std::string filePath);

			open3d::geometry::PointCloud GetLoadedPointCloud();

			open3d::geometry::PointCloud GetOccupancyPointCloud();	//231121-1 sbkwon

			std::tuple<double, double, double> GetOdomentryPos();

			int GetSlamedPointCloudCount() { return nSlamedPointCloudCount; }

			// ���� ������ ���� ���� ��� ����
			void SetMeasurementFolderPath(const std::string& folderPath) 
			{ 
				std::lock_guard<std::mutex> lock(mMeasurementFolderPathMutex);
				mMeasurementFolderPath = folderPath; 
			}

			// 측정 데이터 파일명 앞부분 설정 (예: 20251208120913_test8)
			void SetMeasurementFileName(const std::string& fileName)
			{
				std::lock_guard<std::mutex> lock(mMeasurementFileNameMutex);
				mMeasurementFileName = fileName;
			}

			// LM 측정 시작 시 호출하여 프레임 타임스탬프 기준을 초기화
			void BeginMeasurement();

			// 현재 카메라 프레임을 획득하고, 측정 시작 기준 상대 시각(ms)을 파일명으로 사용해
			// RGB / Depth 이미지를 저장 (측정 폴더 경로 사용)
			void SaveCurrentRgbdFrameWithTimestamp();

			// 외부에서 RGBD 이미지 저장 On/Off 제어
			void SetSaveRgbdFrame(bool enable) { mSaveRgbdFrame = enable; }

			// RGBD 이미지 저장 시간 간격 설정 (초 단위, 0이면 매 프레임마다 저장)
			void SetRgbdFrameSaveInterval(double intervalSeconds);
			
			// 첫 측정 시점의 epoch time 반환 (LMData 타임스탬프 동기화를 위해)
			std::chrono::milliseconds GetFirstMeasurementEpochTimeMs() const;

			// mSlamedPointCloud를 PLY 파일로 저장 (측정 종료 시 호출)
			bool SaveSlamedPointCloudToPly(const std::string& filePath);

		public:
			static RtabmapSlamControl& instance();
			~RtabmapSlamControl();
		};


	};
};
