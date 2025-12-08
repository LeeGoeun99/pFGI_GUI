#pragma once
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <open3d/geometry/Octree.h>

#include "ListModeData.h"
#include "ReconPointCloud.h"
#include "LahgiControl.h"

#include <boost/make_shared.hpp>

namespace HUREL {
	namespace Compton
	{
		using namespace cv;
		using namespace Eigen;

		class LahgiControl;
		const static bool mCodeMask[37][37] = {
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		};

		enum class eRadiationImagingMode
		{
			CODED,
			COMPTON,
			HYBRID
		};

		constexpr double RadiationImageFOV = 130;
		constexpr double RadiationImageSeperateDegree = 5;
		constexpr double RadiationImageResponseSurfaceDistance = 3;

		class RadiationImage
		{
		private:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		public:

			std::vector<ListModeData> mListedListModeData;
			Eigen::Matrix4d mDetectorTransformation;


			//		  ^
			//<-- x   | 
			//        y 
			Mat mDetectorResponseImage;

			//		  ^
			//<-- x   | 
			//        y 
			Mat mComptonImage;
			Mat mCodedImage;
			Mat mHybridImage;
			static void ShowCV_32SAsJet(cv::Mat img, int size);
			static cv::Mat GetCV_32SAsJet(cv::Mat img, int size);
			static cv::Mat GetCV_32SAsJet(cv::Mat img, int size, double minValuePortion);
			static cv::Mat GetCV_32SAsJetZero(cv::Mat img, int size, double minValuePortion);//231025-1 sbkwon
			static cv::Mat GetCV_32SAsJet(cv::Mat img, double minValuePortion);	//231025-1 sbkwon

			static cv::Mat GetAnotation(cv::Mat img, cv::Mat& imgAno, const std::string& IsotopeName, double minValuePortion = 0.8);	//240930 sbkwon : 방사선 영상내 핵종 이름 표시 //250203

			RadiationImage(); //231113-1 sbkwon
			RadiationImage(std::vector<ListModeData>& data);

			RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double m2D, int maxValue);

			
			RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int maxValue = 5, bool bfullRange = false);	//230311 //240326 fullrange = true : 360*180 영상 재구성
			RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D, int maxValue);	//231025-1 sbkwon point cloud
			double OverlayValue(Eigen::Vector3d point, eRadiationImagingMode mode);

			static void OverlayRadimgToP3(cv::Mat& p3, const cv::Mat& radImg);

			//231109-1 sbkwon
			void SetIndexPos();
			static int TotalIndexbyPos[5][7569][2];

			//231120 sbkwon
			void	fillmissing(Eigen::ArrayXd& arr, double dlimit = 0.0);
			int		fillmissing(Eigen::ArrayXd& arr, int nCur, double base, int baseInd, double dlimit);
			int 	fillmissingFront(Eigen::ArrayXd& arr, int nCur, double base, double interval, double dlimit);
			int		fillmissingBack(Eigen::ArrayXd& arr, int nCur, double base, double interval, double dlimit);
			int		findRight(Eigen::ArrayXd& arr, int nIndex);

			void simplifiedFillMissing(Eigen::ArrayXd& arr);
			void interp1Linear(Eigen::ArrayXd& arr, int leftIndex, int rightIndex);

			//test
			static int nCountMat;
		};
	};
};

