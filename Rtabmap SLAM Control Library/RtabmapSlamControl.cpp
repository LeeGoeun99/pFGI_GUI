#include "RtabmapSlamControl.h"
#include <future>
#include <mutex>

using namespace HUREL::Compton;



void ShowCV_32FAsJet(cv::Mat img, int size)
{
	if (img.type() != CV_32F)
	{
		return;
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, &minValue, &maxValue);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<float>(i, j)) - minValue)
				/ (maxValue - minValue) * 255);
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::Mat showImg;

	int sizeHeight = size;
	int sizeWidth = size;

	if (colorImg.size().height > colorImg.size().width)
	{
		sizeWidth = size * colorImg.size().width / colorImg.size().height;
	}
	else
	{
		sizeHeight = size * colorImg.size().height / colorImg.size().width;
	}

	cv::resize(colorImg, showImg, cv::Size(sizeWidth, sizeHeight), 0, 0, cv::INTER_NEAREST_EXACT);
	cv::imshow("img", showImg);
	cv::waitKey(0);
}


HUREL::Compton::RtabmapSlamControl::RtabmapSlamControl()
{

}

bool HUREL::Compton::RtabmapSlamControl::Initiate()
{
	std::string msg;
	std::string* outMessage = &msg;
	try {
		rs2::context ctx = rs2::context();
		rs2::device_list devs = ctx.query_devices();

		rs2::config cfgD455 = rs2::config();
		int devCounts = devs.size();
		if (devCounts == 0)
		{
			*outMessage += "No cameras";
			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
			return false;
		}
		
		bool d455Found = false;
		std::string usbInfo;
		for (int i = 0; i < devCounts; ++i)
		{
			std::string devInfo = devs[i].get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
			if (devInfo == "Intel RealSense D455")
			{
				d455Found = true;
				usbInfo = devs[i].get_info(rs2_camera_info::RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
				break;
			}
		}
		
		if (!d455Found)
		{
			*outMessage += "Intel RealSense D455 camera not found. Available devices: ";
			for (int i = 0; i < devCounts; ++i)
			{
				std::string devInfo = devs[i].get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
				*outMessage += devInfo + " ";
			}
			*outMessage += "\n";
			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
			return false;
		}

		*outMessage = "RtabmapSlamControl: D455 connected with USB " + usbInfo + "\n";

	}
	catch (const rs2::camera_disconnected_error& e)
	{
		*outMessage += "RtabmapSlamControl: Camera was disconnected! Please connect it back ";
		*outMessage += e.what();
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);

		return false;
	}
	// continue with more general cases
	catch (const rs2::recoverable_error& e)
	{
		*outMessage += "RtabmapSlamControl: Operation failed, please try again ";
		*outMessage += e.what();
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);

		return false;
	}
	// you can also catch "anything else" raised from the library by catching rs2::error
	catch (const rs2::error& e)
	{
		*outMessage += "RtabmapSlamControl: Some other error occurred! ";
		*outMessage += e.what();
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);

		return false;
	}

	try
	{
		// librealsense API 버전 확인
		try
		{
			rs2::context ctx = rs2::context();
			*outMessage += "RtabmapSlamControl: RealSense context created successfully\n";
		}
		catch (const rs2::error& e)
		{
			*outMessage += "RtabmapSlamControl: RealSense context creation failed: ";
			*outMessage += e.what();
			*outMessage += "\n";
			*outMessage += "This may be due to API version mismatch. Please check librealsense installation.\n";
			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
			return false;
		}

		mCamera = new rtabmap::CameraRealSense2();

		// D455 카메라의 해상도를 848x480으로 고정 설정
		try
		{
			*outMessage += "RtabmapSlamControl: Setting resolution to 848x480...\n";
			mCamera->setResolution(848, 480);
			mCamera->setDepthResolution(848, 480);
			*outMessage += "RtabmapSlamControl: Resolution set to 848x480 successfully\n";
		}
		catch (const rs2::error& e)
		{
			*outMessage += "RtabmapSlamControl: Resolution setting failed: ";
			*outMessage += e.what();
			*outMessage += "\n";
			*outMessage += "RtabmapSlamControl: Continuing with default resolution...\n";
			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::WARN);
			// 해상도 설정 실패 시에도 계속 진행 (기본값 사용)
		}
		catch (const std::exception& e)
		{
			*outMessage += "RtabmapSlamControl: Resolution setting exception: ";
			*outMessage += e.what();
			*outMessage += "\n";
			*outMessage += "RtabmapSlamControl: Continuing with default resolution...\n";
			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::WARN);
			// 예외 발생 시에도 계속 진행
		}
		catch (...)
		{
			*outMessage += "RtabmapSlamControl: Unknown error during resolution setting\n";
			*outMessage += "RtabmapSlamControl: Continuing with default resolution...\n";
			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::WARN);
			// 알 수 없는 오류 시에도 계속 진행
		}

		const rtabmap::Transform test(0.000f, 0.13f, 0.0f, 0.0f, 0.0f, 0.0f);
		//mCamera->setDualMode(false, test); // t265 사용하지 않는 경우. t265 연결시 false.

		//mCamera->setOdomProvided(true, false, true);
		//mCamera->setImagesRectified(true);
	}
	catch (const rs2::error& e)
	{
		*outMessage += "RtabmapSlamControl: CameraRealSense2 creation error: ";
		*outMessage += e.what();
		*outMessage += "\n";
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
		return false;
	}
	catch (int exp)
	{
		*outMessage += "RtabmapSlamControl: CameraRealSense2 error " + std::to_string(exp);
		*outMessage += "\n";
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
		return false;
	}
	catch (const std::exception& e)
	{
		*outMessage += "RtabmapSlamControl: CameraRealSense2 exception: ";
		*outMessage += e.what();
		*outMessage += "\n";
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
		return false;
	}
	catch (...)
	{
		*outMessage += "RtabmapSlamControl: CameraRealSense2 unknown error occurred\n";
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
		return false;
	}
	
	// 카메라 초기화 시도
	try
	{
		if (!mCamera->init(".", ""))
		{
			*outMessage += "RtabmapSlamControl: Initiate failed";
			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);

			mIsInitiate = false;
			return false;
		}
		else
		{
			*outMessage += "RtabmapSlamControl: Initiate success";
			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::INFO);
			StartVideoStream();
			mIsInitiate = true;
			return true;
		}
	}
	catch (const rs2::error& e)
	{
		*outMessage += "RtabmapSlamControl: Camera init error: ";
		*outMessage += e.what();
		*outMessage += "\n";
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
		return false;
	}
	catch (const std::exception& e)
	{
		*outMessage += "RtabmapSlamControl: Camera init exception: ";
		*outMessage += e.what();
		*outMessage += "\n";
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
		return false;
	}
	catch (...)
	{
		*outMessage += "RtabmapSlamControl: Camera init unknown error occurred\n";
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
		return false;
	}
}

RtabmapSlamControl& HUREL::Compton::RtabmapSlamControl::instance()
{
	static RtabmapSlamControl* instance = new RtabmapSlamControl();
	return *instance;
}

HUREL::Compton::RtabmapSlamControl::~RtabmapSlamControl()
{
	delete mCamera;
	delete mCameraThread;
}

static std::future<void> t1;

void HUREL::Compton::RtabmapSlamControl::StartVideoStream()
{
	if (!mIsInitiate || mIsVideoStreamOn)
	{
		return;
	}
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl start video stream", eLoggerType::INFO);

	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;


	mCameraThread = new rtabmap::CameraThread(mCamera);
	mCameraThread->start();
	mIsVideoStreamOn = true;


	//auto func = std::bind(&RtabmapSlamControl::VideoStream, this);
	//t1 = std::async(std::launch::async, func);
}

void HUREL::Compton::RtabmapSlamControl::StopVideoStream()
{
	if (!mIsVideoStreamOn)
	{
		return;
	}
	mIsVideoStreamOn = false;

	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "Stop video stream", eLoggerType::INFO);

	//if (mCameraThread != nullptr)
	{
		mCameraThread->join(true);
		mCameraThread->kill();

		/*delete mCamera;
		delete mCameraThread;*/
		mCameraThread = nullptr;
	}
	//t1.get();
	//HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl stop video stream", eLoggerType::INFO);

}

static std::mutex videoStreamMutex;
static std::mutex pcMutex;
static std::mutex LMDataVideoMutex;

void HUREL::Compton::RtabmapSlamControl::IntrinsicParamters()
{
	rs2::pipeline pipeline;
	rs2::config config;
	config.enable_stream(RS2_STREAM_DEPTH);

	rs2::pipeline_profile pipeline_profile = pipeline.start(config);
	rs2::depth_sensor depth_sensor = pipeline_profile.get_device().first<rs2::depth_sensor>();
	rs2_intrinsics intrinsics = depth_sensor.get_stream_profiles()[0].as<rs2::video_stream_profile>().get_intrinsics();

	std::cout << "Width: " << intrinsics.width << std::endl;
	std::cout << "Height: " << intrinsics.height << std::endl;
	std::cout << "FX: " << intrinsics.fx << std::endl;
	std::cout << "FY: " << intrinsics.fy << std::endl;

}

cv::Mat ConvertDepthTo3DPoint(cv::Mat& depthI, float fx, float fy, float cx, float cy)
{
	//float x,y,z
	cv::Mat points, chans[3];

	static int rows = 0;
	static int cols = 0;
	static cv::Mat uMat, vMat;
	cv::Mat depth;
	depthI.convertTo(depth, CV_32FC1);

	if (rows != depth.rows || cols != depth.cols)
	{
		rows = depth.rows;
		cols = depth.cols;
		if (rows == 0 || cols == 0)
		{
			return points;
		}

		uMat = cv::Mat(depth.rows, depth.cols, CV_32FC1);
		vMat = cv::Mat(depth.rows, depth.cols, CV_32FC1);

		for (int u = 0; u < rows; ++u)
		{
			uMat.row(u).setTo(u);
		}
		for (int v = 0; v < cols; ++v)
		{
			vMat.col(v).setTo(v);
		}
	}


	cv::Mat x_over_z = (cx - uMat) / fx;
	cv::Mat y_over_z = (cy - vMat) / fy;
	cv::Mat sqrtVlaue;
	//ShowCV_32FAsJet(x_over_z, 600);
	//ShowCV_32FAsJet(y_over_z, 600);
	cv::Mat before = 1 + x_over_z.mul(x_over_z) + y_over_z.mul(y_over_z);
	cv::sqrt(before, sqrtVlaue);
	chans[2] = depth.mul(1 / sqrtVlaue);
	chans[1] = x_over_z.mul(depth);
	chans[0] = y_over_z.mul(depth);

	cv::merge(chans, 3, points);

	return points;
}

void HUREL::Compton::RtabmapSlamControl::VideoStream()
{
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "VideoStream Start", eLoggerType::INFO);

	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;


	mCameraThread = new rtabmap::CameraThread(mCamera);
	mCameraThread->start();
	mIsVideoStreamOn = true;

	while (mIsVideoStreamOn)
	{
		rtabmap::SensorData data = mCamera->takeImage();

		if (data.isValid())
		{

			auto img = data.imageRaw();
			auto imgDepth = data.depthOrRightRaw();
			//float fxValue = static_cast<float>(data.stereoCameraModels()[0].left().fx());
			//float fyValue = static_cast<float>(data.stereoCameraModels()[0].left().fy());
			//float cxValue = static_cast<float>(data.stereoCameraModels()[0].left().cx());
			//float cyValue = static_cast<float>(data.stereoCameraModels()[0].left().cy());

			//240105 : SetCurrentFrame()���� �����ϱ� ������ �Ʒ� �ּ� ó��
			//videoStreamMutex.lock();
			//if (img.cols > 0)
			//{
			//	mCurrentVideoFrame = img;
			//}
			//if (imgDepth.cols > 0)
			//{
			//	mCurrentDepthFrame = imgDepth;
			//	//cv::Mat point3 = ConvertDepthTo3DPoint(imgDepth, fxValue, fyValue, cxValue, cyValue);
			//}
			//if (mOdo != nullptr && &mOdo->getPose() != nullptr && mIsSlamPipeOn == true)
			//{

			//	//mCurrentOdometry = mOdo->getPose().toEigen4d() * t265toLACCAxisTransform;
			//	videoStreamMutex.unlock();
			//}
			//else
			//{
			//	videoStreamMutex.unlock();
			//}
			//240105 :

			//pcMutex.lock();
			//mRealtimePointCloud = *(rtabmap::util3d::cloudRGBFromSensorData(data, 4,           // image decimation before creating the clouds
			//	6.0f,        // maximum depth of the cloud
			//	0.5f));
			//pcMutex.unlock();
		}
	}

	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "VideoStream End", eLoggerType::INFO);

	//if (mCameraThread != nullptr)
	{
		mCameraThread->join(true);
		mCameraThread->kill();
		delete mCamera;
		delete mCameraThread;
		mCameraThread = nullptr;
	}
}


void HUREL::Compton::RtabmapSlamControl::ResetSlam()
{
	if (mOdo == nullptr)
	{
		return;
	}
	videoStreamMutex.lock();
	if (mOdo != nullptr)
	{
		mOdo->reset();
	}
	mOdoInit = false;
	videoStreamMutex.unlock();
}

//240106
void HUREL::Compton::RtabmapSlamControl::SetCurrentFrame()
{
	static bool isCamrerParam2 = false;

	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{
			LMDataVideoMutex.lock();
			mCurrentVideoFrame = data.imageRaw();

			cv::Mat img;

			img = data.depthRaw();

			if (isCamrerParam2 == false)
			{
				m_fxValue = static_cast<float>(data.cameraModels()[0].fx());
				m_fyValue = static_cast<float>(data.cameraModels()[0].fy());
				m_cxValue = static_cast<float>(data.cameraModels()[0].cx());
				m_cyValue = static_cast<float>(data.cameraModels()[0].cy());
				isCamrerParam2 = true;
			}

			img.convertTo(mCurrentDepthFrame, CV_32F, 0.001);

			/*mCurrentVideoFrameCopy = mCurrentVideoFrame.clone();
			mCurrentDepthFrameCopy = mCurrentDepthFrame.clone();*/
			LMDataVideoMutex.unlock();
		}
		else
		{
			mCurrentVideoFrame = cv::Mat();	//241219 sbkwon : camera disconnect set
		}
	}
}

void HUREL::Compton::RtabmapSlamControl::SetCurrentFrame1()
{
	static bool isCamrerParam2 = false;
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "SetCurrentFrame--------------", eLoggerType::ERROR_t);

	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{
			LMDataVideoMutex.lock();
			mCurrentVideoFrame = data.imageRaw();

			cv::Mat img;

			img = data.depthRaw();

			img.convertTo(mCurrentDepthFrame, CV_32F, 0.001);

			/*mCurrentVideoFrameCopy = mCurrentVideoFrame.clone();
			mCurrentDepthFrameCopy = mCurrentDepthFrame.clone();*/
			LMDataVideoMutex.unlock();
		}
	}
}

//240315
cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentVideoFrame1(bool bCopy)
{
	cv::Mat rgb;

	LMDataVideoMutex.lock();
	rgb = mCurrentVideoFrame.clone();

	//240315
	/*if (bCopy)
		mCurrentVideoFrameCopy = mCurrentVideoFrame.clone();*/

	LMDataVideoMutex.unlock();

	return rgb;
}

//240315
cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentVideoFrame2()
{
	cv::Mat rgb;

	LMDataVideoMutex.lock();
	rgb = mCurrentVideoFrameCopy.clone();
	LMDataVideoMutex.unlock();

	return rgb;
}


//240106
cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentDepthFrame1()
{
	cv::Mat depth;

	//LMDataVideoMutex.lock();
	depth = mCurrentDepthFrame.clone();
	//LMDataVideoMutex.unlock();

	return depth;
}

void HUREL::Compton::RtabmapSlamControl::SetCurrentVideoFrame()
{
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{

			mCurrentVideoFrame = data.imageRaw();
		}
		else
			mCurrentVideoFrame = cv::Mat();	//241219 sbkwon : camera disconnect set
	}
}

cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentVideoFrame()
{
	cv::Mat img;
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{

			img = data.imageRaw();
		}
	}
	return img;
}

void HUREL::Compton::RtabmapSlamControl::SetCurrentDepthFrame()
{
	static bool isCamrerParam1 = false;
	cv::Mat img;
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{

			img = data.depthRaw();

			if (isCamrerParam1 == false)
			{
				m_fxValue = static_cast<float>(data.cameraModels()[0].fx());
				m_fyValue = static_cast<float>(data.cameraModels()[0].fy());
				m_cxValue = static_cast<float>(data.cameraModels()[0].cx());
				m_cyValue = static_cast<float>(data.cameraModels()[0].cy());
				isCamrerParam1 = true;
			}

			//
			img.convertTo(mCurrentDepthFrame, CV_32F, 0.001);
		}
	}
}

cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentDepthFrame()
{
	static bool isCamrerParam = false;

	cv::Mat img;
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{
			img = data.depthRaw();

			if (isCamrerParam == false)
			{
				m_fxValue = static_cast<float>(data.cameraModels()[0].fx());
				m_fyValue = static_cast<float>(data.cameraModels()[0].fy());
				m_cxValue = static_cast<float>(data.cameraModels()[0].cx());
				m_cyValue = static_cast<float>(data.cameraModels()[0].cy());
				isCamrerParam = true;
			}

			//
			img.convertTo(img, CV_32F, 0.001);
		}
	}

	return img;
}

cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentPointsFrame(double res)
{
	cv::Mat img;
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{
			cv::Mat dImg = data.depthRaw();

			if (res < 1.0)
			{
				res = 1.0;
			}
			int newRows = dImg.rows / res;
			int newCols = dImg.cols / res;

			cv::Mat dImgSmall;
			cv::resize(dImg, dImgSmall, cv::Size(newCols, newRows));


			float fxValue = static_cast<float>(data.cameraModels()[0].fx());
			float fyValue = static_cast<float>(data.cameraModels()[0].fy());
			float cxValue = static_cast<float>(data.cameraModels()[0].cx() / res);
			float cyValue = static_cast<float>(data.cameraModels()[0].cy() / res);

			if (dImg.cols > 0)
			{
				img = ConvertDepthTo3DPoint(dImg, fxValue, fyValue, cxValue, cyValue);
			}
		}
	}
	return img;
}

void HUREL::Compton::RtabmapSlamControl::LockVideoFrame()
{
	videoStreamMutex.lock();

}

void HUREL::Compton::RtabmapSlamControl::UnlockVideoFrame()
{
	videoStreamMutex.unlock();
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetRTPointCloud()
{
	pcMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mRealtimePointCloud;
	pcMutex.unlock();

	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{
		if (isnan(tmp[i].x) || isinf(tmp[i].x))
		{
			continue;
		}
		Eigen::Vector3d color(tmp[i].r / 255.0, tmp[i].g / 255.0, tmp[i].b / 255.0);
		Eigen::Vector3d point(tmp[i].x, tmp[i].y, tmp[i].z);
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(point);
	}

	return tmpOpen3dPc.Transform(t265toLACCAxisTransform);
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetRTPointCloudTransposed()
{
	pcMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mRealtimePointCloud;
	pcMutex.unlock();
	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{
		if (isnan(tmp[i].x) || isinf(tmp[i].x))
		{
			continue;
		}
		Eigen::Vector3d color(tmp[i].b / 255.0, tmp[i].g / 255.0, tmp[i].r / 255.0);
		Eigen::Vector3d point(tmp[i].x, tmp[i].y, tmp[i].z);
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(point);
	}

	return tmpOpen3dPc.Transform(GetOdomentry() * t265toLACCAxisTransform);
}

static std::future<void> t2;

void HUREL::Compton::RtabmapSlamControl::StartSlamPipe()
{
	if (!mIsVideoStreamOn || !mIsInitiate)
	{
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "StartSlamPipe fail to start slam pipe, Initiation failed", eLoggerType::ERROR_t);
	}

	if (mIsSlamPipeOn)
	{
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "StartSlamPipe is already on", eLoggerType::INFO);

		return;
	}
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "StartSlamPipe start slam pipe", eLoggerType::INFO);
	auto func = std::bind(&RtabmapSlamControl::SlamPipe, this);
	t2 = std::async(std::launch::async, func);

}

void HUREL::Compton::RtabmapSlamControl::StopSlamPipe()
{
	if (mIsSlamPipeOn)
	{
		//250214
		pointcloudBackup = GetSlamPointCloud();

		mIsSlamPipeOn = false;

		t2.get();
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "StopSlamPipe", eLoggerType::INFO);
	}
	else {
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "StopSlamPipe is alread stop", eLoggerType::INFO);

	}


}

static std::mutex slamPipeMutex;

void HUREL::Compton::RtabmapSlamControl::SlamPipe()
{
	mOdo = rtabmap::Odometry::create();
	rtabmap::OdometryThread odomThread(mOdo);
	// Create RTAB-Map to process OdometryEvent
	rtabmap::Rtabmap* rtabmap = new rtabmap::Rtabmap();
	rtabmap->init();
	rtabmap::RtabmapThread rtabmapThread(rtabmap); // ownership is transfered
	// Setup handlers
	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();

	UEventsManager::createPipe(mCameraThread, &odomThread, "CameraEvent");
	// Let's start the threads
	rtabmapThread.start();
	odomThread.start();

	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	rtabmap::OccupancyGrid grid;
	mIsSlamPipeOn = true;
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "SlamPipe Start", eLoggerType::INFO);

	nSlamedPointCloudCount = 0;
	bool shotSave = false;
	bool shotSaveSlamedPcl = true;
	while (mIsSlamPipeOn)
	{

		std::map<int, rtabmap::Signature> nodes;
		std::map<int, rtabmap::Transform> optimizedPoses;
		std::multimap<int, rtabmap::Link> links;
		if (!mOdoInit)
		{
			videoStreamMutex.lock();
			rtabmap->resetMemory();
			mInitOdo = t265toLACCAxisTransform * mOdo->getPose().toEigen4d();
			mOdoInit = true;
			videoStreamMutex.unlock();
			continue;
		}
		std::map<int, rtabmap::Signature> tmpnodes;
		std::map<int, rtabmap::Transform> tmpOptimizedPoses;

		rtabmap->getGraph(tmpOptimizedPoses, links, true, true, &tmpnodes, true, true, true, false);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		int i = 0;
		rtabmap->getStatistics();

		nodes = tmpnodes;
		optimizedPoses = tmpOptimizedPoses;
		std::vector < Eigen::Matrix4d> tempPoses;
		tempPoses.reserve(optimizedPoses.size());

		int k = 0;
		int count = optimizedPoses.size();

		//https://cpp.hotexamples.com/examples/-/Rtabmap/-/cpp-rtabmap-class-examples.html
		for (std::map<int, rtabmap::Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
		{
			++k;
			if (nodes.count(iter->first) == 0)
			{
				continue;
			}
			rtabmap::Signature node = nodes.find(iter->first)->second;
			// uncompress data
			cv::Mat ground, obstacles, empty;   //231121-1 sbkwon : Not Use
			//node.sensorData().uncompressData(0,0,0,0,&ground,&obstacles,&empty);
			node.sensorData().uncompressData();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = rtabmap::util3d::cloudRGBFromSensorData(
				node.sensorData(),
				4,           // image decimation before creating the clouds
				4.0f,        // maximum depth of the cloud
				0.3f);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
			std::vector<int> index;
			pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);
			//grid.addToCache(iter->first, ground, obstacles, empty);
			// 
			// 
			if (!tmpNoNaN->empty())
			{
				*cloud += *rtabmap::util3d::transformPointCloud(tmpNoNaN, iter->second); // transform the point cloud to its pose
			}
			tempPoses.push_back(t265toLACCAxisTransform * iter->second.toEigen4d());
			++i;
			//pintf("iter %d \n", i);
			tmpNoNaN.reset();
			//delete test;

			if (k == count)
			{
				open3d::geometry::PointCloud tmpOpen3dPc;

				if (shotSave)
				{
					std::chrono::milliseconds timeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
					std::string fileName = std::to_string(timeInMili.count());
					rtabmap::SensorData data = mCamera->takeImage();
					//const char* home = getenv("HOME");

					std::string folderPath = "E:\\HUREL\\HUREL_Compton_20240311\\HUREL_Compton\\HUREL Imager GUI\\bin\\x64\\Release\\net6.0-windows\\scenedata\\";
					// = home + folderPath;
					//make folder 
					std::string folderFullPath = folderPath + fileName;
					//cv::imwrite(folderFullPath + "_depth.png", data.depthRaw());
					//cv::imwrite(folderFullPath + "_rgb.png", data.imageRaw());
					open3d::io::WritePointCloudOption option;
					if (shotSaveSlamedPcl)
					{
						mSlamedPointCloud = *cloud;
						tmpOpen3dPc = GetSlamPointCloud();
						open3d::io::WritePointCloudToPLY(folderFullPath + "_.ply", tmpOpen3dPc, option);
					}
				}
			}
		}
		//grid.update(optimizedPoses);

		slamPipeMutex.lock();

		mSlamedPointCloud = *cloud;
		nSlamedPointCloudCount = mSlamedPointCloud.size();
		mPoses = tempPoses;
		slamPipeMutex.unlock();

		//231121-1 sbkwon
		double gridWidth = 0; double gridHeight = 0; double minX = 0; double minZ = 0;
		float res = 0.02;
		CalOccupancySize(res, &gridWidth, &gridHeight, &minX, &minZ);
		mgridWith = gridWidth;
		mgridHeight = gridHeight;
		mminX = minX;
		mminZ = minZ;
		mOccupancyPCLGrid = createOccupancyPCL(res);
		//231121-1 sbkwon

		Sleep(0);
	}
}

//231214
//void HUREL::Compton::RtabmapSlamControl::SlamPipe()
//{
//	mOdo = rtabmap::Odometry::create();
//	rtabmap::OdometryThread odomThread(mOdo);
//	// Create RTAB-Map to process OdometryEvent
//	rtabmap::Rtabmap* rtabmap = new rtabmap::Rtabmap();
//	rtabmap->init();
//	rtabmap::RtabmapThread rtabmapThread(rtabmap); // ownership is transfered
//	// Setup handlers
//	odomThread.registerToEventsManager();
//	rtabmapThread.registerToEventsManager();
//
//	UEventsManager::createPipe(mCameraThread, &odomThread, "CameraEvent");
//	// Let's start the threads
//	rtabmapThread.start();
//	odomThread.start();
//
//	Eigen::Matrix4d t265toLACCAxisTransform;
//	t265toLACCAxisTransform << 0, 1, 0, 0,
//		0, 0, 1, 0,
//		1, 0, 0, 0,
//		0, 0, 0, 1;
//	rtabmap::OccupancyGrid grid;
//	mIsSlamPipeOn = true;
//	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "SlamPipe Start", eLoggerType::INFO);
//	
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);	//����
//	std::vector < Eigen::Matrix4d> tempPoses;
//
//	int endFirst = 0;
//
//	while (mIsSlamPipeOn)
//	{
//
//		std::map<int, rtabmap::Signature> nodes;
//		std::map<int, rtabmap::Transform> optimizedPoses;
//		std::multimap<int, rtabmap::Link> links;
//		if (!mOdoInit)
//		{
//			videoStreamMutex.lock();
//			rtabmap->resetMemory();
//			mInitOdo = t265toLACCAxisTransform * mOdo->getPose().toEigen4d();
//			mOdoInit = true;
//			videoStreamMutex.unlock();
//			continue;
//		}
//		std::map<int, rtabmap::Signature> tmpnodes;
//		std::map<int, rtabmap::Transform> tmpOptimizedPoses;
//
//		rtabmap->getGraph(tmpOptimizedPoses, links, true, true, &tmpnodes, true, true, true, false);
//
//		rtabmap->getStatistics();
//
//		nodes = tmpnodes;
//		optimizedPoses = tmpOptimizedPoses;
//		/*std::vector < Eigen::Matrix4d> tempPoses;
//		tempPoses.reserve(optimizedPoses.size());*/
//
//		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "node count : " + std::to_string(optimizedPoses.size()), eLoggerType::INFO);
//
//		bool bFind = false;
//		int firstIndex = 0;
//		//https://cpp.hotexamples.com/examples/-/Rtabmap/-/cpp-rtabmap-class-examples.html
//		for (std::map<int, rtabmap::Transform>::reverse_iterator iter = optimizedPoses.rbegin(); iter != optimizedPoses.rend(); ++iter)
//		{
//			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "all First : " + std::to_string(iter->first), eLoggerType::INFO);
//			if (iter->first == endFirst && endFirst > 0)
//				break;
//
//			if (nodes.count(iter->first) == 0)
//			{
//				continue;
//			}
//			rtabmap::Signature node = nodes.find(iter->first)->second;
//			// uncompress data
//			//cv::Mat ground, obstacles, empty;	//231121-1 sbkwon : Not Use
//			//node.sensorData().uncompressData(0,0,0,0,&ground,&obstacles,&empty);
//			node.sensorData().uncompressData();
//
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = rtabmap::util3d::cloudRGBFromSensorData(
//				node.sensorData(),
//				4,           // image decimation before creating the clouds
//				4.0f,        // maximum depth of the cloud
//				0.3f);
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
//			std::vector<int> index;
//			pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);
//			//grid.addToCache(iter->first, ground, obstacles, empty);
//			// 
//			// 
//			if (!tmpNoNaN->empty())
//			{
//				if (bFind == false)	//�ڿ��� ù��° ����� first�� ����
//				{
//					bFind = true;
//					firstIndex = iter->first;
//				}
//				*cloud += *rtabmap::util3d::transformPointCloud(tmpNoNaN, iter->second); // transform the point cloud to its pose
//				
//				HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "Node : " + std::to_string(iter->first) + "," + std::to_string(endFirst), eLoggerType::INFO);
//			}
//			tempPoses.push_back(t265toLACCAxisTransform * iter->second.toEigen4d());
//			//pintf("iter %d \n", i);
//			tmpNoNaN.reset();
//			//delete test;
//		}
//		//grid.update(optimizedPoses);
//
//		if(bFind)
//			endFirst = firstIndex;
//
//		slamPipeMutex.lock();
//
//		mSlamedPointCloud = *cloud;
//		mPoses = tempPoses;
//		slamPipeMutex.unlock();
//		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "mSlamedPointCloud : " + std::to_string(mSlamedPointCloud.size()), eLoggerType::INFO);
//		//231121-1 sbkwon
//		double gridWidth = 0; double gridHeight = 0; double minX = 0; double minZ = 0;
//		float res = 0.05;
//		CalOccupancySize(res, &gridWidth, &gridHeight, &minX, &minZ);
//		mgridWith = gridWidth;
//		mgridHeight = gridHeight;
//		mminX = minX;
//		mminZ = minZ;
//		mOccupancyPCLGrid = createOccupancyPCL(res);
//		//231121-1 sbkwon
//
//
//		Sleep(0);
//	}
//
//	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "SlamPipe End", eLoggerType::INFO);
//
//	mOdoInit = false;
//	//mCurrentOdometry = Eigen::Matrix4d::Identity();
//	mInitOdo = Eigen::Matrix4d::Identity();
//
//	rtabmapThread.unregisterFromEventsManager();
//	odomThread.unregisterFromEventsManager();
//
//	odomThread.join(true);
//	rtabmapThread.join(true);
//	odomThread.kill();
//	rtabmapThread.kill();
//}

//231121-1 sbkwon
void HUREL::Compton::RtabmapSlamControl::CalOccupancySize(float res, double* outWidth, double* outHeight, double* outMinX, double* outMinZ)
{
	slamPipeMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mSlamedPointCloud;
	slamPipeMutex.unlock();
	const double Y_threshold = 0.5;
	double minX = 0; double maxX = 0; double minZ = 0; double maxZ = 0;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	open3d::geometry::PointCloud tmpOpen3dPc;
	for (const auto& point : tmp.points)
	{
		Eigen::Vector4d transbeforepoint(point.x, point.y, point.z, 1);
		Eigen::Vector4d transFormedpoint = t265toLACCAxisTransform * transbeforepoint;
		if (transFormedpoint.y() < Y_threshold && transFormedpoint.y() > -0.1)
		{
			minX = std::min(minX, transFormedpoint.x());
			maxX = std::max(maxX, transFormedpoint.x());
			minZ = std::min(minZ, transFormedpoint.z());
			maxZ = std::max(maxZ, transFormedpoint.z());
			Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
			tmpOpen3dPc.points_.push_back(inputpoint);
		}
	}

	int GRID_WIDTH = *outWidth = static_cast<int>(maxX - minX) / res;
	int GRID_HEIGHT = *outHeight = static_cast<int>(maxZ - minZ) / res;
	/*
	double X_range; double Z_range;
	if (abs(minX) < abs(maxX))
	{
		X_range = abs(minX);
	}
	else
	{
		X_range = abs(maxX);
	}
	if (abs(minZ) < abs(maxZ))
	{
		Z_range = abs(minZ);
	}
	else
	{
		Z_range = abs(maxZ);
	}

	*outMinX = X_range;
	*outMinZ = Z_range;
	*/
	*outMinX = minX;
	*outMinZ = minZ;
}

//231121-1 sbkwon
open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::createOccupancyPCL(float res)
{
	slamPipeMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mSlamedPointCloud;
	slamPipeMutex.unlock();

	const double Y_threshold = 0.3;

	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;

	open3d::geometry::PointCloud tmpOpen3dPc;
	for (const auto& point : tmp.points)
	{
		Eigen::Vector4d transbeforepoint(point.x, point.y, point.z, 1);
		Eigen::Vector4d transFormedpoint = t265toLACCAxisTransform * transbeforepoint;
		if (transFormedpoint.y() < Y_threshold && transFormedpoint.y() > -0.1)
		{
			Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
			tmpOpen3dPc.points_.push_back(inputpoint);
		}
	}

	cv::Mat grid(mgridHeight, mgridWith, CV_8UC3, cv::Scalar(255, 255, 255));

	open3d::geometry::PointCloud gridCloud;
	Eigen::Vector3d point;
	Eigen::Vector3d color;

	double absminX = abs(mminX);
	double absminZ = abs(mminZ);

	//double resMulti = 1 / res;

	for (size_t k = 0; k < tmpOpen3dPc.points_.size(); ++k)
	{
		if (tmpOpen3dPc.points_[k].y() < Y_threshold && tmpOpen3dPc.points_[k].y() > -0.2)
		{
			int x = static_cast<int>((tmpOpen3dPc.points_[k].x() + abs(mminX)) / res);	//static_cast<int>((tmpOpen3dPc.points_[k].x() + abs(mminX)) / res);
			int z = static_cast<int>((tmpOpen3dPc.points_[k].z() + abs(mminZ)) / res);	//static_cast<int>((tmpOpen3dPc.points_[k].z() + abs(mminZ)) / res);
			
			//int x = static_cast<int>(floor((tmpOpen3dPc.points_[k].x() - mminX) / res)+1);
			//int z = static_cast<int>(floor((tmpOpen3dPc.points_[k].x() - mminZ) / res)+1);
			
			if (x < 0 || x >= mgridWith || z < 0 || z >= mgridHeight)
			{
				continue;
			}
			if (grid.at<cv::Vec3b>(z, x) == cv::Vec3b(0, 0, 0))
			{
				continue;
			}			
			else
			{
				grid.at<cv::Vec3b>(z, x) = cv::Vec3b(0, 0, 0);
				point(0) = tmpOpen3dPc.points_[k].x();
				point(1) = 0.3;
				point(2) = tmpOpen3dPc.points_[k].z();
				color(0) = 0;
				color(1) = 0;
				color(2) = 0;
				gridCloud.points_.push_back(point);
				gridCloud.colors_.push_back(color);
			}
		}
	}
	return gridCloud;
}

//231121-1 sbkwon
open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetOccupancyPointCloud()
{
	//Logger::Instance().InvokeLog("RtabmapSlamControl", std::to_string(mOccupancyPCLGrid.colors_.size()) + " : " + std::to_string(mOccupancyPCLGrid.points_.size()), eLoggerType::INFO);
	return mOccupancyPCLGrid;
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetSlamPointCloud()
{
	if (!mIsSlamPipeOn)
	{
		return open3d::geometry::PointCloud();
	}
	slamPipeMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mSlamedPointCloud;
	slamPipeMutex.unlock();

	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{
		Eigen::Vector3d color(tmp[i].r / 255.0, tmp[i].g / 255.0, tmp[i].b / 255.0);
		Eigen::Vector4d point(tmp[i].x, tmp[i].y, tmp[i].z, 1);
		Eigen::Vector4d transFormedpoint = t265toLACCAxisTransform * point;

		Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(inputpoint);
	}

	open3d::geometry::PointCloud returnPC = *tmpOpen3dPc.VoxelDownSample(0.02);

	//std::chrono::milliseconds timeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	//std::string fileName = std::to_string(timeInMili.count());
	

	std::chrono::milliseconds timeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	std::string fileName = std::to_string(timeInMili.count());
	//cv::imwrite(fileName + "_depth.png", RtabmapSlamControl::instance().GetCurrentDepthFrame());
	//cv::imwrite(fileName + "_rgb.png", RtabmapSlamControl::instance().GetCurrentVideoFrame());
	//open3d::io::WritePointCloudOption option;
	//open3d::io::WritePointCloudToPLY(fileName + ".ply", returnPC, option);


	return returnPC;
}

std::vector<double> HUREL::Compton::RtabmapSlamControl::getMatrix3DOneLineFromPoseData()
{
	Eigen::Matrix4d m = GetOdomentry();
	auto Matrix3Dtype = m.adjoint();
	std::vector<double> matrix3DOneLine;
	int idx = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			matrix3DOneLine.push_back(Matrix3Dtype(i, j));
			idx++;
		}
	}
	return matrix3DOneLine;
}

bool HUREL::Compton::RtabmapSlamControl::LoadPlyFile(std::string filePath)
{
	open3d::io::ReadPointCloudOption opt;
	return open3d::io::ReadPointCloudFromPLY(filePath, mLoadedPointcloud, opt);
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetLoadedPointCloud()
{
	return mLoadedPointcloud;
}

Eigen::Matrix4d HUREL::Compton::RtabmapSlamControl::GetOdomentry()
{
	Eigen::Matrix4d odo = Eigen::Matrix4d::Identity();;

	//videoStreamMutex.lock();
	if (mOdo != nullptr && &mOdo->getPose() != nullptr && mIsSlamPipeOn == true)
	{
		odo = mOdo->getPose().toEigen4d();
	}
	//videoStreamMutex.unlock();
	return odo;
}

std::vector<Eigen::Matrix4d> HUREL::Compton::RtabmapSlamControl::GetOptimizedPoses()
{

	slamPipeMutex.lock();
	std::vector<Eigen::Matrix4d> tempPoses = mPoses;
	slamPipeMutex.unlock();
	return tempPoses;
}
//231025-1 sbkwon
pcl::PointCloud<pcl::PointXYZRGB>::Ptr HUREL::Compton::RtabmapSlamControl::generatePointCloud()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cv::Mat depth;
	cv::Mat rgb;

	float fxValue = 1.0;
	float fyValue = 1.0;
	float cxValue = 0.0;
	float cyValue = 0.0;

	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{
			rgb = data.imageRaw();
			depth = data.depthRaw();
			cv::imshow("depth img", rgb);


			fxValue = static_cast<float>(data.cameraModels()[0].fx());
			fyValue = static_cast<float>(data.cameraModels()[0].fy());
			cxValue = static_cast<float>(data.cameraModels()[0].cx());
			cyValue = static_cast<float>(data.cameraModels()[0].cy());

			for (int row = 0; row < depth.rows; row++)
			{
				for (int col = 0; col < depth.cols; col++)
				{
					// Get 3D coordinates
					float d = depth.at<float>(row, col);
					if (d < 0)
					{
						//continue;
						d = 0;
					}

					pcl::PointXYZRGB point;
					point.z = d;
					float X = (col - cxValue) * point.z / fxValue;
					float Y = (row - cyValue) * point.z / fyValue;
					point.x = (-1) * X;
					point.y = (-1) * Y;
					cv::Vec3b color = rgb.at<cv::Vec3b>(row, col);
					point.b = color[0];
					point.g = color[1];
					point.r = color[2];

					cloud->points.push_back(point);
				}
			}

			cloud->height = 1;
			cloud->width = static_cast<uint32_t>(cloud->points.size());
			cloud->is_dense = false;
		}
	}	

	return cloud;
}

//231031-1 sbkwon
pcl::PointCloud<pcl::PointXYZRGB>::Ptr HUREL::Compton::RtabmapSlamControl::generatePointCloud(cv::Mat& depth, cv::Mat& rgb)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//240711 camera ���� �ʱ�ȭ
	if (m_fxValue == 0 || m_fyValue)
		GetCurrentDepthFrame();
			
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::RtabmapSlamControl", "cx, cy, fx, fy : " 
				+ std::to_string(m_cxValue) + " : "
				+ std::to_string(m_cyValue) + " : "
				+ std::to_string(m_fxValue) + " : "
				+ std::to_string(m_fyValue)
				, eLoggerType::INFO);

	for (int m = 0; m < depth.rows; m++)//480 
	{
		for (int n = 0; n < depth.cols; n++)//848
		{
			// Get 3D coordinates
			float d = depth.at<float>(m, n);
			if (d < 0)
			{
				//continue;
				d = 0;
			}

			pcl::PointXYZRGB point;
			point.z = d;
			float X = (n - m_cxValue) * point.z / m_fxValue;
			float Y = (m - m_cyValue) * point.z / m_fyValue;
			point.x = (-1) * X;
			point.y = (-1) * Y;
			cv::Vec3b color = rgb.at<cv::Vec3b>(m, n);
			point.b = color[0];
			point.g = color[1];
			point.r = color[2];

			cloud->points.push_back(point);


			////test
			//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::RtabmapSlamControl", "X, Y, Z, B, G, R : " 
			//	+ std::to_string(point.x) + " : "
			//	+ std::to_string(point.y) + " : "
			//	+ std::to_string(point.z) + " : "
			//	+ std::to_string(point.b) + " : "
			//	+ std::to_string(point.g) + " : "
			//	+ std::to_string(point.r) + " : "
			//	+ std::to_string(d) + " : "
			//	, eLoggerType::INFO);
			////
		}
	}

	cloud->height = 1;
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->is_dense = false;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr HUREL::Compton::RtabmapSlamControl::generatePointClouddowin(cv::Mat& depth, cv::Mat& rgb, int down)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//240711 camera ���� �ʱ�ȭ
	if (m_fxValue == 0 || m_fyValue)
		GetCurrentDepthFrame();

	for (int m = 0; m < depth.rows; m++)//480 
	{
		if(m % down == 0)
		{
			for (int n = 0; n < depth.cols; n++)//848
			{
				if (n % down == 0)
				{
					// Get 3D coordinates
					float d = depth.at<float>(m, n);
					if (d < 0)
					{
						//continue;
						d = 0;
					}

					pcl::PointXYZRGB point;
					point.z = d;
					float X = (n - m_cxValue) * point.z / m_fxValue;
					float Y = (m - m_cyValue) * point.z / m_fyValue;
					point.x = (-1) * X;
					point.y = (-1) * Y;
					cv::Vec3b color = rgb.at<cv::Vec3b>(m, n);
					point.b = color[0];
					point.g = color[1];
					point.r = color[2];

					cloud->points.push_back(point);
				}
			}
		}
	}

	cloud->height = 1;
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->is_dense = false;

	return cloud;
}

//231110
pcl::PointCloud<pcl::PointXYZRGB>::Ptr HUREL::Compton::RtabmapSlamControl::downsamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& inputCloud, float voxelSize, pcl::PointIndices& indicesPtr)
{
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(inputCloud);
	sor.setLeafSize(voxelSize, voxelSize, voxelSize);
	//sor.setSaveLeafLayout(true);

	// To get the indices of the points selected by the voxel grid
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	sor.filter(*outputCloud);
	sor.getRemovedIndices(indicesPtr);
	//sor.setIndices(indicesPtr);
	//indicesPtr = sor.getIndices();
	
	// The outputCloud is now downsampled
	return outputCloud;
}

//231025-1 sbkwon
open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::PclToOpen3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud = *tmp;

	/*Eigen::Matrix4d t265toLACCPosTranslate;
	Eigen::Matrix4d t265toLACCPosTransformInv;
	t265toLACCPosTranslate << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		0, 1, 0, 1,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;
	t265toLACCPosTransformInv = t265toLACCPosTranslate.inverse();*/

	open3d::geometry::PointCloud tmpOpen3dPc;
	for (auto cloud_it = cloud->begin(); cloud_it != cloud->end(); ++cloud_it)
	{
		Eigen::Vector3d color((cloud_it->r >> 8), (cloud_it->g >> 8), (cloud_it->b >> 8));
		Eigen::Vector4d point(cloud_it->x, cloud_it->y, cloud_it->z, 1);
		Eigen::Vector3d inputpoint(point.x(), point.y(), point.z());

		/*Eigen::Vector4d transFormedpoint = t265toLACCPosTransformInv * point;
		Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());*/

		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(inputpoint);
	}

	//	tmpOpen3dPc.Transform(t265toLACCPosTransformInv);

	//	open3d::geometry::PointCloud returnPC = *tmpOpen3dPc.VoxelDownSample(0.1);
	open3d::geometry::PointCloud returnPC = tmpOpen3dPc;

	return returnPC;
}

//231106-2 sbkwon
open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::RTPointCloudTransposed(open3d::geometry::PointCloud& initialPC, Eigen::Matrix4d transMatrix)
{
	open3d::geometry::PointCloud totalPCtrans;
	totalPCtrans = initialPC;
	totalPCtrans.Transform(transMatrix);
	Eigen::Matrix4d t265toLACCPosTranslate;
	Eigen::Matrix4d t265toLACCPosTransformInv;

	t265toLACCPosTranslate << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;
	t265toLACCPosTransformInv = t265toLACCPosTranslate.inverse();

	return totalPCtrans.Transform(t265toLACCPosTransformInv);
}

std::tuple<double, double, double> HUREL::Compton::RtabmapSlamControl::GetOdomentryPos()
{
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;

	Eigen::Matrix4d deviceTransformation = t265toLACCAxisTransform * GetOdomentry();

	double x = deviceTransformation(0, 3);
	double y = deviceTransformation(1, 3);
	double z = deviceTransformation(2, 3);
	
	return std::make_tuple(x, y, z);
}
