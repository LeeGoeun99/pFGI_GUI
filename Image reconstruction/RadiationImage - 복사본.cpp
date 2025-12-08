#include "RadiationImage.h"

using namespace Eigen;

// Makting Detector Response Image
constexpr double Det_W = 0.312;
constexpr double Mask_W = 0.370;
constexpr double Mpix = 37;
constexpr double S2M = 1;
constexpr double M2D = 0.07;
constexpr double SP = S2M - M2D;// Source to Mask distance(mm)
constexpr double M = 1 + M2D / S2M; // projection ratio((a + b) / a)
constexpr double Dproj = Det_W / (Mask_W / Mpix * M); // projection mask to Detector pixel Length(mm)
constexpr double ReconPlaneWidth = S2M / M2D * Det_W;
constexpr double ResImprov = 5;
int PixelCount = static_cast<int>(round(Dproj * ResImprov));

inline int findIndex(double value, double min, double pixelSize)
{
	if (value - min <= 0)
	{
		return -1;
	}
	return static_cast<int>(floor((value - min) / pixelSize));
}

static cv::Mat CodedMaskMat()
{
	static bool isMaskMade = false;
	static cv::Mat mask;
	if (isMaskMade)
	{
		return mask;
	}
	else
	{
		mask = cv::Mat(37, 37, CV_32S);
		for (int i = 0; i < 37; ++i)
		{
			for (int j = 0; j < 37; ++j)
			{
				if (HUREL::Compton::mCodeMask[i][j])
				{
					//mask.at<int>(j, 36 - i) = -1;
					mask.at<int>(i, j) = 1;
				}
				else
				{
					//mask.at<int>(j, 36 - i) = 1;
					mask.at<int>(i, j) = -1;
				}
			}
		}
		isMaskMade = true;
		return mask;
	}
}

void HUREL::Compton::RadiationImage::ShowCV_32SAsJet(cv::Mat img, int size)
{
	if (img.type() != CV_32S)
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
			normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue)
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


cv::Mat HUREL::Compton::RadiationImage::GetCV_32SAsJet(cv::Mat img, int size)
{
	cv::Mat showImg;
	if (img.type() != CV_32S)
	{
		return showImg;
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, &minValue, &maxValue);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue) / (maxValue - minValue) * 255);
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::cvtColor(colorImg, colorImg, COLOR_BGR2BGRA);

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

	return showImg;
}

cv::Mat HUREL::Compton::RadiationImage::GetCV_32SAsJet(cv::Mat img, int size, double minValuePortion)
{
	cv::Mat showImg;
	if (img.type() != CV_32S)
	{
		return showImg;
	}
	if (img.empty())
	//if (img.rows == 0)
	{
		return cv::Mat();
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, nullptr, &maxValue);
	minValue = maxValue * minValuePortion;
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			if (img.at<int>(i, j) < minValue)//231023 sbkwon : 중복 연산 제거
			{
				normImg.at<uchar>(i, j) = 0;
			}
			else
			{
				normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue) / (maxValue - minValue) * 255);
			}
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::cvtColor(colorImg, colorImg, COLOR_BGR2BGRA);
	for (int i = 0; i < colorImg.rows; i++)
	{
		for (int j = 0; j < colorImg.cols; j++)
		{
			auto& pixel = colorImg.at<cv::Vec4b>(i, j);
			if (pixel[0] == 128 && pixel[1] == 0 && pixel[2] == 0)
			{
				pixel[3] = 0;
			}
		}
	}
	int sizeHeight = size;
	int sizeWidth = size;

	//231023 sbkwon : 기존 - size 변경
	if (colorImg.size().height > colorImg.size().width)
	{
		sizeWidth = size * colorImg.size().width / colorImg.size().height;
	}
	else
	{
		sizeHeight = size * colorImg.size().height / colorImg.size().width;
	}

	cv::resize(colorImg, showImg, cv::Size(sizeWidth, sizeHeight), 0, 0, cv::INTER_NEAREST_EXACT);

	return showImg;

	//231023 sbkwon : 수정 - size 변경없이 그대로 전달
	//return colorImg;
}

//231025-1 sbkwon
cv::Mat HUREL::Compton::RadiationImage::GetCV_32SAsJet(cv::Mat img, double minValuePortion)
{
	cv::Mat showImg;
	if (img.type() != CV_32S)
	{
		return showImg;
	}
	if (img.rows == 0)
	{
		return cv::Mat();
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, nullptr, &maxValue);
	minValue = maxValue * minValuePortion;
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			if (img.at<int>(i, j) < minValue)//231023 sbkwon : 중복 연산 제거
			{
				normImg.at<uchar>(i, j) = 0;
			}
			else
			{
				normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue) / (maxValue - minValue) * 255);
			}
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::cvtColor(colorImg, colorImg, COLOR_BGR2BGRA);
	for (int i = 0; i < colorImg.rows; i++)
	{
		for (int j = 0; j < colorImg.cols; j++)
		{
			auto& pixel = colorImg.at<cv::Vec4b>(i, j);
			if (pixel[0] == 128 && pixel[1] == 0 && pixel[2] == 0)
			{
				pixel[3] = 0;
			}
		}
	}

	return showImg;
}

HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData>& data)
{
	Mat responseImg(PixelCount, PixelCount, CV_32S, Scalar(0));
	Mat comptonImg(PixelCount, PixelCount, CV_32S, Scalar(1));
	__int32* responseImgPtr = static_cast<__int32*>(static_cast<void*>(responseImg.data));
	__int32* comptonImgPtr = static_cast<__int32*>(static_cast<void*>(comptonImg.data));
	int codedImageCount = 0;
	int comptonImageCount = 0;

#pragma omp parallel for
	for (int i = 0; i < data.size(); ++i)
	{

		ListModeData& lm = data[i];
		//if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 600 || lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy > 720)
		//{
		//	continue;
		//}
		/*if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 1000 || lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy > 1500)
		{
			continue;
		}*/

		if (lm.Type == eInterationType::CODED)
		{
			//continue;
			double& interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
			double& interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];

			int iX = findIndex(interactionPoseX, -Det_W / 2, Det_W / PixelCount);
			int iY = findIndex(interactionPoseY, -Det_W / 2, Det_W / PixelCount);
			if (iX >= 0 && iY >= 0 && iX < PixelCount && iY < PixelCount)
			{
				++responseImgPtr[PixelCount * iY + iX];
				++codedImageCount;
			}
		}




		if (lm.Type == eInterationType::COMPTON)
		{
			if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 200)
			{
				continue;
			}
			++comptonImageCount;
			for (int i = 0; i < PixelCount; ++i)
			{
				for (int j = 0; j < PixelCount; ++j)
				{
					double imagePlaneX = ReconPlaneWidth / PixelCount * i + ReconPlaneWidth / PixelCount * 0.5 - ReconPlaneWidth / 2;
					double imagePlaneY = ReconPlaneWidth / PixelCount * j + ReconPlaneWidth / PixelCount * 0.5 - ReconPlaneWidth / 2;
					double imagePlaneZ = S2M + M2D + 0.02;
					Eigen::Vector3d imgPoint;
					imgPoint[0] = imagePlaneX;
					imgPoint[1] = imagePlaneY;
					imgPoint[2] = imagePlaneZ;
					comptonImgPtr[PixelCount * (PixelCount - j - 1) + PixelCount - i - 1] += ReconPointCloud::SimpleComptonBackprojectionUntransformed(lm, imgPoint);
				}
			}

		}
	}
	//std::cout << "Lm Count: " << data.size() << " Coded count: " << codedImageCount << " Compton count: " << comptonImageCount << std::endl;
	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(37 * ResImprov, 37 * ResImprov), 0, 0, INTER_NEAREST_EXACT);
	Mat reconImg;
	//cv::filter2D(responseImg, reconImg, CV_32S, scaleG);	//231017 sbkwon : 기존
	cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);//231017 sbkwon : 수정

	double maxValue;
	cv::minMaxLoc(reconImg, nullptr, &maxValue);
	Mat idxImg(PixelCount, PixelCount, CV_32S, Scalar(1));

	cv::max(reconImg, idxImg, mCodedImage);

	//mCodedImage = reconImg;

	mDetectorResponseImage = responseImg;
	mComptonImage = comptonImg;

	mHybridImage = mCodedImage.mul(mComptonImage);


	if (data.size() == 0)
	{
		return;
	}

	mDetectorTransformation = data[0].DetectorTransformation;
	mListedListModeData = data;

	//ShowCV_32SAsJet(mDetectorResponseImage, 1000);
	//ShowCV_32SAsJet(mCodedImage, 1000);
	//ShowCV_32SAsJet(mComptonImage, 1000);
	//ShowCV_32SAsJet(mHybridImage, 1000);
}

//231020 sbkwon : 기존
//HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov)
//{
//	double m = 1 + m2D / s2M;
//	double reconPlaneWidth = s2M / m2D * det_W;
//	double dproj = det_W / (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
//	int pixelCount = static_cast<int>(round(dproj * resImprov));
//
//
//
//	Mat responseImg(pixelCount, pixelCount, CV_32S, Scalar(0));
//	Mat comptonImg(pixelCount, pixelCount, CV_32S, Scalar(1));
//	__int32* responseImgPtr = static_cast<__int32*>(static_cast<void*>(responseImg.data));
//	__int32* comptonImgPtr = static_cast<__int32*>(static_cast<void*>(comptonImg.data));
//	int codedImageCount = 0;
//	int comptonImageCount = 0;
//
//	//231013 sbkwon : 중복 연산 제거
//	double det_w_div2 = -det_W / 2;
//	double pixelSize = det_W / pixelCount;
//
//#pragma omp parallel for
//	for (int i = 0; i < data.size(); ++i)
//	{
//		ListModeData& lm = data[i];
//		if (lm.Type == eInterationType::CODED)
//		{
//			double& interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
//			double& interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];
//
//			int iX = findIndex(interactionPoseX, det_w_div2, pixelSize);	//231013 sbkwon : 중복 연산 제거, 기존 : int iX = findIndex(interactionPoseX, -det_W / 2, det_W / pixelCount);
//			int iY = findIndex(interactionPoseY, det_w_div2, pixelSize);	//231013 sbkwon : 중복 연산 제거, 기존 : int iY = findIndex(interactionPoseY, -det_W / 2, det_W / pixelCount);
//			if (iX >= 0 && iY >= 0 && iX < pixelCount && iY < pixelCount)
//			{
//				++responseImgPtr[pixelCount * iY + iX];
//				++codedImageCount;
//			}
//		}
//		else if (lm.Type == eInterationType::COMPTON)	//230907 sbkwon
//		{
//			if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 400)
//			{
//				continue;
//			}
//			if (lm.Scatter.InteractionEnergy < 10)
//			{
//				continue;
//			}
//			++comptonImageCount;
//			double comptonScatterAngle = nan("");
//			double sigmacomptonScatteringAngle = nan("");
//			Eigen::Vector3d sToAVector;
//			double imagePlaneZ = s2M + 0.07;
//
//			//231013 sbkwon : 중복 연산 제거
//			double reconPlaneWidth_dev_pixelCount = reconPlaneWidth / pixelCount;
//			double reconPlaneWidth_dev_2 = reconPlaneWidth / 2;
//			int nPixelIndex = pixelCount - 1;
//
//			for (int i = 0; i < pixelCount; ++i)
//			{
//				double imagePlaneX = reconPlaneWidth_dev_pixelCount * i + reconPlaneWidth_dev_pixelCount * 0.5 - reconPlaneWidth_dev_2;	//231013 sbkwon : 중복 연산 제거, 기존 : double imagePlaneX = reconPlaneWidth / pixelCount * i + reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;
//				
//				for (int j = 0; j < pixelCount; ++j)
//				{
//					double imagePlaneY = reconPlaneWidth_dev_pixelCount * j + reconPlaneWidth_dev_pixelCount * 0.5 - reconPlaneWidth_dev_2;	//231013 sbkwon : 중복 연산 제거, 기존 : double imagePlaneY = reconPlaneWidth / pixelCount * j + reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;
//					
//					Eigen::Vector3d imgPoint;
//					imgPoint[0] = imagePlaneX;
//					imgPoint[1] = imagePlaneY;
//					imgPoint[2] = imagePlaneZ;
//					comptonImgPtr[pixelCount * (nPixelIndex - j) + nPixelIndex - i] += ReconPointCloud::SimpleComptonBackprojectionUntransformed(lm, imgPoint, &comptonScatterAngle, &sigmacomptonScatteringAngle, &sToAVector);	//231013 sbkwon : 중복 연산 제거,
//				}
//			}
//		}
//	}
//	//std::cout << "Lm Count: " << data.size() << " Coded count: " << codedImageCount << " Compton count: " << comptonImageCount << std::endl;
//	Mat scaleG;
//	cv::resize(CodedMaskMat(), scaleG, Size(37 * resImprov, 37 * resImprov), 0, 0, INTER_NEAREST_EXACT);
//	Mat reconImg;
//	//cv::filter2D(responseImg, reconImg, CV_32S, scaleG);//231017 sbkwon : 기존
//	cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);//231017 sbkwon : 수정
//
//	//reconImg = -reconImg;
//	//double maxValue;
//	//cv::minMaxLoc(reconImg, nullptr, &maxValue);
//	//Mat idxImg(pixelCount, pixelCount, CV_32S, Scalar(maxValue * 0.01));
//	//mCodedImage = reconImg;
//	//cv::max(reconImg, idxImg, mCodedImage);
//
//
//	double fovHeight = 2 * tan((hFov / 2) * M_PI / 180.0) * (s2M + m2D + 0.02);
//	double fovWidth = 2 * tan((wFov / 2) * M_PI / 180.0) * (s2M + m2D + 0.02);
//
//	//height correction
//	constexpr double heightDiff = 0.28;
//
//	double heightPixelSize = reconPlaneWidth / pixelCount;
//	int offSetPixelCount = heightDiff / heightPixelSize;
//
//	int heightPixelCount = pixelCount * (fovHeight / reconPlaneWidth);
//	int widthPixelCount = pixelCount * (fovWidth / reconPlaneWidth);
//
//	int minHeightPixleCount = (pixelCount - heightPixelCount) / 2 + offSetPixelCount;
//	int maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 + offSetPixelCount;
//
//	if (minHeightPixleCount < 0)
//	{
//		minHeightPixleCount = 0;
//	}
//	if (maxHeightPixleCount > pixelCount)
//	{
//		maxHeightPixleCount = pixelCount;
//	}
//	if (widthPixelCount > pixelCount)
//	{
//		widthPixelCount = pixelCount;
//	}
//
//	mDetectorResponseImage = responseImg;
//	cv::Mat nonFiltered = reconImg;
//	cv::Mat Filtered;
//
//	nonFiltered.convertTo(nonFiltered, CV_32F);
//	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
//	Filtered.convertTo(Filtered, CV_32S);
//	reconImg = Filtered;
//
//	mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
//	
//
//	nonFiltered.convertTo(nonFiltered, CV_32F);
//	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
//	Filtered.convertTo(Filtered, CV_32S);
//	reconImg = Filtered;
//
//	nonFiltered = comptonImg;
//	Filtered;
//
//	nonFiltered.convertTo(nonFiltered, CV_32F);
//	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
//	Filtered.convertTo(Filtered, CV_32S);
//	comptonImg = Filtered;
//
//	mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
//	nonFiltered = mCodedImage.mul(mComptonImage);
//	Filtered;
//	nonFiltered.convertTo(nonFiltered, CV_32F);
//	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 1.5);
//	Filtered.convertTo(Filtered, CV_32S);
//
//
//	double minVal;
//	double maxVal;
//	Point minLoc;
//	Point maxLoc;
//
//	minMaxLoc(Filtered, &minVal, &maxVal, &minLoc, &maxLoc);
//	if (maxVal > 400)
//	{
//	}
//
//	mHybridImage = Filtered;
//
//	Logger::Instance().InvokeLog("RadImage", "Hybrid Max: " + std::to_string(maxVal), eLoggerType::INFO);
//	if (data.size() == 0)
//	{
//		return;
//	}
//
//	mDetectorTransformation = data[0].DetectorTransformation;
//	mListedListModeData = data;
//}

//231020 sbkwon : 수정, Detector FOV : 130도
HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov)
{
	//사용여부 확인, 영상 확인 필요.
	if (data.size() == 0)
		return;

	double m = 1 + m2D / s2M;	//s2m 비율
	double reconPlaneWidth = s2M / m2D * det_W;	//source 위치에서의 영상 사이즈
	//double reconPlaneWidth2 = 2 * tan(65 * M_PI / 180.0) * (s2M + m2D);	//fov 130으로 recon plane width 계산
	double dproj = det_W / (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
	int pixelCount = static_cast<int>(round(dproj * resImprov));	//compton size
	int pixelCountCode = static_cast<int>(round(dproj));;	//Detector Pixel size 대입

	Mat responseImg(pixelCountCode, pixelCountCode, CV_32S, Scalar(0));
	Mat comptonImg(pixelCount, pixelCount, CV_32S, Scalar(1));
	__int32* responseImgPtr = static_cast<__int32*>(static_cast<void*>(responseImg.data));
	__int32* comptonImgPtr = static_cast<__int32*>(static_cast<void*>(comptonImg.data));

	double det_w_div2 = -det_W / 2;
	double pixelSize = det_W / pixelCountCode;

	//중복 연산 제거
	double preCalc1 = reconPlaneWidth / pixelCount;	//영상화 영역 한 픽셀 길이
	double preCalc2 = reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;	//영상화 영역의 반 픽셀 길이 - 중심을 (0,0)으로 만들기 위해

	double imagePlaneZ = s2M + M2D;

#pragma omp parallel for
	for (int i = 0; i < data.size(); ++i)
	{
		ListModeData& lm = data[i];
		if (lm.Type == eInterationType::CODED)
		{
			double& interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
			double& interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];

			int iX = findIndex(interactionPoseX, det_w_div2, pixelSize);
			int iY = findIndex(interactionPoseY, det_w_div2, pixelSize);
			if (iX >= 0 && iY >= 0 && iX < pixelCountCode && iY < pixelCountCode)
			{
				++responseImgPtr[pixelCountCode * iY + iX];
			}
		}
		else if (lm.Type == eInterationType::COMPTON)	//230907 sbkwon
		{
			if ((lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy) < 400 || lm.Scatter.InteractionEnergy < 50)
				continue;

			for (int i = 0; i < pixelCount; ++i)
			{
				double imagePlaneX = preCalc1 * i + preCalc2;

				for (int j = 0; j < pixelCount; ++j)
				{
					double imagePlaneY = preCalc1 * j + preCalc2;

					Eigen::Vector3d imgPoint;
					//imgPoint[0] = imagePlaneX - T265_TO_LAHGI_OFFSET_X;
					//imgPoint[1] = imagePlaneY - T265_TO_LAHGI_OFFSET_Y;
					//imgPoint[2] = imagePlaneZ - T265_TO_LAHGI_OFFSET_Z;
					imgPoint[0] = imagePlaneX;
					imgPoint[1] = imagePlaneY;
					imgPoint[2] = imagePlaneZ;
					comptonImgPtr[pixelCount * (pixelCount - j - 1) + pixelCount - i - 1] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
				}
			}
		}
	}

	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(Mpix, Mpix), 0, 0, INTER_NEAREST);
	Mat reconImg;

	cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);
	//cv::rotate(reconImg, reconImg, ROTATE_90_COUNTERCLOCKWISE);

	//reconImg.convertTo(reconImg, CV_32F);	//todo : 확인 필요.
	if (resImprov > 1.0)
		cv::resize(reconImg, reconImg, Size(pixelCount, pixelCount), 0, 0, INTER_NEAREST_EXACT);	//compton size로 변경 : 하이브리드 영상용

	double fovHeight = 2 * tan((hFov / 2) * M_PI / 180.0) * (s2M + m2D);
	double fovWidth = 2 * tan((wFov / 2) * M_PI / 180.0) * (s2M + m2D);

	//height correction
	constexpr double heightDiff = 0.35;

	double heightPixelSize = reconPlaneWidth / pixelCount;
	int offSetPixelCount = heightDiff / heightPixelSize;

	int heightPixelCount = pixelCount * (fovHeight / reconPlaneWidth);
	int widthPixelCount = pixelCount * (fovWidth / reconPlaneWidth);

	int minHeightPixleCount = (pixelCount - heightPixelCount) / 2 - offSetPixelCount;
	int maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 - offSetPixelCount;

	int minWidthPixleCount = (pixelCount - widthPixelCount) / 2 - offSetPixelCount;
	int maxWidthPixleCount = (pixelCount + widthPixelCount) / 2 - offSetPixelCount;
	if (minHeightPixleCount < 0)
	{
		minHeightPixleCount = 0;
	}
	if (maxHeightPixleCount > pixelCount)
	{
		maxHeightPixleCount = pixelCount;
	}
	if (widthPixelCount > pixelCount)
	{
		widthPixelCount = pixelCount;
	}

	if (minWidthPixleCount < 0)
	{
		minWidthPixleCount = 0;
	}
	if (maxWidthPixleCount > pixelCount)
	{
		maxWidthPixleCount = pixelCount;
	}

	mDetectorResponseImage = responseImg;	//not use

	cv::Mat nonFiltered = reconImg;
	cv::Mat Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	reconImg = Filtered;

	//mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	//mCodedImage = reconImg(Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2), Range(minHeightPixleCount, maxHeightPixleCount));
	mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	//cv::resize(reconImg, mCodedImage, Size(widthPixelCount, maxHeightPixleCount - minHeightPixleCount), 0, 0, INTER_NEAREST_EXACT);	//code 영상은 그냥 resize만

	nonFiltered = comptonImg;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 2);
	Filtered.convertTo(Filtered, CV_32S);
	comptonImg = Filtered;

	//mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	//mComptonImage = comptonImg(Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2), Range(minHeightPixleCount, maxHeightPixleCount));
	mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));

	//231106-1 sbkwon : 
	//normalize 해서 픽셀곱해주는 부분 추가 필요
	Mat mCodedImagenorm;
	Mat mComptonImagenorm;
	cv::normalize(mCodedImage, mCodedImagenorm, 0, 255, cv::NORM_MINMAX);
	cv::normalize(mComptonImage, mComptonImagenorm, 0, 255, cv::NORM_MINMAX);
	mHybridImage = mCodedImagenorm.mul(mComptonImagenorm);

	nonFiltered = mHybridImage;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	mHybridImage = Filtered;

	//minMaxLoc(Filtered, &minVal, &maxVal, &minLoc, &maxLoc);
	//if (maxVal > 400)
	//{
	//}

	//mHybridImage = Filtered;

	//Logger::Instance().InvokeLog("RadImage", "Hybrid Max: " + std::to_string(maxVal), eLoggerType::INFO);

	if (data.size() == 0)
	{
		return;
	}

	mDetectorTransformation = data[0].DetectorTransformation;
	mListedListModeData = data;
}

//231025-1 sbkwon 
HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D)
{	
	int datasize = data.size();

	if (datasize <= 0)
	{
		return;
	}

	SetIndexPos();

	mDetectorTransformation = data[datasize - 1].DetectorTransformation;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gencloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//*gencloud = *RtabmapSlamControl::instance().generatePointCloud();

	cv::Mat rgb;
	cv::Mat depth;

	rgb = RtabmapSlamControl::instance().GetCurrentVideoFrame();
	depth = RtabmapSlamControl::instance().GetCurrentDepthFrame();


	if (rgb.empty() || depth.empty())
		return;

	*gencloud = *RtabmapSlamControl::instance().generatePointCloud(depth, rgb);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointIndices& removedindices = *new pcl::PointIndices;
	float leafsize = 0.08;

	downsampled = HUREL::Compton::RtabmapSlamControl::instance().downsamplePointCloud(gencloud, leafsize, removedindices);

	open3d::geometry::PointCloud reconPointCloud = RtabmapSlamControl::instance().PclToOpen3d(gencloud);

	//open3d::geometry::PointCloud downsampledPointCloud = HUREL::Compton::RtabmapSlamControl::instance().PclToOpen3d(downsampled);

	open3d::geometry::PointCloud recontransPC;
	recontransPC = HUREL::Compton::RtabmapSlamControl::instance().RTPointCloudTransposed(reconPointCloud, mDetectorTransformation);

	//open3d::io::WritePointCloudOption option;
	//open3d::io::WritePointCloudToPLY("C:\Users\Compton\source\repos\triplehoon\HUREL_Compton\HUREL Imager GUI\bin\x64\Release\net6.0-windows\shot.ply", reconPointCloud, option);

	/*if (recontransPC.IsEmpty())
		return;*/

	HUREL::Compton::ReconPointCloud reconPCtrans = HUREL::Compton::ReconPointCloud(recontransPC);

	/*HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(reconPointCloud);
	open3d::geometry::PointCloud recontransPC;
	open3d::geometry::PointCloud recontransPCFOVlim;
	Eigen::MatrixXd fovchk(1, reconPC.points_.size());*/

	double m = 1 + M2D / s2M;
	double imagePlaneZ = s2M + M2D;
	double dproj = det_W / (Mask_W / Mpix * m);
	int pixelCount = static_cast<int>(round(dproj * resImprov));
	int pixelCountcoded = 29;	// static_cast<int>(round(dproj));
		
	// should be fixed
	//reconPC.imspaceLim(reconPointCloud, wFov, hFov, &reconPCFOVlim, &fovchk);
	//reconPC.imspaceLim(reconPointCloud, 50, 50, mDetectorTransformation, &recontransPCFOVlim, &recontransPC, &fovchk);

	int pixelLength = static_cast<int>(round(dproj * 1)); // pixel length of detector

	Mat comptonImg(480, 848, CV_32S, Scalar(1)); //848*480
	int32_t* comptonImgPtr = static_cast<int32_t*>(static_cast<void*>(comptonImg.data));
	int codedImageCount = 0;
	int comptonImageCount = 0;

	Mat responseImg(pixelCountcoded, pixelCountcoded, CV_32S, Scalar(0));
	__int32* responseImgPtr = static_cast<__int32*>(static_cast<void*>(responseImg.data));

	double det_w_div2 = -det_W / 2;
	double pixelSize = det_W / pixelCountcoded;

	int32_t maxVal = 0;
	Eigen::Vector3d maxValLoc;
	maxValLoc[0] = 0;
	maxValLoc[1] = 0;
	maxValLoc[2] = 0;

	for (int i = 0; i < data.size(); ++i)
	{
		ListModeData& lm = data[i];
				
		if (lm.Type == eInterationType::COMPTON)
		{
			if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 400 || lm.Scatter.InteractionEnergy < 50)
			{
				continue;
			}

			int pointCount = 0;
			int dwsIndexCount = 0;

			int width = 848;
			int height = 480;
			int indx = 0;
			//for (int rows = 0; rows < height; ++rows)
			//{
			//	for (int cols = 0; cols < width; ++cols)
			//	{
			//		if (pointCount < 0 )
			//		{
			//			std::cerr << "Pont count is over the point cloud size" << std::endl;
			//			break;
			//		}
			//		Eigen::Vector3d imgPoint;
			//		//double FOVchk = fovchk(pointCount);
			//		imgPoint[0] = reconPCtrans.points_[pointCount].x();
			//		imgPoint[1] = reconPCtrans.points_[pointCount].y();
			//		imgPoint[2] = reconPCtrans.points_[pointCount].z();

			//		//pixelCount * (pixelCount - j - 1) + pixelCount - i - 1]
			//		//comptonImgPtr[848 * (480 - cols - 1) + 848 - rows - 1] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
			//		//comptonImgPtr[480 * rows + cols] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
			//		//indx = rows * width + cols;
			//		//indx = rows * height + cols;
			//		//comptonImgPtr[indx] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
			//		comptonImg.at< int32_t>(rows, cols) += ReconPointCloud::SimpleComptonBackprojectionTransformed(lm, imgPoint, 1);

			//		if (maxVal < comptonImg.at< int32_t>(rows, cols))
			//		{
			//			maxVal = comptonImg.at< int32_t>(rows, cols);
			//			maxValLoc = imgPoint;
			//		}
			//		//--pointCount;
			//		++pointCount;
			//	}
			//}

			//down
#pragma omp parallel for
			for (int rows = 0; rows < height; ++rows)
			{
				for (int cols = 0; cols < width; ++cols, ++pointCount)
				{
					int dwsIndex = (dwsIndexCount < removedindices.indices.size()) ? removedindices.indices[dwsIndexCount] : -1;

					if (pointCount == dwsIndex)
					{
						++dwsIndexCount;
						continue;
					}

					if (pointCount >= reconPCtrans.points_.size())
					{
						std::cerr << "Pont count is over the point cloud size" << std::endl;
						break;
					}

					Eigen::Vector3d imgPoint;
					//double FOVchk = fovchk(pointCount);
					imgPoint[0] = reconPCtrans.points_[pointCount].x();
					imgPoint[1] = reconPCtrans.points_[pointCount].y();
					imgPoint[2] = reconPCtrans.points_[pointCount].z();

					comptonImg.at< int32_t>(rows, cols) += ReconPointCloud::SimpleComptonBackprojectionTransformed(lm, imgPoint, 1);

					if (maxVal < comptonImg.at< int32_t>(rows, cols))
					{
						maxVal = comptonImg.at< int32_t>(rows, cols);
						maxValLoc = imgPoint;
					}
				}
			}

		}
	}

	cv::Mat nonFiltered = comptonImg;
	cv::Mat Filtered;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	mComptonImage = Filtered;
	// ------Compton imaging done---------------


	mCodedImage = Filtered;
	mHybridImage = Filtered;

	// ------Coded imageing Start---------------
	//

	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(Mpix, Mpix), 0, 0, INTER_NEAREST);

	open3d::geometry::PointCloud codedTransPC3d = recontransPC;

	Mat codedImg(480, 848, CV_32S, Scalar(0)); //848*480
	int32_t* codedImgPtr = static_cast<int32_t*>(static_cast<void*>(codedImg.data));

	int pcdDownCount = recontransPC.points_.size() * 0.04;

	if (pcdDownCount == 0)
		pcdDownCount = recontransPC.points_.size() > 0 ? recontransPC.points_.size() : 1;

	int pcdDownInterval = recontransPC.points_.size() / pcdDownCount;

	//open3d::geometry::PointCloud transPC3d = codedTransPC3d.Transform(data[0].DetectorTransformation.inverse());	//마지막 trans 를 적용
	//HUREL::Compton::ReconPointCloud transPC = HUREL::Compton::ReconPointCloud(transPC3d);
//
//#pragma omp parallel for
	for (int i = 0; i < data.size(); ++i)
	{
		ListModeData& lm = data[i];

		if (lm.Type == eInterationType::CODED)
		{
			codedImageCount++;
			int ncount = 0;
			//현재와 다음 transformation matrix의 차이 계산
			if (i < data.size() - 1)
			{
				Eigen::Matrix4d curTrans = lm.DetectorTransformation;
				Eigen::Matrix4d nextTrans = data[i + 1].DetectorTransformation;
				for (int ii = 0; ii < 4; ii++)
				{
					for (int jj = 0; jj < 4; jj++)
					{
						if (abs(nextTrans(ii, jj) - curTrans(ii, jj)) <= 0.002)
							ncount++;
					}
				}
			}

			//현재는 count
			double& interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
			double& interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];

			int iX = findIndex(interactionPoseX, det_w_div2, pixelSize);
			int iY = findIndex(interactionPoseY, det_w_div2, pixelSize);
			if (iX >= 0 && iY >= 0 && iX < pixelCountcoded && iY < pixelCountcoded)
			{
				++responseImgPtr[pixelCountcoded * iY + iX];
			}

			if (ncount < 10)	//다음 데이터와 다른 위치여서 좌표간 영상 변환 시작 및 완료 후 데이터 삭제
			{
				//point cloud transformation
				//open3d::geometry::PointCloud transPC3d = codedTransPC3d.Transform(lm.DetectorTransformation.inverse());	//마지막 trans 를 적용 //recontransPC
				codedTransPC3d.Transform(lm.DetectorTransformation.inverse());	//마지막 trans 를 적용 
				HUREL::Compton::ReconPointCloud transPC = HUREL::Compton::ReconPointCloud(codedTransPC3d);//recontransPC

				//PDC Z 평균 구하기
				double zSum = 0.0;
				for (int pci = 0; pci < pcdDownCount; pci++)
				{
					int nIndex = pci * pcdDownInterval;
					zSum += transPC.points_[nIndex].z();
				}

				double zMean = zSum / pcdDownCount;

				//LUT Z Index 구하기
				int zIndex = 0;
				if (zMean < 1.5)
					zIndex = 0;
				else if (zMean >= 1.5 && zMean < 3)
					zIndex = 1;
				else if (zMean >= 3 && zMean < 4.5)
					zIndex = 2;
				else if (zMean >= 4.5 && zMean < 6)
					zIndex = 3;
				else
					zIndex = 4;

				Logger::Instance().InvokeLog("RadImage", "Z Index: " + std::to_string(zIndex) + ", Data : " + std::to_string(data.size()) + ", I : " + std::to_string(i)
					+ " Coded No : " + std::to_string(codedImageCount), eLoggerType::INFO);
				//zIndex = 1;

				//영상화
				Mat reconImg;
				cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);
				//cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);

				int resize = pixelCountcoded * 3;
				cv::resize(reconImg, reconImg, Size(resize, resize), 0, 0, INTER_NEAREST_EXACT);	//87*87

				{
					cv::Mat nonFiltered1 = reconImg;
					cv::Mat Filtered1;

					nonFiltered1.convertTo(nonFiltered1, CV_32F);
					cv::GaussianBlur(nonFiltered1, Filtered1, Size(15, 15), 5);
					Filtered1.convertTo(Filtered1, CV_32S);
					reconImg = Filtered1;
				}

				//cv::imwrite("C:/Users/Compton/Downloads/reconImg.png", reconImg);

				//reconImg < 0 then 0
				for (int iii = 0; iii < resize; iii++)
				{
					for (int jjj = 0; jjj < resize; jjj++)
					{
						if (reconImg.at<int32_t>(iii, jjj) < 0)
							reconImg.at<int32_t>(iii, jjj) = 0;
					}
				}

				Logger::Instance().InvokeLog("RadImage", "1 image Start", eLoggerType::INFO);
				for (int iii = 0; iii < resize; iii++)
				{
					std::string line;
					for (int jjj = 0; jjj < resize; jjj++)
					{
						line += std::to_string(reconImg.at<int32_t>(iii, jjj)); line += ",";
					}
					Logger::Instance().InvokeLog("RadImage", line, eLoggerType::INFO);
				}

				Logger::Instance().InvokeLog("RadImage", "1 image End", eLoggerType::INFO);
								
				Mat angresponse_matched(91, 91, CV_32S, Scalar(0));	//구형 좌표 영상 값
				//Mat angresponse_mask(91, 91, CV_8U, Scalar(1));	//구형 좌표 영상 값
				//직교 => 구
				int nindexCount = 0;
//#pragma omp parallel for
				for (int height = 0; height < 91; height++)//91
				{
					for (int width = 0; width < 91; width++)//91
					{
						//int nInex = height * resize + width;

						int xIndex = TotalIndexbyPos[zIndex][nindexCount][0] + 45;
						int yIndex = TotalIndexbyPos[zIndex][nindexCount][1] + 45;

						/*if (TotalIndexbyPos[zIndex][nindexCount][0] == 0 && TotalIndexbyPos[zIndex][nindexCount][1] == 0)
						{
							nindexCount++;
							continue;
						}
						else*/
						{
							//angresponse_matched.at<int32_t>(yIndex, xIndex) += reconImg.at<int32_t>(height, width);
							angresponse_matched.at<int32_t>(yIndex, xIndex) += reconImg.at<int32_t>(width, height);
							//angresponse_mask.at<int32_t>(yIndex, xIndex) = 0;
							nindexCount++;
						}
					}
				}

				Logger::Instance().InvokeLog("RadImage", "2 image Start", eLoggerType::INFO);
				for (int iii = 0; iii < resize; iii++)
				{
					std::string line;
					for (int jjj = 0; jjj < resize; jjj++)
					{
						line += std::to_string(angresponse_matched.at<int32_t>(iii, jjj)); line += ",";
					}
					Logger::Instance().InvokeLog("RadImage", line, eLoggerType::INFO);
				}

				Logger::Instance().InvokeLog("RadImage", "2 image End", eLoggerType::INFO);

				/////////////////////////////////////////////
				//fillmissing 함수를 이용해서 세로, 가로 방향 총 2번 보간
				/*cv::resize(angresponse_matched, angresponse_matched, Size(31, 31), 0, 0, INTER_NEAREST);
				cv::resize(angresponse_matched, angresponse_matched, Size(91, 91), 0, 0, INTER_NEAREST);*/


				/*Mat angresponse_sor(91, 91, CV_16U, Scalar(0));
				angresponse_matched.convertTo(angresponse_sor, CV_16U);
				Mat angresponse_result(91, 91, CV_16U, Scalar(0));
				cv::inpaint(angresponse_sor, angresponse_mask, angresponse_result, 2, INPAINT_NS);

				angresponse_result.convertTo(angresponse_matched, CV_32S);*/

				//cv::imwrite("C:/Users/Compton/Downloads/angres.png", angresponse_matched);

				//구 => 포인트 클라우드				
						
				//down
				int pointCount = 0;
				int dwsIndexCount = 0;

				int width = 848;
				int height = 480;
//#pragma omp parallel for
				for (int rows = 0; rows < height; ++rows)
				{
					for (int cols = 0; cols < width; ++cols, ++pointCount)
					{
						int dwsIndex = (dwsIndexCount < removedindices.indices.size()) ? removedindices.indices[dwsIndexCount] : -1;

						if (pointCount == dwsIndex)
						{
							++dwsIndexCount;
							continue;
						}

						if (pointCount >= reconPCtrans.points_.size())
						{
							std::cerr << "Pont count is over the point cloud size" << std::endl;
							break;
						}

						double azP = RAD2DEG(std::atan2((transPC.points_[pointCount].x() * -1), transPC.points_[pointCount].z()));	//az = std::atan2(y, x); 
						double polP = RAD2DEG(std::atan2(transPC.points_[pointCount].y(), std::sqrt(transPC.points_[pointCount].z() * transPC.points_[pointCount].z()
										+ transPC.points_[pointCount].x() * transPC.points_[pointCount].x())));	//pol = std::atan2(z, std::sqrt(x * x + y * y));

						int azi = round(azP);
						int poli = round(polP);

						if (abs(azi) > 45 || abs(poli) > 45)
							continue;

						double reconValue_CCBP = angresponse_matched.at<int32_t>(poli + 45, azi + 45);
						//double reconValue_CCBP = angresponse_matched.at<int32_t>(azi + 45, poli + 45);

						if (reconValue_CCBP != 0)
						{
							codedImg.at< int32_t>(rows, cols) += reconValue_CCBP;
						}
					}
				}

				/////////////////////////////////////////////////


				//초기화
				/*for (int ri = 0; ri < pixelCountcoded; ri++)
				{
					for (int  ry = 0; ry < pixelCountcoded; ry++)
					{
						responseImg.at<int32_t>(ri, ry) = 0;
					}
				}*/

				for (int ri = 0; ri < pixelCountcoded * pixelCountcoded; ri++)
				{
					responseImgPtr[ri] = 0;
				}
				Logger::Instance().InvokeLog("RadImage", "Debug Index: 5", eLoggerType::INFO);

				//codedTransPC3d = transPC3d;

				reconImg.release();
			}
		}
	}
//
//	//최종 결과 : codedImg
	nonFiltered = codedImg;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	Filtered.convertTo(Filtered, CV_32S);
	mCodedImage = Filtered;

	// ------Coded imageing done----------------

	///spdlog::info("RadImage: Hybrid Max: " + std::to_string(maxVal));
	//mListedListModeData = lmData;
}

//231113-1 sbkwon
 HUREL::Compton::RadiationImage::RadiationImage()
{
	SetIndexPos();
}

 int HUREL::Compton::RadiationImage::nCountMat = 0;	//test

//231109-1 sbkwon
int HUREL::Compton::RadiationImage::TotalIndexbyPos[5][7569][2] = { 0, };
void HUREL::Compton::RadiationImage::SetIndexPos()
{
	static bool bInit = false;

	if (bInit)
		return;

	//Z = 1, 2, 4, 5, 6 m

	//double m = 1 + M2D / S2M;	//S2M : 1m일경우
	//double reconPlaneWidth = S2M / M2D * Det_W;	//source 위치에서의 영상 사이즈
	//double dproj = Det_W / (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
	//int pixelCount = static_cast<int>(round(dproj));

	int pixelCount = 29 * 3;

	//구좌표 각도
	int nSphericalCount = 91;

	Eigen::MatrixXd SphericalAz = Eigen::MatrixXd::Zero(nSphericalCount, nSphericalCount);
	Eigen::MatrixXd SphericalPol = Eigen::MatrixXd::Zero(nSphericalCount, nSphericalCount);

	for (int i = 0; i < nSphericalCount; i++)
	{
		for (int j = 0; j < nSphericalCount; j++)
		{
			SphericalAz(i, j) = j - 45;
			SphericalPol(i, j) = i - 45;
			/*SphericalAz(i, j) = j - 45;
			SphericalPol(i, j) = i - 45;*/
		}
	}

	//Detector pos
	double minWidth = -(Det_W / 2);
	double interval = Det_W / (pixelCount - 1);

	double detectPos[87] = { 0.0, };

	for (int i = 0; i < pixelCount; i++)
	{
		detectPos[i] = minWidth + interval * i;
	}

	double ZPos[5] = { 1.0, 2.0, 4.0, 5.0, 7.0 };

#pragma omp parallel for
	for (int i = 0; i < 5; i++)
	{
		double z = ZPos[i];
		double imagePlaneZ_pow2 = z * z;

		double s2m = z - M2D;

		double CAxis = s2m / M2D;

		//직교 영상 -> 구좌표 영상 변환  : 방위 (z, x, y => x, y, z)
		for (int w = 0; w < pixelCount; w++)
		{
			double indexWidth = (minWidth + interval * w) * CAxis;
			double indexWidth_pow2 = indexWidth * indexWidth;
			double azC = RAD2DEG(std::atan2(indexWidth, z));//az = std::atan2(y, x); 

			for (int h = 0; h < pixelCount; h++)
			{
				double indexHeight = (minWidth + interval * h) * CAxis;
				double polC = RAD2DEG(std::atan2(indexHeight, std::sqrt(imagePlaneZ_pow2 + indexWidth_pow2)));	//pol = std::atan2(z, std::sqrt(x * x + y * y));

				Eigen::MatrixXd az_diff = (SphericalAz.array() - azC).abs().pow(2);
				Eigen::MatrixXd pol_diff = (SphericalPol.array() - polC).abs().pow(2);
				Eigen::MatrixXd sum_diff = az_diff.matrix() + pol_diff.matrix();

				std::ptrdiff_t minIndexX, minIndexY;
				double minValue = sum_diff.minCoeff(&minIndexX, &minIndexY);

				if (minValue <= 0.25)
				{
					int nIndex = w * 87 + h;
					TotalIndexbyPos[i][nIndex][0] = SphericalAz(minIndexX, minIndexY);
					TotalIndexbyPos[i][nIndex][1] = SphericalPol(minIndexX, minIndexY);
				}

				/*int nMin = 9999999;
				int nIndexX = -1;
				int nIndexY = -1;

				for (int j = 0; j < 91; j++)
				{
					for (int k = 0; k < 91; k++)
					{
						double az_dif = pow(abs(azC - SphericalAz(j, k)), 2);
						double pol_dif = pow(abs(polC - SphericalPol(j, k)), 2);
						double sum_dif = sqrt(az_dif + pol_dif);

						if (sum_dif <= 0.5 && nMin > sum_dif)
						{
							nIndexX = j;
							nIndexY = k;
							nMin = sum_dif;
						}
					}
				}

				if (nIndexX >= 0 && nIndexY >= 0)
				{
					int nIndex = w * 87 + h;
					TotalIndexbyPos[i][nIndex][0] = SphericalAz(nIndexY, nIndexX);
					TotalIndexbyPos[i][nIndex][1] = SphericalPol(nIndexY, nIndexX);
				}*/
			}
		}
	}
	

	/// <summary>
	/*std::ofstream saveaz, savepol;
	saveaz.open("C:/Users/Compton/Downloads/angres_az.csv");
	savepol.open("C:/Users/Compton/Downloads/angres_pol.csv");*/

	/*if (saveaz.is_open())
	{
		for (int i = 0; i < pixelCount; i++)
		{
			std::string line = "";
			for (int j = 0; j < pixelCount; j++)
			{
				int nIndex = i + pixelCount * j;
				line += std::to_string(TotalIndexbyPos[1][nIndex][0]); line += ",";
			}

			saveaz << line << std::endl;
		}

		saveaz.close();
	}

	if (savepol.is_open())
	{
		for (int i = 0; i < pixelCount; i++)
		{
			std::string line = "";
			for (int j = 0; j < pixelCount; j++)
			{
				int nIndex = i + pixelCount * j;
				line += std::to_string(TotalIndexbyPos[1][nIndex][1]); line += ",";
			}

			savepol << line << std::endl;
		}

		savepol.close();
	}*/


	////az, pol : 
	//for (int zindex = 0; zindex < 5; zindex++)
	//{

	//	std::ofstream saveaz;
	//	saveaz.open("C:/Users/Compton/Downloads/angres_7569_" + std::to_string(zindex) + ".csv");

	//	if (saveaz.is_open())
	//	{
	//		for (int i = 0; i < pixelCount; i++)
	//		{
	//			for (int j = 0; j < pixelCount; j++)
	//			{
	//				int nIndex = i * pixelCount + j;
	//				saveaz << std::to_string(TotalIndexbyPos[zindex][nIndex][0]) << "," << std::to_string(TotalIndexbyPos[zindex][nIndex][1]) << std::endl;
	//			}
	//		}

	//		saveaz.close();
	//	}
	//}

	/*std::ofstream saveaz, savepol;
	saveaz.open("C:/Users/Compton/Downloads/angres_SP_az.csv");
	savepol.open("C:/Users/Compton/Downloads/angres_SP_pol.csv");

	if (saveaz.is_open())
	{
		for (int i = 0; i < 91; i++)
		{
			std::string line = "";
			for (int j = 0; j < 91; j++)
			{
				line += std::to_string(SphericalAz(i, j)); line += ",";
			}

			saveaz << line << std::endl;
		}

		saveaz.close();
	}

	if (savepol.is_open())
	{
		for (int i = 0; i < 91; i++)
		{
			std::string line = "";
			for (int j = 0; j < 91; j++)
			{
				line += std::to_string(SphericalPol(i, j)); line += ",";
			}

			savepol << line << std::endl;
		}

		savepol.close();
	}*/
	/// </summary>

	bInit = true;

	Logger::Instance().InvokeLog("RadImage", "Successfully initiate Total Index Z Pos", eLoggerType::INFO);
}

double HUREL::Compton::RadiationImage::OverlayValue(Eigen::Vector3d point, eRadiationImagingMode mode)
{

	constexpr double imagePlaneZ = S2M + M2D + 0.011;
	Eigen::Vector3d detectorNormalVector(0, 0, 1);
	Eigen::Vector4d point4d(point.x(), point.y(), point.z(), 1);
	Eigen::Vector4d transformedPoint = ((mDetectorTransformation).inverse() * point4d);
	if (transformedPoint.z() <= imagePlaneZ || transformedPoint.z() >= 5)
	{
		//std::cout << transformedPoint << std::endl;
		return 0;
	}
	double xPoseOnImgPlane = transformedPoint.x() * imagePlaneZ / transformedPoint.z();
	double yPoseOnImgPlane = transformedPoint.y() * imagePlaneZ / transformedPoint.z();


	int iY = findIndex(xPoseOnImgPlane, -ReconPlaneWidth / 2, ReconPlaneWidth / PixelCount);
	int iX = findIndex(yPoseOnImgPlane, -ReconPlaneWidth / 2, ReconPlaneWidth / PixelCount);
	int tempiY = iY;
	iY = iX;
	iX = tempiY;

	if (iX >= 0 && iY >= 0 && iX < PixelCount && iY < PixelCount)
	{
		__int32 value = 0;
		switch (mode)
		{
		case HUREL::Compton::eRadiationImagingMode::CODED:
			value = static_cast<__int32*>(static_cast<void*>(mCodedImage.ptr()))[PixelCount * (PixelCount - iY) + PixelCount - iX];
			break;
		case HUREL::Compton::eRadiationImagingMode::COMPTON:
			value = static_cast<__int32*>(static_cast<void*>(mComptonImage.ptr()))[PixelCount * (PixelCount - iY) + PixelCount - iX];
		case HUREL::Compton::eRadiationImagingMode::HYBRID:
			value = static_cast<__int32*>(static_cast<void*>(mHybridImage.ptr()))[PixelCount * (PixelCount - iY) + PixelCount - iX];
			break;
		default:
			assert(false);
			return 0.0;
			break;
		}
		return static_cast<double>(value);
	}
	else
	{
		return 0.0;
	}





}

void HUREL::Compton::RadiationImage::OverlayRadimgToP3(cv::Mat& p3, const cv::Mat& radImg)
{


}

