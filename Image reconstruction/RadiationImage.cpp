#include "RadiationImage.h"

using namespace Eigen;

// Makting Detector Response Image
constexpr double Det_W = 0.146;
constexpr double Mask_W = 0.180;
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
			if (img.at<int>(i, j) < minValue)//231023 sbkwon : �ߺ� ���� ����
			{
				normImg.at<uchar>(i, j) = 0;
			}
			else
			{
				normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue) / (maxValue - minValue) * 255);
			}
		}
	}

	//240927 : sameple data write
#if 0
	std::string spath = "E:\\sample\\" + std::to_string(nCountMat) + "_normImg.png";
	cv::imwrite(spath, normImg);

#endif

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

	//231023 sbkwon : ���� - size ����
	if (colorImg.size().height > colorImg.size().width)
	{
		sizeWidth = size * colorImg.size().width / colorImg.size().height;
	}
	else
	{
		sizeHeight = size * colorImg.size().height / colorImg.size().width;
	}

	cv::resize(colorImg, showImg, cv::Size(sizeWidth, sizeHeight), 0, 0, cv::INTER_NEAREST_EXACT);

	//240927 : sameple data write
#if 0
	std::string spath1 = "E:\\sample\\" + std::to_string(nCountMat++) + "_colorImg.png";
	cv::imwrite(spath1, showImg);

#endif

	return showImg;
}

cv::Mat HUREL::Compton::RadiationImage::GetCV_32SAsJetZero(cv::Mat img, int size, double minValuePortion)
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
			if (img.at<int>(i, j) < minValue)//231023 sbkwon : �ߺ� ���� ����
			{
				normImg.at<uchar>(i, j) = 0;
			}
			else
			{
				normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue) / (maxValue - minValue) * 255);
			}
		}
	}

	//240927 : sameple data write
#if 0
	std::string spath = "D:\\sample\\" + std::to_string(nCountMat++) + "_normImg.png";
	cv::imwrite(spath, normImg);

#endif

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
				pixel[0] = 0;
				pixel[3] = 0;
			}
		}
	}
	int sizeHeight = size;
	int sizeWidth = size;

	//231023 sbkwon : ���� - size ����
	if (colorImg.size().height > colorImg.size().width)
	{
		sizeWidth = size * colorImg.size().width / colorImg.size().height;
	}
	else
	{
		sizeHeight = size * colorImg.size().height / colorImg.size().width;
	}

	cv::resize(colorImg, showImg, cv::Size(sizeWidth, sizeHeight), 0, 0, cv::INTER_NEAREST_EXACT);

	//240927 : sameple data write
#if 0
	std::string spath1 = "D:\\sample\\" + std::to_string(nCountMat++) + "_colorImg.png";
	cv::imwrite(spath1, showImg);

#endif

	return showImg;
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
			if (img.at<int>(i, j) < minValue)//231023 sbkwon : �ߺ� ���� ����
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


//250203 : �����̸� �и�
cv::Mat HUREL::Compton::RadiationImage::GetAnotation(cv::Mat img, cv::Mat& imgAno, const std::string& IsotopeName, double minValuePortion)
{

	if (img.empty() || IsotopeName.size() <= 0)
	{
		return img;
	}

	//250203 : ���� ������ Ȯ�� - ��ü : cv::Mat imgAno(img.rows, img.cols, CV_8UC4, cv::Scalar(0));
	if (img.size() != imgAno.size())
		cv::resize(imgAno, imgAno, img.size(), 0, 0, cv::INTER_NEAREST_EXACT);

	//gray ��ȯ
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

	cv::threshold(gray, gray, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

	// ������ ã��
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	double maxValue = 700 * minValuePortion;

	for (size_t i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area >= maxValue && area < 10000)
		{
			////����
			//// �߽��� ���
			//cv::Moments moments = cv::moments(contours[i]);
			//cv::Point center(moments.m10 / moments.m00, moments.m01 / moments.m00);


			//int baseline = 0;
			//cv::Size textSize = cv::getTextSize(IsotopeName, cv::FONT_ITALIC, 0.8, 2, &baseline);

			////��� ���� ��ġ : �߽��������� ��ġ
			//center.x -= textSize.width / 2;
			//center.y += textSize.height + 30;

			//cv::Rect textBackground(center.x, center.y - textSize.height, textSize.width - 10, textSize.height + baseline);
			//
			////�ؽ�Ʈ �Է� �̹��� ����
			//cv::Mat anotation(cv::Size(textBackground.width, textBackground.height), CV_8UC4, cv::Scalar(128, 0, 0, 0));
	
			//// ��� �簢�� �׸��� (������)
			////cv::rectangle(anotation, textBackground, cv::Scalar(50, 50, 50), cv::FILLED);

			////text ��� ��ġ ����
			//center.x += 7;
			//center.y += 2;


			////���� : text ���� �̹��� ������ puttext -> ���� �ռ�
			//cv::putText(anotation, IsotopeName, cv::Point(7, 20), cv::FONT_ITALIC, 0.6,
			//	cv::Scalar(255, 255, 255, 255), 2);
			//
			////cv::addWeighted(img, 1, anotation, 0.7, 0.0, img);

			//Mat mROI = img(textBackground);
			//addWeighted(mROI, 0.7, anotation, 1, 0, mROI);
			////����

			//����
			// �߽��� ���
			cv::Moments moments = cv::moments(contours[i]);
			cv::Point center(moments.m10 / moments.m00, moments.m01 / moments.m00);

			int baseline = 0;
			cv::Size textSize = cv::getTextSize(IsotopeName, cv::FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);

			//��� ���� ��ġ
			center.x -= textSize.width / 2;
			center.y += textSize.height + 30;

			// ��� �簢�� �׸��� (������)
			//cv::Rect textBackground(center.x, center.y - textSize.height, textSize.width - 10, textSize.height + baseline);
			//cv::rectangle(imgAno, textBackground, cv::Scalar(240, 240, 240, 100), cv::FILLED);	//250203

			//text ��� ��ġ ����
			center.x += 28;
			center.y += 2;

			cv::putText(imgAno, IsotopeName, center, cv::FONT_ITALIC, 0.6, 
				cv::Scalar(255, 255, 255, 255), 2);	//250203
			//����			
		}
	}

	//������ ����ũ
	cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(1));
	if (contours.size() > 0)
	{
		cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED);
		return mask;
	}
	else
		return cv::Mat();
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
	cv::rotate(scaleG, scaleG, cv::ROTATE_90_COUNTERCLOCKWISE); //lge

	Mat reconImg;
	//cv::filter2D(responseImg, reconImg, CV_32S, scaleG);	//231017 sbkwon : ����
	cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);//231017 sbkwon : ����

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

//20251208 OUTDOOR MODE
HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int maxValue, bool bfullRange)
{
	//��뿩�� Ȯ��, ���� Ȯ�� �ʿ�.
	if (data.size() == 0)
	{
		//240126 : ����ȭ ���� �� ���õ� ������ ���� ��� ���� clear
		mComptonImage = Mat::zeros(480, 848, CV_32S);
		mCodedImage = Mat::zeros(480, 848, CV_32S);
		mHybridImage = Mat::zeros(480, 848, CV_32S);

		return;
	}

	if (bfullRange == true)
	{
		//240326 : Detector FOV : 360, 180�� �����籸��
		if (data.size() == 0)
		{
			//240126 : ����ȭ ���� �� ���õ� ������ ���� ��� ���� clear
			mComptonImage = Mat::zeros(480, 848, CV_32S);
			mCodedImage = Mat::zeros(480, 848, CV_32S);
			mHybridImage = Mat::zeros(480, 848, CV_32S);

			return;
		}
		if (data.size() == 0)
		{
			return;
		}
		int pixelCountX = 360; int pixelCountXHf = pixelCountX / 2;
		int pixelCountY = 180; int pixelCountYHf = pixelCountY / 2;

		Mat comptonImg(pixelCountY, pixelCountX, CV_32S, Scalar(1));
		__int32* comptonImgPtr = static_cast<__int32*>(static_cast<void*>(comptonImg.data));

		double imagePlaneR = s2M + M2D;
		Eigen::MatrixXd SphericalXAxis = Eigen::MatrixXd::Zero(360, 1);


		for (int i = 0; i < (pixelCountX); i++)
		{
			SphericalXAxis(i, 0) = DEG2RAD(pixelCountYHf * (+1) - i); //90 -270
		}

		Eigen::MatrixXd SphericalYAxis = Eigen::MatrixXd::Zero(180, 1);
		for (int j = 0; j < (pixelCountY); j++)
		{
			SphericalYAxis(j, 0) = DEG2RAD(pixelCountYHf * (+1) - j);  //90 1 -90
		}

#pragma omp parallel for
		for (int i = 0; i < data.size(); ++i)
		{
			ListModeData& lm = data[i];
			if (lm.Type == eInterationType::COMPTON)
			{
				if ((lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy) < 100)
					continue;
				//row y direction, cols x direction
				for (int row = 0; row < (pixelCountY); ++row) //polar
				{
					double phi = SphericalYAxis(row, 0);

					for (int cols = 0; cols < (pixelCountX); ++cols) //azi
					{
						double theta = SphericalXAxis(cols, 0);

						Eigen::Vector3d imgPoint;
						imgPoint[2] = imagePlaneR * sin(phi); //z
						double rcoselev = imagePlaneR * cos(phi);
						imgPoint[0] = (rcoselev * cos(theta)); //x
						imgPoint[1] = rcoselev * sin(theta); //y

						//comptonImg.at< int32_t>(row, cols) += ReconPointCloud::SimpleComptonBackprojectionSphere(lm, imgPoint);

						comptonImg.at< int32_t>(row, cols) += ReconPointCloud::SqComptonBackprojectionSphere(lm, imgPoint);

					}
				}
			}
		}

		cv::Mat nonFiltered;
		cv::Mat Filtered;
		cv::Mat rgbFiltered;

		nonFiltered = comptonImg;
		nonFiltered.convertTo(nonFiltered, CV_32F);
		cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 2,2);
		Filtered.convertTo(Filtered, CV_32S);
		mComptonImage = Filtered;
		rgbFiltered = Filtered;
		rgbFiltered.convertTo(rgbFiltered, CV_16U);
		//cv::imwrite("E:/Filtered.png", rgbFiltered);

		//240311 : coded, compton : max value 
		double maxValueCompton = 0;

		for (int row = 0; row < mComptonImage.rows; row++)
		{
			for (int col = 0; col < mComptonImage.cols; col++)
			{
				if (mComptonImage.at<int32_t>(row, col) > maxValueCompton)
				{
					maxValueCompton = mComptonImage.at<int32_t>(row, col);
				}
			}
		}
		if (maxValueCompton <= maxValue)
		{
			for (int row = 0; row < mComptonImage.rows; row++)
			{
				for (int col = 0; col < mComptonImage.cols; col++)
				{
					mComptonImage.at<int32_t>(row, col) = 0;
				}
			}
		}

		//240326 : resize
		cv::resize(mComptonImage, mComptonImage, Size(800, 400), 0, 0, INTER_NEAREST_EXACT);

		mCodedImage = mComptonImage;
		mHybridImage = mComptonImage;

		mDetectorTransformation = data[0].DetectorTransformation;
		mListedListModeData = data;

		return;
	}

	///////////////////////////////////////////////////////////////////////

	double m = 1 + m2D / s2M;	//s2m ����
	double reconPlaneWidth = s2M / m2D * det_W;	//source ��ġ������ ���� ������
	//double reconPlaneWidth2 = 2 * tan(65 * M_PI / 180.0) * (s2M + m2D);	//fov 130���� recon plane width ���
	double dproj = det_W / (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
	int pixelCount = 299;// static_cast<int>(round(dproj * resImprov));	//compton size
	int pixelCountCode = static_cast<int>(round(dproj * resImprov));// static_cast<int>(round(dproj));;	//Detector Pixel size ����

	Mat responseImg(pixelCountCode, pixelCountCode, CV_32S, Scalar(0));
	Mat comptonImg(pixelCount, pixelCount, CV_32S, Scalar(1));
	__int32* responseImgPtr = static_cast<__int32*>(static_cast<void*>(responseImg.data));
	__int32* comptonImgPtr = static_cast<__int32*>(static_cast<void*>(comptonImg.data));

	double det_w_div2 = -det_W / 2;
	double pixelSize = det_W / pixelCountCode;

	//�ߺ� ���� ����
	double preCalc1 = reconPlaneWidth / pixelCount;	//����ȭ ���� �� �ȼ� ����
	double preCalc2 = reconPlaneWidth / pixelCount * 0.5 - reconPlaneWidth / 2;	//����ȭ ������ �� �ȼ� ���� - �߽��� (0,0)���� ����� ����

	double imagePlaneZ = s2M + M2D;

#pragma region background count
	////231121-1 sbkwon
	//int nbgdCountX = 24;
	//Eigen::MatrixXi bgdCountY1 = Eigen::MatrixXi::Zero(pixelCountCode, pixelCountCode);
	//Eigen::MatrixXi bgdCountY2 = Eigen::MatrixXi::Zero(pixelCountCode, pixelCountCode);
#pragma endregion

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

#pragma region background count
				////231121-1 sbkwon
				////background count ���� : ListModeData EnergyCheck�� �̿��Ͽ� Min / Max count �Ѵ�.
				//double dIntervalE = (lm.EnergyCheck.maxE - lm.EnergyCheck.minE) / nbgdCountX;
				//double y1Max = lm.EnergyCheck.minE + dIntervalE;
				//double y2Min = lm.EnergyCheck.maxE - dIntervalE;

				//if (lm.EnergyCheck.minE < lm.Scatter.InteractionEnergy && lm.Scatter.InteractionEnergy <= y1Max)// Y1 range (ECheck.minE, y1Max)
				//{
				//	bgdCountY1(iY, iX)++;
				//}

				//if (y2Min <= lm.Scatter.InteractionEnergy && lm.Scatter.InteractionEnergy < lm.EnergyCheck.maxE)// Y1 range (y2Min, ECheck.maxE)
				//{
				//	bgdCountY2(iY, iX)++;
				//}
#pragma endregion
			}
		}
		else if (lm.Type == eInterationType::COMPTON)	//230907 sbkwon
		{
			if ((lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy) < 100)
				continue;

			for (int i = 0; i < pixelCount; ++i)
			{
				double imagePlaneX = preCalc1 * i + preCalc2;

				for (int j = 0; j < pixelCount; ++j)
				{
					double imagePlaneY = preCalc1 * j + preCalc2;

					Eigen::Vector3d imgPoint;
					imgPoint[0] = imagePlaneX;
					imgPoint[1] = imagePlaneY;
					imgPoint[2] = imagePlaneZ;
					comptonImgPtr[pixelCount * (pixelCount - j - 1) + pixelCount - i - 1] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);

					//240227 lge
					//comptonImgPtr[pixelCount * (pixelCount - j - 1) + pixelCount - i - 1] += ReconPointCloud::SqComptonBackprojectionTransformed(lm, imgPoint);

				}
			}
		}
	}



	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(Mpix * resImprov, Mpix * resImprov), 0, 0, INTER_NEAREST);
	cv::rotate(scaleG, scaleG, cv::ROTATE_90_COUNTERCLOCKWISE); //lge

	Mat reconImg;

	cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);

	//reconImg.convertTo(reconImg, CV_32F);	//todo : Ȯ�� �ʿ�.
	if (resImprov > 1.0)
		cv::resize(reconImg, reconImg, Size(pixelCount, pixelCount), 0, 0, INTER_NEAREST_EXACT);	//compton size�� ���� : ���̺긮�� �����

	double fovHeight = 2 * tan((hFov / 2) * M_PI / 180.0) * (s2M + m2D);
	double fovWidth = 2 * tan((wFov / 2) * M_PI / 180.0) * (s2M + m2D);

	//height correction
	////////////////////////////////////////////////////////
	//constexpr double heightDiff = 0.35;
	//
	//double heightPixelSize = reconPlaneWidth / pixelCount;
	//int offSetPixelCount = heightDiff / heightPixelSize;
	//
	//int heightPixelCount = pixelCount * (fovHeight / reconPlaneWidth);
	//int widthPixelCount = pixelCount * (fovWidth / reconPlaneWidth);
	//
	//int minHeightPixleCount = (pixelCount - heightPixelCount) / 2 - offSetPixelCount;
	//int maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 - offSetPixelCount;
	//
	//int minWidthPixleCount = (pixelCount - widthPixelCount) / 2 - offSetPixelCount;
	//int maxWidthPixleCount = (pixelCount + widthPixelCount) / 2 - offSetPixelCount;
	//if (minHeightPixleCount < 0)
	//{
	//	minHeightPixleCount = 0;
	//}
	//if (maxHeightPixleCount > pixelCount)
	//{
	//	maxHeightPixleCount = pixelCount;
	//}
	//if (widthPixelCount > pixelCount)
	//{
	//	widthPixelCount = pixelCount;
	//}
	//
	//if (minWidthPixleCount < 0)
	//{
	//	minWidthPixleCount = 0;
	//}
	//if (maxWidthPixleCount > pixelCount)
	//{
	//	maxWidthPixleCount = pixelCount;
	//}
	//
	//mDetectorResponseImage = responseImg;	//not use
	//
	//cv::Mat nonFiltered = reconImg;
	//cv::Mat Filtered;
	//
	//nonFiltered.convertTo(nonFiltered, CV_32F);
	//cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
	//Filtered.convertTo(Filtered, CV_32S);
	//reconImg = Filtered;
	//
	////mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	////mCodedImage = reconImg(Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2), Range(minHeightPixleCount, maxHeightPixleCount));
	//mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	////cv::resize(reconImg, mCodedImage, Size(widthPixelCount, maxHeightPixleCount - minHeightPixleCount), 0, 0, INTER_NEAREST_EXACT);	//code ������ �׳� resize��
	//
	//nonFiltered = comptonImg;
	//nonFiltered.convertTo(nonFiltered, CV_32F);
	//cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 2);
	//Filtered.convertTo(Filtered, CV_32S);
	//comptonImg = Filtered;
	//
	////mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	////mComptonImage = comptonImg(Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2), Range(minHeightPixleCount, maxHeightPixleCount));
	//mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));
	//
	/////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////
	double heightDiff = abs(T265_TO_LAHGI_OFFSET_Y) *1.17;
	double widthDiff = abs(T265_TO_LAHGI_OFFSET_X) * 1.35;

	double heightPixelSize = reconPlaneWidth / pixelCount;
	int offSetPixelCount = heightDiff / heightPixelSize;
	int offSetWidthPixelCount = widthDiff / heightPixelSize;

	int heightPixelCount = pixelCount * (fovHeight / reconPlaneWidth);
	int widthPixelCount = pixelCount * (fovWidth / reconPlaneWidth);

	int minHeightPixleCount;
	int maxHeightPixleCount;
	int minWidthPixleCount;
	int maxWidthPixleCount;

	if (T265_TO_LAHGI_OFFSET_Y < 0)
	{
		minHeightPixleCount = (pixelCount - heightPixelCount) / 2 - offSetPixelCount;
		maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 - offSetPixelCount;
	}
	else
	{
		minHeightPixleCount = (pixelCount - heightPixelCount) / 2 + offSetPixelCount;
		maxHeightPixleCount = (pixelCount + heightPixelCount) / 2 + offSetPixelCount;
	}


	if (T265_TO_LAHGI_OFFSET_X < 0)
	{
		minWidthPixleCount = (pixelCount - widthPixelCount) / 2 - offSetWidthPixelCount;
		maxWidthPixleCount = (pixelCount + widthPixelCount) / 2 - offSetWidthPixelCount;
	}
	else
	{
		minWidthPixleCount = (pixelCount - widthPixelCount) / 2 + offSetWidthPixelCount;
		maxWidthPixleCount = (pixelCount + widthPixelCount) / 2 + offSetWidthPixelCount;
	}


	if (minHeightPixleCount < 0)
	{
		minHeightPixleCount = 0;
	}
	if (maxHeightPixleCount > pixelCount)
	{
		maxHeightPixleCount = pixelCount;
	}

	if (minWidthPixleCount < 0)
	{
		minWidthPixleCount = 0;
	}
	if (maxWidthPixleCount > pixelCount)
	{
		maxWidthPixleCount = pixelCount;
	}

	for (int i = 0; i < reconImg.rows; ++i)
	{
		for (int j = 0; j < reconImg.cols; ++j)
		{
			if (reconImg.at<float>(i, j) < 0)
			{
				reconImg.at<float>(i, j) = 0;
			}
		}
	}

	//mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));


	mCodedImage = reconImg(Range(minHeightPixleCount, maxHeightPixleCount), Range(minWidthPixleCount, maxWidthPixleCount));
	///
	cv::resize(mCodedImage, mCodedImage, Size(848, 480), 0, 0, INTER_NEAREST_EXACT);	//compton size�� ���� : ���̺긮�� �����


	//mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range((pixelCount - widthPixelCount) / 2, (pixelCount + widthPixelCount) / 2));


	mComptonImage = comptonImg(Range(minHeightPixleCount, maxHeightPixleCount), Range(minWidthPixleCount, maxWidthPixleCount));
	cv::resize(mComptonImage, mComptonImage, Size(848, 480), 0, 0, INTER_NEAREST_EXACT);	//resize size(col, row)

	//231106-1 sbkwon : 
	Mat mCodedImagenorm;
	Mat mComptonImagenorm;
	cv::normalize(mCodedImage, mCodedImagenorm, 0, 255, cv::NORM_MINMAX);
	cv::normalize(mComptonImage, mComptonImagenorm, 0, 255, cv::NORM_MINMAX);
	//mHybridImage = mCodedImagenorm.mul(mComptonImagenorm);
	mHybridImage = mCodedImage.mul(mComptonImage);

	cv::Mat nonFiltered = mCodedImage;
	cv::Mat Filtered;

	nonFiltered.convertTo(nonFiltered, CV_32F);
	//cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 2,2);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 4, 4);
	Filtered.convertTo(Filtered, CV_32S);
	mCodedImage = Filtered;


	nonFiltered = mComptonImage;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 4,4);
	Filtered.convertTo(Filtered, CV_32S);
	mComptonImage = Filtered;

	nonFiltered = mHybridImage;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 4, 4);
	Filtered.convertTo(Filtered, CV_32S);
	mHybridImage = Filtered;

	//mHybridImage = Filtered;

	//Logger::Instance().InvokeLog("RadImage", "Hybrid Max: " + std::to_string(maxVal), eLoggerType::INFO);

	//240311 : coded, compton : max value 
	double maxValueCoded = 0;
	double maxValueCompton = 0;
	double maxValueHybrid = 0;

	for (int row = 0; row < mHybridImage.rows; row++)
	{
		for (int col = 0; col < mHybridImage.cols; col++)
		{
			if (mHybridImage.at<int32_t>(row, col) > maxValueHybrid)
			{
				maxValueHybrid = mHybridImage.at<int32_t>(row, col);
			}
			if (mCodedImage.at<int32_t>(row, col) > maxValueCoded)
			{
				maxValueCoded = mCodedImage.at<int32_t>(row, col);
			}
			if (mComptonImage.at<int32_t>(row, col) > maxValueCompton)
			{
				maxValueCompton = mComptonImage.at<int32_t>(row, col);
			}
		}
	}
	if (maxValueHybrid <= maxValue)
	{
		for (int row = 0; row < mHybridImage.rows; row++)
		{
			for (int col = 0; col < mHybridImage.cols; col++)
			{
				mHybridImage.at<int32_t>(row, col) = 0;
			}
		}
	}
	//240311 : coded
	if (maxValueCoded <= maxValue)
	{
		for (int row = 0; row < mCodedImage.rows; row++)
		{
			for (int col = 0; col < mCodedImage.cols; col++)
			{
				mCodedImage.at<int32_t>(row, col) = 0;
			}
		}
	}
	//240311 : compton 
	if (maxValueCompton <= maxValue)
	{
		for (int row = 0; row < mComptonImage.rows; row++)
		{
			for (int col = 0; col < mComptonImage.cols; col++)
			{
				mComptonImage.at<int32_t>(row, col) = 0;
			}
		}
	}

	if (data.size() == 0)
	{
		return;
	}


	//240927 : sameple data write
#if 0
	cv::Mat mat_16u;
	cv::normalize(mHybridImage, mat_16u, 0, 65535, cv::NORM_MINMAX);
	mat_16u.convertTo(mat_16u, CV_16U);

	std::string spath = "D:\\sample\\" + std::to_string(nCountMat++) + "_mHybridImage.png";
	cv::imwrite(spath, mat_16u);

#endif


	mDetectorTransformation = data[0].DetectorTransformation;
	mListedListModeData = data;
}


//231025-1 sbkwon : point cloud
//HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D)
//{	
//	int datasize = data.size();
//
//	if (datasize <= 0)
//	{
//		return;
//	}
//
//	SetIndexPos();
//
//	mDetectorTransformation = data[datasize - 1].DetectorTransformation;
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gencloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	//*gencloud = *RtabmapSlamControl::instance().generatePointCloud();
//
//	cv::Mat rgb;
//	cv::Mat depth;
//
//	rgb = RtabmapSlamControl::instance().GetCurrentVideoFrame();
//	depth = RtabmapSlamControl::instance().GetCurrentDepthFrame();
//
//
//	if (rgb.empty() || depth.empty())
//		return;
//
//	*gencloud = *RtabmapSlamControl::instance().generatePointCloud(depth, rgb);
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointIndices& removedindices = *new pcl::PointIndices;
//	float leafsize = 0.08;
//
//	downsampled = HUREL::Compton::RtabmapSlamControl::instance().downsamplePointCloud(gencloud, leafsize, removedindices);
//
//	open3d::geometry::PointCloud reconPointCloud = RtabmapSlamControl::instance().PclToOpen3d(gencloud);
//
//	//open3d::geometry::PointCloud downsampledPointCloud = HUREL::Compton::RtabmapSlamControl::instance().PclToOpen3d(downsampled);
//
//	open3d::geometry::PointCloud recontransPC;
//	recontransPC = HUREL::Compton::RtabmapSlamControl::instance().RTPointCloudTransposed(reconPointCloud, mDetectorTransformation);
//
//	//open3d::io::WritePointCloudOption option;
//	//open3d::io::WritePointCloudToPLY("C:\Users\Compton\source\repos\triplehoon\HUREL_Compton\HUREL Imager GUI\bin\x64\Release\net6.0-windows\shot.ply", reconPointCloud, option);
//
//	/*if (recontransPC.IsEmpty())
//		return;*/
//
//	HUREL::Compton::ReconPointCloud reconPCtrans = HUREL::Compton::ReconPointCloud(recontransPC);
//
//	/*HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(reconPointCloud);
//	open3d::geometry::PointCloud recontransPC;
//	open3d::geometry::PointCloud recontransPCFOVlim;
//	Eigen::MatrixXd fovchk(1, reconPC.points_.size());*/
//
//	double m = 1 + M2D / s2M;
//	double imagePlaneZ = s2M + M2D;
//	double dproj = det_W / (Mask_W / Mpix * m);
//	int pixelCount = static_cast<int>(round(dproj * resImprov));
//	int pixelCountcoded = 29;	// static_cast<int>(round(dproj));
//		
//	// should be fixed
//	//reconPC.imspaceLim(reconPointCloud, wFov, hFov, &reconPCFOVlim, &fovchk);
//	//reconPC.imspaceLim(reconPointCloud, 50, 50, mDetectorTransformation, &recontransPCFOVlim, &recontransPC, &fovchk);
//
//	int pixelLength = static_cast<int>(round(dproj * 1)); // pixel length of detector
//
//	Mat comptonImg(480, 848, CV_32S, Scalar(1)); //848*480
//	int32_t* comptonImgPtr = static_cast<int32_t*>(static_cast<void*>(comptonImg.data));
//	int codedImageCount = 0;
//	int comptonImageCount = 0;
//
//	Mat responseImg(pixelCountcoded, pixelCountcoded, CV_32S, Scalar(0));
//	__int32* responseImgPtr = static_cast<__int32*>(static_cast<void*>(responseImg.data));
//
//	double det_w_div2 = -det_W / 2;
//	double pixelSize = det_W / pixelCountcoded;
//
//	int32_t maxVal = 0;
//	Eigen::Vector3d maxValLoc;
//	maxValLoc[0] = 0;
//	maxValLoc[1] = 0;
//	maxValLoc[2] = 0;
//
//#pragma omp parallel for
//	for (int i = 0; i < data.size(); ++i)
//	{
//		ListModeData& lm = data[i];
//				
//		if (lm.Type == eInterationType::COMPTON)
//		{
//			if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 400 || lm.Scatter.InteractionEnergy < 50)
//			{
//				continue;
//			}
//
//			int pointCount = 0;
//			int dwsIndexCount = 0;
//
//			int width = 848;
//			int height = 480;
//			int indx = 0;
//			//for (int rows = 0; rows < height; ++rows)
//			//{
//			//	for (int cols = 0; cols < width; ++cols)
//			//	{
//			//		if (pointCount < 0 )
//			//		{
//			//			std::cerr << "Pont count is over the point cloud size" << std::endl;
//			//			break;
//			//		}
//			//		Eigen::Vector3d imgPoint;
//			//		//double FOVchk = fovchk(pointCount);
//			//		imgPoint[0] = reconPCtrans.points_[pointCount].x();
//			//		imgPoint[1] = reconPCtrans.points_[pointCount].y();
//			//		imgPoint[2] = reconPCtrans.points_[pointCount].z();
//
//			//		//pixelCount * (pixelCount - j - 1) + pixelCount - i - 1]
//			//		//comptonImgPtr[848 * (480 - cols - 1) + 848 - rows - 1] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
//			//		//comptonImgPtr[480 * rows + cols] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
//			//		//indx = rows * width + cols;
//			//		//indx = rows * height + cols;
//			//		//comptonImgPtr[indx] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
//			//		comptonImg.at< int32_t>(rows, cols) += ReconPointCloud::SimpleComptonBackprojectionTransformed(lm, imgPoint, 1);
//
//			//		if (maxVal < comptonImg.at< int32_t>(rows, cols))
//			//		{
//			//			maxVal = comptonImg.at< int32_t>(rows, cols);
//			//			maxValLoc = imgPoint;
//			//		}
//			//		//--pointCount;
//			//		++pointCount;
//			//	}
//			//}
//
//			//down
//			for (int rows = 0; rows < height; ++rows)
//			{
//				for (int cols = 0; cols < width; ++cols, ++pointCount)
//				{
//					int dwsIndex = (dwsIndexCount < removedindices.indices.size()) ? removedindices.indices[dwsIndexCount] : -1;
//
//					if (pointCount == dwsIndex)
//					{
//						++dwsIndexCount;
//						continue;
//					}
//
//					if (pointCount >= reconPCtrans.points_.size())
//					{
//						std::cerr << "Pont count is over the point cloud size" << std::endl;
//						break;
//					}
//
//					Eigen::Vector3d imgPoint;
//					//double FOVchk = fovchk(pointCount);
//					imgPoint[0] = reconPCtrans.points_[pointCount].x();
//					imgPoint[1] = reconPCtrans.points_[pointCount].y();
//					imgPoint[2] = reconPCtrans.points_[pointCount].z();
//
//					comptonImg.at< int32_t>(rows, cols) += ReconPointCloud::SimpleComptonBackprojectionTransformed(lm, imgPoint, 1);
//
//					if (maxVal < comptonImg.at< int32_t>(rows, cols))
//					{
//						maxVal = comptonImg.at< int32_t>(rows, cols);
//						maxValLoc = imgPoint;
//					}
//				}
//			}
//
//		}
//	}
//
//	cv::Mat nonFiltered = comptonImg;
//	cv::Mat Filtered;
//	nonFiltered.convertTo(nonFiltered, CV_32F);
//	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 2);
//	Filtered.convertTo(Filtered, CV_32S);
//	mComptonImage = Filtered;
//	// ------Compton imaging done---------------
//
//
//	mCodedImage = Filtered;
//	mHybridImage = Filtered;
//
//	// ------Coded imageing Start---------------
//	//
//
//	Mat scaleG;
//	cv::resize(CodedMaskMat(), scaleG, Size(Mpix, Mpix), 0, 0, INTER_NEAREST);
//	cv::rotate(scaleG, scaleG, cv::ROTATE_90_COUNTERCLOCKWISE); //lge
//
//	open3d::geometry::PointCloud codedTransPC3d = recontransPC;
//
//	Mat codedImg(480, 848, CV_32S, Scalar(0)); //848*480
//	int32_t* codedImgPtr = static_cast<int32_t*>(static_cast<void*>(codedImg.data));
//
//	int pcdDownCount = recontransPC.points_.size() * 0.04;
//
//	if (pcdDownCount == 0)
//		pcdDownCount = recontransPC.points_.size() > 0 ? recontransPC.points_.size() : 1;
//
//	int pcdDownInterval = recontransPC.points_.size() / pcdDownCount;
//
//	//open3d::geometry::PointCloud transPC3d = codedTransPC3d.Transform(data[0].DetectorTransformation.inverse());	//������ trans �� ����
//	//HUREL::Compton::ReconPointCloud transPC = HUREL::Compton::ReconPointCloud(transPC3d);
////
////#pragma omp parallel for
//	for (int i = 0; i < data.size(); ++i)
//	{
//		ListModeData& lm = data[i];
//
//		if (lm.Type == eInterationType::CODED)
//		{
//			codedImageCount++;
//			int ncount = 0;
//			//����� ���� transformation matrix�� ���� ���
//			if (i < data.size() - 1)
//			{
//				Eigen::Matrix4d curTrans = lm.DetectorTransformation;
//				Eigen::Matrix4d nextTrans = data[i + 1].DetectorTransformation;
//				for (int ii = 0; ii < 4; ii++)
//				{
//					for (int jj = 0; jj < 4; jj++)
//					{
//						if (abs(nextTrans(ii, jj) - curTrans(ii, jj)) <= 0.002)
//							ncount++;
//					}
//				}
//			}
//
//			//����� count
//			double& interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
//			double& interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];
//
//			int iX = findIndex(interactionPoseX, det_w_div2, pixelSize);
//			int iY = findIndex(interactionPoseY, det_w_div2, pixelSize);
//			if (iX >= 0 && iY >= 0 && iX < pixelCountcoded && iY < pixelCountcoded)
//			{
//				++responseImgPtr[pixelCountcoded * iY + iX];
//			}
//
//			if (ncount < 10)	//���� �����Ϳ� �ٸ� ��ġ���� ��ǥ�� ���� ��ȯ ���� �� �Ϸ� �� ������ ����
//			{
//				//point cloud transformation
//				//open3d::geometry::PointCloud transPC3d = codedTransPC3d.Transform(lm.DetectorTransformation.inverse());	//������ trans �� ���� //recontransPC
//				codedTransPC3d.Transform(lm.DetectorTransformation.inverse());	//������ trans �� ���� 
//				HUREL::Compton::ReconPointCloud transPC = HUREL::Compton::ReconPointCloud(codedTransPC3d);//recontransPC
//
//				//PDC Z ��� ���ϱ�
//				double zSum = 0.0;
//				for (int pci = 0; pci < pcdDownCount; pci++)
//				{
//					int nIndex = pci * pcdDownInterval;
//					zSum += transPC.points_[nIndex].z();
//				}
//
//				double zMean = zSum / pcdDownCount;
//
//				//LUT Z Index ���ϱ�
//				int zIndex = 0;
//				if (zMean < 1.5)
//					zIndex = 0;
//				else if (zMean >= 1.5 && zMean < 3)
//					zIndex = 1;
//				else if (zMean >= 3 && zMean < 4.5)
//					zIndex = 2;
//				else if (zMean >= 4.5 && zMean < 6)
//					zIndex = 3;
//				else
//					zIndex = 4;
//
//				Logger::Instance().InvokeLog("RadImage", "Z Index: " + std::to_string(zIndex) + ", Data : " + std::to_string(data.size()) + ", I : " + std::to_string(i)
//					+ " Coded No : " + std::to_string(codedImageCount), eLoggerType::INFO);
//				//zIndex = 1;
//
//				//����ȭ
//				Mat reconImg;
//				cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);
//				//cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);
//
//				int resize = pixelCountcoded * 3;
//				cv::resize(reconImg, reconImg, Size(resize, resize), 0, 0, INTER_NEAREST_EXACT);	//87*87
//
//				nonFiltered = reconImg;
//
//				nonFiltered.convertTo(nonFiltered, CV_32F);
//				cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
//				Filtered.convertTo(Filtered, CV_32S);
//				reconImg = Filtered;
//
//				//cv::imwrite("C:/Users/Compton/Downloads/reconImg.png", reconImg);
//
//				//reconImg < 0 then 0
//				for (int iii = 0; iii < resize; iii++)
//				{
//					for (int jjj = 0; jjj < resize; jjj++)
//					{
//						if (reconImg.at<int32_t>(iii, jjj) < 0)
//							reconImg.at<int32_t>(iii, jjj) = 0;
//					}
//				}
//
//				Logger::Instance().InvokeLog("RadImage", "1 image Start", eLoggerType::INFO);
//				for (int iii = 0; iii < resize; iii++)
//				{
//					std::string line;
//					for (int jjj = 0; jjj < resize; jjj++)
//					{
//						line += std::to_string(reconImg.at<int32_t>(iii, jjj)); line += ",";
//					}
//					Logger::Instance().InvokeLog("RadImage", line, eLoggerType::INFO);
//				}
//
//				Logger::Instance().InvokeLog("RadImage", "1 image End", eLoggerType::INFO);
//
//				//int32_t* reconImgPtr = static_cast<int32_t*>(static_cast<void*>(reconImg.data));
//
//				Mat angresponse_matched(91, 91, CV_32S, Scalar(0));	//���� ��ǥ ���� ��
//				//Mat angresponse_mask(91, 91, CV_8U, Scalar(1));	//���� ��ǥ ���� ��
//				//���� => ��
//				int nindexCount = 0;
////#pragma omp parallel for
//				for (int height = 0; height < 91; height++)//87
//				{
//					for (int width = 0; width < 91; width++)//87
//					{
//						//int nInex = height * resize + width;
//
//						int xIndex = TotalIndexbyPos[zIndex][nindexCount][0] + 45;
//						int yIndex = TotalIndexbyPos[zIndex][nindexCount][1] + 45;
//
//						/*if (TotalIndexbyPos[zIndex][nindexCount][0] == 0 && TotalIndexbyPos[zIndex][nindexCount][1] == 0)
//						{
//							nindexCount++;
//							continue;
//						}
//						else*/
//						{
//							//angresponse_matched.at<int32_t>(yIndex, xIndex) += reconImg.at<int32_t>(height, width);
//							angresponse_matched.at<int32_t>(yIndex, xIndex) += reconImg.at<int32_t>(width, height);
//							//angresponse_mask.at<int32_t>(yIndex, xIndex) = 0;
//							//angresponse_matched.at<int32_t>(yIndex, xIndex) += reconImgPtr[nindexCount];
//							nindexCount++;
//						}
//					}
//				}
//
//				Logger::Instance().InvokeLog("RadImage", "2 image Start", eLoggerType::INFO);
//				for (int iii = 0; iii < 91; iii++)
//				{
//					std::string line;
//					for (int jjj = 0; jjj < 81; jjj++)
//					{
//						line += std::to_string(angresponse_matched.at<int32_t>(iii, jjj)); line += ",";
//					}
//					Logger::Instance().InvokeLog("RadImage", line, eLoggerType::INFO);
//				}
//
//				Logger::Instance().InvokeLog("RadImage", "2 image End", eLoggerType::INFO);
//
//				/////////////////////////////////////////////
//				//fillmissing �Լ��� �̿��ؼ� ����, ���� ���� �� 2�� ����
//				/*cv::resize(angresponse_matched, angresponse_matched, Size(31, 31), 0, 0, INTER_NEAREST);
//				cv::resize(angresponse_matched, angresponse_matched, Size(91, 91), 0, 0, INTER_NEAREST);*/
//
//
//				/*Mat angresponse_sor(91, 91, CV_16U, Scalar(0));
//				angresponse_matched.convertTo(angresponse_sor, CV_16U);
//				Mat angresponse_result(91, 91, CV_16U, Scalar(0));
//				cv::inpaint(angresponse_sor, angresponse_mask, angresponse_result, 2, INPAINT_NS);
//
//				angresponse_result.convertTo(angresponse_matched, CV_32S);*/
//
//				//cv::imwrite("C:/Users/Compton/Downloads/angres.png", angresponse_matched);
//
//				//�� => ����Ʈ Ŭ����
//				//for (int pci = 0; pci < pcdDownCount; pci++)
//				//{
//				//	int nIndex = pci * pcdDownInterval;
//
//				//	//����(z, x, y = > x, y, z)
//				//	double azP = RAD2DEG(std::atan2(transPC3d.points_[nIndex].x(), transPC3d.points_[nIndex].z()));	//az = std::atan2(y, x); 
//				//	double polP = RAD2DEG(std::atan2(transPC3d.points_[nIndex].y(), std::sqrt(transPC3d.points_[nIndex].z() * transPC3d.points_[nIndex].z() + transPC3d.points_[nIndex].x() * transPC3d.points_[nIndex].x())));	//pol = std::atan2(z, std::sqrt(x * x + y * y));
//
//				//	int azi = round(azP);
//				//	int poli = round(polP);
//
//				//	if (abs(azi) > 45 || abs(poli) > 45)
//				//		continue;
//
//				//	double reconValue_CCBP = angresponse_matched.at<int32_t>(azi + 46, poli + 46);
//
//				//	if (reconValue_CCBP != 0)
//				//	{
//				//		codedImgPtr[nIndex] += reconValue_CCBP;
//				//	}
//				//}
//						
//				//down
//				int pointCount = 0;
//				int dwsIndexCount = 0;
//
//				int width = 848;
//				int height = 480;
////#pragma omp parallel for
//				for (int rows = 0; rows < height; ++rows)
//				{
//					for (int cols = 0; cols < width; ++cols, ++pointCount)
//					{
//						int dwsIndex = (dwsIndexCount < removedindices.indices.size()) ? removedindices.indices[dwsIndexCount] : -1;
//
//						if (pointCount == dwsIndex)
//						{
//							++dwsIndexCount;
//							continue;
//						}
//
//						if (pointCount >= reconPCtrans.points_.size())
//						{
//							std::cerr << "Pont count is over the point cloud size" << std::endl;
//							break;
//						}
//
//						double azP = RAD2DEG(std::atan2((transPC.points_[pointCount].x() * -1), transPC.points_[pointCount].z()));	//az = std::atan2(y, x); 
//						double polP = RAD2DEG(std::atan2(transPC.points_[pointCount].y(), std::sqrt(transPC.points_[pointCount].z() * transPC.points_[pointCount].z()
//										+ transPC.points_[pointCount].x() * transPC.points_[pointCount].x())));	//pol = std::atan2(z, std::sqrt(x * x + y * y));
//
//						int azi = round(azP);
//						int poli = round(polP);
//
//						if (abs(azi) > 45 || abs(poli) > 45)
//							continue;
//
//						double reconValue_CCBP = angresponse_matched.at<int32_t>(poli + 45, azi + 45);
//						//double reconValue_CCBP = angresponse_matched.at<int32_t>(azi + 45, poli + 45);
//
//						if (reconValue_CCBP != 0)
//						{
//							codedImg.at< int32_t>(rows, cols) += reconValue_CCBP;
//						}
//					}
//				}
//
//				/////////////////////////////////////////////////
//
//
//				//�ʱ�ȭ
//				/*for (int ri = 0; ri < pixelCountcoded; ri++)
//				{
//					for (int  ry = 0; ry < pixelCountcoded; ry++)
//					{
//						responseImg.at<int32_t>(ri, ry) = 0;
//					}
//				}*/
//
//				/*for (int ri = 0; ri < pixelCountcoded * pixelCountcoded; ri++)
//				{
//					responseImgPtr[ri] = 0;
//				}*/
//				Logger::Instance().InvokeLog("RadImage", "Debug Index: 5", eLoggerType::INFO);
//
//				//codedTransPC3d = transPC3d;
//			}
//		}
//	}
////
////	//���� ��� : codedImg
//	nonFiltered = codedImg;
//	nonFiltered.convertTo(nonFiltered, CV_32F);
//	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
//	Filtered.convertTo(Filtered, CV_32S);
//	mCodedImage = Filtered;
//
//	// ------Coded imageing done----------------
//
//	///spdlog::info("RadImage: Hybrid Max: " + std::to_string(maxVal));
//	//mListedListModeData = lmData;
//}

//20251208 INDOOR MODE
HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D, int maxValue)
{
	int datasize = data.size();

	if (datasize <= 0)
	{
		//240126 : ����ȭ ���� �� ���õ� ������ ���� ��� ���� clear
		mComptonImage = Mat::zeros(480, 848, CV_32S);
		mCodedImage = Mat::zeros(480, 848, CV_32S);
		mHybridImage = Mat::zeros(480, 848, CV_32S);
		return;
	}

	SetIndexPos();

	mDetectorTransformation = data[datasize - 1].DetectorTransformation;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gencloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//*gencloud = *RtabmapSlamControl::instance().generatePointCloud();

	cv::Mat rgb;
	cv::Mat depth;

	/////////////////240315
	//RtabmapSlamControl::instance().SetCurrentFrame();
	/////////////////

	rgb = RtabmapSlamControl::instance().GetCurrentVideoFrame1(true);	//240315
	depth = RtabmapSlamControl::instance().GetCurrentDepthFrame1();


	if (rgb.empty() || depth.empty())
		return;

	*gencloud = *RtabmapSlamControl::instance().generatePointCloud(depth, rgb);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointIndices& removedindices = *new pcl::PointIndices;
	pcl::IndicesConstPtr indicesPtr(new std::vector<int>);
	float leafsize = 0.08;

	//downsampled = HUREL::Compton::RtabmapSlamControl::instance().downsamplePointCloud(gencloud, leafsize, removedindices);

	open3d::geometry::PointCloud reconPointCloud = RtabmapSlamControl::instance().PclToOpen3d(gencloud);

	//open3d::geometry::PointCloud downsampledPointCloud = HUREL::Compton::RtabmapSlamControl::instance().PclToOpen3d(downsampled);
	//Logger::Instance().InvokeLog("RadImage", "downsampledPointCloud size : " + std::to_string(indicesPtr->size()), eLoggerType::INFO);

	open3d::geometry::PointCloud recontransPC;
	recontransPC = HUREL::Compton::RtabmapSlamControl::instance().RTPointCloudTransposed(reconPointCloud, mDetectorTransformation);

	HUREL::Compton::ReconPointCloud reconPCtrans = HUREL::Compton::ReconPointCloud(recontransPC);

	double m = 1 + M2D / s2M;
	double imagePlaneZ = s2M + M2D;
	double dproj = det_W / (Mask_W / Mpix * m);
	int pixelCount = static_cast<int>(round(dproj * resImprov));
	int pixelCountcoded = pixelCount;// static_cast<int>(round(dproj));

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

	int dwsIndex = 1;
#pragma omp parallel for
	for (int i = 0; i < data.size(); ++i)
	{
		ListModeData& lm = data[i];

		if (lm.Type == eInterationType::COMPTON)
		{
			if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 100) //400, 50 -> 100, 10
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
			//
			//		//pixelCount * (pixelCount - j - 1) + pixelCount - i - 1]
			//		//comptonImgPtr[848 * (480 - cols - 1) + 848 - rows - 1] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
			//		//comptonImgPtr[480 * rows + cols] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
			//		//indx = rows * width + cols;
			//		//indx = rows * height + cols;
			//		//comptonImgPtr[indx] += ReconPointCloud::SimpleComptonBackprojection(lm, imgPoint, 1);
			//		comptonImg.at< int32_t>(rows, cols) += ReconPointCloud::SimpleComptonBackprojectionTransformed(lm, imgPoint, 1);
			//
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
			//for (int rows = 0; rows < height; ++rows)
			//{
			//	for (int cols = 0; cols < width; ++cols, ++pointCount)
			//	{
			//		int dwsIndex = (dwsIndexCount < removedindices.indices.size()) ? removedindices.indices[dwsIndexCount] : -1;
			//
			//		if (pointCount == dwsIndex)
			//		{
			//			++dwsIndexCount;
			//			continue;
			//		}
			//
			//		if (pointCount >= reconPCtrans.points_.size())
			//		{
			//			std::cerr << "Pont count is over the point cloud size" << std::endl;
			//			break;
			//		}
			//
			//		Eigen::Vector3d imgPoint;
			//		//double FOVchk = fovchk(pointCount);
			//		imgPoint[0] = reconPCtrans.points_[pointCount].x();
			//		imgPoint[1] = reconPCtrans.points_[pointCount].y();
			//		imgPoint[2] = reconPCtrans.points_[pointCount].z();
			//
			//		comptonImg.at< int32_t>(rows, cols) += ReconPointCloud::SimpleComptonBackprojectionTransformed(lm, imgPoint, 1);
			//
			//		if (maxVal < comptonImg.at< int32_t>(rows, cols))
			//		{
			//			maxVal = comptonImg.at< int32_t>(rows, cols);
			//			maxValLoc = imgPoint;
			//		}
			//	}
			//}

			for (int rows = 0; rows < height; ++rows)
			{
				for (int cols = 0; cols < width; ++cols, ++pointCount)
				{
					if (pointCount % dwsIndex == 0)
					{
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
						//240227 lge
						//comptonImg.at< int32_t>(rows, cols) += ReconPointCloud::SqComptonBackprojectionTransformed(lm, imgPoint);
						if (maxVal < comptonImg.at< int32_t>(rows, cols))
						{
							maxVal = comptonImg.at< int32_t>(rows, cols);
							maxValLoc = imgPoint;
						}
					}
				}
			}
		}
		else if (lm.Type == eInterationType::CODED)
			codedImageCount++;
	}

	for (int rows = 0; rows < 480; ++rows)
	{
		for (int cols = 0; cols < 848; ++cols)
		{
			if (comptonImg.at< int32_t>(rows, cols) <= 0)
				comptonImg.at< int32_t>(rows, cols) = 1;
		}
	}


	cv::Mat nonFiltered = comptonImg;
	cv::Mat Filtered;
	nonFiltered.convertTo(nonFiltered, CV_32F);
//	cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 2,2);
	cv::GaussianBlur(nonFiltered, Filtered, Size(0, 0), 4, 4);
	Filtered.convertTo(Filtered, CV_32S);
	mComptonImage = Filtered;
	//// ------Compton imaging done---------------

	//mHybridImage = Filtered;

	// ------Coded imageing Start---------------
	//1. List Mode Data�� ��ȸ �ϸ鼭 Trans ���� �ٸ����� �����Ѵ�.(�� ���� ���� <= 0.002 �� ��Ұ� 10�� �̻��̸� ������ ��ġ��� �����Ѵ�.
	//

	open3d::geometry::PointCloud codedTransPC3d = recontransPC;

	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(Mpix * resImprov, Mpix * resImprov), 0, 0, INTER_NEAREST);
	cv::rotate(scaleG, scaleG, cv::ROTATE_90_COUNTERCLOCKWISE); //lge

	Mat codedImg(480, 848, CV_32S, Scalar(0)); //848*480
	int32_t* codedImgPtr = static_cast<int32_t*>(static_cast<void*>(codedImg.data));

	int pcdDownCount = recontransPC.points_.size() * 0.04;

	if (pcdDownCount == 0)
		pcdDownCount = recontransPC.points_.size() > 0 ? recontransPC.points_.size() : 1;

	int pcdDownInterval = recontransPC.points_.size() / pcdDownCount;

#pragma region background count
	////231121-1 sbkwon
	//int nbgdCountX = 5;
	//Eigen::MatrixXi bgdCountY1 = Eigen::MatrixXi::Zero(pixelCountcoded, pixelCountcoded);
	//Eigen::MatrixXi bgdCountY2 = Eigen::MatrixXi::Zero(pixelCountcoded, pixelCountcoded);
#pragma endregion

	int tempcodecount = 0;
	//#pragma omp parallel for
	for (int i = 0; i < data.size(); ++i)
	{
		ListModeData& lm = data[i];

		if (lm.Type == eInterationType::CODED)
		{
			tempcodecount++;

			double& interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
			double& interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];

			int iX = findIndex(interactionPoseX, det_w_div2, pixelSize);
			int iY = findIndex(interactionPoseY, det_w_div2, pixelSize);
			if (iX >= 0 && iY >= 0 && iX < pixelCountcoded && iY < pixelCountcoded)
			{
				++responseImgPtr[pixelCountcoded * iY + iX];

#pragma region background count
				////231121-1 sbkwon
				////background count ���� : ListModeData EnergyCheck�� �̿��Ͽ� Min / Max count �Ѵ�.
				//double dIntervalE = (lm.EnergyCheck.maxE - lm.EnergyCheck.minE) / nbgdCountX;
				//double y1Max = lm.EnergyCheck.minE + dIntervalE;
				//double y2Min = lm.EnergyCheck.maxE - dIntervalE;
				//
				//if (lm.EnergyCheck.minE < lm.Scatter.InteractionEnergy && lm.Scatter.InteractionEnergy <= y1Max)// Y1 range (ECheck.minE, y1Max)
				//{
				//	bgdCountY1(iY, iX)++;
				//}
				//
				//if (y2Min <= lm.Scatter.InteractionEnergy && lm.Scatter.InteractionEnergy < lm.EnergyCheck.maxE)// Y1 range (y2Min, ECheck.maxE)
				//{
				//	bgdCountY2(iY, iX)++;
				//}
#pragma endregion
			}

			int ncount = 0;
			//����� ���� transformation matrix�� ���� ���
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

			if (ncount < 8 || tempcodecount == codedImageCount)	//���� �����Ϳ� �ٸ� ��ġ���� ��ǥ�� ���� ��ȯ ���� �� �Ϸ� �� ������ ����
			{
				//codedTransPC3d.Transform(mDetectorTransformation.inverse());	//������ trans �� ����
				codedTransPC3d = codedTransPC3d.Transform(lm.DetectorTransformation.inverse());	//������ trans �� ����
				HUREL::Compton::ReconPointCloud transPC = HUREL::Compton::ReconPointCloud(codedTransPC3d);

				//PDC Z ��� ���ϱ�
				double zSum = 0.0;
				int nDownCount = 0;
				for (int pci = 0; pci < transPC.points_.size(); pci++)
				{
					if (pci % dwsIndex == 0)
					{
						zSum += transPC.points_[pci].z();
						nDownCount++;
					}
				}

				double zMean = zSum / nDownCount;

				//LUT Z Index ���ϱ�
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


#pragma region background count
				////231121-1 sbkwon
				////background count ����(��ü count ��) : ���� - (Y1 + Y2) * nbgdCountX / 2
//				double nbgdCountX_2 = nbgdCountX / 2;
////#pragma omp parallel for
//				for (int iY = 0; iY < pixelCountcoded; iY++)
//				{
//					for (int iX = 0; iX < pixelCountcoded; iX++)
//					{
//						if (bgdCountY1(iY, iX) > 0 || bgdCountY2(iY, iX) > 0)	// ���� ���� ���
//						{
//							int nCount = floor((bgdCountY1(iY, iX) + bgdCountY2(iY, iX)) * nbgdCountX_2);	//ceil
//							responseImg.at<int32_t>(iX, iY) -= nCount;
//						}
//
//						//Logger::Instance().InvokeLog("RadImage", "iY : " + std::to_string(iY) + ", iX : " + std::to_string(iX) + " : " + std::to_string(bgdCountY1(iY, iX)) + " : " + std::to_string(bgdCountY2(iY, iX)), eLoggerType::INFO);
//					}
//				}
#pragma endregion

				//����ȭ
				Mat reconImg;
				cv::filter2D(responseImg, reconImg, CV_32S, scaleG, Point(-1, -1), 0.0, BORDER_CONSTANT);


				/*std::ofstream savescaleG, savegaussian;
				savescaleG.open("E:/scaleG.csv");
				savegaussian.open("E:/gaussian.csv");

				if (savescaleG.is_open())
				{
					for (int i = 0; i < pixelCountcoded; i++)
					{
						std::string line = "";
						for (int j = 0; j < pixelCountcoded; j++)
						{
							line += std::to_string(reconImg.at<int32_t>(i, j)); line += ",";
						}

						savescaleG << line << std::endl;
					}

					savescaleG.close();
				}*/

				//cv::resize(reconImg, reconImg, Size(pixelCount, pixelCount), 0, 0, INTER_NEAREST_EXACT);	//resimprov ����

				//20240304 lge
				//nonFiltered = reconImg;
				//nonFiltered.convertTo(nonFiltered, CV_32F);
				//cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 5);
				//Filtered.convertTo(Filtered, CV_32S);
				//reconImg = Filtered;


				int resize = 42;
				cv::resize(reconImg, reconImg, Size(resize, resize), 0, 0, INTER_NEAREST_EXACT);


				/*if (savegaussian.is_open())
				{
					for (int i = 0; i < resize; i++)
					{
						std::string line = "";
						for (int j = 0; j < resize; j++)
						{
							line += std::to_string(reconImg.at<int32_t>(i, j)); line += ",";
						}

						savegaussian << line << std::endl;
					}

					savegaussian.close();
				}*/


				for (int iY = 0; iY < resize; iY++)
				{
					for (int iX = 0; iX < resize; iX++)
					{
						if (reconImg.at<int32_t>(iY, iX) <= 0)
							reconImg.at<int32_t>(iY, iX) = 1;	//231221 : matlab code ����
					}
				}

				Eigen::MatrixXd angresponse_Eigen = Eigen::MatrixXd::Zero(91, 91);

				//���� => ��
#pragma omp parallel for
				for (int height = 0; height < resize; height++)
				{
					for (int width = 0; width < resize; width++)
					{
						if (reconImg.at<int32_t>(width, height) == 0)
							continue;

						//int nInex = height + resize * width;
						int nInex = height * resize + width;
						int xIndex = TotalIndexbyPos[zIndex][nInex][0] + 45;
						int yIndex = TotalIndexbyPos[zIndex][nInex][1] + 45;

						//Logger::Instance().InvokeLog("RadImage", std::to_string(xIndex) + ", " + std::to_string(yIndex) + ", " + std::to_string(nInex) + ", " + std::to_string(height) + ", " + std::to_string(width), eLoggerType::INFO);

						if (TotalIndexbyPos[zIndex][nInex][0] == -100 && TotalIndexbyPos[zIndex][nInex][1] == -100)
							continue;
						else
						{
							//angresponse_matched.at<int32_t>(yIndex, xIndex) += reconImg.at<int32_t>(width, height);
							angresponse_Eigen(yIndex, xIndex) += reconImg.at<int32_t>(width, height);
							//angresponse_Eigen(xIndex, yIndex) += reconImg.at<int32_t>(width, height);
						}
					}
				}

				
				//col ���� : Y�� ����
				for (int i = 0; i < 91; i++)
				{
					Eigen::ArrayXd temp = angresponse_Eigen.col(i).array();

					//mean(), maxCoeff()
					/*double tempLimit = temp.maxCoeff() * 0.8;
					fillmissing(temp, tempLimit);*/

					simplifiedFillMissing(temp);

					for (int j = 0; j < 91; j++)
					{
						angresponse_Eigen(j, i) = temp(j);
					}
				}

				for (int i = 0; i < 91; i++)
				{
					Eigen::ArrayXd temp = angresponse_Eigen.row(i).array();

					//mean(), maxCoeff()
					/*double tempLimit = temp.maxCoeff() * 0.8;
					fillmissing(temp, tempLimit);*/

					simplifiedFillMissing(temp);

					for (int j = 0; j < 91; j++)
					{
						angresponse_Eigen(i, j) = temp(j);
					}
				}

				//Gauss filter after simply fill missing 20240304
				cv::Mat gauss_Eigen  = Mat::zeros(91, 91, CV_32S);

				for (int k = 0; k < 91; k++)
				{
					for (int t = 0; t < 91; t++)
					{
						gauss_Eigen.at<int32_t>(k, t) = angresponse_Eigen(k, t);
					}
				}
				
				nonFiltered = gauss_Eigen;
				nonFiltered.convertTo(nonFiltered, CV_32F);
				cv::GaussianBlur(nonFiltered, Filtered, Size(7, 7), 3, 3);
				Filtered.convertTo(Filtered, CV_32S);
				gauss_Eigen = Filtered;

				for (int p = 0; p < 91; p++)
				{
					for (int q = 0; q < 91; q++)
					{
						angresponse_Eigen(p, q) = gauss_Eigen.at<int32_t>(p, q);
					}
				}

#pragma region Origin
				/*
				for (int i = 0; i < 91; i++)
				{
					Eigen::ArrayXd temp = angresponse_Eigen.col(i).array();

				//	//mean(), maxCoeff()
				//	//double tempLimit = temp.maxCoeff() * 0.8;
					fillmissing(temp);

					for (int j = 0; j < 91; j++)
					{
						angresponse_Eigen(j, i) = temp(j);//j
					}
				}
				*/

				//for	(int i = 0; i < 91; i++)
				//{
				//	Eigen::ArrayXd temp = angresponse_Eigen.row(i).array();
				//
				//	//mean(), maxCoeff()
				//	//double tempLimit = temp.maxCoeff() * 0.8;
				//	fillmissing(temp);
				//
				//	for (int j = 0; j < 91; j++)
				//	{
				//		angresponse_Eigen(i, j) = temp(j);
				//	}
				//}
#pragma endregion

				/*for (int i = 0; i < 91; i++)
				{
					for (int j = 0; j < 91; j++)
					{
						if (angresponse_Eigen(i, j) < 0)
							angresponse_Eigen(i, j) = 0;
					}
				}*/

				//file save
				/*if (saveafterfill.is_open())
				{
					for (int i = 0; i < 91; i++)
					{
						std::string line = "";
						for (int j = 0; j < 91; j++)
						{
							line += std::to_string(angresponse_Eigen(i, j)); line += ",";
						}

						saveafterfill << line << std::endl;
					}

					saveafterfill.close();
				}*/

				//angresponse_Eigen.colwise().reverse();//colwize : ����
				//angresponse_Eigen.rowwise().reverse();// rowwize : �¿�

				//�� => ����Ʈ Ŭ����	
				//down
				int pointCount = 0;
				int dwsIndexCount = 0;

				int width = 848;
				int height = 480;
#pragma region Origin
				//
				//#pragma omp parallel for
				//				for (int rows = 0; rows < height; ++rows)
				//				{
				//					for (int cols = 0; cols < width; ++cols/*, ++pointCount*/)
				//					{
				//						//pointCount = rows + width * cols;
				//						pointCount = rows * width + cols;
				//						int dwsIndex = (dwsIndexCount < removedindices.indices.size()) ? removedindices.indices[dwsIndexCount] : -1;
				//
				//						if (pointCount == dwsIndex)
				//						{
				//							Logger::Instance().InvokeLog("RadImage", std::to_string(pointCount) + " : " + std::to_string(dwsIndexCount), eLoggerType::INFO);
				//							++dwsIndexCount;
				//							continue;
				//						}
				//
				//						if (pointCount >= transPC.points_.size())
				//						{
				//							std::cerr << "Pont count is over the point cloud size" << std::endl;
				//							break;
				//						}
				//
				//						double azP = RAD2DEG(std::atan2((transPC.points_[pointCount].x() * -1), transPC.points_[pointCount].z()));	//az = std::atan2(y, x); 
				//						double polP = RAD2DEG(std::atan2((transPC.points_[pointCount].y() * -1), std::sqrt(transPC.points_[pointCount].z() * transPC.points_[pointCount].z()
				//							+ transPC.points_[pointCount].x() * transPC.points_[pointCount].x())));	//pol = std::atan2(z, std::sqrt(x * x + y * y));
				//
				//						int azi = round(azP);
				//						int poli = round(polP);
				//						//poli = (-1) * poli;
				//						//Logger::Instance().InvokeLog("RadImage", std::to_string(pointCount) + " : " + std::to_string(azi) + ", " + std::to_string(poli), eLoggerType::INFO);
				//
				//						if (abs(azi) > 45 || abs(poli) > 45)
				//							continue;
				//
				//						//double reconValue_CCBP = angresponse_Eigen(azi + 45, poli + 45);
				//						double reconValue_CCBP = angresponse_Eigen(poli + 45, azi + 45);
				//
				//						if (reconValue_CCBP != 0)
				//						{
				//							codedImg.at< int32_t>(rows, cols) += reconValue_CCBP;
				//						}
				//					}
				//				}
#pragma endregion

#pragma region Down
//				//down
#pragma omp parallel for
				for (int rows = 0; rows < height; ++rows)
				{
					int nRowIndex = rows * width;
					for (int cols = 0; cols < width; ++cols)
					{
						pointCount = nRowIndex + cols;
						if (pointCount % dwsIndex == 0)
						{
							if (pointCount >= transPC.points_.size())
							{
								std::cerr << "Pont count is over the point cloud size" << std::endl;
								break;
							}

							double azP = RAD2DEG(std::atan2((transPC.points_[pointCount].x() * -1), transPC.points_[pointCount].z()));	//az = std::atan2(y, x); 
							double polP = RAD2DEG(std::atan2((transPC.points_[pointCount].y() * -1), std::sqrt(transPC.points_[pointCount].z() * transPC.points_[pointCount].z()
								+ transPC.points_[pointCount].x() * transPC.points_[pointCount].x())));	//pol = std::atan2(z, std::sqrt(x * x + y * y));

							int azi = round(azP);
							int poli = round(polP);

							//Logger::Instance().InvokeLog("RadImage", std::to_string(pointCount) + " : " + std::to_string(azi) + ", " + std::to_string(poli), eLoggerType::INFO);

							if (abs(azi) > 45 || abs(poli) > 45)
								continue;

							//double reconValue_CCBP = angresponse_Eigen(azi + 45, poli + 45);
							double reconValue_CCBP = angresponse_Eigen(poli + 45, azi + 45);

							if (reconValue_CCBP != 0)
							{
								codedImg.at< int32_t>(rows, cols) += reconValue_CCBP;
							}
						}
					}
				}
				for (int rows = 0; rows < height; ++rows)
				{
					for (int cols = 0; cols < width; ++cols)
					{
						if (codedImg.at< int32_t>(rows, cols) <= 0)
							codedImg.at< int32_t>(rows, cols) = 1;
					}
				}

#pragma endregion

				//�ʱ�ȭ
				for (int row = 0; row < pixelCountcoded; row++)
				{
					for (int col = 0; col < pixelCountcoded; col++)
					{
						responseImg.at<int32_t>(row, col) = 0;
					}
				}
			}
		}
	}

	
	nonFiltered = codedImg;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 3,3);
	Filtered.convertTo(Filtered, CV_32S);

	mCodedImage = Filtered;

	//cv::flip(mCodedImage, mCodedImage, 0);

	//mCodedImage = codedImg;

	// ------Coded imageing done----------------

	Mat mCodedImagenorm;
	Mat mComptonImagenorm;
	cv::normalize(codedImg, mCodedImagenorm, 0, 255, cv::NORM_MINMAX);
	cv::normalize(comptonImg, mComptonImagenorm, 0, 255, cv::NORM_MINMAX);
	mHybridImage = mCodedImagenorm.mul(mComptonImagenorm);
	//mHybridImage = mCodedImage.mul(mComptonImage);

	nonFiltered = mHybridImage;
	nonFiltered.convertTo(nonFiltered, CV_32F);
	cv::GaussianBlur(nonFiltered, Filtered, Size(15, 15), 3,3);
	Filtered.convertTo(Filtered, CV_32S);
	mHybridImage = Filtered;

	//coded, compton : max value 
	double maxValueCoded = 0;
	double maxValueCompton = 0;
	double maxValueHybrid = 0;

	for (int row = 0; row < 480; row++)
	{
		for (int col = 0; col < 848; col++)
		{
			if (mHybridImage.at<int32_t>(row, col) > maxValueHybrid)
			{
				maxValueHybrid = mHybridImage.at<int32_t>(row, col);
			}
			//240122 : coded
			if (mCodedImage.at<int32_t>(row, col) > maxValueCoded)
			{
				maxValueCoded = mCodedImage.at<int32_t>(row, col);
			}
			//240122 : compton
			if (mComptonImage.at<int32_t>(row, col) > maxValueCompton)
			{
				maxValueCompton = mComptonImage.at<int32_t>(row, col);
			}
		}
	}
	if (maxValueHybrid <= maxValue)
	{
		for (int row = 0; row < 480; row++)
		{
			for (int col = 0; col < 848; col++)
			{
				mHybridImage.at<int32_t>(row, col) = 0;
			}
		}
	}
	//240122 : coded
	if (maxValueCoded <= maxValue)
	{
		for (int row = 0; row < 480; row++)
		{
			for (int col = 0; col < 848; col++)
			{
				mCodedImage.at<int32_t>(row, col) = 0;
			}
		}
	}
	//240122 : compton 
	if (maxValueCompton <= maxValue)
	{
		for (int row = 0; row < 480; row++)
		{
			for (int col = 0; col < 848; col++)
			{
				mComptonImage.at<int32_t>(row, col) = 0;
			}
		}
	}
}

//231113-1 sbkwon
HUREL::Compton::RadiationImage::RadiationImage()
{
	SetIndexPos();
}

int HUREL::Compton::RadiationImage::nCountMat = 0;	//test

//231109-1 sbkwon
int HUREL::Compton::RadiationImage::TotalIndexbyPos[5][1764][2] = { -100, };

void HUREL::Compton::RadiationImage::SetIndexPos()
{
	static bool bInit = false;

	if (bInit)
		return;

	for (int z = 0; z < 5; z++)
	{
		for (int i = 0; i < 1764; i++)
		{
			TotalIndexbyPos[z][i][0] = -100;
			TotalIndexbyPos[z][i][1] = -100;
		}
	}

	//Z = 1, 2, 4, 5, 6 m

	//double m = 1 + M2D / S2M;	//S2M : 1m�ϰ��
	//double reconPlaneWidth = S2M / M2D * Det_W;	//source ��ġ������ ���� ������
	//double dproj = Det_W / (Mask_W / Mpix * m); // projection mask to Detector pixel Length(mm)
	//int pixelCount = static_cast<int>(round(dproj));

	int pixelCount = 42;

	//����ǥ ����
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

	double detectPos[42] = { 0.0, };

	for (int i = 0; i < pixelCount; i++)
	{
		detectPos[i] = minWidth + interval * i;
	}

	double ZPos[5] = { 1.0, 2.0, 3.0, 5.0, 7.0 };

#pragma omp parallel for
	for (int i = 0; i < 5; i++)
	{
		double z = ZPos[i];
		double imagePlaneZ_pow2 = z * z;

		double s2m = z - M2D;

		double CAxis = s2m / M2D;

		//���� ���� -> ����ǥ ���� ��ȯ  : ���� (z, x, y => x, y, z)
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

				if (minValue <= 0.2) //0.25 -> 0.3 20240304 lge
				{
					//int nIndex = w + 87 * h;
					int nIndex = w * 42 + h;
					TotalIndexbyPos[i][nIndex][0] = SphericalAz(minIndexX, minIndexY);
					TotalIndexbyPos[i][nIndex][1] = SphericalPol(minIndexX, minIndexY);
				}
			}
		}
	}


	/*/// <summary>
	std::ofstream saveaz, savepol;
	saveaz.open("C:/Users/Compton/Downloads/angres_az.csv");
	savepol.open("C:/Users/Compton/Downloads/angres_pol.csv");

	if (saveaz.is_open())
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
	//
	//	std::ofstream saveaz;
	//	saveaz.open("E:/angres_7569_" + std::to_string(zindex) + ".csv");
	//
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
	//
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

//231120 sbkwon
void HUREL::Compton::RadiationImage::fillmissing(Eigen::ArrayXd& arr, double dlimit)
{
	int nMax = arr.size() - 2;

	//�׵θ� ������ ������� �ʴ´�.
	for (int i = 1; i < nMax; i++)	//for (int i = 0; i < nCnt; i++)
	{
		if (arr(i) == 0)
		{
			double left = arr(i - 1);	//left = i > 0 ? arr(i - 1) : 0;
			double right = arr(i + 1);	//right = i < nMax ? arr(i + 1) : 0;

			if (left != 0 && right != 0)
				arr(i) = left + (right - left) * 0.5;
			else if (left == 0)
			{
				if (right == 0) //�ΰ� ã��
				{
					int nIndex1 = findRight(arr, i + 2);
					if (nIndex1 == 0)	//��� 0�� ���
						break;
					else
					{
						int nIndex2 = findRight(arr, nIndex1 + 1);
						if (nIndex2 == 0)	//�ϳ��� ���� ���
						{
							//i = fillmissingFront(arr, i, arr(nIndex1), 0, dlimit); //lge
						}
						else
						{
							double dInterval = (arr(nIndex2) - arr(nIndex1)) / (nIndex1 - nIndex2);
							i = fillmissingBack(arr, nIndex1 - 1, arr(nIndex1), dInterval, dlimit);
						}
					}
				}
				else // �ϳ� ã�� : i == 1 �ϰ�츸 �߻�
				{
					i = fillmissing(arr, i, right, i + 1, dlimit);
				}
			}
			else if (left != 0)	//������ �ϳ� ã��
			{
				i = fillmissing(arr, i, left, i - 1, dlimit);
			}
		}
	}
}

//231120 sbkwon
int HUREL::Compton::RadiationImage::fillmissing(Eigen::ArrayXd& arr, int nCur, double base, int baseInd, double dlimit)
{
	int nIndex = findRight(arr, nCur + 2);

	if (nIndex == 0)	//�����ʿ� ���� ���� ���
	{
		if (nCur < baseInd)
			arr(nCur) = base;
		else
		{
			if (nCur == 1)
				return fillmissingFront(arr, nCur, base, 0.0, dlimit);	//���� ������ ä���
			else
			{
				double dInterval = (base - arr(nCur - 2));
				return fillmissingFront(arr, nCur, base, dInterval, dlimit);
			}
		}
	}
	else
	{
		double dInterval = (arr(nIndex) - base) / (nIndex - baseInd);
		return fillmissingFront(arr, nCur, base, dInterval, dlimit);
	}

	return arr.size();
}

//231120 sbkwon
int HUREL::Compton::RadiationImage::fillmissingFront(Eigen::ArrayXd& arr, int nCur, double base, double interval, double dlimit)
{
	for (int i = nCur; i < arr.size(); i++)
	{
		if (arr(i) != 0)
			return i;

		double temp = base + interval;
		arr(i) = temp > dlimit ? dlimit : temp;
		base += interval;
	}

	return arr.size();
}

//231120 sbkwon
int HUREL::Compton::RadiationImage::fillmissingBack(Eigen::ArrayXd& arr, int nCur, double base, double interval, double dlimit)
{
	for (int i = nCur; i > 0; i--)
	{
		if (arr(i) != 0)
			return nCur;

		double temp = base + interval;
		arr(i) = temp > dlimit ? dlimit : temp;
		base += interval;
	}

	return nCur;
}

//231120 sbkwon
int HUREL::Compton::RadiationImage::findRight(Eigen::ArrayXd& arr, int nIndex)
{
	if (arr.size() <= nIndex)
		return 0;

	for (int i = nIndex; i < arr.size(); i++)
	{
		if (arr(i) > 0)
		{
			return i;
		}
	}

	return 0;	//Not Find
}

void HUREL::Compton::RadiationImage::simplifiedFillMissing(Eigen::ArrayXd& arr)
{
	int nMax = arr.size() - 2;

	for (int i = 1; i < nMax; i++)	//���۰� ���� �˻����� �ʴ´�.
	{
		if (arr(i) == 0)	//Nan
		{
			double left = arr(i - 1);	//left = i > 0 ? arr(i - 1) : 0;
			double right = arr(i + 1);	//right = i < nMax ? arr(i + 1) : 0;

			if (left != 0 && right != 0)
				arr(i) = left + (right - left) * 0.5;	//�߰��� ä���
			else if (left != 0)
			{	//���翡�� �����ʿ� ���� �����ϴ��� ã�´�.
				int nIndex = findRight(arr, i + 2);	//���簪 : i, right : i + 1, �˻� : i + 2

				if (nIndex == 0)
					break;	//�����ʿ� ���� ���� ��� ����
				else
				{
					//���� �ִ� �� ���� ä�� �ֱ�
					interp1Linear(arr, i - 1, nIndex);
					i = nIndex;
				}
			}
			//else : ���ʿ� ���� ���� ��츸, ���� ��� ä���� �ʴ´�
		}
	}
}

//leftIndex : Nan ���� ���� �ִ� index
//rightIndex : Nan ������ ���� �ִ� index
//leftIndex - rightIndex ���� Nan ���� ��� ä���.
void HUREL::Compton::RadiationImage::interp1Linear(Eigen::ArrayXd& arr, int leftIndex, int rightIndex)
{
	double left = arr(leftIndex);
	double right = arr(rightIndex);

	double addValue = (left - right) / (leftIndex - rightIndex);

	for (int i = leftIndex + 1; i < rightIndex; i++)
	{
		if (arr(i) != 0)	//Ȥ�� ����
			continue;

		double temp = left + addValue;
		arr(i) = temp;
		left += addValue;
	}
}