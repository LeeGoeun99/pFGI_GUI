#include "LahgiControl.h"
#include <future>
#include <mutex>
#include <open3d/visualization/utility/DrawGeometry.h>
#include "Recon3dPostProcess.h"	//2404 : MLEM
#include <fstream>
#include <filesystem>

static std::mutex mListModeDataMutex;
static std::mutex mResetListModeDataMutex;
static std::mutex mListModeImageMutex;
static std::mutex mResetEnergySpectrumMutex;
static std::mutex mRListEnergyTimeDataMutex;	//231123 sbkwon

using namespace std;
using namespace HUREL;
using namespace Compton;


//230911 sbkwon : Energy check �߰� - ���� ���� �з�
ListModeData HUREL::Compton::LahgiControl::MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double& scatterEnergy, double& absorberEnergy, Eigen::Matrix4d& transformation, std::chrono::milliseconds& timeInMili, sEnergyCheck echk)
{
	InteractionData scatter;
	InteractionData absorber;
	ListModeData listmodeData;

	if (!isnan(scatterEnergy))
	{
		scatter.InteractionEnergy = scatterEnergy;
	}
	scatter.RelativeInteractionPoint = scatterPoint;
	scatter.TransformedInteractionPoint = transformation * scatterPoint;

	if (!isnan(absorberEnergy))
	{
		absorber.InteractionEnergy = absorberEnergy;
	}
	if (iType == eInterationType::COMPTON)
	{

		absorber.RelativeInteractionPoint = absorberPoint;
		absorber.TransformedInteractionPoint = transformation * absorberPoint;
	}



	listmodeData.Type = iType;
	listmodeData.Scatter = scatter;
	listmodeData.Absorber = absorber;
	listmodeData.DetectorTransformation = transformation;
	listmodeData.InteractionTimeInMili = timeInMili;
	listmodeData.EnergyCheck = echk;	//230911 sbkwon : Energy check �߰� - ���� ���� �з�

	return listmodeData;
}

ListModeData HUREL::Compton::LahgiControl::MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double& scatterEnergy, double& absorberEnergy, Eigen::Matrix4d& transformation)
{
	InteractionData scatter;
	InteractionData absorber;
	ListModeData listmodeData;

	if (!isnan(scatterEnergy))
	{
		scatter.InteractionEnergy = scatterEnergy;
	}
	scatter.RelativeInteractionPoint = scatterPoint;
	scatter.TransformedInteractionPoint = transformation * scatterPoint;

	if (!isnan(absorberEnergy))
	{
		absorber.InteractionEnergy = absorberEnergy;
	}
	if (iType == eInterationType::COMPTON)
	{

		absorber.RelativeInteractionPoint = absorberPoint;
		absorber.TransformedInteractionPoint = transformation * absorberPoint;
	}

	listmodeData.Type = iType;
	listmodeData.Scatter = scatter;
	listmodeData.Absorber = absorber;
	listmodeData.DetectorTransformation = transformation;
	listmodeData.InteractionTimeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());;

	return listmodeData;
}


HUREL::Compton::LahgiControl::LahgiControl() :
	mAbsorberModules(NULL),
	mScatterModules(NULL),
	mModuleType(HUREL::Compton::eMouduleType::MONO)
{
	bUseFaultDiagnosis = false;	//240228

	Eigen::Matrix4d test;
	test = Matrix4d::Ones();
	Eigen::Vector3d test2 = Eigen::Vector3d(1, 1, 1);

	Eigen::Vector4d test3;
	test3 = Eigen::Vector4d(1, 2, 3, 4);
	test3.normalize();


	t265toLACCPosTransform << 0, 1, 0, T265_TO_LAHGI_OFFSET_X,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Y,
		1, 0, 0, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;

	t265toLACCPosTranslate << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;

	t265toLACCPosTransformInv = t265toLACCPosTransform.inverse();

	t265toLACCPosTransCalc = t265toLACCPosTransform * t265toLACCPosTransformInv * t265toLACCPosTranslate;	//231012 sbkwon : ���� ����

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Logger loaded in cpp!", eLoggerType::INFO);

	ListModeDataListeningThread = std::async([this]
		{
			this->ListModeDataListening();
		});

}

void HUREL::Compton::LahgiControl::ListModeDataListening()
{
	mIsListModeDataListeningThreadStart = true;
	mIsListModeDataListeningThreadRun = true;

	size_t bufferSize = 3000;	//1000000
	std::vector<std::array<unsigned short, 144>> tempVector;
	tempVector.reserve(bufferSize);

	while (mIsListModeDataListeningThreadStart)
	{
		//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Before pop out ByteData", eLoggerType::INFO);
		std::array<unsigned short, 144> out;
		while (mShortByteDatas.try_pop(out) && tempVector.size() < bufferSize)
		{
			tempVector.push_back(out);
		}
		if (tempVector.size() == 0)
		{
			continue;
		}

		//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "After pop out ByteData", eLoggerType::INFO);
		RtabmapSlamControl::instance().SetCurrentFrame();

		//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "temp size() : " + std::to_string(tempVector.size()) + " : " + std::to_string(mShortByteDatas.unsafe_size()), eLoggerType::INFO);


		//tempVector.clear();
		//continue;

		mListModeDataMutex.lock();
		mResetEnergySpectrumMutex.lock();
		eChksMutex.lock();

		std::chrono::milliseconds timeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		Eigen::Matrix4d deviceTransformation = t265toLACCPosTransform * RtabmapSlamControl::instance().GetOdomentry() * t265toLACCPosTransformInv * t265toLACCPosTranslate;	//231023 sbkwon ����
		//Eigen::Matrix4d deviceTransformation = t265toLACCPosTransCalc * RtabmapSlamControl::instance().GetOdomentry();	//231023 sbkwon ����

		//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Before MLPE", eLoggerType::INFO);
#pragma omp parallel for
		for (int i = 0; i < tempVector.size(); ++i)
		{
			//240228
			if (bUseFaultDiagnosis == true)
				AddListModeDataWithTransformationLoopFD(tempVector[i], timeInMili, deviceTransformation);
			else
				AddListModeDataWithTransformationLoop(tempVector[i], timeInMili, deviceTransformation);
		}
		//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "After MLPE", eLoggerType::INFO);
		mListModeDataMutex.unlock();
		mResetEnergySpectrumMutex.unlock();
		eChksMutex.unlock();
		//printf("done\n");

		////
		//int nCount = tempVector.size();
		////

		tempVector.clear();


		//240104 : time test �ʿ�
		/*RtabmapSlamControl::instance().SetCurrentVideoFrame();
		RtabmapSlamControl::instance().SetCurrentDepthFrame();*/

		//		RtabmapSlamControl::instance().SetCurrentFrame();

				//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "List Mode Data add and RGB/Depth Set : " + std::to_string(nCount), eLoggerType::INFO);

				/*cv::Mat rgb;
				cv::Mat depth;

				rgb = RtabmapSlamControl::instance().GetCurrentVideoFrame1();
				depth = RtabmapSlamControl::instance().GetCurrentDepthFrame1();

				if (rgb.empty() == false && depth.empty() == false)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr gencloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					*gencloud = *RtabmapSlamControl::instance().generatePointCloud(depth, rgb);
					open3d::geometry::PointCloud reconPointCloud = RtabmapSlamControl::instance().PclToOpen3d(gencloud);

					open3d::geometry::PointCloud recontransPC;
					recontransPC = HUREL::Compton::RtabmapSlamControl::instance().RTPointCloudTransposed(reconPointCloud, deviceTransformation);
				HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "List Mode Data add" + std::to_string(bufferSize) + " : " + std::to_string(tempVector.size()), eLoggerType::INFO);
				}*/

	}

	mIsListModeDataListeningThreadRun = false;
}

HUREL::Compton::LahgiControl& HUREL::Compton::LahgiControl::instance()
{
	static LahgiControl& instance = *(new LahgiControl());
	return instance;
}


void HUREL::Compton::LahgiControl::SetEchk(std::vector<sEnergyCheck> eChksInput)
{
	eChksMutex.lock();

	eChk = eChksInput;

	eChksMutex.unlock();
}

//240123
void HUREL::Compton::LahgiControl::SelectEchk(std::vector<int> eChksInput)
{
	eSelectChksMutex.lock();

	eSelectChk = eChksInput;

	//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Select Check : " + std::to_string(eSelectChk.size()), eLoggerType::INFO);

	eSelectChksMutex.unlock();
}


bool HUREL::Compton::LahgiControl::SetType(eMouduleType type)
{
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::LahgiControl", "Set type", eLoggerType::INFO);
	mListModeDataMutex.lock();

	mListedListModeData.reserve(50000);
	mListModeDataMutex.unlock();
	mModuleType = type;
	switch (type)
	{
	case HUREL::Compton::eMouduleType::MONO:
		assert(false);
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		mScatterModules = new Module * [1];
		mAbsorberModules = new Module * [1];
		double offset = 0.083;
		// 3 0
		// 2 1
		//
		double xOffset[1]{ 0 };
		double yOffset[1]{ 0 };
		string scatterSerial = "2001";
		//string absorberSerial = "50785";

		//string scatterSerial = "50777";
		string absorberSerial = "2002";

		for (int i = 0; i < 1; ++i)
		{
			double gain[10];
			double offsetZ = -(0.160);
			mScatterModules[i] = new Module(eMouduleType::QUAD, "config\\QUAD", scatterSerial + string("_Scint") + to_string(i), xOffset[i], yOffset[i], -0.015); //-0.055
			if (!mScatterModules[i]->IsModuleSet())
			{
				return false;
			}
			mAbsorberModules[i] = new Module(eMouduleType::QUAD, "config\\QUAD", absorberSerial + string("_Scint") + to_string(i), xOffset[i], yOffset[i], -0.015 + offsetZ);
			if (!mAbsorberModules[i]->IsModuleSet())
			{
				return false;
			}
		}
		break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		break;
	}
	case HUREL::Compton::eMouduleType::TEST:
	{
		mScatterModules = new Module * [1];
		mAbsorberModules = new Module * [1];
		mScatterModules[0] = new Module();
		mAbsorberModules[0] = new Module();
		break;
	}
	default:
		assert(false);
		break;
	}

	return true;
}

HUREL::Compton::LahgiControl::~LahgiControl()
{
	mIsListModeDataListeningThreadStart = false;

	//wait thread end
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Close 1", eLoggerType::INFO);

	MSG   message;
	while (mIsListModeDataListeningThreadRun)
	{
		if (::PeekMessage(&message, NULL, 0, 0, PM_REMOVE))
		{
			::TranslateMessage(&message);
			::DispatchMessage(&message);
		}
		Sleep(10);
	}

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Close 2", eLoggerType::INFO);

	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		assert(false);
		break;
	case HUREL::Compton::eMouduleType::QUAD:
		for (int i = 0; i < 1; ++i)
		{
			delete mScatterModules[i];
			delete mAbsorberModules[i];
		}
		break;
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
		break;
	case HUREL::Compton::eMouduleType::TEST:
		delete mScatterModules[0];
		delete mAbsorberModules[0];
		break;
	default:
		assert(false);
		break;
	}
	delete[] mScatterModules;
	delete[] mAbsorberModules;

	ListModeDataListeningThread.get();
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Close 3", eLoggerType::INFO);
}

void HUREL::Compton::LahgiControl::AddListModeDataWithTransformation(const unsigned short byteData[144])
{
	std::array<unsigned short, 144> tempArray;
	for (int i = 0; i < 144; ++i)
	{
		tempArray[i] = byteData[i];
	}

	mShortByteDatas.push(tempArray);
}

void HUREL::Compton::LahgiControl::AddListModeDataWithTransformationLoop(std::array<unsigned short, 144> byteData, std::chrono::milliseconds& timeInMili, Eigen::Matrix4d& deviceTransformation)
{
	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		Eigen::Array<float, 1, 9> scatterShorts[1];
		Eigen::Array<float, 1, 9> absorberShorts[1];

		//Channel 1 (scatter)
		for (int i = 1; i < 2; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				scatterShorts[i - 1][j] = static_cast<double>(byteData[i * 9 + j]);
			}
		}

		//Channel 9 (absorber)
		for (int i = 9; i < 10; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				absorberShorts[i - 9][j] = static_cast<double>(byteData[i * 9 + j]);
			}
		}

		double scattersEnergy[1];
		double absorbersEnergy[1];

		int scatterInteractionCount = 0;
		int absorberInteractionCount = 0;
		int scatterInteractModuleNum[1];
		int absorberInteractModuleNum[1];

		for (int i = 0; i < 1; ++i)
		{
			scattersEnergy[i] = mScatterModules[i]->GetEcal(scatterShorts[i]);
			if (!isnan(scattersEnergy[i]))
			{
				EnergyTimeData eTime{ i + 1, scattersEnergy[i], timeInMili };
				mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i]);

				mListedEnergyTimeData.push_back(eTime);
				scatterInteractModuleNum[scatterInteractionCount++] = i;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				EnergyTimeData eTime{ i + 9, absorbersEnergy[i], timeInMili };
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);

				mListedEnergyTimeData.push_back(eTime);
				absorberInteractModuleNum[absorberInteractionCount++] = i;

			}
		}


		if (scatterInteractionCount == 1)
		{
			double sEnergy = scattersEnergy[scatterInteractModuleNum[0]];
			double aEnergy = nan("");
			if (absorberInteractionCount == 1)
			{
				double aEnergy = absorbersEnergy[absorberInteractModuleNum[0]];
				//Compton
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{
						eInterationType type = eInterationType::COMPTON;

						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum[0]]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum[0]]);
						Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum[0]]->FastMLPosEstimation(absorberShorts[absorberInteractModuleNum[0]]);

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili, eChk[i]));//230911 sbkwon : Energy check �߰�
					}
				}

				//eChksMutex.unlock();
			}
			else if (absorberInteractionCount == 0)
			{

				//Coded Apature
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy < eChk[i].maxE && sEnergy> eChk[i].minE)
					{
						eInterationType type = eInterationType::CODED;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum[0]]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum[0]]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);
						aEnergy = nan("");
						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili, eChk[i]));//230911 sbkwon : Energy check �߰�
					}

				}
				//eChksMutex.unlock();
			}
		}
		else if (scatterInteractionCount > 1)
		{
			eInterationType type = eInterationType::CODED;
			for (int interDet = 0; interDet < scatterInteractionCount; ++interDet)
			{
				double sEnergy = scattersEnergy[scatterInteractModuleNum[interDet]];
				double aEnergy = nan("");

				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy < eChk[i].maxE && sEnergy> eChk[i].minE)
					{
						eInterationType type = eInterationType::CODED;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum[interDet]]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum[interDet]]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);
						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili, eChk[i]));//230911 sbkwon : Energy check �߰�
					}

				}
			}

		}

		break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		break;
	}
	default:
	{
		//Do nothing
		break;
	}
	}
}

//240228 : ���� �˻� �� Data ����
void HUREL::Compton::LahgiControl::AddListModeDataWithTransformationLoopFD(std::array<unsigned short, 144> byteData, std::chrono::milliseconds& timeInMili, Eigen::Matrix4d& deviceTransformation)
{
	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		Eigen::Array<float, 1, 9> scatterShorts[1];
		Eigen::Array<float, 1, 9> absorberShorts[1];

		//Channel 1 (scatter)
		for (int i = 1; i < 2; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				scatterShorts[i - 1][j] = static_cast<double>(byteData[i * 9 + j]);
			}
		}

		//Channel 9 (absorber)
		for (int i = 9; i < 10; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				absorberShorts[i - 9][j] = static_cast<double>(byteData[i * 9 + j]);
			}
		}

		double scattersEnergy[1];
		double absorbersEnergy[1];

		for (int i = 0; i < 1; ++i)
		{
			//���� �˻�� : PMT 9�� * gain
			//scatter
			if (scatterShorts[i].sum() > 0)
			{
				Eigen::Array<float, 1, 9> data = mScatterModules[i]->CalGain(scatterShorts[i]);
				PMTData pmt{ i + 1, data };
				mListedPMTGainData.push_back(pmt);
			}
			//absorber
			if (absorberShorts[i].sum() > 0)
			{
				Eigen::Array<float, 1, 9> data = mAbsorberModules[i]->CalGain(absorberShorts[i]);
				PMTData pmt{ i + 9, data };
				mListedPMTGainData.push_back(pmt);
			}

			//spectrum data
			scattersEnergy[i] = mScatterModules[i]->GetEcal(scatterShorts[i]);
			if (!isnan(scattersEnergy[i]))
			{
				EnergyTimeData eTime{ i + 1, scattersEnergy[i], timeInMili };
				mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i]);

				mListedEnergyTimeData.push_back(eTime);
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				EnergyTimeData eTime{ i + 9, absorbersEnergy[i], timeInMili };
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);

				mListedEnergyTimeData.push_back(eTime);

			}
		}

		break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		break;
	}
	default:
	{
		//Do nothing
		break;
	}
	}
}


void HUREL::Compton::LahgiControl::AddListModeDataWithTransformationVerification(const unsigned short byteData[])
{
	Eigen::Matrix4d t265toLACCPosTransform;
	t265toLACCPosTransform << 0, 1, 0, T265_TO_LAHGI_OFFSET_X,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Y,
		1, 0, 0, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;
	Eigen::Matrix4d deviceTransformation = t265toLACCPosTransform * RtabmapSlamControl::instance().GetOdomentry();

	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		const unsigned short* scatterShorts[1];
		const unsigned short* absorberShorts[1];

		for (int i = 1; i < 2; ++i)
		{
			scatterShorts[i - 1] = &byteData[i * 9];
		}

		for (int i = 9; i < 10; ++i)
		{
			absorberShorts[i - 9] = &byteData[i * 9];
		}


		double scattersEnergy[1];
		double absorbersEnergy[1];

		int scatterInteractionCount = 0;
		int absorberInteractionCount = 0;
		int scatterInteractModuleNum = 4;
		int absorberInteractModuleNum = 4;

		for (int i = 0; i < 1; ++i)
		{
			scattersEnergy[i] = mScatterModules[i]->GetEcal(scatterShorts[i]);
			if (!isnan(scattersEnergy[i]))
			{
				mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i]);
				scatterInteractModuleNum = i;
				++scatterInteractionCount;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);
				absorberInteractModuleNum = i;
				++absorberInteractionCount;
			}
		}

		if (scatterInteractionCount == 1)
		{
			double sEnergy = scattersEnergy[scatterInteractModuleNum];
			double aEnergy = absorbersEnergy[absorberInteractModuleNum];

			if (absorberInteractionCount == 1)
			{
				//Compton
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{
						eInterationType type = eInterationType::COMPTON;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimationVerification(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum]->FastMLPosEstimationVerification(absorberShorts[absorberInteractModuleNum]);

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
					}
				}
			}
			else if (absorberInteractionCount == 0)
			{
				//Coded Apature
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy < eChk[i].maxE && sEnergy> eChk[i].minE)
					{
						eInterationType type = eInterationType::CODED;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimationVerification(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));

					}

				}

			}
		}
		else
		{
			eInterationType type = eInterationType::NONE;
		}
		break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		break;
	}
	default:
	{
		//Do nothing
		break;
	}
	}
}

void HUREL::Compton::LahgiControl::AddListModeData(const unsigned short(byteData)[144], Eigen::Matrix4d deviceTransformation)
{
	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		unsigned short scatterShorts[1][9];
		unsigned short absorberShorts[1][9];
		//Channel 4
		for (int i = 4; i < 5; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				scatterShorts[i - 4][j] = byteData[i * 9 + j];
			}
		}

		//Channel 12
		for (int i = 12; i < 13; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				absorberShorts[i - 12][j] = byteData[i * 9 + j];
			}
		}


		double scattersEnergy[1];
		double absorbersEnergy[1];

		int scatterInteractionCount = 0;
		int absorberInteractionCount = 0;
		int scatterInteractModuleNum = 1;
		int absorberInteractModuleNum = 1;

		for (int i = 0; i < 1; ++i)
		{
			scattersEnergy[i] = mScatterModules[i]->GetEcal(scatterShorts[i]);
			if (!isnan(scattersEnergy[i]))
			{
				mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i]);
				scatterInteractModuleNum = i;
				++scatterInteractionCount;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);
				absorberInteractModuleNum = i;
				++absorberInteractionCount;
			}
		}

		if (scatterInteractionCount == 1)
		{
			double sEnergy = scattersEnergy[scatterInteractModuleNum];
			double aEnergy = absorbersEnergy[absorberInteractModuleNum];

			if (absorberInteractionCount == 1)
			{
				//Compton
				eInterationType type = eInterationType::COMPTON;
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{

						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum]->FastMLPosEstimation(scatterShorts[absorberInteractModuleNum]);
						mListModeDataMutex.lock();

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
						mListModeDataMutex.unlock();

					}
				}
			}
			else if (absorberInteractionCount == 0)
			{
				//Coded Apature
				eInterationType type = eInterationType::CODED;
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{

						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);
						mListModeDataMutex.lock();

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
						mListModeDataMutex.unlock();

					}
				}

			}
		}
		else
		{
			eInterationType type = eInterationType::NONE;
		}



		break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		break;
	}
	default:
	{
		//Do nothing
		break;
	}
	}
}

HUREL::Compton::eMouduleType HUREL::Compton::LahgiControl::GetDetectorType()
{
	return mModuleType;
}

const std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData() const
{
	mResetListModeDataMutex.lock();
	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;
	lmData.reserve(size);
	for (int i = 0; i < size; ++i)
	{
		lmData.push_back(mListedListModeData[i]);
	}


	mResetListModeDataMutex.unlock();
	return lmData;
}

size_t  HUREL::Compton::LahgiControl::GetListedListModeDataSize()
{
	return mListedListModeData.size();
}


const std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData(long long timeInMiliSecond) const
{

	if (timeInMiliSecond == 0)
	{
		return GetListedListModeData();
	}

	mResetListModeDataMutex.lock();
	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	size_t getIndexStart = 0;
	for (int i = size - 1; i > 0; --i)
	{
		if (t.count() - mListedListModeData[i].InteractionTimeInMili.count() > timeInMiliSecond)
		{
			getIndexStart = i;
			break;
		}
	}

	int reconStartIndex = 0;

	lmData.reserve(size - getIndexStart);
	for (int i = getIndexStart; i < size; ++i)
	{
		lmData.push_back(mListedListModeData[i]);
	}

	mResetListModeDataMutex.unlock();

	return lmData;
}

std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData()
{
	mResetListModeDataMutex.lock();

	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;
	lmData.reserve(size);
	for (int i = 0; i < size; ++i)
	{
		lmData.push_back(mListedListModeData[i]);
	}
	mResetListModeDataMutex.unlock();


	return lmData;
}

std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData(long long timeInMililseconds)
{
	if (timeInMililseconds == 0)
	{
		return GetListedListModeData();
	}
	mResetListModeDataMutex.lock();

	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	size_t getIndexStart = 0;
	long long currentTime = t.count();
	for (int i = size - 1; i > 0; --i)
	{
		long long interactionTime = mListedListModeData[i].InteractionTimeInMili.count();
		if (currentTime - interactionTime > timeInMililseconds)
		{
			getIndexStart = i;
			break;
		}
	}

	int reconStartIndex = 0;

	lmData.reserve(size - getIndexStart);
	for (int i = getIndexStart; i < size; ++i)
	{
		lmData.push_back(mListedListModeData[i]);
	}
	mResetListModeDataMutex.unlock();


	return lmData;
}

//231121-1 sbkwon : ��ȿ ���� ��ŭ ��������
//240122 sbkwon : echek�� �̿��Ͽ� ���� ���õ� ������ ���� efectcount��ŭ �����´�
#define EACH_EFECTCOUNT	//���õ� ������ ������ efectcount ��ŭ �����ö�
std::vector<ListModeData> HUREL::Compton::LahgiControl::GetEfectListedListModeData(int nEfectCount, long long time)
{
	if (nEfectCount == 0)
	{
		return GetListedListModeData();
	}

	//mResetListModeDataMutex.lock();

	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;

#ifdef EACH_EFECTCOUNT
	//echk
	eSelectChksMutex.lock();

	std::vector<int> selectChk = eSelectChk;

	eSelectChksMutex.unlock();
	if (selectChk.size() <= 0)
		return lmData;

	//

	int nTotalCnt = nEfectCount * selectChk.size();
	lmData.reserve(nTotalCnt);

	for (int ek = 0; ek < selectChk.size(); ek++)
	{
		if (time > 0)
		{
			std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
			long long currentTime = t.count();

			int nCount = 0;

			for (int i = size - 1; i > 0; --i)
			{
				ListModeData& d = mListedListModeData[i];
				long long interactionTime = d.InteractionTimeInMili.count();
				if (currentTime - interactionTime > time || nCount > nEfectCount)
				{
					break;
				}

				if (d.EnergyCheck.nElement == selectChk[ek])
				{
					nCount++;
					lmData.push_back(mListedListModeData[i]);
				}
			}
		}
		else
		{
			int nCount = 0;
			for (int i = size - 1; i > 0; --i)
			{
				ListModeData& d = mListedListModeData[i];
				if (nCount > nEfectCount)
				{
					break;
				}
				if (d.EnergyCheck.nElement == selectChk[ek])
				{
					nCount++;
					lmData.push_back(mListedListModeData[i]);
				}
			}
		}
	}
#else
	//echk befor
	if (time > 0)
	{
		std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		long long currentTime = t.count();

		lmData.reserve(nEfectCount);
		int nCount = 0;

		for (int i = size - 1; i > 0; --i)
		{
			long long interactionTime = mListedListModeData[i].InteractionTimeInMili.count();
			if (currentTime - interactionTime > time || nCount > nEfectCount)
			{
				break;
			}
			nCount++;
			lmData.push_back(mListedListModeData[i]);
		}

	}
	else
	{
		int getIndexStart = size - nEfectCount;
		if (getIndexStart < 0)
			getIndexStart = 0;

		lmData.reserve(nEfectCount);

		for (int i = getIndexStart; i < size; ++i)
		{
			lmData.push_back(mListedListModeData[i]);
		}
	}

#endif // EACH_EFECTCOUNT

	//mResetListModeDataMutex.unlock();

	return lmData;
}


//240930 sbkwon : Ư�� ������ ����
std::vector<ListModeData> HUREL::Compton::LahgiControl::GetEfectListedListModeData(const int& nElement, const int& nEfectCount, const long long& time)
{
	std::vector<ListModeData> lmData;
	size_t size = mListedListModeData.size();

	if (nEfectCount <= 0)
	{
		if (time > 0)
		{
			std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
			size_t getIndexStart = 0;
			long long incudeTime = t.count() - time;
			int nCount = 0;

			for (int i = size - 1; i > 0; --i)
			{
				ListModeData& d = mListedListModeData[i];

				if (incudeTime > d.InteractionTimeInMili.count())
				{
					getIndexStart = i;
					break;
				}

				if (d.EnergyCheck.nElement == nElement)
					nCount++;
			}

			lmData.reserve(nCount);

			for (int i = getIndexStart; i < size; ++i)
			{
				ListModeData& d = mListedListModeData[i];
				if (d.EnergyCheck.nElement == nElement)
					lmData.push_back(d);
			}
		}
		else
		{
			lmData.reserve(10000);

			for (int i = size - 1; i > 0; --i)
			{
				ListModeData& d = mListedListModeData[i];

				if (d.EnergyCheck.nElement == nElement)
				{
					lmData.push_back(d);
				}
			}
		}
	}
	else
	{
		lmData.reserve(nEfectCount);
		int nCount = 0;

		if (time > 0)
		{
			std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
			long long incudeTime = t.count() - time;

			int nCount = 0;

			for (int i = size - 1; i > 0; --i)
			{
				ListModeData& d = mListedListModeData[i];

				if (incudeTime > d.InteractionTimeInMili.count() || nCount > nEfectCount)
				{
					break;
				}

				if (d.EnergyCheck.nElement == nElement)
				{
					nCount++;
					lmData.push_back(d);
				}
			}
		}
		else
		{
			int nCount = 0;

			for (int i = size - 1; i > 0; --i)
			{
				ListModeData& d = mListedListModeData[i];

				if (nCount > nEfectCount)
				{
					break;
				}

				if (d.EnergyCheck.nElement == nElement)
				{
					nCount++;
					lmData.push_back(d);
				}
			}
		}
	}

	return lmData;
}

//240930 sbkwon 
std::vector<int> HUREL::Compton::LahgiControl::GetSelectEchek()
{
	eSelectChksMutex.lock();

	std::vector<int> selectChk = eSelectChk;

	eSelectChksMutex.unlock();

	return selectChk;
}


std::vector<ListModeData> HUREL::Compton::LahgiControl::GetEfectListedListModeData(int nEfectCount)
{
	if (nEfectCount == 0)
	{
		return GetListedListModeData();
	}

	//mResetListModeDataMutex.lock();

	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;


	int getIndexStart = size - nEfectCount;
	if (getIndexStart < 0)
		getIndexStart = 0;

	lmData.reserve(nEfectCount);

	for (int i = getIndexStart; i < size; ++i)
	{
		lmData.push_back(mListedListModeData[i]);
	}

	//mResetListModeDataMutex.unlock();
	return lmData;
}

//230911 sbkwon
std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData(sEnergyCheck echk)
{
	mResetListModeDataMutex.lock();

	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;
	lmData.reserve(size);
	for (int i = 0; i < size; ++i)
	{
		ListModeData& d = mListedListModeData[i];
		if (d.EnergyCheck == echk)
		{
			lmData.push_back(d);
		}
	}
	mResetListModeDataMutex.unlock();

	return lmData;
}

//230911 sbkwon
std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData(long long timeInMililseconds, sEnergyCheck echk)
{
	if (timeInMililseconds <= 0)
	{
		return GetListedListModeData(echk);
	}

	mResetListModeDataMutex.lock();

	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	size_t getIndexStart = 0;
	long long currentTime = t.count();

	for (int i = size - 1; i > 0; --i)
	{
		long long interactionTime = mListedListModeData[i].InteractionTimeInMili.count();
		if (currentTime - interactionTime > timeInMililseconds)
		{
			getIndexStart = i;
			break;
		}
	}

	lmData.reserve(size - getIndexStart);
	for (int i = getIndexStart; i < size; ++i)
	{
		ListModeData& d = mListedListModeData[i];
		if (d.EnergyCheck == echk)
		{
			lmData.push_back(d);
		}
	}
	mResetListModeDataMutex.unlock();

	return lmData;
}

//231123 sbkwon
std::vector<EnergyTimeData> HUREL::Compton::LahgiControl::GetListedEnergyTimeData(int fpgaChannelNumber, long long timeInMililseconds)
{
	mRListEnergyTimeDataMutex.lock();	//231100

	size_t size = mListedEnergyTimeData.size();
	std::vector<EnergyTimeData> lmData;
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	size_t getIndexStart = 0;

	long long time = t.count() - timeInMililseconds;

	for (int i = size - 1; i > 0; --i)
	{
		if (time > mListedEnergyTimeData[i].InteractionTimeInMili.count())
		{
			getIndexStart = i;
			break;
		}
	}

	int reconStartIndex = 0;

	lmData.reserve(size - getIndexStart);
	for (int i = getIndexStart; i < size; ++i)
	{
		if (mListedEnergyTimeData[i].InteractionChannel == fpgaChannelNumber)
		{
			lmData.push_back(mListedEnergyTimeData[i]);
		}
	}
	mRListEnergyTimeDataMutex.unlock();	//231100
	return lmData;

}

std::vector<EnergyTimeData> HUREL::Compton::LahgiControl::GetListedEnergyTimeData(long long timeInMiliSecond)
{
	mResetListModeDataMutex.lock();	//231100

	size_t size = mListedEnergyTimeData.size();
	std::vector<EnergyTimeData> lmData;
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	size_t getIndexStart = 0;

	for (int i = size - 1; i > 0; --i)
	{
		if (t.count() - mListedEnergyTimeData[i].InteractionTimeInMili.count() > timeInMiliSecond)
		{
			getIndexStart = i;
			break;
		}
	}

	int reconStartIndex = 0;

	lmData.reserve(size - getIndexStart + 1);
	for (int i = getIndexStart; i < size; ++i)
	{
		lmData.push_back(mListedEnergyTimeData[i]);
	}
	mResetListModeDataMutex.unlock();	//231100
	return lmData;

}
std::vector<EnergyTimeData> HUREL::Compton::LahgiControl::GetListedEnergyTimeData()
{
	mResetListModeDataMutex.lock();	//231100
	std::vector<EnergyTimeData> lmData;

	size_t size = mListedEnergyTimeData.size();


	lmData.reserve(size);
	for (int i = 0; i < size; ++i)
	{
		lmData.push_back(mListedEnergyTimeData[i]);
	}
	mResetListModeDataMutex.unlock();	//231100
	return lmData;
}
const std::vector<EnergyTimeData> HUREL::Compton::LahgiControl::GetListedEnergyTimeData(long long timeInMiliSecond) const
{
	mResetListModeDataMutex.lock();	//231100

	size_t size = mListedEnergyTimeData.size();
	std::vector<EnergyTimeData> lmData;
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	size_t getIndexStart = 0;
	for (int i = size - 1; i > 0; --i)
	{
		if (t.count() - mListedEnergyTimeData[i].InteractionTimeInMili.count() > timeInMiliSecond)
		{
			getIndexStart = i;
			break;
		}
	}

	int reconStartIndex = 0;

	lmData.reserve(size - getIndexStart);
	for (int i = getIndexStart; i < size; ++i)
	{
		lmData.push_back(mListedEnergyTimeData[i]);
	}
	mResetListModeDataMutex.unlock();	//231100
	return lmData;
}


void HUREL::Compton::LahgiControl::ResetListedListModeData()
{
	mResetListModeDataMutex.lock();
	mListModeDataMutex.lock();
	mListedListModeData.clear();
	mListedListModeData.shrink_to_fit();
	mListedListModeData.reserve(50000);

	mListModeDataMutex.unlock();
	mResetListModeDataMutex.unlock();

	mRListEnergyTimeDataMutex.lock();	//231123 sbkwon
	mListedEnergyTimeData.clear();
	mListedEnergyTimeData.shrink_to_fit();
	mListedEnergyTimeData.reserve(50000);
	mRListEnergyTimeDataMutex.unlock();	//231123 sbkwon

	//240228 : PMT ����
	mListedPMTGainData.clear();
	mListedPMTGainData.shrink_to_fit();
	mListedPMTGainData.reserve(1000000);

	//240228 : PMT ���纻
	mListedPMTGainDataCopy.clear();
	mListedPMTGainDataCopy.shrink_to_fit();
	mListedPMTGainDataCopy.reserve(1000000);
}

void HUREL::Compton::LahgiControl::SaveListedListModeData(std::string fileName)
{
	std::ofstream saveFile;
	saveFile.open(fileName + "_LmData.csv");
	if (!saveFile.is_open())
	{
		std::cout << "File is not opened" << endl;
		saveFile.close();
		return;
	}
	std::vector<ListModeData> data = this->GetListedListModeData();
	for (unsigned int i = 0; i < data.size(); ++i)
	{
		ListModeData& d = data[i];
		saveFile << d.WriteListModeData() << std::endl;
	}
	saveFile.close();

	saveFile.open(fileName + "_LmDEnergyData.csv");
	if (!saveFile.is_open())
	{
		std::cout << "File is not opened" << endl;
		saveFile.close();
		return;
	}
	std::vector<EnergyTimeData> edata = this->GetListedEnergyTimeData();
	for (unsigned int i = 0; i < edata.size(); ++i)
	{
		EnergyTimeData& d = edata[i];

		string line = "";
		line += std::to_string(d.InteractionTimeInMili.count());	line += ",";
		line += std::to_string(d.InteractionChannel);	line += ",";
		line += std::to_string(d.Energy);

		saveFile << line << std::endl;
	}
	saveFile.close();

	return;
}

bool replace(std::string& str, const std::string& from, const std::string& to) {
	size_t start_pos = str.find(from);
	if (start_pos == std::string::npos)
		return false;
	str.replace(start_pos, from.length(), to);
	return true;
}


bool HUREL::Compton::LahgiControl::LoadListedListModeData(std::string fileName)
{
	std::ifstream loadFile;
	loadFile.open(fileName);
	if (!loadFile.is_open())
	{
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Fail to open file", eLoggerType::ERROR_t);
		loadFile.close();
		return false;
	}
	mListedListModeData.clear();

	for (int i = 0; i < 1; ++i)
	{
		mScatterModules[i]->GetEnergySpectrum().Reset();
		mAbsorberModules[i]->GetEnergySpectrum().Reset();
	}

	string buffer;
	char line[2048];
	while (loadFile.good())
	{
		ListModeData temp;
		getline(loadFile, buffer);
		if (temp.ReadListModeData(buffer))
		{
			mListedListModeData.push_back(temp);


		}
	}

	loadFile.close();
	for (unsigned int i = 0; i < 16; ++i)
	{
		std::string tempFileName = fileName;
		replace(tempFileName, "_LmData", "_EnergyList_Channel_" + std::to_string(i));

	}


	if (mListedListModeData.size() == 0)
	{

		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Fail to load lm data", eLoggerType::ERROR_t);
		return false;
	}

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Load lm data: " + fileName, eLoggerType::INFO);
	return true;
}


bool HUREL::Compton::LahgiControl::LoadListedEnergyTimeData(std::string fileName)
{
	std::ifstream loadFile;
	loadFile.open(fileName);
	if (!loadFile.is_open())
	{
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Fail to open file : ListedEnergyTimeData", eLoggerType::ERROR_t);
		loadFile.close();
		return false;
	}
	mListedEnergyTimeData.clear();


	string buffer;
	char line[2048];
	while (loadFile.good())
	{
		EnergyTimeData temp;
		getline(loadFile, buffer);

		std::vector<string> words;
		stringstream sstream(buffer);
		string word;
		while (getline(sstream, word, ','))
		{
			words.push_back(word);
		}

		if (words.size() == 0)
			continue;

		if (words.size() != 3)
		{
			return false;
		}

		temp.InteractionTimeInMili = chrono::milliseconds(stoll(words[0]));
		temp.InteractionChannel = stoi(words[1]);
		temp.Energy = stod(words[2]);

		mListedEnergyTimeData.push_back(temp);
	}

	loadFile.close();

	if (mListedEnergyTimeData.size() == 0)
	{

		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Fail to load lm data : ListedEnergyTimeData", eLoggerType::ERROR_t);
		return false;
	}

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Load LoadListedEnergyTimeData: " + fileName, eLoggerType::INFO);
	return true;
}

EnergySpectrum& HUREL::Compton::LahgiControl::GetEnergySpectrum(int fpgaChannelNumber)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			return mScatterModules[0]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber == 8)
		{
			return mAbsorberModules[0]->GetEnergySpectrum();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber == 4 || fpgaChannelNumber == 1)
		{
			return mScatterModules[0]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber == 12 || fpgaChannelNumber == 9)
		{
			return mAbsorberModules[0]->GetEnergySpectrum();
		}

		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 8]->GetEnergySpectrum();
		}
		break;
	default:
		break;
	}
	EnergySpectrum e = EnergySpectrum(5, 3000);
	return e;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetSumEnergySpectrum()
{
	// All spectrum은 5000 범위로 설정 (Scatter + Absorber)
	// 원본 에너지 데이터를 사용하여 5000 범위로 변환
	EnergySpectrum spect(5, 5000);
	std::vector<EnergyTimeData> lmData = GetListedEnergyTimeData();
	
	for (int i = 0; i < lmData.size(); ++i)
	{
		// 모든 채널의 에너지 데이터 추가
		if (lmData[i].Energy >= 0 && lmData[i].Energy < 5000)
		{
			spect.AddEnergy(lmData[i].Energy);
		}
	}
	
	return spect;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetAbsorberSumEnergySpectrum()
{
	// Absorber spectrum 범위를 5000 keV로 설정
	// 원본 에너지 데이터를 사용하여 5000 범위로 변환
	EnergySpectrum spect(5, 5000);
	std::vector<EnergyTimeData> lmData = GetListedEnergyTimeData();
	
	for (int i = 0; i < lmData.size(); ++i)
	{
		// Absorber 채널만 선택 (InteractionChannel >= 8)
		if (lmData[i].InteractionChannel >= 8)
		{
			if (lmData[i].Energy >= 0 && lmData[i].Energy < 5000)
			{
				spect.AddEnergy(lmData[i].Energy);
			}
		}
	}
	
	return spect;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetScatterSumEnergySpectrum()
{
	EnergySpectrum spect(5, 3000);
	switch (mModuleType)
	{
	case eMouduleType::MONO:

		spect = mScatterModules[0]->GetEnergySpectrum();
		break;

	case eMouduleType::QUAD:
		for (int i = 0; i < 1; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();

		}
		break;
	case eMouduleType::QUAD_DUAL:
		for (int i = 0; i < 8; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();

		}
		break;
	default:
		break;
	}
	return spect;
}

std::tuple<double, double, double> HUREL::Compton::LahgiControl::GetEcalValue(int fpgaChannelNumber)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			return mScatterModules[0]->GetEnergyCalibration();
		}
		else if (fpgaChannelNumber == 8)
		{
			return mAbsorberModules[0]->GetEnergyCalibration();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber == 4 || fpgaChannelNumber == 1)
		{
			return mScatterModules[0]->GetEnergyCalibration();
		}
		else if (fpgaChannelNumber == 12 || fpgaChannelNumber == 9)
		{
			return mAbsorberModules[0]->GetEnergyCalibration();
		}

		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber]->GetEnergyCalibration();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 8]->GetEnergyCalibration();
		}
		break;
	default:
		break;
	}
	return tuple<double, double, double>();
}

void HUREL::Compton::LahgiControl::SetEcalValue(int fpgaChannelNumber, std::tuple<double, double, double> ecal)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			mScatterModules[0]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		else if (fpgaChannelNumber == 8)
		{
			mAbsorberModules[0]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber == 4 || fpgaChannelNumber == 1)
		{
			mScatterModules[0]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		else if (fpgaChannelNumber == 12 || fpgaChannelNumber == 9)
		{
			mAbsorberModules[0]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 8]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		break;
	default:
		break;
	}
}

void HUREL::Compton::LahgiControl::ResetEnergySpectrum()
{
	mResetEnergySpectrumMutex.lock();

	for (int i = 0; i < 16; ++i)
	{
		ResetEnergySpectrum(i);
	}

	mResetEnergySpectrumMutex.unlock();
}

void HUREL::Compton::LahgiControl::ResetEnergySpectrum(int fpgaChannelNumber)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			mScatterModules[0]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber == 8)
		{
			mAbsorberModules[0]->GetEnergySpectrum().Reset();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber == 4 || fpgaChannelNumber == 1)
		{
			mScatterModules[0]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber == 12 || fpgaChannelNumber == 9)
		{
			mAbsorberModules[0]->GetEnergySpectrum().Reset();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 8]->GetEnergySpectrum().Reset();
		}
		break;
	default:
		break;
	}
	return;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconRealtimePointCloudComptonUntransformed(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	std::vector<ListModeData> tempLMData = GetListedListModeData();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if ((t - tempLMData[i].InteractionTimeInMili).count() < seconds * 1000)
		{
			reconStartIndex = i;
			break;
		}
	}
#pragma omp parallel for
	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPoint(tempLMData[i], ReconPointCloud::SimpleComptonBackprojectionUntransformed);
	}

	std::cout << "End Recon: " << tempLMData.size() << std::endl;


	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconRealtimePointCloudCompton(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	std::vector<ListModeData> tempLMData = GetListedListModeData();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (seconds == 0)
		{
			reconStartIndex = 0;
			break;
		}
		if (t.count() - tempLMData[i].InteractionTimeInMili.count() < static_cast<__int64>(seconds))
		{
			reconStartIndex = i;
			break;
		}

	}

	std::vector<ListModeData> reconLm;
	reconLm.reserve(tempLMData.size());
	//assert(0, "temp energy search");
	for (const auto lm : tempLMData)
	{
		if (lm.Absorber.InteractionEnergy + lm.Scatter.InteractionEnergy > 620 && lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 700)
		{
			reconLm.push_back(lm);
		}

	}

#pragma omp parallel for
	for (int i = 0; i < reconLm.size(); ++i)
	{
		reconPC.CalculateReconPoint(reconLm[i], ReconPointCloud::SimpleComptonBackprojection);
	}
	//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "GetReconRealtimePointCloudCompton End Recon: " + reconLm.size(), eLoggerType::INFO);


	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconOverlayPointCloudCoded(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	mListModeImageMutex.lock();

	std::vector<RadiationImage> tempLMData;
	mListModeImageMutex.unlock();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<__int64>(seconds))
		{
			reconStartIndex = i;
			break;
		}

	}

	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPointCoded(tempLMData[i]);
	}
	std::cout << "End GetReconOverlayPointCloudCoded: " << tempLMData.size() << std::endl;


	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconOverlayPointCloudCompton(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	mListModeImageMutex.lock();

	std::vector<RadiationImage> tempLMData;
	mListModeImageMutex.unlock();

	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<__int64>(seconds))
		{
			reconStartIndex = i;
			break;
		}

	}

	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPointCompton(tempLMData[i]);
	}
	//std::cout << "End GetReconOverlayPointCloudCompton: " << tempLMData.size() << std::endl;


	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconOverlayPointCloudHybrid(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::vector <ListModeData> lmData = GetListedListModeData();
	RadiationImage radimg = RadiationImage(lmData);
	reconPC.CalculateReconPointHybrid(radimg);

	return reconPC;
}

inline int findIndex(double value, double min, double pixelSize)
{
	if (value - min <= 0)
	{
		return -1;
	}
	return static_cast<int>(floor((value + 0.00001 - min) / pixelSize));
}

cv::Mat HUREL::Compton::LahgiControl::GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter)
{
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	std::vector<ListModeData> tempLMData = GetListedListModeData(timeInSeconds * 1000);

	constexpr double Det_W = 0.400;
	Mat responseImg(pixelCount, pixelCount, CV_32S, Scalar(0));
	__int32* responseImgPtr = static_cast<__int32*>(static_cast<void*>(responseImg.data));

	for (ListModeData lm : tempLMData)
	{

		double interactionPoseX = -999999;
		double interactionPoseY = -999999;

		switch (lm.Type)
		{
		case eInterationType::COMPTON:
			if (isScatter)
			{
				interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
				interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];
			}
			else
			{
				interactionPoseX = lm.Absorber.RelativeInteractionPoint[0];
				interactionPoseY = lm.Absorber.RelativeInteractionPoint[1];
			}

			break;
		case eInterationType::CODED:
			if (isScatter)
			{
				interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
				interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];
			}
			break;
		default:
			continue;
			break;
		}


		int iX = findIndex(interactionPoseX, -Det_W / 2, Det_W / pixelCount);
		int iY = findIndex(interactionPoseY, -Det_W / 2, Det_W / pixelCount);
		if (iX >= 0 && iY >= 0 && iX < pixelCount && iY < pixelCount)
		{
			++responseImgPtr[pixelCount * iY + iX];
		}
	}


	return HUREL::Compton::RadiationImage::GetCV_32SAsJet(responseImg, imgSize);
}
//240228 : ���� �� ������ �ð��� ����˻縦 �����ϱ� ���Ͽ� PTM Data�� �����ϴ� ����, �˻� ���۽� �ѹ��� ����
void HUREL::Compton::LahgiControl::CopyListedPMTEnergyData()
{
	size_t size = mListedPMTGainData.size();

	mListedPMTGainDataCopy.clear();
	mListedPMTGainDataCopy.shrink_to_fit();
	mListedPMTGainDataCopy.reserve(size);

	for (size_t i = 0; i < size; i++)
	{
		mListedPMTGainDataCopy.push_back(mListedPMTGainData[i]);
	}
}

std::vector<double> HUREL::Compton::LahgiControl::GetGainref(int fpgaChannelNumber)
{
	std::vector<double> gainref;
	gainref.reserve(9);

	Eigen::Array<float, 1, 9> gain;

	if (fpgaChannelNumber == 4)
	{
		int ch = 0;
		gain = mScatterModules[ch]->GetGain();
	}
	else if (fpgaChannelNumber == 12)
	{
		int ch = 0;
		gain = mAbsorberModules[ch]->GetGain();
	}

	for (size_t i = 0; i < 9; i++)
	{
		gainref.push_back(gain(i));
	}

	return gainref;
}

//240228 : PTM Data�� Energy Data�� ��ȯ�Ͽ� ����. �Լ� ȣ�� �������� ��� �����͸� ��ȯ
std::vector<double> HUREL::Compton::LahgiControl::GetListedPMTEnergyData(int fpgaChannelNumber)
{
	size_t size = mListedPMTGainDataCopy.size();
	std::vector<double> lmData;

	lmData.reserve(size);

	for (int i = 0; i < size; ++i)
	{
		if (mListedPMTGainDataCopy[i].InteractionChannel == fpgaChannelNumber)
		{
			double dEnergy = mListedPMTGainDataCopy[i].data.sum();
			lmData.push_back(dEnergy);
		}
	}

	return lmData;
}

//240228 : dCorrMatIn ���ڸ� �̿��Ͽ� PTM Data�� Energy Data�� ��ȯ�Ͽ� ����.
std::vector<double> HUREL::Compton::LahgiControl::GetListedPMTEnergyData(int fpgaChannelNumber, std::vector<double> dCorrMatIn)
{
	size_t size = mListedPMTGainDataCopy.size();
	std::vector<double> lmData;

	Eigen::Array<float, 1, 9> newGain;
	for (size_t i = 0; i < 9; i++)
	{
		newGain[i] = dCorrMatIn[i];
	}

	lmData.reserve(size);

	for (int i = 0; i < size; ++i)
	{
		if (mListedPMTGainDataCopy[i].InteractionChannel == fpgaChannelNumber)
		{
			double dEnergy = (mListedPMTGainDataCopy[i].data * newGain).sum() + dCorrMatIn[9];
			lmData.push_back(dEnergy);
		}
	}

	return lmData;
}

//240228 : ����/���� ���� �̿��Ͽ� gain[10]���� ��ȯ
//erange_bkg index : ¦�� (loser), Ȧ�� (upper)
std::vector<double> HUREL::Compton::LahgiControl::GetPMTCorrMatIn(int fpgaChannelNumber, std::vector<int> usedPeak, std::vector<double> erange_bkg)
{
	size_t size = mListedPMTGainDataCopy.size();
	std::vector<double> gain;
	gain.reserve(10);

	//usedpead �� ������ �ش��ϴ� �ּ� �̺�Ʈ �� Ȯ��
	//���� ��ũ�� �̺�Ʈ ���� 3000���� �ȵǸ�, ���� ���� �̺�Ʈ ���� ����
	int nData = 3000;	//�ִ� �̺�Ʈ ��
	int IndexArray[4][3000] = { 0, };	//�̺�Ʈ ������ �ε��� ����

	for (size_t i = 0; i < 4; i++)	//usedPeak �� �ݺ�
	{
		int nIndex = i * 2;
		int nCount = 0;

		for (int j = 0; j < size; ++j)
		{
			if (mListedPMTGainDataCopy[j].InteractionChannel == fpgaChannelNumber)	//ä�� Ȯ��
			{
				float sum = mListedPMTGainDataCopy[j].data.sum();	//������ ��
				if (sum > erange_bkg[nIndex] && sum < erange_bkg[nIndex + 1])	//��Ʈ ������ �ش��ϴ��� Ȯ�� : (nIndex - lower), (nIndex+1 - upper)
				{
					if (nCount < 3000)//�ִ� 3000������ �ε����� �����Ѵ�.
						IndexArray[i][nCount] = j;
					else
						break;

					nCount++;
				}
			}
		}

		if (nData > nCount)
			nData = nCount;	//���� �̺�Ʈ ���� ���� ��ũ�� ������ŭ�� ������ ��
	}

	//��������ȭ�� ���� ���� �ʱ�ȭ
	//x�� ��ũ�� ��ġ�� ä��
	//y�� ��ũ�� ������
	//�� Ư�� ä�ο� ��ġ�� ��ũ�� oo keV �������� ��ũ�� ��� �˷��ִ� ������
	int nNo = nData * 4;	//4���� usedPeak�� �� �̺�Ʈ ��
	Eigen::MatrixXd fitdatax = Eigen::MatrixXd::Zero(nNo, 10);	//���õ� PMT ������ matrix 9���� ��ȣ data�� 10��°�� 1�� ������
	Eigen::MatrixXd fitdataE = Eigen::MatrixXd::Zero(nNo, 1);	//usedPeak�� ������ ������

	//���� ����ȭ�� �ϱ� ���� �̺�Ʈ�� ��ũ ���� �����ϴ� �ڵ�
	for (size_t i = 0; i < 4; i++)
	{
		int nIndex = i * nData;	//��ũ�� ���� �ε���

		//usedPeak�� ������ �߰�
		for (int j = 0; j < nData; ++j)
		{
			int nrows = nIndex + j;
			fitdataE(nrows, 0) = usedPeak[i];	//��ũ ������

			int nPMTIndex = IndexArray[i][j];

			for (size_t k = 0; k < 9; k++)
			{
				fitdatax(nrows, k) = mListedPMTGainDataCopy[nPMTIndex].data(k);
			}

			fitdatax(nrows, 9) = 1;
		}
	}

	//X=(A'*A)\A'*B; % X: ä���� �������� ��ȯ���ִ� matrix (ä���� 9���̹Ƿ�)
	Eigen::MatrixXd tranX = fitdatax.transpose();

	Eigen::MatrixXd A = tranX * fitdatax;	//(A'*A) ' => inverse() ?

	//A'*B ����
	//�ѹ��� Matrix ����
	Eigen::MatrixXd transAB = tranX * fitdataE;
	Eigen::VectorXd B = transAB.array();	//A'*B �����



	Eigen::VectorXd vX = A.colPivHouseholderQr().solve(B);	//A \ B

	for (size_t i = 0; i < 10; i++)
	{
		gain.push_back(vX(i));
	}

	return gain;
}

//240315 : ���� ���� gain�� ����/���� ���� �̿��Ͽ� gain[10]���� ��ȯ
//erange_bkg index : ¦�� (loser), Ȧ�� (upper)
std::vector<double> HUREL::Compton::LahgiControl::GetPMTCorrMatInBeforGain(int fpgaChannelNumber, std::vector<int> usedPeak, std::vector<double> erange_bkg, std::vector<double> dCorrMatIn)
{
	size_t size = mListedPMTGainDataCopy.size();
	std::vector<double> gain;
	gain.reserve(10);

	Eigen::Array<float, 1, 9> beforGain;
	for (size_t i = 0; i < 9; i++)
	{
		beforGain[i] = dCorrMatIn[i];
	}

	//usedpead �� ������ �ش��ϴ� �ּ� �̺�Ʈ �� Ȯ��
	//���� ��ũ�� �̺�Ʈ ���� 3000���� �ȵǸ�, ���� ���� �̺�Ʈ ���� ����
	int nData = 3000;	//�ִ� �̺�Ʈ ��
	int IndexArray[4][3000] = { 0, };	//�̺�Ʈ ������ �ε��� ����

	for (size_t i = 0; i < 4; i++)	//usedPeak �� �ݺ�
	{
		int nIndex = i * 2;
		int nCount = 0;

		for (int j = 0; j < size; ++j)
		{
			if (mListedPMTGainDataCopy[j].InteractionChannel == fpgaChannelNumber)	//ä�� Ȯ��
			{
				float sum = (mListedPMTGainDataCopy[j].data * beforGain).sum() + dCorrMatIn[9];	//������ ��
				if (sum > erange_bkg[nIndex] && sum < erange_bkg[nIndex + 1])	//��Ʈ ������ �ش��ϴ��� Ȯ�� : (nIndex - lower), (nIndex+1 - upper)
				{
					if (nCount < 3000)//�ִ� 3000������ �ε����� �����Ѵ�.
						IndexArray[i][nCount] = j;
					else
						break;

					nCount++;
				}
			}
		}

		if (nData > nCount)
			nData = nCount;	//���� �̺�Ʈ ���� ���� ��ũ�� ������ŭ�� ������ ��
	}


	int nNo = nData * 4;	//4���� usedPeak�� �� �̺�Ʈ ��
	Eigen::MatrixXd fitdatax = Eigen::MatrixXd::Zero(nNo, 10);	//���õ� PMT ������ matrix 9���� ��ȣ data�� 10��°�� 1�� ������
	Eigen::MatrixXd fitdataE = Eigen::MatrixXd::Zero(nNo, 1);	//usedPeak�� ������ ������

	//���� ����ȭ�� �ϱ� ���� �̺�Ʈ�� ��ũ ���� �����ϴ� �ڵ�
	for (size_t i = 0; i < 4; i++)
	{
		int nIndex = i * nData;	//��ũ�� ���� �ε���

		//usedPeak�� ������ �߰�
		for (int j = 0; j < nData; ++j)
		{
			int nrows = nIndex + j;
			fitdataE(nrows, 0) = usedPeak[i];	//��ũ ������

			int nPMTIndex = IndexArray[i][j];

			Eigen::Array<float, 1, 9> PMTData = mListedPMTGainDataCopy[nPMTIndex].data * beforGain;	//PMT Data�� ���� ����� gain ����

			for (size_t k = 0; k < 9; k++)
			{
				fitdatax(nrows, k) = PMTData(k);
			}

			fitdatax(nrows, 9) = 1;
		}
	}

	//X=(A'*A)\A'*B; % X: ä���� �������� ��ȯ���ִ� matrix (ä���� 9���̹Ƿ�)
	Eigen::MatrixXd tranX = fitdatax.transpose();

	Eigen::MatrixXd A = tranX * fitdatax;	//(A'*A) ' => inverse() ?

	//A'*B ����
	//�ѹ��� Matrix ����
	Eigen::MatrixXd transAB = tranX * fitdataE;
	Eigen::VectorXd B = transAB.array();	//A'*B �����

	//�� �྿ ����
	//Eigen::VectorXd B(10);	//A'*B �����
	//for (size_t i = 0; i < 10; i++)
	//{
	//	Eigen::MatrixXd value = tranX.row(i) * fitdataE;
	//	B(i) = value(0);	//value(0, 0)
	//}

	Eigen::VectorXd vX = A.colPivHouseholderQr().solve(B);	//A \ B

	for (size_t i = 0; i < 10; i++)
	{
		gain.push_back(vX(i));
	}

	return gain;
}

//2404 : MLEM
int HUREL::Compton::LahgiControl::GetSystemMatrixIndex(double energy)
{
	int nIndex = -1;
	//system matrix file data energy
	//10, 30, 50, 100, 200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800, 3000 keV 
	if (energy < 30)
		nIndex = 0;
	else if (energy < 50)
		nIndex = 1;
	else if (energy < 100)
		nIndex = 2;
	else if (energy < 200)
		nIndex = 3;
	else if (energy < 400)
		nIndex = 4;
	else if (energy < 600)
		nIndex = 5;
	else if (energy < 800)
		nIndex = 6;
	else if (energy < 1000)
		nIndex = 7;
	else if (energy < 1200)
		nIndex = 8;
	else if (energy < 1400)
		nIndex = 9;
	else if (energy < 1600)
		nIndex = 10;
	else if (energy < 1800)
		nIndex = 11;
	else if (energy < 2000)
		nIndex = 12;
	else if (energy < 2200)
		nIndex = 13;
	else if (energy < 2400)
		nIndex = 14;
	else if (energy < 2600)
		nIndex = 15;
	else if (energy < 2800)
		nIndex = 16;
	else if (energy <= 3000)
		nIndex = 17;

	return nIndex;
}


//2D �׽�Ʈ �Ϸ�� ���� ���. �Ʒ� TEST_2D �ּ��� ��� 3D ���� �˻�, �ּ� ������ ��� 2D ���а˻� �׽�Ʈ (Data load, �ҷ����� �� ���)
//#define TEST_2D

//2404 : MLEM
//240621 : load == true�� ��� FilePath => ply, rgb, depth ���� ���, : ���� + �ð�
bool HUREL::Compton::LahgiControl::LoadMLEMData(const std::string& FilePath, const std::string& LMDPath, bool bLoad, int nSize)
{
	//240429 data �ʱ�ȭ
	m_vMLEMPC.clear();
	m_vMLEMPC.shrink_to_fit();

	if (nSize > 0)
		m_vMLEMPC.reserve(nSize);
	else
		m_vMLEMPC.reserve(1);

	mMLEMPointcloud = open3d::geometry::PointCloud();

	//m_2DMLEM : 250324
	m_2DMLEM.clear();
	m_2DMLEM.shrink_to_fit();

	if (nSize > 0)
		m_2DMLEM.reserve(nSize);
	else
		m_2DMLEM.reserve(1);

	//test log
//	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "File Path: " + FilePath, eLoggerType::INFO);

	if (bLoad == true)
	{
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Data Load Set : " + std::to_string(bLoad), eLoggerType::INFO);
		//mMLEMPointcloud;


		//#ifdef TEST_2D
		//240621
		std::string rgbpath = FilePath + "_rgb.png";
		std::string depthpath = FilePath + "_depth.png";

		cv::Mat rgb = cv::imread(rgbpath/*, IMREAD_COLOR*/);
		//cv::Mat depth = cv::imread(depthpath);
		cv::Mat depth = cv::imread(depthpath, IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
		depth.convertTo(depth, CV_32F);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr gencloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		if (rgb.empty() || depth.empty())
			return false;


		//resize
		cv::Mat rergb, redepth;
		int down = 2;
		/*cv::resize(rgb, rergb, Size(rgb.cols / down, rgb.rows / down), INTER_LINEAR);
		cv::resize(depth, redepth, Size(rgb.cols / down, rgb.rows / down), INTER_LINEAR);*/

		/*cv::imshow("rgb", rergb);
		cv::waitKey();*/

		*gencloud = *RtabmapSlamControl::instance().generatePointClouddowin(depth, rgb, down);
		//*gencloud = *RtabmapSlamControl::instance().generatePointCloud(redepth, rergb);
		mStillPointCloud = RtabmapSlamControl::instance().PclToOpen3d(gencloud);

		m_rgb = rgb;
		//#endif // TEST_2D


		//point cloud data load
		std::string plypath = FilePath + "_SlamData.ply";

		//250428 ply  file ���� ���� Ȯ�� �� ������ ply load,, ������ depth �̹����� �̿��� ����ƮŬ���� ������ �����.

		std::ifstream file(plypath);
		if (file.is_open())
		{
			open3d::io::ReadPointCloudOption opt;
			if (open3d::io::ReadPointCloudFromPLY(plypath, mMLEMPointcloud, opt) == false)
			{
				HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "PLY Data Load Fail", eLoggerType::ERROR_t);
				//return false;		//250310 sbkwon : point cloud �� �ɼ�,, �����Ͱ� ������� ǥ�� ����,, 2D�� ǥ��
			}

			RtabmapSlamControl::instance().LoadPlyFile(plypath);
		}
		else
		{
			//depth ������ �̿��Ѵ�.
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Depth Image to Point Cloud : ", eLoggerType::INFO);
			mMLEMPointcloud = RtabmapSlamControl::instance().PclToOpen3d(gencloud);
		}


		//list mode data load
		if (LoadListedListModeData(LMDPath) == false)
			return false;


		//������ Ÿ�� ������
		std::string engpath = FilePath + "_LmDEnergyData.csv";
		if (LoadListedEnergyTimeData(engpath) == false)
			return false;;




	}
	else
	{
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM File Data Load: " + std::to_string(bLoad), eLoggerType::INFO);

		mMLEMPointcloud = RtabmapSlamControl::instance().GetSlamPointCloud();//���� ; �ǽð� ���� data

		if (mMLEMPointcloud.points_.size() <= 0)
		{
			mMLEMPointcloud = RtabmapSlamControl::instance().GetOccupancyPointCloud();//stopslam���� ������ data
		}

		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "GetSlamPointCloud : " + std::to_string(mMLEMPointcloud.points_.size()), eLoggerType::INFO);

		//#ifdef TEST_2D
		//240621 

		//���� �������� �۾���� ���� �߻�	: 250325
		std::string rgbpath = FilePath + "_rgb.png";
		std::string depthpath = FilePath + "_depth.png";

		//data load
		//cv::Mat rgb = cv::imread(rgbpath/*, IMREAD_COLOR*/);
		////cv::Mat depth = cv::imread(depthpath);
		//cv::Mat depth = cv::imread(depthpath, IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
		//depth.convertTo(depth, CV_32F);

		//����
		cv::Mat rgb = RtabmapSlamControl::instance().GetCurrentVideoFrame1();
		cv::Mat depth = RtabmapSlamControl::instance().GetCurrentDepthFrame1();
		//����

		if (rgb.empty())
		{
			//rgb = RtabmapSlamControl::instance().GetCurrentVideoFrame1();
			rgb = cv::imread(rgbpath/*, IMREAD_COLOR*/);
		}

		if (depth.empty())
		{
			//depth = RtabmapSlamControl::instance().GetCurrentDepthFrame1();
			depth = cv::imread(depthpath, IMREAD_ANYDEPTH | IMREAD_ANYCOLOR);
			depth.convertTo(depth, CV_32F);
		}

		LoadListedListModeData(LMDPath);

		//resize
		//cv::Mat rergb, redepth;
		int down = 2;
		/*cv::resize(rgb, rergb, Size(rgb.cols / down, rgb.rows / down), INTER_LINEAR);
		cv::resize(depth, redepth, Size(rgb.cols / down, rgb.rows / down), INTER_LINEAR);*/

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr gencloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		*gencloud = *RtabmapSlamControl::instance().generatePointClouddowin(depth, rgb, down);
		//*gencloud = *RtabmapSlamControl::instance().generatePointCloud(redepth, rergb);

		mStillPointCloud = RtabmapSlamControl::instance().PclToOpen3d(gencloud);

		m_rgb = rgb;


		savepath = FilePath;
		//#endif // TEST_2D

	}

	return true;
}

//240429 �������� �������� ������ �ִ� ���� ����
bool HUREL::Compton::LahgiControl::CalMLEMList(const std::string& systemMPath, const std::vector<double>& energy, const std::vector<double>& EgateMin,
	const std::vector<double>& EgateMax, const double& minValuePer)
{
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Run count : " + std::to_string(energy.size()), eLoggerType::INFO);

#ifdef Test_Log
	std::ostringstream oss;
	for (size_t i = 0; i < energy.size(); ++i) {
		oss << energy[i];
		oss << ", "; // ������
		oss << EgateMin[i];
		oss << ", "; // ������
		oss << EgateMax[i];
		oss << " : "; // ������
	}

	oss << minValuePer;

	std::string result = oss.str();
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "CalMLEMList value : " + result, eLoggerType::INFO);
#endif // Test_Log


	if (energy.size() == 1)
	{
		if (CalMLEM(systemMPath, energy[0], EgateMin[0], EgateMax[0], minValuePer) == true)
		{
			if (mCalMLEMPointcloud.points_.size() > 0)	//250310 sbkwon : 
				m_vMLEMPC.push_back(mCalMLEMPointcloud);

			//250605 ����þ� ���� ����
			cv::Mat Filtered;
			cv::GaussianBlur(m_Cal2D, Filtered, Size(7, 7), 2, 2);
			m_Cal2D = Filtered;

			//250123 : RGB ����� ����
			cv::addWeighted(m_rgb, 1, m_Cal2D, 0.4, 0, m_Cal2D);
			m_2DMLEM.push_back(m_Cal2D.clone());	//250107
			m_Cal2D.setTo(cv::Scalar(0, 0, 0));

		}
		else
		{
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Run Fail " + std::to_string(energy[0]), eLoggerType::ERROR_t);
			return false;
		}
	}
	else if (energy.size() > 1)
	{
		std::shared_ptr < open3d::geometry::PointCloud> pc = std::make_shared < open3d::geometry::PointCloud >(mMLEMPointcloud);

		HUREL::Compton::ReconPointCloud reconPC_all = HUREL::Compton::ReconPointCloud(*pc);

		HUREL::Compton::ReconPointCloud reconPC = *reconPC_all.VoxelDownSample(0.04);	//0.06
		for (int i = 0; i < reconPC.colors_.size(); i++)
		{
			reconPC.colors_[i](0) = 0;
			reconPC.colors_[i](1) = 0;
			reconPC.colors_[i](2) = 0;
		}

		//point cloud ȹ�� ���� ��� �Ʒ� �ڵ� ���� ������ Ȯ�� �ʿ�..
		std::shared_ptr < open3d::geometry::PointCloud> reconpc_ptr_total = std::make_shared<open3d::geometry::PointCloud>(reconPC);

		cv::Mat caltemp(m_rgb.rows, m_rgb.cols, CV_8UC3, cv::Scalar(0, 0, 0));	//250107
		//��� �������� ���� EM������ ������ �� ������ ���� ���Ѵ�
		for (size_t i = 0; i < energy.size(); i++)
		{
			if (CalMLEM(systemMPath, energy[i], EgateMin[i], EgateMax[i], minValuePer) == true)
			{
				if (reconpc_ptr_total->points_.size() > 0)	//250310 sbkwon : 
				{
					for (int i = 0; i < reconpc_ptr_total->colors_.size(); ++i)
					{
						reconpc_ptr_total->colors_[i](0) += mCalMLEMPointcloud.colors_[i](0);
						reconpc_ptr_total->colors_[i](1) += mCalMLEMPointcloud.colors_[i](1);
						reconpc_ptr_total->colors_[i](2) += mCalMLEMPointcloud.colors_[i](2);
					}
				}

				//2D : 250107
				for (size_t i = 0; i < caltemp.rows; i++)
				{
					for (size_t j = 0; j < caltemp.cols; j++)
					{
						caltemp.at<cv::Vec3b>(i, j)[0] += m_Cal2D.at<cv::Vec3b>(i, j)[0];
						caltemp.at<cv::Vec3b>(i, j)[1] += m_Cal2D.at<cv::Vec3b>(i, j)[1];
						caltemp.at<cv::Vec3b>(i, j)[2] += m_Cal2D.at<cv::Vec3b>(i, j)[2];
					}
				}

				m_Cal2D.setTo(cv::Scalar(0, 0, 0));
			}
			else
			{
				HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Run Fail " + std::to_string(energy[i]), eLoggerType::ERROR_t);
				return false;
			}
		}

		if (reconpc_ptr_total->points_.size() > 0)
			m_vMLEMPC.push_back(*reconpc_ptr_total);
		else
			m_vMLEMPC.push_back(open3d::geometry::PointCloud());

		//250605 ����þ� ���� ����
		cv::Mat Filtered;
		cv::GaussianBlur(caltemp, Filtered, Size(7, 7), 2, 2);
		caltemp = Filtered;

		//250123 : RGB ����� ����
		cv::addWeighted(m_rgb, 1, caltemp, 0.4, 0, caltemp);
		m_2DMLEM.push_back(caltemp);	//250107
		return true;
	}
	else
	{
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Run Fail : energy size is 0", eLoggerType::ERROR_t);
		return false;
	}

	return true;
}
//2404 : MLEM
bool HUREL::Compton::LahgiControl::CalMLEM(const std::string& systemMPath, const double& energy, const double& EgateMin, const double& EgateMax, const double& minValuePer)
{
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Run : " + std::to_string(energy), eLoggerType::INFO);
	//Systemmatrix index,
	int energyIndex = GetSystemMatrixIndex(energy);
	if (energyIndex < 0 || energyIndex > 19)
	{
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "System Matrix Index error : energy - " + std::to_string(energy), eLoggerType::ERROR_t);
		return false;
	}

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "system matrix Index : " + std::to_string(energyIndex), eLoggerType::INFO);

	std::shared_ptr < open3d::geometry::PointCloud> pc = std::make_shared < open3d::geometry::PointCloud >(mMLEMPointcloud);
	std::vector<ListModeData> lmData = GetListedListModeData();

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "System matrix load", eLoggerType::INFO);
	//Load Coded Geant4 systemmatrix
	Eigen::MatrixXd Geant4SystemMatrix;
	FILE* pFile = fopen(systemMPath.c_str(), "rb"); //read mode 

	if (pFile == NULL)
	{
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "System Matrix File Open error", eLoggerType::ERROR_t);
		return false;
	}

	double buffer[23];
	Geant4SystemMatrix = MatrixXd(14641, 3600); //20250416
	//                  detetor pos(i, j)
	//           (-0.16, -0.16) (-0.12, -0.16) (-0.08, -0.16) ......
	// (azm, alt) --------------------------------------------------
	// (-65, -65) |
	// (-60, -65) | System Matrix
	// (-55, -65) |
	int indexIazm = 0;
	/* 20250416
	for (double azm = -65; azm <= 65; ++azm)
	{
		int indexIalt = 0;
		for (double alt = -65; alt <= 65; ++alt)
		{
			int indexIx = 0;
			for (double x = -147.5; x <= 147.5; x += 5)
			{
				int indexIy = 0;
				for (double y = -147.5; y <= 147.5; y += 5)
				{
					int test = fread(buffer, sizeof(double), 23, pFile);
					double value = buffer[energyIndex + 3];	//
					int i = indexIazm + indexIalt * 131;
					int j = indexIx + indexIy * 60;
					Geant4SystemMatrix(i, j) = value;
					++indexIy;
				}
				++indexIx;
			}
			++indexIalt;
		}
		++indexIazm;
	}
	*/
	for (double azm = -60; azm <= 60; ++azm)
	{
		int indexIalt = 0;
		for (double alt = -60; alt <= 60; ++alt)
		{
			int indexIx = 0;
			for (double x = -147.5; x <= 147.5; x += 5)
			{
				int indexIy = 0;
				for (double y = -147.5; y <= 147.5; y += 5)
				{
					int test = fread(buffer, sizeof(double), 23, pFile);
					double value = buffer[energyIndex + 3];
					int i = indexIazm + indexIalt * 121;
					int j = indexIx + indexIy * 60;
					Geant4SystemMatrix(i, j) = value;
					++indexIy;
				}
				++indexIx;
			}
			++indexIalt;
		}
		++indexIazm;
	}

	//MatrixXd sensitivityMatrix = Geant4SystemMatrix.rowwise().sum();    //not used
	fclose(pFile);          //���� �ݱ�

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "System matrix End", eLoggerType::INFO);

	//Get Listmode data corresponding to Egate, Compton events and Coded events each
	Eigen::MatrixXd Egate(1, 2);
	Egate(0, 0) = EgateMin;
	Egate(0, 1) = EgateMax;

	double drawingPortion = minValuePer;

	Recon3dPostProcess Postrecon;

	//test log
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Cal E : " + std::to_string(energy), eLoggerType::INFO);


	std::vector<ListModeData> ComptonlmDataEff = Postrecon.GetListedComptonListModeData(lmData, Egate);
	std::vector<ListModeData> CodedlmDataEff = Postrecon.GetListedCodedListModeData(lmData, Egate);


#ifdef Test_Log
	std::ostringstream oss;
	oss << lmData.size() << " , CC :" << ComptonlmDataEff.size() << ", CA : " << CodedlmDataEff.size();
	std::string stCnt = oss.str();
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "LM Count : " + stCnt, eLoggerType::INFO);
#endif // Test_Log



	//Hybrid EM 
	HUREL::Compton::ReconPointCloud reconPC_all = HUREL::Compton::ReconPointCloud(*pc);
	//
	for (int i = 0; i < reconPC_all.points_.size(); ++i)
	{
		reconPC_all.reconValues_[i] = 0;
	}

	HUREL::Compton::ReconPointCloud reconPC = *reconPC_all.VoxelDownSample(0.04);//0.06
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "downsmple size : " + std::to_string(reconPC.points_.size())
		+ ", pointcloud size : " + std::to_string(reconPC_all.points_.size()), eLoggerType::INFO);

	if (ComptonlmDataEff.size() <= 0)
	{
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Compton LM Data size : " + std::to_string(ComptonlmDataEff.size()), eLoggerType::WARN);
		return false;
	}


	//250310 sbkwon : 3D ������ �ɼ�,, 2D ���� �켱
	// 
	//250214 : �����ð��� ª�� ��� pointCloud ������ �ȵǴ� ���� �߻�
	//240621 2D MLEM
	//������ transformation ����

	//mStillPointCloud = mStillPointCloud.Transform(lmData[lmData.size() - 1].DetectorTransformation);

#ifdef Test_Log
	std::stringstream ss;
	ss << lmData[lmData.size() - 1].DetectorTransformation;
	std::string matrix_str = ss.str();
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "DetectorTransformation: " + matrix_str, eLoggerType::INFO);
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "StillPointCloud cnt: " + std::to_string(mStillPointCloud.points_.size()), eLoggerType::INFO);
#endif

	std::shared_ptr < open3d::geometry::PointCloud> pc2D = std::make_shared < open3d::geometry::PointCloud >(mStillPointCloud);

	//������ transformation ����
	(*pc2D).Transform(lmData[lmData.size() - 1].DetectorTransformation);

	HUREL::Compton::ReconPointCloud recon2DPC = HUREL::Compton::ReconPointCloud(*pc2D);

	recon2DPC.maxReconValue = -DBL_MAX;
	for (int i = 0; i < recon2DPC.points_.size(); ++i)
	{
		recon2DPC.reconValues_[i] = 0;
	}

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Reconstruct2dPostProcessing  start: ", eLoggerType::INFO);
	Reconstruct2dPostProcessing(ComptonlmDataEff, CodedlmDataEff, Egate, Geant4SystemMatrix, *pc2D, &recon2DPC);
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Reconstruct2dPostProcessing  End: ", eLoggerType::INFO);

	std::shared_ptr < open3d::geometry::PointCloud> reconpc_2D_ptr_total = std::make_shared<open3d::geometry::PointCloud>(recon2DPC);

	int width = 848, height = 480;
	int down = 2;
	int ndownW = width / down;
	int ndownH = height / down;
	cv::Mat down2DImage(ndownH, ndownW, CV_8UC3, Scalar(0, 0, 0));

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Max : "
		+ std::to_string(recon2DPC.maxReconValue), eLoggerType::INFO);

	int nPCDIndex = 0;

	for (size_t m = 0; m < ndownH; m++)
	{
		for (size_t n = 0; n < ndownW; n++)
		{
			if (recon2DPC.reconValues_[nPCDIndex] < drawingPortion * recon2DPC.maxReconValue)
			{
				//grayscale
				double grayScaleValue = reconpc_2D_ptr_total->colors_[nPCDIndex](0) * 0.299 + reconpc_2D_ptr_total->colors_[nPCDIndex](1) * 0.587
					+ reconpc_2D_ptr_total->colors_[nPCDIndex](2) * 0.114;
				reconpc_2D_ptr_total->colors_[nPCDIndex](0) = grayScaleValue;
				reconpc_2D_ptr_total->colors_[nPCDIndex](1) = grayScaleValue;
				reconpc_2D_ptr_total->colors_[nPCDIndex](2) = grayScaleValue;

			}
			else
			{
				RGBA_t rgb = HUREL::Compton::ReconPointCloud::ColorScaleJet(recon2DPC.reconValues_[nPCDIndex], drawingPortion * recon2DPC.maxReconValue, recon2DPC.maxReconValue);
				reconpc_2D_ptr_total->colors_[nPCDIndex](0) = rgb.R;
				reconpc_2D_ptr_total->colors_[nPCDIndex](1) = rgb.G;
				reconpc_2D_ptr_total->colors_[nPCDIndex](2) = rgb.B;


				down2DImage.at<cv::Vec3b>(m, n)[0] = rgb.B * 255;
				down2DImage.at<cv::Vec3b>(m, n)[1] = rgb.G * 255;
				down2DImage.at<cv::Vec3b>(m, n)[2] = rgb.R * 255;

			}
			nPCDIndex++;
		}
	}

	cv::resize(down2DImage, down2DImage, Size(m_rgb.cols, m_rgb.rows), INTER_LANCZOS4);

	//����þ� ���� ����
	/*cv::Mat Filtered;
	cv::GaussianBlur(down2DImage, Filtered, Size(7, 7), 2, 2);
	down2DImage = Filtered;*/

	m_Cal2D = down2DImage.clone();	//250107

#ifdef Test_Log
	////test save
	std::string name = std::to_string(energy);
	std::string path = "E:\\" + name + ".png";
	cv::imwrite(path, down2DImage.clone());

	open3d::io::WritePointCloudOption option;
	std::string filePath = "E:\\" + name + "_SlamData.ply";
	//open3d::io::WritePointCloudToPLY(filePath, pc, option);
	{
		open3d::io::WritePointCloud(filePath, *reconpc_2D_ptr_total, option);
	}
	////test save
#endif


	//3D
	if (reconPC.points_.size() <= 0)
	{
		/*HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "downsmple size : " + std::to_string(reconPC.points_.size())
			+ ", pointcloud size : " + std::to_string(reconPC_all.points_.size()), eLoggerType::WARN);*/
			//return false;

		mCalMLEMPointcloud = open3d::geometry::PointCloud();
	}
	else
	{

		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Reconstruct3dPostProcessing  start: ", eLoggerType::INFO);

		Reconstruct3dPostProcessing(ComptonlmDataEff, CodedlmDataEff, Egate, Geant4SystemMatrix, *pc, &reconPC);

		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Reconstruct3dPostProcessing  End: ", eLoggerType::INFO);

		std::shared_ptr < open3d::geometry::PointCloud> reconpc_ptr_total = std::make_shared<open3d::geometry::PointCloud>(reconPC);

		for (int i = 0; i < reconpc_ptr_total->colors_.size(); ++i)
		{
			Vector3d color;
			Vector3d point;

			if (reconPC.reconValues_[i] < drawingPortion * reconPC.maxReconValue)
			{
				//grayscale
				double grayScaleValue = reconpc_ptr_total->colors_[i](0) * 0.299 + reconpc_ptr_total->colors_[i](1) * 0.587 + reconpc_ptr_total->colors_[i](2) * 0.114;
				reconpc_ptr_total->colors_[i](0) = grayScaleValue;
				reconpc_ptr_total->colors_[i](1) = grayScaleValue;
				reconpc_ptr_total->colors_[i](2) = grayScaleValue;
				//color
				//reconpc_ptr_total->colors_[i] = reconpc_ptr_total->colors_[i];
			}
			else
			{
				RGBA_t rgb = HUREL::Compton::ReconPointCloud::ColorScaleJet(reconPC.reconValues_[i], drawingPortion * reconPC.maxReconValue, reconPC.maxReconValue);
				reconpc_ptr_total->colors_[i](0) = rgb.R;
				reconpc_ptr_total->colors_[i](1) = rgb.G;
				reconpc_ptr_total->colors_[i](2) = rgb.B;
			}
		}

		mCalMLEMPointcloud = *reconpc_ptr_total;


#ifdef Test_Log
		open3d::io::WritePointCloudOption option1;
		std::string filePath1 = "E:\\" + name + "_3D__SlamData.ply";
		//open3d::io::WritePointCloudToPLY(filePath, pc, option);
		{
			open3d::io::WritePointCloud(filePath1, *reconpc_ptr_total, option1);
		}
		////test save
#endif

	}
	//3D



	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM End", eLoggerType::INFO);

	return true;//reconPC
}

//2404 : MLEM : NEC Data ����
open3d::geometry::PointCloud HUREL::Compton::LahgiControl::Reconstruct3dPostProcessing(std::vector<ListModeData>& ComptonlmDataEff,
	std::vector<ListModeData>& CodedlmDataEff, Eigen::MatrixXd& Egate,
	Eigen::MatrixXd& systemmatrix, open3d::geometry::PointCloud& pc, HUREL::Compton::ReconPointCloud* outReconPC)
{
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Reconstruct3dPostProcessing Start : " + std::to_string(ComptonlmDataEff.size()) + ", " +
		std::to_string(CodedlmDataEff.size()), eLoggerType::INFO);

	HUREL::Compton::ReconPointCloud tempreconPCtotal_all = HUREL::Compton::ReconPointCloud(pc);
	HUREL::Compton::ReconPointCloud tempreconPCtotal = *tempreconPCtotal_all.VoxelDownSample(0.04); //0.06
	HUREL::Compton::ReconPointCloud& reconPC = *outReconPC;

	HUREL::Compton::ReconPointCloud CCtempreconPC;
	HUREL::Compton::ReconPointCloud CAtempreconPC;
	HUREL::Compton::ReconPointCloud ComptonreconPC;
	HUREL::Compton::ReconPointCloud CodedreconPC;

	size_t posenum = 0;
	int casenum2 = 0;
	int Comptonlmposchkstart = 0;
	int Comptonlmposchkend = 0;
	int Codedlmposchkstart = 0;
	int Codedlmposchkend = 0;

	//double dZero = 0.0; double InfN = 1 / dZero;
	int tmphy = 0;	//���� ���� �ε���
	int tmpcc = 0;
	int tmpca = 0;
	int tmpCodedlmdatasize = 0;
	int posaccumN = 15;

	Eigen::MatrixXd CCSystemMatrix(1, reconPC.points_.size());
	Eigen::MatrixXd CASystemMatrix(reconPC.points_.size(), 3600); //60*60 pixel size
	Eigen::MatrixXd emptyCCSystemMatrix;
	Eigen::MatrixXd emptyCASystemMatrix;
	Eigen::MatrixXd posCCSystemMatrix;
	Eigen::MatrixXd posCASystemMatrix;
	Eigen::MatrixXd tmpCCSystemMatrix;
	Eigen::MatrixXd tmpCASystemMatrix;
	Eigen::MatrixXd CASystemMatrixResized;

	double peakEnergy;
	int fovsizechk = 0;
	Eigen::MatrixXd CodedNECSignal = MatrixXd::Zero(1, posaccumN);
	Eigen::MatrixXd CodedNECBackground = MatrixXd::Zero(1, posaccumN);
	Eigen::MatrixXd ComptonNECSignal = MatrixXd::Zero(1, posaccumN);
	Eigen::MatrixXd ComptonNECBackground = MatrixXd::Zero(1, posaccumN);
	double SignalCount;   double NoiseCount; double NEC;

	while (posenum == 0)
	{
		bool bReset = false;
		Recon3dPostProcess Post3drecon;

		////////////////////////////////////////////////////////////// ����
		Codedlmposchkend = Post3drecon.sortposition2(Codedlmposchkstart, CodedlmDataEff);	//coded end ���� ã��
		//Comptonlmposchkend = Post3drecon.sortpositionCompton(Comptonlmposchkstart, ComptonlmDataEff);	//compton end ���� ã��

		if (Codedlmposchkstart >= 0 && Codedlmposchkstart < CodedlmDataEff.size())
			Comptonlmposchkend = Post3drecon.sortposition3(Comptonlmposchkstart, ComptonlmDataEff, CodedlmDataEff[Codedlmposchkstart].DetectorTransformation);	//coded end ���� ã��
		else
		{
			Comptonlmposchkend = 0;
			Codedlmposchkend = 0;
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "3D start else : " + std::to_string(Comptonlmposchkstart), eLoggerType::INFO);
		}

#ifdef Test_Log
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "3D start : "
			+ std::to_string(Codedlmposchkstart) + " : "
			+ std::to_string(Codedlmposchkend) + " : "
			+ std::to_string(Comptonlmposchkstart) + " : "
			+ std::to_string(Comptonlmposchkend), eLoggerType::INFO);
#endif // Test_Log

		std::vector<ListModeData> posComptonlmDataEff;	//���� ���� compton listmode data
		std::vector<ListModeData> posCodedlmDataEff;	//���� ���� coded listmode data
		/*std::vector<ListModeData> posComptonlmDataNEC;
		std::vector<ListModeData> posCodedlmDataNEC;*/

		HUREL::Compton::ReconPointCloud reconPCFOVlim;
		HUREL::Compton::ReconPointCloud reconPCFOVlimtranposed;
		Eigen::MatrixXd fovchk(1, reconPC.points_.size());
		Eigen::Matrix4d transformation(4, 4);

		//List Mode data ���� ���� ������
		casenum2 = Post3drecon.sortposlmData(Codedlmposchkend, Codedlmposchkstart,
			Comptonlmposchkstart, Comptonlmposchkend,
			ComptonlmDataEff, CodedlmDataEff,
			&posCodedlmDataEff, &posComptonlmDataEff,
			&transformation);

#ifdef Test_Log
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
			"3D Sort : "
			+ std::to_string(casenum2) + " : "
			+ std::to_string(posCodedlmDataEff.size()) + " : "
			+ std::to_string(posComptonlmDataEff.size()), eLoggerType::INFO);
#endif // Test_Log
		//Post3drecon.sortposlmDataNEC(CodedlmDataNEC, ComptonlmDataNEC,&posCodedlmDataNEC, &posComptonlmDataNEC, transformation);

		Post3drecon.ComptonNECCal(posComptonlmDataEff, tmphy,
			Egate, &ComptonNECSignal, &ComptonNECBackground);	//signal, backgroun value calc
		Post3drecon.CodedNECCal(posCodedlmDataEff, tmphy, Egate,
			&CodedNECSignal, &CodedNECBackground);	//signal, backgroun value calc

		SignalCount = ComptonNECSignal.sum() + CodedNECSignal.sum();
		NoiseCount = ComptonNECBackground.sum() + CodedNECBackground.sum();
		NEC = (SignalCount * SignalCount) / (SignalCount + NoiseCount);


#ifdef Test_Log
		std::stringstream ss1;
		ss1 << SignalCount << " : ";
		ss1 << NoiseCount << " : ";
		ss1 << NEC << " : ";
		std::string nec_str = ss1.str();
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
			"SignalCount : NoiseCount : NEC - " + nec_str, eLoggerType::INFO);
#endif

		Post3drecon.imspacelim(reconPC, &transformation, 65, 65,
			&reconPCFOVlim, &fovchk);	//���� ���� ���� ���� �� fovchk�� �ش� ���� 1�� ����

		reconPCFOVlimtranposed = reconPCFOVlim;	//���� ������ ����ƮŬ���� ������
		Post3drecon.imspacetranspose(reconPCFOVlim, &transformation,
			&reconPCFOVlimtranposed);
		++tmphy;

#ifdef Test_Log
		std::stringstream ss;
		ss << transformation;
		std::string matrix_str = ss.str();
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "imspacetranspose Transformation: " + matrix_str, eLoggerType::INFO);
#endif

		/*HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "coded : " + std::to_string(posCodedlmDataEff.size())
			+ ", compton : " + std::to_string(posComptonlmDataEff.size())
			+ ", Nec : " + std::to_string(NEC)
			+ ", tmphy : " + std::to_string(tmphy)
			+ ", casenum2 : " + std::to_string(casenum2), eLoggerType::INFO);*/

		if (tmphy > posaccumN)	//�ִ� ���� �̻��� ��� ���� ���� ������ �ʱ�ȭ �� �ٽ� ����
		{
#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Clear" + std::to_string(tmphy), eLoggerType::INFO);
#endif // Test_Log

			CCSystemMatrix.resize(1, reconPC.points_.size()); tmphy = 0;//reset tmpCodedlmdatasize = 0; //reset
			tmpcc = 0; tmpca = 0; tmpCodedlmdatasize = 0;
			CASystemMatrix.resize(reconPC.points_.size(), 3600);
			Codedlmposchkstart = Codedlmposchkend; Comptonlmposchkstart = Comptonlmposchkend;
			CodedNECSignal = MatrixXd::Zero(1, posaccumN);
			CodedNECBackground = MatrixXd::Zero(1, posaccumN);
			ComptonNECSignal = MatrixXd::Zero(1, posaccumN);
			ComptonNECBackground = MatrixXd::Zero(1, posaccumN);


			continue;
		}

		//case 0 : Hybrid, 1 : coded, 2 : compton,  3 : nothing
		if (casenum2 == 0 || casenum2 == 1) //coded image recon
		{
			if (posCodedlmDataEff.size() < 10)	//���� ���� listmode data���� ����
			{
				Comptonlmposchkstart = Comptonlmposchkend;	//���� ���� ������ ������
				HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "coded min 10" + std::to_string(posCodedlmDataEff.size()), eLoggerType::INFO);
			}
			else
			{
				tmpca++;
				tmpCodedlmdatasize += posCodedlmDataEff.size();

				HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "coded recon : " + std::to_string(tmpCodedlmdatasize), eLoggerType::INFO);

				CodedreconPC = reconPCFOVlim;	//not used
				CAtempreconPC = reconPCFOVlimtranposed;

				posCASystemMatrix = MatrixXd::Zero(reconPCFOVlimtranposed.points_.size(), 3600);
				tmpCASystemMatrix = MatrixXd::Zero(reconPC.points_.size(), 3600);

				for (int i = 0; i < CAtempreconPC.points_.size(); ++i)
				{
					CAtempreconPC.reconValues_[i] = 0;
				}

				CAtempreconPC.CalculateCodedMLEMSystemMatrix(posCodedlmDataEff,
					systemmatrix, reconPCFOVlimtranposed,
					transformation, &posCASystemMatrix);

				fovsizechk = 0;
				for (size_t size1 = 0; size1 < reconPC.points_.size(); ++size1)
				{
					if (fovchk(0, size1) != 0)
					{
						tmpCASystemMatrix.row(size1) = posCASystemMatrix.row(fovsizechk);
						fovsizechk += 1;
					}
				}
				emptyCASystemMatrix.resize(CASystemMatrix.rows(), tmpCASystemMatrix.cols() + CASystemMatrix.cols());
				emptyCASystemMatrix << CASystemMatrix, tmpCASystemMatrix;
				CASystemMatrix.resize(emptyCASystemMatrix.rows(), emptyCASystemMatrix.cols());
				CASystemMatrix = emptyCASystemMatrix;
				Codedlmposchkstart = Codedlmposchkend;

				HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Case 0 or 1", eLoggerType::INFO);
			}
		}

		if (casenum2 == 0 || casenum2 == 2) //Compton image recon
		{
			tmpcc++;
			ComptonreconPC = reconPCFOVlim; CCtempreconPC = reconPCFOVlimtranposed;
			posCCSystemMatrix.resize(posComptonlmDataEff.size(), reconPCFOVlim.points_.size());
			tmpCCSystemMatrix.resize(posComptonlmDataEff.size(), reconPC.points_.size());

			for (int i = 0; i < CCtempreconPC.points_.size(); ++i)
			{
				CCtempreconPC.reconValues_[i] = 0;
			}

			CCtempreconPC.CalculateComptonMLEMSystemMatrix(posComptonlmDataEff, reconPCFOVlimtranposed, &posCCSystemMatrix, HUREL::Compton::ReconPointCloud::SimpleComptonMLEM);

			for (size_t size1 = 0; size1 < posComptonlmDataEff.size(); ++size1)
			{
				fovsizechk = 0;
				for (size_t size2 = 0; size2 < reconPC.points_.size(); ++size2)
				{
					if (fovchk(0, size2) != 0)
					{
						tmpCCSystemMatrix(size1, size2) = posCCSystemMatrix(size1, fovsizechk);
						fovsizechk = fovsizechk + 1;
					}
					else
					{
						tmpCCSystemMatrix(size1, size2) = 0;
					}
				}
			}
			emptyCCSystemMatrix.resize(CCSystemMatrix.rows() + tmpCCSystemMatrix.rows(), CCSystemMatrix.cols());
			emptyCCSystemMatrix << CCSystemMatrix,
				tmpCCSystemMatrix;
			CCSystemMatrix.resize(emptyCCSystemMatrix.rows(), emptyCCSystemMatrix.cols());
			CCSystemMatrix = emptyCCSystemMatrix;
			Comptonlmposchkstart = Comptonlmposchkend;

			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM case 0 or 2", eLoggerType::INFO);
		}

		if (casenum2 == 3) //coded image recon
		{
			posenum = 1;
			//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM case 3", eLoggerType::INFO);
			break;
		}

		// Compton Em iteration		
		if (CCSystemMatrix.rows() >= 16)
		{
			Post3drecon.removeRow(CCSystemMatrix, 0);  //CCSystemMatrix ù��° �� �����
			int iterationtotalN = 8;
			tempreconPCtotal.maxReconValue = -DBL_MAX;
			for (int k = 0; k < reconPC.points_.size(); ++k)
			{
				tempreconPCtotal.reconValues_[k] = 0;
			}

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"3D MLEM Compton Iteration : "
				+ std::to_string(CCSystemMatrix.rows()) + ", "
				+ std::to_string(CCSystemMatrix.cols()) + ", "
				+ std::to_string(tmpcc),
				eLoggerType::INFO);
#endif // Test_Log

			tempreconPCtotal.ReconComptonMLEMIteration(iterationtotalN, CCSystemMatrix);	//CCSystemMatrix ��� ���� �� tempreconPCtotal.reconValues_ ����

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"3D Compton Iteration maxReconValue : "
				+ std::to_string(iterationtotalN) + ", "
				+ std::to_string(tempreconPCtotal.maxReconValue),
				eLoggerType::INFO);

			int nCount = 0;
			int nNega = 0;
			double nSum = 0.0;
#endif // Test_Log

			if (tempreconPCtotal.maxReconValue > 0)	//max �� Ȯ�� �ʿ�
			{
				for (int k = 0; k < reconPC.points_.size(); ++k)
				{
					reconPC.reconValues_[k] = reconPC.reconValues_[k] + tempreconPCtotal.reconValues_[k] / tmpcc;// tmpcc;
#ifdef Test_Log
					//count 
					if (tempreconPCtotal.reconValues_[k] > 0)
						nCount++;

					if (tempreconPCtotal.reconValues_[k] > 1)
						nNega++;

					nSum += tempreconPCtotal.reconValues_[k];
#endif // Test_Log
				}
			}


#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"3D Compton Iteration C, N,, S : "
				+ std::to_string(nCount) + ", "
				+ std::to_string(nNega) + ", "
				+ std::to_string(nSum),
				eLoggerType::INFO);
#endif // Test_Log

			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Compton Iteration : " + std::to_string(CCSystemMatrix.rows()), eLoggerType::INFO);

			CCSystemMatrix.resize(1, reconPC.points_.size()); bReset = true;  tmpcc = 0; //reset
		}

		// Coded Em iteration

		if (tmpCodedlmdatasize >= 600)
		{
			CASystemMatrixResized.resize(CASystemMatrix.rows(), CASystemMatrix.cols() - 3600);
			CASystemMatrixResized = CASystemMatrix.block(0, 3600, CASystemMatrix.rows(), CASystemMatrix.cols() - 3600);
			int iterationtotalN = 10;
			tempreconPCtotal.maxReconValue = -DBL_MAX;
			for (int k = 0; k < reconPC.points_.size(); ++k)
			{
				tempreconPCtotal.reconValues_[k] = 0;
			}
			tempreconPCtotal.ReconCodedMLEMIteration(iterationtotalN, CASystemMatrix);

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"3D Coded Iteration maxReconValue : "
				+ std::to_string(tmpCodedlmdatasize) + ", "
				+ std::to_string(iterationtotalN) + ", "
				+ std::to_string(tempreconPCtotal.maxReconValue),
				eLoggerType::INFO);

			int nCount = 0;
			int nNega = 0;
			double nSum = 0.0;
#endif // Test_Log

			if (tempreconPCtotal.maxReconValue > 0)
			{
				for (int k = 0; k < reconPC.points_.size(); ++k)
				{
					reconPC.reconValues_[k] = reconPC.reconValues_[k]
						+ tempreconPCtotal.reconValues_[k] / tmphy;
#ifdef Test_Log
					//count 
					if (tempreconPCtotal.reconValues_[k] > 0)
						nCount++;

					if (tempreconPCtotal.reconValues_[k] > 1)
						nNega++;

					nSum += tempreconPCtotal.reconValues_[k];
#endif // Test_Log
				}
			}

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"3D Coded Iteration C, N,, S : "
				+ std::to_string(nCount) + ", "
				+ std::to_string(nNega) + ", "
				+ std::to_string(nSum) + ", "
				+ std::to_string(tmphy),
				eLoggerType::INFO);

			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM code Iteration : " + std::to_string(CASystemMatrix.rows()) + ", "
				+ std::to_string(CASystemMatrix.cols()) + ", "
				+ std::to_string(tmpCodedlmdatasize) + ", " + std::to_string(tmphy), eLoggerType::INFO);
#endif // Test_Log

			CASystemMatrix.resize(reconPC.points_.size(), 3600); tmpCodedlmdatasize = 0; bReset = true; tmpca = 0; //reset
		}


		//Hybrid EM iteration
//        if (tmpCodedlmdatasize >= 600 || CCSystemMatrix.rows() >= 16)
		if (NEC >= 500)
		{
			Post3drecon.removeRow(CCSystemMatrix, 0);  //CCSystemMatrix ù��° �� �����
			CASystemMatrixResized.resize(CASystemMatrix.rows(), CASystemMatrix.cols() - 3600);
			CASystemMatrixResized = CASystemMatrix.block(0, 3600, CASystemMatrix.rows(), CASystemMatrix.cols() - 3600);

			int iterationtotalN = 12;
			tempreconPCtotal.maxReconValue = -DBL_MAX;
			for (int k = 0; k < reconPC.points_.size(); ++k)
			{
				tempreconPCtotal.reconValues_[k] = 0;
			}
			tempreconPCtotal.ReconHybridMLEMIteration(iterationtotalN, CCSystemMatrix, CASystemMatrix, 10);

			if (tempreconPCtotal.maxReconValue > 0)
			{
				for (int k = 0; k < reconPC.points_.size(); ++k)
				{
					reconPC.reconValues_[k] = reconPC.reconValues_[k] + tempreconPCtotal.reconValues_[k] / tmphy;
				}
			}

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"3D MLEM Hybrid Iteration : "
				+ std::to_string(CCSystemMatrix.rows()) + ", "
				+ std::to_string(CCSystemMatrix.cols()) + ", "
				+ std::to_string(CASystemMatrix.rows()) + ", "
				+ std::to_string(CASystemMatrix.cols()) + ", "
				+ std::to_string(tempreconPCtotal.maxReconValue) + ", "
				+ std::to_string(iterationtotalN) + ", " + std::to_string(tmphy),
				eLoggerType::INFO);
#endif // Test_Log


			CCSystemMatrix.resize(1, reconPC.points_.size()); bReset = true; tmphy = 0; //reset
			CASystemMatrix.resize(reconPC.points_.size(), 3600); tmpCodedlmdatasize = 0;//reset    
			CodedNECSignal = MatrixXd::Zero(1, posaccumN);
			CodedNECBackground = MatrixXd::Zero(1, posaccumN);
			ComptonNECSignal = MatrixXd::Zero(1, posaccumN);
			ComptonNECBackground = MatrixXd::Zero(1, posaccumN);

			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Hybrid Iteration : " + std::to_string(NEC), eLoggerType::INFO);

			/*
			reconPC.maxReconValue = -DBL_MAX;
			for (size_t j = 0; j < reconPC.points_.size(); ++j)
			{
			   if (reconPC.reconValues_[j] > reconPC.maxReconValue)
			   {
				  reconPC.maxReconValue = reconPC.reconValues_[j];
			   }
			}
			std::shared_ptr < open3d::geometry::PointCloud> reconPCFOVlimpoint = std::make_shared<open3d::geometry::PointCloud>(reconPC);
			for (int i = 0; i < reconPCFOVlimpoint->colors_.size(); ++i)
			{
			   RGBA_t rgb = HUREL::Compton::ReconPointCloud::ColorScaleJet(reconPC.reconValues_[i], 0.1 * reconPC.maxReconValue, reconPC.maxReconValue);
			   reconPCFOVlimpoint->colors_[i](0) = rgb.R;
			   reconPCFOVlimpoint->colors_[i](1) = rgb.G;
			   reconPCFOVlimpoint->colors_[i](2) = rgb.B;
			}
			open3d::visualization::DrawGeometries({ reconPCFOVlimpoint });
		   HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Compton Iteration", eLoggerType::INFO);
			*/
		}

		/*if (bReset == true)
			tmphy = 0;*/
	}

	reconPC.maxReconValue = -DBL_MAX;
	for (size_t j = 0; j < reconPC.points_.size(); ++j)
	{
		if (reconPC.reconValues_[j] > reconPC.maxReconValue)
		{
			reconPC.maxReconValue = reconPC.reconValues_[j];
		}
	}

	//
	//std::shared_ptr < open3d::geometry::PointCloud> reconPCFOVlimpoint = std::make_shared<open3d::geometry::PointCloud>(reconPC);
	//
	//for (int i = 0; i < reconPCFOVlimpoint->colors_.size(); ++i)
	//{
	//	RGBA_t rgb = HUREL::Compton::ReconPointCloud::ColorScaleJet(reconPC.reconValues_[i], 0.1 * reconPC.maxReconValue, reconPC.maxReconValue);
	//	reconPCFOVlimpoint->colors_[i](0) = rgb.R;
	//	reconPCFOVlimpoint->colors_[i](1) = rgb.G;
	//	reconPCFOVlimpoint->colors_[i](2) = rgb.B;
	//}

	////Ȯ�ο� ȭ��â ǥ��
	//open3d::visualization::DrawGeometries({ reconPCFOVlimpoint });

#ifdef Test_Log
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
		"3D Recon Max Value : " + std::to_string(reconPC.maxReconValue), eLoggerType::INFO);
#endif // Test_Log
	return pc;
}

//240429
open3d::geometry::PointCloud HUREL::Compton::LahgiControl::GetMLEMPointCloud(const int& nNo)
{
	if (m_vMLEMPC.size() > nNo && nNo >= 0)
		return m_vMLEMPC[nNo];
	else
		return open3d::geometry::PointCloud();
}

//250107 MLEM 2D
bool HUREL::Compton::LahgiControl::GetMLEM2D(const std::string& FilePath, const int& nNo)
{
	std::string save2D = FilePath + "_MLEM2D_" + std::to_string(nNo) + ".png";



	if (m_2DMLEM.size() > nNo && nNo >= 0)
	{
		//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM 2d save" + savepath, eLoggerType::INFO);
		bool ret = cv::imwrite(save2D, m_2DMLEM[nNo].clone());
		//cv::imwrite("_MLEM2DSelect.png", m_2DMLEM[nNo].clone());


		//test log
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM 2D : Path, " + save2D
			+ " , " + std::to_string(nNo) + " , " + std::to_string(ret), eLoggerType::INFO);

		return ret;
	}
	else
	{
		//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM 2d fail" + save2D, eLoggerType::INFO);
		return false;
	}
}

//240621 2D MLEM
void HUREL::Compton::LahgiControl::Reconstruct2dPostProcessing(std::vector<ListModeData>& ComptonlmDataEff, std::vector<ListModeData>& CodedlmDataEff,
	Eigen::MatrixXd& Egate, Eigen::MatrixXd& systemmatrix,
	open3d::geometry::PointCloud& pc, HUREL::Compton::ReconPointCloud* outReconPC)
{
	HUREL::Compton::ReconPointCloud& reconPC = *outReconPC;

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Reconstruct2dPostProcessing Start : " + std::to_string(ComptonlmDataEff.size()) + ", " +
		std::to_string(CodedlmDataEff.size()), eLoggerType::INFO);

	reconPC.maxReconValue = -DBL_MAX;
	for (int k = 0; k < reconPC.points_.size(); ++k)
	{
		reconPC.reconValues_[k] = 0;
	}

	HUREL::Compton::ReconPointCloud tempreconPCtotal = HUREL::Compton::ReconPointCloud(pc);
	//HUREL::Compton::ReconPointCloud tempreconPCtotal = *tempreconPCtotal_all.VoxelDownSample(0.06);

	HUREL::Compton::ReconPointCloud CCtempreconPC;
	HUREL::Compton::ReconPointCloud CAtempreconPC;
	HUREL::Compton::ReconPointCloud ComptonreconPC;
	HUREL::Compton::ReconPointCloud CodedreconPC;

	size_t posenum = 0;
	int casenum2 = 0;
	int Comptonlmposchkstart = 0;
	int Comptonlmposchkend = 0;
	int Codedlmposchkstart = 0;
	int Codedlmposchkend = 0;

	int tmpCodedlmdatasize = 0;
	int posaccumN = 15;
	int tmphy = 0;	//���� ���� �ε���
	int tmpcc = 0;
	int tmpca = 0;

	Eigen::MatrixXd CASystemMatrix(reconPC.points_.size(), 3600); //60*60 pixel size
	Eigen::MatrixXd emptyCASystemMatrix;
	Eigen::MatrixXd CCSystemMatrix(1, reconPC.points_.size());
	Eigen::MatrixXd emptyCCSystemMatrix;

	Eigen::MatrixXd CodedNECSignal = MatrixXd::Zero(1, posaccumN);
	Eigen::MatrixXd CodedNECBackground = MatrixXd::Zero(1, posaccumN);
	Eigen::MatrixXd ComptonNECSignal = MatrixXd::Zero(1, posaccumN);
	Eigen::MatrixXd ComptonNECBackground = MatrixXd::Zero(1, posaccumN);
	double SignalCount;   double NoiseCount; double NEC;

	Recon3dPostProcess Post3drecon;

	Eigen::MatrixXd posCCSystemMatrix;
	Eigen::MatrixXd posCASystemMatrix;


	while (posenum == 0)
	{

		//pose�� list mode data �з�
		Codedlmposchkend = Post3drecon.sortposition2(Codedlmposchkstart, CodedlmDataEff);	//coded end ���� ã��
		//Comptonlmposchkend = Post3drecon.sortpositionCompton(Comptonlmposchkstart, ComptonlmDataEff);	//compton end ���� ã��

		if (Codedlmposchkstart >= 0 && Codedlmposchkstart < CodedlmDataEff.size())
			Comptonlmposchkend = Post3drecon.sortposition3(Comptonlmposchkstart, ComptonlmDataEff, CodedlmDataEff[Codedlmposchkstart].DetectorTransformation);	//coded end ���� ã��
		else
		{
			Comptonlmposchkend = 0;
			Codedlmposchkend = 0;
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "2D start else : " + std::to_string(Comptonlmposchkstart), eLoggerType::INFO);
		}

#ifdef Test_Log
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "2D start : "
			+ std::to_string(Codedlmposchkstart) + " : "
			+ std::to_string(Codedlmposchkend) + " : "
			+ std::to_string(Comptonlmposchkstart) + " : "
			+ std::to_string(Comptonlmposchkend), eLoggerType::INFO);
#endif // Test_Log

		//�з��� data
		std::vector<ListModeData> posComptonlmDataEff;	//���� ���� compton listmode data
		std::vector<ListModeData> posCodedlmDataEff;	//���� ���� coded listmode data

		HUREL::Compton::ReconPointCloud reconPCFOVlimtranposed;	//���� pose�� transformation ������ PCD
		Eigen::Matrix4d transformation(4, 4);	//���� pose�� transformation data
		//List Mode data ���� ���� ������ - ����ȭ ��� casenum2�� ����
		casenum2 = Post3drecon.sortposlmData(Codedlmposchkend, Codedlmposchkstart,
			Comptonlmposchkstart, Comptonlmposchkend,
			ComptonlmDataEff, CodedlmDataEff,
			&posCodedlmDataEff,
			&posComptonlmDataEff, &transformation);

#ifdef Test_Log
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
			"2D Sort : "
			+ std::to_string(casenum2) + " : "
			+ std::to_string(posCodedlmDataEff.size()) + " : "
			+ std::to_string(posComptonlmDataEff.size()), eLoggerType::INFO);
#endif // Test_Log


		//NEC ���
		Post3drecon.ComptonNECCal(posComptonlmDataEff, tmphy, Egate, &ComptonNECSignal, &ComptonNECBackground);	//signal, backgroun value calc
		Post3drecon.CodedNECCal(posCodedlmDataEff, tmphy, Egate, &CodedNECSignal, &CodedNECBackground);	//signal, backgroun value calc

		SignalCount = ComptonNECSignal.sum() + CodedNECSignal.sum();
		NoiseCount = ComptonNECBackground.sum() + CodedNECBackground.sum();
		NEC = (SignalCount * SignalCount) / (SignalCount + NoiseCount);

#ifdef Test_Log
		std::stringstream ss1;
		ss1 << SignalCount << " : ";
		ss1 << NoiseCount << " : ";
		ss1 << NEC << " : ";
		std::string nec_str = ss1.str();
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
			"SignalCount : NoiseCount : NEC - " + nec_str, eLoggerType::INFO);
#endif

		//transformation ����
		reconPCFOVlimtranposed = reconPC;
		Post3drecon.imspacetranspose2D(reconPC, &transformation,
			&reconPCFOVlimtranposed);

#ifdef Test_Log
		std::stringstream ss;
		ss << transformation;
		std::string matrix_str = ss.str();
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "imspacetranspose2D Transformation: " + matrix_str, eLoggerType::INFO);
#endif

		++tmphy;

		//pose �ʱ�ȭ
		if (tmphy > posaccumN)	//�ִ� ���� �̻��� ��� ���� ���� ������ �ʱ�ȭ �� �ٽ� ����
		{
#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM 2D Clear" + std::to_string(tmphy), eLoggerType::INFO);
#endif // Test_Log

			tmphy = 0; tmpcc = 0; tmpca = 0;
			tmpCodedlmdatasize = 0;
			Codedlmposchkstart = Codedlmposchkend; Comptonlmposchkstart = Comptonlmposchkend;

			CASystemMatrix.resize(reconPC.points_.size(), 3600);
			CCSystemMatrix.resize(1, reconPC.points_.size());

			CodedNECSignal = MatrixXd::Zero(1, posaccumN);
			CodedNECBackground = MatrixXd::Zero(1, posaccumN);
			ComptonNECSignal = MatrixXd::Zero(1, posaccumN);
			ComptonNECBackground = MatrixXd::Zero(1, posaccumN);

			continue;
		}


		//case 0 : Hybrid, 1 : coded, 2 : compton,  3 : nothing
		if (casenum2 == 0 || casenum2 == 1) //coded image recon
		{
			if (posCodedlmDataEff.size() < 10)	//���� ���� listmode data���� ����
			{
				Comptonlmposchkstart = Comptonlmposchkend;	//���� ���� ������ ������
				HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "2D - coded min 10" + std::to_string(posCodedlmDataEff.size()), eLoggerType::INFO);
			}
			else
			{
				tmpca++;
				tmpCodedlmdatasize += posCodedlmDataEff.size();

#ifdef Test_Log
				HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "coded recon : " + std::to_string(tmpCodedlmdatasize), eLoggerType::INFO);
#endif // Test_Log

				CAtempreconPC = reconPCFOVlimtranposed;

				posCASystemMatrix = MatrixXd::Zero(reconPCFOVlimtranposed.points_.size(), 3600);	//system matrix ���� ���� ���

				for (int i = 0; i < CAtempreconPC.points_.size(); ++i)
				{
					CAtempreconPC.reconValues_[i] = 0;
				}

				//coded count �� system matrix ����
				CAtempreconPC.CalculateCodedMLEMSystemMatrix(posCodedlmDataEff, systemmatrix,
					reconPCFOVlimtranposed, transformation, &posCASystemMatrix);

				emptyCASystemMatrix.resize(CASystemMatrix.rows(), posCASystemMatrix.cols() + CASystemMatrix.cols());
				emptyCASystemMatrix << CASystemMatrix, posCASystemMatrix;
				CASystemMatrix.resize(emptyCASystemMatrix.rows(), emptyCASystemMatrix.cols());
				CASystemMatrix = emptyCASystemMatrix;

				Codedlmposchkstart = Codedlmposchkend;

#ifdef Test_Log
				HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "2D MLEM Case 0 or 1", eLoggerType::INFO);
#endif // Test_Log
			}
		}

		if (casenum2 == 0 || casenum2 == 2) //Compton image recon
		{
			tmpcc++;
			CCtempreconPC = reconPCFOVlimtranposed;

			posCCSystemMatrix.resize(posComptonlmDataEff.size(), reconPCFOVlimtranposed.points_.size());

			for (int i = 0; i < CCtempreconPC.points_.size(); ++i)
			{
				CCtempreconPC.reconValues_[i] = 0;
			}

			//compton count
			CCtempreconPC.CalculateComptonMLEMSystemMatrix(posComptonlmDataEff,
				reconPCFOVlimtranposed, &posCCSystemMatrix,
				HUREL::Compton::ReconPointCloud::SimpleComptonMLEM2D);

			emptyCCSystemMatrix.resize(CCSystemMatrix.rows() + posCCSystemMatrix.rows(),
				CCSystemMatrix.cols());
			emptyCCSystemMatrix << CCSystemMatrix,
				posCCSystemMatrix;
			CCSystemMatrix.resize(emptyCCSystemMatrix.rows(), emptyCCSystemMatrix.cols());
			CCSystemMatrix = emptyCCSystemMatrix;
			Comptonlmposchkstart = Comptonlmposchkend;

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "2D MLEM case 0 or 2", eLoggerType::INFO);
#endif // Test_Log
		}

		if (casenum2 == 3) //nothing
		{
			posenum = 1;
			//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM case 3", eLoggerType::INFO);
			break;
		}

		//EM Interation
		// Compton Em iteration		
		if (CCSystemMatrix.rows() >= 16)	//LMD 16�� �̻��� ���
		{
			Post3drecon.removeRow(CCSystemMatrix, 0);  //CCSystemMatrix ù��° �� �����

			int iterationtotalN = 8;

			//reset
			tempreconPCtotal.maxReconValue = -DBL_MAX;
			for (int k = 0; k < reconPC.points_.size(); ++k)
			{
				tempreconPCtotal.reconValues_[k] = 0;
			}

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"2D MLEM Compton Iteration : "
				+ std::to_string(CCSystemMatrix.rows()) + ", "
				+ std::to_string(CCSystemMatrix.cols()) + ", "
				+ std::to_string(NEC) + ", "
				+ std::to_string(tmpcc),
				eLoggerType::INFO);
#endif // Test_Log

			tempreconPCtotal.ReconComptonMLEMIteration(iterationtotalN, CCSystemMatrix);	//CCSystemMatrix ��� ���� �� tempreconPCtotal.reconValues_ ����

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"2D Compton Iteration maxReconValue : "
				+ std::to_string(iterationtotalN) + ", "
				+ std::to_string(tempreconPCtotal.maxReconValue),
				eLoggerType::INFO);

			int nCount = 0;
			int nNega = 0;
			double nSum = 0.0;
#endif // Test_Log

			if (tempreconPCtotal.maxReconValue > 0)
			{
				for (int k = 0; k < reconPC.points_.size(); ++k)
				{
					reconPC.reconValues_[k] = reconPC.reconValues_[k] +
						tempreconPCtotal.reconValues_[k] / tmpcc; // ����ġ

#ifdef Test_Log
					//count 
					if (tempreconPCtotal.reconValues_[k] > 0)
						nCount++;

					if (tempreconPCtotal.reconValues_[k] > 1)
						nNega++;

					nSum += tempreconPCtotal.reconValues_[k];
#endif // Test_Log
				}
			}

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"2D Compton Iteration C, N,, S : "
				+ std::to_string(nCount) + ", "
				+ std::to_string(nNega) + ", "
				+ std::to_string(nSum),
				eLoggerType::INFO);
#endif // Test_Log

			//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "MLEM Compton Iteration : " + std::to_string(CCSystemMatrix.rows()), eLoggerType::INFO);

			CCSystemMatrix.resize(1, reconPC.points_.size()); tmpcc = 0; //reset
			//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "2D  Compton Em iteration", eLoggerType::INFO);
		}

		if (tmpCodedlmdatasize >= 600)	//������ LMD ����
		{
			int iterationtotalN = 10;

			//reset
			tempreconPCtotal.maxReconValue = -DBL_MAX;
			for (int k = 0; k < reconPC.points_.size(); ++k)
			{
				tempreconPCtotal.reconValues_[k] = 0;
			}

			tempreconPCtotal.ReconCodedMLEMIteration(iterationtotalN, CASystemMatrix);

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"2D Coded Iteration maxReconValue : "
				+ std::to_string(iterationtotalN) + ", "
				+ std::to_string(tmpCodedlmdatasize) + ", "
				+ std::to_string(tempreconPCtotal.maxReconValue),
				eLoggerType::INFO);

			int nCount = 0;
			int nNega = 0;
			double nSum = 0.0;
#endif // Test_Log


			//Co60 1332 ���������� Coded �������� ���� ��ü�� �̻������� ����
			if (tempreconPCtotal.maxReconValue > 1)		//0 -> 1
			{
				for (int k = 0; k < reconPC.points_.size(); ++k)
				{
					reconPC.reconValues_[k] = reconPC.reconValues_[k] +
						tempreconPCtotal.reconValues_[k] / tmphy;

#ifdef Test_Log
					//count 
					if (tempreconPCtotal.reconValues_[k] > 0)
						nCount++;
					if (tempreconPCtotal.reconValues_[k] == 0)
						nNega++;

					nSum += tempreconPCtotal.reconValues_[k];
#endif // Test_Log

				}
			}

#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"2D Coded Iteration C, N,, S : "
				+ std::to_string(nCount) + ", "
				+ std::to_string(nNega) + ", "
				+ std::to_string(nSum) + ", "
				+ std::to_string(tmphy),
				eLoggerType::INFO);

			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"2D MLEM code Iteration : "
				+ std::to_string(CASystemMatrix.rows()) + ", "
				+ std::to_string(CASystemMatrix.cols()) + ", "
				+ std::to_string(tmpCodedlmdatasize) + ", " + std::to_string(tmphy),
				eLoggerType::INFO);
#endif // Test_Log

			CASystemMatrix.resize(reconPC.points_.size(), 3600); tmpCodedlmdatasize = 0; tmpca = 0; //reset

			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "2D  Coded Em iteration", eLoggerType::INFO);
		}

		//Hybrid EM iteration
		if (NEC >= 500)
		{
#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"2D MLEM Hybrid Iteration : "
				+ std::to_string(CCSystemMatrix.rows()) + ", "
				+ std::to_string(CCSystemMatrix.cols()),
				eLoggerType::INFO);
#endif // Test_Log

			Post3drecon.removeRow(CCSystemMatrix, 0);  //CCSystemMatrix ù��° �� �����

			int iterationtotalN = 10; // 3D : 12

			//reset
			tempreconPCtotal.maxReconValue = -DBL_MAX;
			for (int k = 0; k < reconPC.points_.size(); ++k)
			{
				tempreconPCtotal.reconValues_[k] = 0;
			}

			tempreconPCtotal.ReconHybridMLEMIteration(iterationtotalN, CCSystemMatrix, CASystemMatrix, 10);

			if (tempreconPCtotal.maxReconValue > 0)
			{
				for (int k = 0; k < reconPC.points_.size(); ++k)
				{
					reconPC.reconValues_[k] = reconPC.reconValues_[k] +
						tempreconPCtotal.reconValues_[k] / tmphy;
				}
			}


#ifdef Test_Log
			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"2D MLEM Hybrid Iteration : "
				+ std::to_string(CCSystemMatrix.rows()) + ", "
				+ std::to_string(CCSystemMatrix.cols()) + ", "
				+ std::to_string(CASystemMatrix.rows()) + ", "
				+ std::to_string(CASystemMatrix.cols()) + ", "
				+ std::to_string(tempreconPCtotal.maxReconValue) + ", "
				+ std::to_string(iterationtotalN) + ", " + std::to_string(tmphy),
				eLoggerType::INFO);
#endif // Test_Log


			CCSystemMatrix.resize(1, reconPC.points_.size()); tmphy = 0; //reset
			CASystemMatrix.resize(reconPC.points_.size(), 3600); tmpCodedlmdatasize = 0;//reset    
			CodedNECSignal = MatrixXd::Zero(1, posaccumN);
			CodedNECBackground = MatrixXd::Zero(1, posaccumN);
			ComptonNECSignal = MatrixXd::Zero(1, posaccumN);
			ComptonNECBackground = MatrixXd::Zero(1, posaccumN);

			HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
				"MLEM Hybrid Iteration : " + std::to_string(NEC), eLoggerType::INFO);
		}
	}

	reconPC.maxReconValue = -DBL_MAX;
	for (size_t j = 0; j < reconPC.points_.size(); ++j)
	{
		if (reconPC.reconValues_[j] > reconPC.maxReconValue)
		{
			reconPC.maxReconValue = reconPC.reconValues_[j];
		}
	}

#ifdef Test_Log
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl",
		"2D Recon Max Value : " + std::to_string(reconPC.maxReconValue), eLoggerType::INFO);
#endif // Test_Log
}