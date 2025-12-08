#include "LahgiControl.h"
#include "RtabmapSlamControl.h"
#include "CppWrapper.h"

using namespace HUREL::Compton;

sBitMapUnmanged GetCvToPointers(cv::Mat color, uint8_t** outPoint)
{

	sBitMapUnmanged outStruct{ *outPoint, 0, 0 , 0, 0 };


	if (outStruct.ptr != nullptr)
	{
		delete[] outStruct.ptr;
	}

	int imagesize = 0;


	if (color.empty())
		//if (color.cols == 0)
	{
		return outStruct;
	}
	outStruct.width = color.cols;
	outStruct.height = color.rows;
	outStruct.step = color.step;
	outStruct.channelSize = color.channels();
	imagesize = outStruct.width * outStruct.height * outStruct.channelSize;

	//RtabmapCppWrapper::instance().UnlockVideoFrame();
	outStruct.ptr = new uchar[imagesize];
	memcpy(outStruct.ptr, color.data, imagesize);

	return outStruct;
}


LahgiCppWrapper& HUREL::Compton::LahgiCppWrapper::instance()
{
	static LahgiCppWrapper& inst = *(new LahgiCppWrapper());
	static bool isLogHandled = false;
	if (!isLogHandled)
	{
		HUREL::Logger::Instance().Handle(HUREL::Compton::WrapperCaller::Logging);
		isLogHandled = true;
	}
	return inst;
}

bool HUREL::Compton::LahgiCppWrapper::SetType(eModuleCppWrapper type)
{
	HUREL::Compton::eMouduleType moduleType = HUREL::Compton::eMouduleType::MONO;
	switch (type)
	{
	case HUREL::Compton::eModuleCppWrapper::MONO:
		moduleType = HUREL::Compton::eMouduleType::MONO;
		break;
	case HUREL::Compton::eModuleCppWrapper::QUAD:
		moduleType = HUREL::Compton::eMouduleType::QUAD;

		break;
	case HUREL::Compton::eModuleCppWrapper::QUAD_DUAL:
		moduleType = HUREL::Compton::eMouduleType::QUAD_DUAL;
		break;
	default:
		break;
	}
	return LahgiControl::instance().SetType(moduleType);
}

void HUREL::Compton::LahgiCppWrapper::SetEchks(std::vector<std::vector<double>> echks, std::vector<int> elements)
{
	std::vector<sEnergyCheck> eChkUnmanagedVector;
	eChkUnmanagedVector.reserve(echks.size());
	/*for (auto echk : echks)
	{
		sEnergyCheck eChkUnmanaged;
		eChkUnmanaged.minE = echk[0];
		eChkUnmanaged.maxE = echk[1];

		eChkUnmanagedVector.push_back(eChkUnmanaged);
	}*/

	if (echks.size() != elements.size())
		return;

	for (size_t i = 0; i < echks.size(); i++)
	{
		sEnergyCheck eChk;
		eChk.minE = echks[i][0];
		eChk.maxE = echks[i][1];
		eChk.nElement = elements[i];

		eChkUnmanagedVector.push_back(eChk);
	}
	LahgiControl::instance().SetEchk(eChkUnmanagedVector);
}

//240123
void HUREL::Compton::LahgiCppWrapper::SelectEchks(std::vector<int> elements)
{
	LahgiControl::instance().SelectEchk(elements);
}

void HUREL::Compton::LahgiCppWrapper::AddListModeDataWithTransformation(const unsigned short* byteData)
{

	LahgiControl::instance().AddListModeDataWithTransformation(byteData);
}

std::vector<ListModeDataCppWrapper> HUREL::Compton::LahgiCppWrapper::GetRelativeListModeData()
{
	std::vector<ListModeData> lists = LahgiControl::instance().GetListedListModeData();
	std::vector<ListModeDataCppWrapper> cppLists;
	for (auto lm : lists)
	{
		ListModeDataCppWrapper lmCpp;
		lmCpp.ScatterRelativeInteractionPointX = lm.Scatter.RelativeInteractionPoint[0];
		lmCpp.ScatterRelativeInteractionPointY = lm.Scatter.RelativeInteractionPoint[1];
		lmCpp.ScatterRelativeInteractionPointZ = lm.Scatter.RelativeInteractionPoint[2];
		lmCpp.ScatterInteractionEnergy = lm.Scatter.InteractionEnergy;
		lmCpp.AbsorberRelativeInteractionPointX = lm.Absorber.RelativeInteractionPoint[0];
		lmCpp.AbsorberRelativeInteractionPointY = lm.Absorber.RelativeInteractionPoint[1];
		lmCpp.AbsorberRelativeInteractionPointZ = lm.Absorber.RelativeInteractionPoint[2];
		lmCpp.AbsorberInteractionEnergy = lm.Absorber.InteractionEnergy;
		cppLists.push_back(lmCpp);
	}
	return cppLists;
}

size_t HUREL::Compton::LahgiCppWrapper::GetListedListModeDataSize()
{
	return LahgiControl::instance().GetListedListModeDataSize();
}


void HUREL::Compton::LahgiCppWrapper::ResetListedListModeData()
{
	LahgiControl::instance().ResetListedListModeData();
	for (int i = 0; i < 16; ++i)
	{
		LahgiControl::instance().ResetEnergySpectrum(i);
	}
}

void HUREL::Compton::LahgiCppWrapper::RestEnergySpectrum(int channelNumber)
{
	LahgiControl::instance().ResetEnergySpectrum(channelNumber);
}

std::tuple<double, double, double> HUREL::Compton::LahgiCppWrapper::GetEcalValue(unsigned int fpgaChannelNumber)
{
	return LahgiControl::instance().GetEcalValue(fpgaChannelNumber);
}

void HUREL::Compton::LahgiCppWrapper::SetEcalValue(int fpgaChannelNumber, std::tuple<double, double, double> ecal)
{
	LahgiControl::instance().SetEcalValue(fpgaChannelNumber, ecal);
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetSpectrum(int channelNumber)
{
	return LahgiControl::instance().GetEnergySpectrum(channelNumber).GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetSumSpectrum()
{
	return LahgiControl::instance().GetSumEnergySpectrum().GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetAbsorberSumSpectrum()
{
	return LahgiControl::instance().GetAbsorberSumEnergySpectrum().GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetScatterSumSpectrum()
{
	return  LahgiControl::instance().GetScatterSumEnergySpectrum().GetHistogramEnergy();
}

//231123 sbkwon
std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetSpectrum(int channelNumber, int time)
{
	std::vector<EnergyTimeData> lmData = LahgiControl::instance().GetListedEnergyTimeData(channelNumber, time * 1000);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());



	int reconStartIndex = 0;


	EnergySpectrum spectClass = EnergySpectrum(10, 3000);;
	for (int i = 0; i < lmData.size(); ++i)
	{
		spectClass.AddEnergy(lmData[i].Energy);
	}

	return spectClass.GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetScatterSumSpectrum(int time)
{
	std::vector<EnergyTimeData> lmData = LahgiControl::instance().GetListedEnergyTimeData(time * 1000);

	//std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	//int reconStartIndex = 0;

	EnergySpectrum spectClass = EnergySpectrum(10, 3000);;
	for (int i = 0; i < lmData.size(); ++i)
	{
		if (lmData[i].InteractionChannel < 8)
		{
			spectClass.AddEnergy(lmData[i].Energy);
		}
	}

	return spectClass.GetHistogramEnergy();
}

//250410
std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetSpectrumData(int type, int ch)
{
	std::vector<EnergyTimeData> lmData = LahgiControl::instance().GetListedEnergyTimeData();
		
	EnergySpectrum spectClass = EnergySpectrum(10, 3000);;
	for (int i = 0; i < lmData.size(); ++i)
	{
		if (type == 0)	//scater
		{
			if (lmData[i].InteractionChannel == 4)
			{
				spectClass.AddEnergy(lmData[i].Energy);
			}
		}
		else if (type == 1)	//apsorberf
		{
			if (lmData[i].InteractionChannel == 12)
			{
				spectClass.AddEnergy(lmData[i].Energy);
			}
		}
		else if (type == 2)//all
		{
			spectClass.AddEnergy(lmData[i].Energy);
		}
		else if (type == 3)	//ch
		{
			if (lmData[i].InteractionChannel == ch)
			{
				spectClass.AddEnergy(lmData[i].Energy);
			}
		}
		
	}

	return spectClass.GetHistogramEnergy();
}

//240311 : ���� �˻� ����
void HUREL::Compton::LahgiCppWrapper::SetUseFD(bool set)
{
	LahgiControl::instance().SetUseFD(set);
}

//240228 : ����˻� PMT Data Copy
void HUREL::Compton::LahgiCppWrapper::CopyPMTData()
{
	LahgiControl::instance().CopyListedPMTEnergyData();
}

//240228 ���� �˻� : used peak range�� �̿��Ͽ� gain�� ��ȯ�Ѵ�
std::vector<double> HUREL::Compton::LahgiCppWrapper::GetGainref(unsigned int channelNumer)
{
	return LahgiControl::instance().GetGainref(channelNumer);
}

//240228 : ����˻�
std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetPMTEnergyData(unsigned int channelNumber)
{
	std::vector<double> lmData = LahgiControl::instance().GetListedPMTEnergyData(channelNumber);

	EnergySpectrum spectClass = EnergySpectrum(2, 3000);
	for (int i = 0; i < lmData.size(); ++i)
	{
		spectClass.AddEnergy(lmData[i]);
	}

	return spectClass.GetHistogramEnergy();
}

//240228 : ����˻� corrMatin ����
std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetPMTEnergyData(unsigned int channelNumber, std::vector<double> corrMatIn)
{
	std::vector<double> lmData = LahgiControl::instance().GetListedPMTEnergyData(channelNumber, corrMatIn);

	EnergySpectrum spectClass = EnergySpectrum(2, 3000);;
	for (int i = 0; i < lmData.size(); ++i)
	{
		spectClass.AddEnergy(lmData[i]);
	}

	return spectClass.GetHistogramEnergy();
}

//240228 ���� �˻� : used peak range�� �̿��Ͽ� gain�� ��ȯ�Ѵ�
std::vector<double> HUREL::Compton::LahgiCppWrapper::GetPMTCorrMatIn(unsigned int channelNumer, std::vector<int> usedPeak, std::vector<double> range_bkg)
{
	return LahgiControl::instance().GetPMTCorrMatIn(channelNumer, usedPeak, range_bkg);
}

//240315
std::vector<double> HUREL::Compton::LahgiCppWrapper::GetPMTCorrMatInBeforGain(unsigned int channelNumer, std::vector<int> usedPeak, std::vector<double> range_bkg, std::vector<double> corrMatIn)
{
	return LahgiControl::instance().GetPMTCorrMatInBeforGain(channelNumer, usedPeak, range_bkg, corrMatIn);
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetAbsorberSumSpectrum(int time)
{
	std::vector<EnergyTimeData> lmData = LahgiControl::instance().GetListedEnergyTimeData(time * 1000);

	//std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	//int reconStartIndex = 0;

	EnergySpectrum spectClass = EnergySpectrum(10, 4000);;  // Absorber spectrum 범위를 4000 keV로 설정
	for (int i = 0; i < lmData.size(); ++i)
	{
		if (lmData[i].InteractionChannel >= 8)
		{
			spectClass.AddEnergy(lmData[i].Energy);
		}
	}

	return spectClass.GetHistogramEnergy();
}

//231100-GUI sbkwon
std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetSumSpectrum(int time)
{
	std::vector<EnergyTimeData> lmData = LahgiControl::instance().GetListedEnergyTimeData(time * 1000);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	int reconStartIndex = 0;

	// All spectrum은 4000 범위로 설정
	EnergySpectrum spectClass = EnergySpectrum(10, 4000);;
	for (int i = 0; i < lmData.size(); ++i)
	{
		if (lmData[i].Energy >= 0 && lmData[i].Energy < 4000)
	{
		spectClass.AddEnergy(lmData[i].Energy);
		}
	}

	return spectClass.GetHistogramEnergy();
}

bool HUREL::Compton::LahgiCppWrapper::SaveListedListModeData(std::string filePath)
{
	LahgiControl::instance().SaveListedListModeData(filePath);
	return true;
}

bool HUREL::Compton::LahgiCppWrapper::LoadListedListModeData(std::string filePath)
{
	return LahgiControl::instance().LoadListedListModeData(filePath);
}

sBitMapUnmanged  HUREL::Compton::LahgiCppWrapper::GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter)
{
	static uint8_t* ptr = nullptr;

	return GetCvToPointers(LahgiControl::instance().GetResponseImage(imgSize, pixelCount, timeInSeconds, isScatter), &ptr);
}

//231113-1 sbkwon
void HUREL::Compton::LahgiCppWrapper::InitRadiationImage()
{
	RadiationImage radimage;
}

std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  HUREL::Compton::LahgiCppWrapper::GetRadiation2dImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion)
{
	static uint8_t* ptrCoded = nullptr;
	static uint8_t* ptrCompton = nullptr;
	static uint8_t* ptrHybrid = nullptr;

	RadiationImage radimage(LahgiControl::instance().GetListedListModeData(timeInMiliSeconds), s2M, det_W, resImprov, m2D, hFov, wFov);

	return std::make_tuple(GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mCodedImage, imgSize, minValuePortion), &ptrCoded),
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mComptonImage, imgSize, minValuePortion), &ptrCompton),
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mHybridImage, imgSize, minValuePortion), &ptrHybrid));

}

//231025-1 sbkwon
std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  HUREL::Compton::LahgiCppWrapper::GetRadiation2dImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double minValuePortion)
{
	static uint8_t* ptrCoded = nullptr;
	static uint8_t* ptrCompton = nullptr;
	static uint8_t* ptrHybrid = nullptr;

	RadiationImage radimage(LahgiControl::instance().GetListedListModeData(timeInMiliSeconds), s2M, det_W, resImprov, m2D, 70);	//240122

	return std::make_tuple(GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mCodedImage, 800, minValuePortion), &ptrCoded),
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mComptonImage, 800, minValuePortion), &ptrCompton),
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mHybridImage, 800, minValuePortion), &ptrHybrid));
}

//231212 : �ǿ�
std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  HUREL::Compton::LahgiCppWrapper::GetRadation2dImageCount(int count, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion, int time, int maxValue, bool fullrange)
{
	static uint8_t* ptrCoded = nullptr;
	static uint8_t* ptrCompton = nullptr;
	static uint8_t* ptrHybrid = nullptr;

	RadiationImage radimage(LahgiControl::instance().GetEfectListedListModeData(count, time * 1000), s2M, det_W, resImprov, m2D, hFov, wFov, maxValue, fullrange);

	return std::make_tuple(GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mCodedImage, imgSize, minValuePortion), &ptrCoded),
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mComptonImage, imgSize, minValuePortion), &ptrCompton),
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mHybridImage, imgSize, minValuePortion), &ptrHybrid));

}

//231100-GUI sbkwon : �ǳ�
std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  HUREL::Compton::LahgiCppWrapper::GetRadation2dImageCount(int count, double s2M, double det_W, double resImprov, double m2D, double minValuePortion, int time, int maxValue)
{
	static uint8_t* ptrCoded = nullptr;
	static uint8_t* ptrCompton = nullptr;
	static uint8_t* ptrHybrid = nullptr;

	RadiationImage radimage(LahgiControl::instance().GetEfectListedListModeData(count, time * 1000), s2M, det_W, resImprov, m2D, maxValue);

	return std::make_tuple(GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mCodedImage, 800, minValuePortion), &ptrCoded),
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mComptonImage, 800, minValuePortion), &ptrCompton),
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mHybridImage, 800, minValuePortion), &ptrHybrid));
}

//240930 sbkwon Label �߰� : Plane
std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  HUREL::Compton::LahgiCppWrapper::GetRadation2dImageCountLabel(int count, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion, int time, int maxValue, bool fullrange)
{
	static uint8_t* ptrCoded = nullptr;
	static uint8_t* ptrCompton = nullptr;
	static uint8_t* ptrHybrid = nullptr;

	std::vector<int> selectChk = LahgiControl::instance().GetSelectEchek();

	cv::Mat compton, coded, hybrid;
	cv::Mat anotation;	//250203 : anotation�� �ϳ��� mat ������ ��ġ��

	int beforChk = -1;

	for (size_t i = 0; i < selectChk.size(); i++)
	{
		if(selectChk[i] != beforChk)
		{
			beforChk = selectChk[i];
			RadiationImage radimage(LahgiControl::instance().GetEfectListedListModeData(selectChk[i], count, time * 1000), s2M, det_W, resImprov, m2D, hFov, wFov, maxValue, fullrange);

			std::string IsotopeName;

			if (isotopeList.find(selectChk[i]) != isotopeList.end())	//�����Ͱ� ���� ���
				IsotopeName = isotopeList[selectChk[i]];
			else
				IsotopeName = std::to_string(selectChk[i]) + " : None";

			if (i == 0)
			{
				//compton
				compton = RadiationImage::GetCV_32SAsJetZero(radimage.mComptonImage, imgSize, minValuePortion);
				//coded
				coded = RadiationImage::GetCV_32SAsJetZero(radimage.mCodedImage, imgSize, minValuePortion);
				//hybrid
				//hybrid = RadiationImage::GetAnotation(RadiationImage::GetCV_32SAsJetZero(radimage.mHybridImage, imgSize, minValuePortion), IsotopeName);
				hybrid = RadiationImage::GetCV_32SAsJetZero(radimage.mHybridImage, imgSize, minValuePortion);	//250203

				anotation = Mat::zeros(hybrid.cols, hybrid.rows, CV_8UC4);	//250203 : �ʱ�ȭ

				cv::Mat mask = RadiationImage::GetAnotation(hybrid, anotation, IsotopeName);

				////test
				//std::string saveHybrid = "E:\\" + std::to_string(i) + "_hybrid.png";
				//cv::imwrite(saveHybrid, hybrid);
				//std::string saveAno = "E:\\" + std::to_string(i) + "_anotation.png";
				//cv::imwrite(saveAno, anotation);

				/*std::string saveAno = "E:\\" + std::to_string(i) + std::to_string(anotation.cols) + IsotopeName + "_mask1.png";
				cv::imwrite(saveAno, mask);*/
			}
			else
			{
				//compton
				cv::Mat comptontemp = RadiationImage::GetCV_32SAsJetZero(radimage.mComptonImage, imgSize, minValuePortion);
				//coded
				cv::Mat codedtemp = RadiationImage::GetCV_32SAsJetZero(radimage.mCodedImage, imgSize, minValuePortion);
				//hybrid : Label �߰�
				//cv::Mat hybridtemp = RadiationImage::GetAnotation(RadiationImage::GetCV_32SAsJetZero(radimage.mHybridImage, imgSize, minValuePortion), IsotopeName);
				cv::Mat hybridtemp = RadiationImage::GetCV_32SAsJetZero(radimage.mHybridImage, imgSize, minValuePortion);
				cv::Mat mask = RadiationImage::GetAnotation(hybridtemp, anotation, IsotopeName);	//250203 : 

				
				//���� ��ġ��
				//compton
				if (!comptontemp.empty())
				{
					if (comptontemp.size() != compton.size())
						cv::resize(comptontemp, comptontemp, compton.size(), 0, 0, cv::INTER_NEAREST_EXACT);

					cv::addWeighted(comptontemp, 1, compton, 1, 0, compton);
				}

				//coded
				if (!codedtemp.empty())
				{
					if (codedtemp.size() != coded.size())
						cv::resize(codedtemp, codedtemp, coded.size(), 0, 0, cv::INTER_NEAREST_EXACT);

					cv::addWeighted(codedtemp, 1, coded, 1, 0, coded);
				}

				//hybrid
				if (!hybridtemp.empty())
				{
					if (hybridtemp.size() != hybrid.size())
						cv::resize(hybridtemp, hybridtemp, hybrid.size(), 0, 0, cv::INTER_NEAREST_EXACT);

					cv::addWeighted(hybridtemp, 1, hybrid, 1, 0, hybrid);	
					//mask �̿�
					//cv::add(hybridtemp, hybrid, hybrid, mask);
				}

				////test
				/*std::string saveHybrid1 = "E:\\" + std::to_string(i) + "_hybrid.png";
				cv::imwrite(saveHybrid1, hybridtemp);
				std::string saveHybrid = "E:\\" + std::to_string(i) + "_hybrid1.png";
				cv::imwrite(saveHybrid, hybrid);*/
				/*std::string saveAno = "E:\\" + std::to_string(i) + std::to_string(anotation.cols) + "_anotation.png";
				cv::imwrite(saveAno, anotation);
				std::string savemask = "E:\\" + std::to_string(i) + std::to_string(anotation.cols) + IsotopeName +"_mask2.png";
				cv::imwrite(savemask, mask);*/
			}
		}
	}

	//250203 : label + hybrid Image
	if (!hybrid.empty() && !anotation.empty())
	{
		if (hybrid.size() != anotation.size())
			cv::resize(anotation, anotation, hybrid.size(), 0, 0, cv::INTER_NEAREST_EXACT);

		cv::addWeighted(hybrid, 1, anotation, 1, 0, hybrid);	//���� ������ ���� �� ���� �ʿ�

		/*std::string savemask = "E:\\_anotation.png";
		cv::imwrite(savemask, anotation);*/
	}

	return std::make_tuple(GetCvToPointers(coded, &ptrCoded), GetCvToPointers(compton, &ptrCompton), GetCvToPointers(hybrid, &ptrHybrid));
}

//240930 sbkwon Label �߰� : point cloud
std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  HUREL::Compton::LahgiCppWrapper::GetRadation2dImageCountLabel(int count, double s2M, double det_W, double resImprov, double m2D, double minValuePortion, int time, int maxValue)
{
	static uint8_t* ptrCoded = nullptr;
	static uint8_t* ptrCompton = nullptr;
	static uint8_t* ptrHybrid = nullptr;

	std::vector<int> selectChk = LahgiControl::instance().GetSelectEchek();

	cv::Mat compton, coded, hybrid;
	cv::Mat anotation;	//250203 : anotation�� �ϳ��� mat ������ ��ġ��

	int beforChk = -1;

	for (size_t i = 0; i < selectChk.size(); i++)
	{
		if (selectChk[i] != beforChk)
		{
			beforChk = selectChk[i];
			RadiationImage radimage(LahgiControl::instance().GetEfectListedListModeData(selectChk[i], count, time * 1000), s2M, det_W, resImprov, m2D, maxValue);

			std::string IsotopeName;

			if (isotopeList.find(selectChk[i]) != isotopeList.end())	//�����Ͱ� ���� ���
				IsotopeName = isotopeList[selectChk[i]];
			else
				IsotopeName = std::to_string(selectChk[i]) + " : None";

			if (i == 0)
			{
				//compton
				compton = RadiationImage::GetCV_32SAsJetZero(radimage.mComptonImage, 800, minValuePortion);
				//coded
				coded = RadiationImage::GetCV_32SAsJetZero(radimage.mCodedImage, 800, minValuePortion);
				//hybrid
				//hybrid = RadiationImage::GetAnotation(RadiationImage::GetCV_32SAsJetZero(radimage.mHybridImage, 800, minValuePortion), IsotopeName);
				hybrid = RadiationImage::GetCV_32SAsJetZero(radimage.mHybridImage, 800, minValuePortion);
			
				anotation = Mat::zeros(hybrid.cols, hybrid.rows, CV_8UC4);	//250203 : �ʱ�ȭ
				RadiationImage::GetAnotation(hybrid, anotation, IsotopeName);
			}
			else
			{
				//compton
				cv::Mat comptontemp = RadiationImage::GetCV_32SAsJetZero(radimage.mComptonImage, 800, minValuePortion);
				//coded
				cv::Mat codedtemp = RadiationImage::GetCV_32SAsJetZero(radimage.mCodedImage, 800, minValuePortion);
				//hybrid : Label �߰�
				//cv::Mat hybridtemp = RadiationImage::GetAnotation(RadiationImage::GetCV_32SAsJetZero(radimage.mHybridImage, 800, minValuePortion), IsotopeName);
				cv::Mat hybridtemp = RadiationImage::GetCV_32SAsJetZero(radimage.mHybridImage, 800, minValuePortion);
				RadiationImage::GetAnotation(hybridtemp, anotation, IsotopeName);	//250203 : 

				//���� ��ġ��
				//compton
				if(!comptontemp.empty())
				{
					if (comptontemp.size() != compton.size())
						cv::resize(comptontemp, comptontemp, compton.size(), 0, 0, cv::INTER_NEAREST_EXACT);

					cv::addWeighted(comptontemp, 1, compton, 1, 0, compton);
				}

				//coded
				if(!codedtemp.empty())
				{
					if (codedtemp.size() != coded.size())
						cv::resize(codedtemp, codedtemp, coded.size(), 0, 0, cv::INTER_NEAREST_EXACT);

					cv::addWeighted(codedtemp, 1, coded, 1, 0, coded);
				}

				//hybrid
				if(!hybridtemp.empty())
				{
					if (hybridtemp.size() != hybrid.size())
						cv::resize(hybridtemp, hybridtemp, hybrid.size(), 0, 0, cv::INTER_NEAREST_EXACT);

					cv::addWeighted(hybridtemp, 1, hybrid, 1, 0, hybrid);
				}
			}
		}
	}

	//250203 : label + hybrid Image
	if (!hybrid.empty() && !anotation.empty())
	{
		if (hybrid.size() != anotation.size())
			cv::resize(anotation, anotation, hybrid.size(), 0, 0, cv::INTER_NEAREST_EXACT);

		cv::addWeighted(hybrid, 1, anotation, 1, 0, hybrid);	//���� ������ ���� �� ���� �ʿ�
	}


	return std::make_tuple(GetCvToPointers(coded, &ptrCoded), GetCvToPointers(compton, &ptrCompton), GetCvToPointers(hybrid, &ptrHybrid));
}

sBitMapUnmanged HUREL::Compton::LahgiCppWrapper::GetTransPoseRadiationImage(int timeInMiliSeconds, double minValuePortion, double resolution)
{
	static uint8_t* ptrImg = nullptr;

	Eigen::Matrix4d deviceTransformation = LahgiControl::instance().t265toLACCPosTransform * RtabmapSlamControl::instance().GetOdomentry()
		* LahgiControl::instance().t265toLACCPosTransformInv * LahgiControl::instance().t265toLACCPosTranslate;

	cv::Mat p3 = RtabmapSlamControl::instance().GetCurrentPointsFrame(resolution);
	cv::Mat xyz[3];
	cv::split(p3, xyz);
	std::vector<ListModeData> data = LahgiControl::instance().GetListedListModeData(timeInMiliSeconds);
	cv::Mat radImgReturn = cv::Mat::zeros(p3.rows, p3.cols, CV_32S);

	if (data.size() == 0)
	{
		return GetCvToPointers(radImgReturn, &ptrImg);
	}

	long long startTime = data[0].InteractionTimeInMili.count();

	int startIndex = 0;
	int endIndex = 0;
	for (int i = 0; i < data.size(); ++i)
	{
		if (startTime != data[i].InteractionTimeInMili.count())
		{
			endIndex = i - 1;

			Eigen::Matrix4d diffMatrix = data[startIndex].DetectorTransformation * deviceTransformation.inverse();
			Eigen::Quaterniond quaternino(diffMatrix.topLeftCorner<3, 3>());


			std::vector<ListModeData>::const_iterator first = data.begin() + startIndex;
			std::vector<ListModeData>::const_iterator last = data.begin() + endIndex + 1;
			std::vector<ListModeData> newVec(first, last);

			RadiationImage radImg = RadiationImage(newVec);
			cv::Mat tempRadImg = cv::Mat::zeros(p3.rows, p3.cols, CV_32S);

			for (int iPix = 0; iPix < radImgReturn.rows; ++iPix)
			{
				for (int jPix = 0; jPix < radImgReturn.cols; ++jPix)
				{
					Eigen::Vector3d point(xyz[0].at<float>(iPix, jPix), xyz[1].at<float>(iPix, jPix), xyz[2].at<float>(iPix, jPix));

					tempRadImg.at<int>(iPix, jPix) = radImg.OverlayValue(point, eRadiationImagingMode::COMPTON);
				}
			}

			radImgReturn += tempRadImg;
			startIndex = i;
		}
	}

	cv::Mat jetImage = RadiationImage::GetCV_32SAsJet(radImgReturn, minValuePortion);


	return GetCvToPointers(jetImage, &ptrImg);
}

//2404 : MLEM
bool HUREL::Compton::LahgiCppWrapper::LoadMLEMData(std::string PLYPath, std::string LMDPath, bool bLoad, int nSize)
{
	return LahgiControl::instance().LoadMLEMData(PLYPath, LMDPath, bLoad, nSize);
}

//240429 : MLEM List
bool HUREL::Compton::LahgiCppWrapper::CalMLEMList(std::string systemMPath, std::vector<double> energy, std::vector<double> EgateMin, std::vector<double> EgateMax, double minValuePer)
{
	return LahgiControl::instance().CalMLEMList(systemMPath, energy, EgateMin, EgateMax, minValuePer);
}

//2404 : MLEM
bool HUREL::Compton::LahgiCppWrapper::CalMLEM(std::string systemMPath, double energy, double EgateMin, double EgateMax, double minValuePer)
{
	return LahgiControl::instance().CalMLEM(systemMPath, energy, EgateMin, EgateMax, minValuePer);
}

//2404 : MLEM
std::vector<ReconPointCppWrapper> HUREL::Compton::LahgiCppWrapper::GetMLEMPointCloud(int nNo)
{
	open3d::geometry::PointCloud  pc = LahgiControl::instance().GetMLEMPointCloud(nNo);

	std::vector<ReconPointCppWrapper> returnPc;

	returnPc.reserve(pc.colors_.size());

	for (int i = 0; i < pc.colors_.size(); ++i)
	{
		ReconPointCppWrapper tmpPoint;
		tmpPoint.pointX = pc.points_[i][0];
		tmpPoint.pointY = pc.points_[i][1];
		tmpPoint.pointZ = pc.points_[i][2];


		tmpPoint.colorR = pc.colors_[i][0];
		tmpPoint.colorG = pc.colors_[i][1];
		tmpPoint.colorB = pc.colors_[i][2];
		tmpPoint.colorA = 1;
		returnPc.push_back(tmpPoint);
	}

	return returnPc;
}

//250107 2D MLEM 
bool  HUREL::Compton::LahgiCppWrapper::Get2DMLEMData(std::string PLYPath, int nNo)
{
	return LahgiControl::instance().GetMLEM2D(PLYPath, nNo);
}

bool HUREL::Compton::RtabmapCppWrapper::GetIsSlamPipeOn()
{
	return RtabmapSlamControl::instance().mIsSlamPipeOn;
}

bool HUREL::Compton::RtabmapCppWrapper::GetIsVideoStreamOn()
{
	return RtabmapSlamControl::instance().mIsVideoStreamOn;
}

bool HUREL::Compton::RtabmapCppWrapper::Initiate()
{
	return RtabmapSlamControl::instance().Initiate();
}

std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetRTPointCloud()
{

	std::vector<ReconPointCppWrapper> points;
	return points;
}

std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetRTPointCloudTransposed()
{
	return std::vector<ReconPointCppWrapper>();
}

void HUREL::Compton::RtabmapCppWrapper::StartVideoStream()
{
	RtabmapSlamControl::instance().StartVideoStream();
}

bool HUREL::Compton::RtabmapCppWrapper::StartSlamPipe()
{
	RtabmapSlamControl::instance().StartSlamPipe();
	return true;
}

void HUREL::Compton::RtabmapCppWrapper::StopVideoStream()
{
	RtabmapSlamControl::instance().StopVideoStream();
}

void HUREL::Compton::RtabmapCppWrapper::StopSlamPipe()
{
	RtabmapSlamControl::instance().StopSlamPipe();
}

void HUREL::Compton::RtabmapCppWrapper::ResetSlam()
{
	RtabmapSlamControl::instance().ResetSlam();
}

std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetSlamPointCloud()
{
	open3d::geometry::PointCloud  pc = RtabmapSlamControl::instance().GetSlamPointCloud();

	std::vector<ReconPointCppWrapper> returnPc;
	returnPc.reserve(pc.colors_.size());

	for (int i = 0; i < pc.colors_.size(); ++i)
	{
		ReconPointCppWrapper tmpPoint;
		tmpPoint.pointX = pc.points_[i][0];
		tmpPoint.pointY = pc.points_[i][1];
		tmpPoint.pointZ = pc.points_[i][2];


		tmpPoint.colorR = pc.colors_[i][0];
		tmpPoint.colorG = pc.colors_[i][0];
		tmpPoint.colorB = pc.colors_[i][0];
		tmpPoint.colorA = 1;
		returnPc.push_back(tmpPoint);
	}

	return returnPc;
}

//231121-1 sbkwon
std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetSLAMOccupancyGrid()
{
	open3d::geometry::PointCloud  pc = RtabmapSlamControl::instance().GetOccupancyPointCloud();

	std::vector<ReconPointCppWrapper> returnPc;
	returnPc.reserve(pc.colors_.size());

	for (int i = 0; i < pc.colors_.size(); ++i)
	{
		ReconPointCppWrapper tmpPoint;
		tmpPoint.pointX = pc.points_[i][0];
		tmpPoint.pointY = pc.points_[i][1];
		tmpPoint.pointZ = pc.points_[i][2];


		tmpPoint.colorR = pc.colors_[i][0];
		tmpPoint.colorG = pc.colors_[i][0];
		tmpPoint.colorB = pc.colors_[i][0];
		tmpPoint.colorA = 1;
		returnPc.push_back(tmpPoint);
	}

	return returnPc;
}

bool HUREL::Compton::RtabmapCppWrapper::LoadPlyFile(std::string filePath)
{
	return RtabmapSlamControl::instance().LoadPlyFile(filePath);
}

//240621 sbkwon : ���� ���� �� ���� 
//depth, rgb file name�� ply��ü ���ϸ� �ڿ� �ٴ� ����
void HUREL::Compton::RtabmapCppWrapper::SavePlyFile(std::string filePath)
{
	open3d::geometry::PointCloud  pc = RtabmapSlamControl::instance().GetSlamPointCloud();
	open3d::io::WritePointCloudOption option;

	//open3d::io::WritePointCloudToPLY(filePath, pc, option);
	open3d::io::WritePointCloudToPLY(filePath + "_SlamData.ply", pc, option);

	//241219 sbkwon : frame data null(empty) check insert
	cv::Mat rgb = RtabmapSlamControl::instance().GetCurrentVideoFrame();
	cv::Mat depth = RtabmapSlamControl::instance().GetCurrentDepthFrame();
	
	if(!depth.empty())
		cv::imwrite(filePath + "_depth.png", depth);
	if (!rgb.empty())
		cv::imwrite(filePath + "_rgb.png", rgb);
	//241219 sbkwon :
}

std::vector<double> HUREL::Compton::RtabmapCppWrapper::getMatrix3DOneLineFromPoseData()
{
	return RtabmapSlamControl::instance().getMatrix3DOneLineFromPoseData();;
}

bool HUREL::Compton::RtabmapCppWrapper::GetCurrentVideoFrame(uint8_t** outImgPtr, int* outWidth, int* outHeight, int* outStride, int* outChannelSize, bool bRealTime)
{
    // 출력 매개변수 초기화
    if (outImgPtr == nullptr || outWidth == nullptr || outHeight == nullptr || 
        outStride == nullptr || outChannelSize == nullptr)
    {
        return false;
    }
    
    *outImgPtr = nullptr;
    *outWidth = 0;
    *outHeight = 0;
    *outStride = 0;
    *outChannelSize = 0;

    try
    {
        // OpenCV Mat 객체로 이미지 획득
        cv::Mat color;
        if (bRealTime)
        {
            color = RtabmapSlamControl::instance().GetCurrentVideoFrame();
        }
        else
        {
            color = RtabmapSlamControl::instance().GetCurrentVideoFrame1();
        }

        // 이미지 유효성 검사
        if (color.empty() || color.cols <= 0 || color.rows <= 0)
        {
            return false;
        }

        // 이미지 크기 및 채널 정보 설정
        *outWidth = color.cols;
        *outHeight = color.rows;
        *outStride = color.step;
        *outChannelSize = color.channels();

        // 이미지 크기 계산 및 검증
        int imagesize = *outWidth * *outHeight * *outChannelSize;
        if (imagesize <= 0 || imagesize > 100000000) // 100MB 제한
        {
            return false;
        }

        // 새로운 메모리 할당
        uchar* newImg = nullptr;
        try
        {
            newImg = new uchar[imagesize];
            if (newImg == nullptr)
            {
                return false;
            }

            // 메모리 유효성 추가 검증
            if (color.data == nullptr || color.data == (uchar*)0xCCCCCCCC || 
                color.data == (uchar*)0xCDCDCDCD || color.data == (uchar*)0xBAADF00D)
            {
                delete[] newImg;
                return false;
            }

            // 이미지 데이터 크기 재검증
            size_t actualDataSize = (size_t)color.rows * color.cols * color.channels();
            if (actualDataSize != (size_t)imagesize || actualDataSize == 0)
            {
                delete[] newImg;
                return false;
            }

            // 안전한 메모리 복사 (바이트 단위로 검증)
            bool copySuccess = true;
            try
            {
                // 메모리 페이지 접근 가능성 테스트
                volatile uchar testByte = color.data[0];
                (void)testByte; // 경고 억제
                
                // 실제 복사 수행
                memcpy(newImg, color.data, imagesize);
            }
            catch (...)
            {
                copySuccess = false;
            }

            if (!copySuccess)
            {
                delete[] newImg;
                return false;
            }

            // 성공 시 출력 포인터 설정
            *outImgPtr = newImg;
            return true;
        }
        catch (...)
        {
            // 메모리 할당 실패 시 정리
            if (newImg != nullptr)
            {
                delete[] newImg;
            }
            return false;
        }
    }
    catch (...)
    {
        // 모든 예외 상황에서 안전하게 처리
        return false;
    }
}



bool HUREL::Compton::RtabmapCppWrapper::GetCurrentVideoFrame1(uint8_t** outImgPtr, int* outWidth, int* outHeight, int* outStride, int* outChannelSize, bool bRealTime)
{
    // 출력 매개변수 초기화
    if (outImgPtr == nullptr || outWidth == nullptr || outHeight == nullptr || 
        outStride == nullptr || outChannelSize == nullptr)
    {
        return false;
    }
    
    *outImgPtr = nullptr;
    *outWidth = 0;
    *outHeight = 0;
    *outStride = 0;
    *outChannelSize = 0;

    try
    {
        // OpenCV Mat 객체로 이미지 획득
        cv::Mat color;
        if (bRealTime)
        {
            color = RtabmapSlamControl::instance().GetCurrentVideoFrame();
        }
        else
        {
            color = RtabmapSlamControl::instance().GetCurrentVideoFrame1();
        }

        // 이미지 유효성 검사
        if (color.empty() || color.cols <= 0 || color.rows <= 0)
        {
            return false;
        }

        // 이미지 크기 및 채널 정보 설정
        *outWidth = color.cols;
        *outHeight = color.rows;
        *outStride = color.step;
        *outChannelSize = color.channels();

        // 이미지 크기 계산 및 검증
        int imagesize = *outWidth * *outHeight * *outChannelSize;
        if (imagesize <= 0 || imagesize > 100000000) // 100MB 제한
        {
            return false;
        }

        // 새로운 메모리 할당
        uchar* newImg = nullptr;
        try
        {
            newImg = new uchar[imagesize];
            if (newImg == nullptr)
            {
                return false;
            }

            // 메모리 유효성 추가 검증
            if (color.data == nullptr || color.data == (uchar*)0xCCCCCCCC || 
                color.data == (uchar*)0xCDCDCDCD || color.data == (uchar*)0xBAADF00D)
            {
                delete[] newImg;
                return false;
            }

            // 이미지 데이터 크기 재검증
            size_t actualDataSize = (size_t)color.rows * color.cols * color.channels();
            if (actualDataSize != (size_t)imagesize || actualDataSize == 0)
            {
                delete[] newImg;
                return false;
            }

            // 안전한 메모리 복사 (바이트 단위로 검증)
            bool copySuccess = true;
            try
            {
                // 메모리 페이지 접근 가능성 테스트
                volatile uchar testByte = color.data[0];
                (void)testByte; // 경고 억제
                
                // 실제 복사 수행
                memcpy(newImg, color.data, imagesize);
            }
            catch (...)
            {
                copySuccess = false;
            }

            if (!copySuccess)
            {
                delete[] newImg;
                return false;
            }

            // 성공 시 출력 포인터 설정
            *outImgPtr = newImg;
            return true;
        }
        catch (...)
        {
            // 메모리 할당 실패 시 정리
            if (newImg != nullptr)
            {
                delete[] newImg;
            }
            return false;
        }
    }
    catch (...)
    {
        // 모든 예외 상황에서 안전하게 처리
        return false;
    }
}

bool HUREL::Compton::RtabmapCppWrapper::GetLMDataVideoFrame(uint8_t** outImgPtr, int* outWidth, int* outHeight, int* outStride, int* outChannelSize, bool bRealTime)
{
    static int imagesize = 0;
    static uchar* currentImg = nullptr;
    
    // 안전한 메모리 해제 - 더 강화된 검증
    if (currentImg != nullptr)
    {
        try
        {
            // 포인터 유효성 추가 검증
            if (currentImg != (uchar*)0xCCCCCCCC && currentImg != (uchar*)0xCDCDCDCD)
            {
                delete[] currentImg;
            }
        }
        catch (...)
        {
            // 예외 발생 시 무시하고 계속 진행
        }
        currentImg = nullptr;
    }

    //240105
    cv::Mat color = cv::Mat();
    if (bRealTime)
        color = RtabmapSlamControl::instance().GetCurrentVideoFrame();		//RealTime 영상
    else
        color = RtabmapSlamControl::instance().GetCurrentVideoFrame1();	//240312 B, //LMData 영상 획득 : GetCurrentVideoFrame1()

    if (color.cols <= 0)
    {
        *outImgPtr = nullptr;
        return false;
    }
    
    *outWidth = color.cols;
    *outHeight = color.rows;
    *outStride = color.step;
    *outChannelSize = color.channels();
    
    // 이미지 크기 계산 및 메모리 할당
    imagesize = *outWidth * *outHeight * *outChannelSize;
    
    // 메모리 할당 전 추가 검증
    if (imagesize <= 0 || imagesize > 100000000) // 100MB 제한
    {
        *outImgPtr = nullptr;
        return false;
    }
    
    try
    {
        currentImg = new uchar[imagesize];
        if (currentImg != nullptr)
        {
            memcpy(currentImg, color.data, imagesize);
            *outImgPtr = currentImg;
            return true;
        }
        else
        {
            *outImgPtr = nullptr;
            return false;
        }
    }
    catch (...)
    {
        if (currentImg != nullptr)
        {
            delete[] currentImg;
            currentImg = nullptr;
        }
        *outImgPtr = nullptr;
        return false;
    }
}


std::vector<ReconPointCppWrapper>  HUREL::Compton::RtabmapCppWrapper::GetReconSLAMPointCloud(double time, eReconCppWrapper reconType, double voxelSize, bool useLoaded)
{
	open3d::geometry::PointCloud reconPC;
	if (useLoaded)
	{
		reconPC = RtabmapSlamControl::instance().GetLoadedPointCloud();
	}
	else
	{
		reconPC = RtabmapSlamControl::instance().GetSlamPointCloud();
	}

	open3d::geometry::PointCloud reconPcDownSampled = *reconPC.VoxelDownSample(voxelSize);

	ReconPointCloud o3dPc = LahgiControl::instance().GetReconOverlayPointCloudHybrid(reconPcDownSampled, time);


	std::vector<ReconPointCppWrapper> pc;

	int pcSize = 0;
	if (o3dPc.points_.size() > o3dPc.colors_.size())
	{
		pcSize = o3dPc.colors_.size();
	}
	else
	{
		pcSize = o3dPc.points_.size();
	}
	pc.reserve(pcSize);



	for (int i = 0; i < pcSize; ++i)
	{
		ReconPointCppWrapper tmpPoint;
		tmpPoint.pointX = o3dPc.points_[i][0];
		tmpPoint.pointY = o3dPc.points_[i][1];
		tmpPoint.pointZ = o3dPc.points_[i][2];

		RGBA_t rgb = ReconPointCloud::ColorScaleJet(o3dPc.reconValues_[i], o3dPc.maxReoconValue * 0.7, o3dPc.maxReoconValue);

		tmpPoint.colorR = rgb.R;

		tmpPoint.colorG = rgb.G;
		tmpPoint.colorB = rgb.B;
		tmpPoint.colorA = rgb.A;

		pc.push_back(tmpPoint);
	}
	return pc;

}

std::vector<std::vector<double>> HUREL::Compton::RtabmapCppWrapper::GetOptimizedPoses()
{

	std::vector<Eigen::Matrix4d> poses = RtabmapSlamControl::instance().GetOptimizedPoses();
	std::vector<std::vector<double>> returnPoses;
	returnPoses.reserve(poses.size());
	for (int i = 0; i < poses.size(); ++i)
	{
		Eigen::Matrix4d m = poses[i];
		auto Matrix3Dtype = m.adjoint();
		std::vector<double> matrix3DOneLine;
		matrix3DOneLine.reserve(16);
		int idx = 0;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				matrix3DOneLine.push_back(Matrix3Dtype(i, j));
				idx++;
			}
		}
		returnPoses.push_back(matrix3DOneLine);
	}



	return returnPoses;
}


std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetLoadedPointCloud()
{
	std::vector<ReconPointCppWrapper> pc;

	open3d::geometry::PointCloud o3dPc = RtabmapSlamControl::instance().GetLoadedPointCloud();
	int pcSize = 0;
	if (o3dPc.points_.size() > o3dPc.colors_.size())
	{
		pcSize = o3dPc.colors_.size();
	}
	else
	{
		pcSize = o3dPc.points_.size();
	}
	pc.reserve(pcSize);
	for (int i = 0; i < pcSize; ++i)
	{
		ReconPointCppWrapper tmpPoint;
		tmpPoint.pointX = o3dPc.points_[i][0];
		tmpPoint.pointY = o3dPc.points_[i][1];
		tmpPoint.pointZ = o3dPc.points_[i][2];

		tmpPoint.colorR = o3dPc.colors_[i][0];

		tmpPoint.colorG = o3dPc.colors_[i][1];
		tmpPoint.colorB = o3dPc.colors_[i][2];
		tmpPoint.colorA = 1;
		pc.push_back(tmpPoint);
	}

	return pc;
}

RtabmapCppWrapper& HUREL::Compton::RtabmapCppWrapper::instance()
{
	static RtabmapCppWrapper& inst = *(new RtabmapCppWrapper());

	return inst;
}

std::tuple<double, double, double> HUREL::Compton::RtabmapCppWrapper::GetOdomentryPos()
{
	return RtabmapSlamControl::instance().GetOdomentryPos();
}

int HUREL::Compton::LahgiCppWrapper::GetSlamedPointCloudCount()
{
	return RtabmapSlamControl::instance().GetSlamedPointCloudCount();
}