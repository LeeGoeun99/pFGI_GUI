#include "pch.h"

#include "Lahgi wrapper.h"

using namespace HUREL::Compton;

HUREL::Compton::LahgiWrapper::LahgiWrapper()
{
}

bool HUREL::Compton::LahgiWrapper::Initiate(eModuleManagedType type)
{
	WrapperLogger::Log::Info("C++CLR::HUREL::Compton::LahgiWrapper", "Initiate");
	HUREL::Compton::eModuleCppWrapper moduleType = HUREL::Compton::eModuleCppWrapper::MONO;
	switch (type)
	{
	case HUREL::Compton::eModuleManagedType::MONO:
		moduleType = HUREL::Compton::eModuleCppWrapper::MONO;
		break;
	case HUREL::Compton::eModuleManagedType::QUAD:
		moduleType = HUREL::Compton::eModuleCppWrapper::QUAD;

		break;
	case HUREL::Compton::eModuleManagedType::QUAD_DUAL:
		moduleType = HUREL::Compton::eModuleCppWrapper::QUAD_DUAL;
		break;
	default:
		break;
	}
	return LahgiCppWrapper::instance().SetType(moduleType);
}

void HUREL::Compton::LahgiWrapper::AddListModeDataWraper(array<unsigned short>^ adcData)
{
	pin_ptr<unsigned short> intParamsPtr = &adcData[0];

	LahgiCppWrapper::instance().AddListModeDataWithTransformation(intParamsPtr);

}

void HUREL::Compton::LahgiWrapper::SetEchks(List<array<double>^>^ echks, List<int>^ elements)
{

	std::vector<std::vector<double>> eChkUnmanagedVector;
	eChkUnmanagedVector.reserve(echks->Count);
	for each (array<double> ^ e in echks)
	{
		std::vector<double> eChkUnmanaged;
		eChkUnmanaged.reserve(2);
		double minE = e[0];
		double maxE = e[1];
		eChkUnmanaged.push_back(minE);
		eChkUnmanaged.push_back(maxE);

		eChkUnmanagedVector.push_back(eChkUnmanaged);
	}

	std::vector<int> eChkElements;
	eChkElements.reserve(elements->Count);
	for each (int el in elements)
	{
		eChkElements.push_back(el);
	}

	LahgiCppWrapper::instance().SetEchks(eChkUnmanagedVector, eChkElements);
}
//240123
void HUREL::Compton::LahgiWrapper::SelectEchks(List<int>^ elements)
{
	std::vector<int> eChkElements;
	eChkElements.reserve(elements->Count);
	for each (int el in elements)
	{
		eChkElements.push_back(el);
	}

	LahgiCppWrapper::instance().SelectEchks(eChkElements);
}


void HUREL::Compton::LahgiWrapper::GetRelativeListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE)
{
	std::vector<ListModeDataCppWrapper> lists = LahgiCppWrapper::instance().GetRelativeListModeData();
	scatterXYZE->Clear();
	absorberXYZE->Clear();

	for (int i = 0; i < lists.size(); ++i)
	{
		array<double>^ tempScatterArray = gcnew array<double>(4);
		array<double>^ tempAbsorberArray = gcnew array<double>(4);
		tempScatterArray[0] = lists[i].ScatterRelativeInteractionPointX;
		tempScatterArray[1] = lists[i].ScatterRelativeInteractionPointY;
		tempScatterArray[2] = lists[i].ScatterRelativeInteractionPointZ;
		tempScatterArray[3] = lists[i].ScatterInteractionEnergy;

		tempAbsorberArray[0] = lists[i].AbsorberRelativeInteractionPointX;
		tempAbsorberArray[1] = lists[i].AbsorberRelativeInteractionPointY;
		tempAbsorberArray[2] = lists[i].AbsorberRelativeInteractionPointZ;
		tempAbsorberArray[3] = lists[i].AbsorberInteractionEnergy;


		scatterXYZE->Add(tempScatterArray);
		absorberXYZE->Add(tempAbsorberArray);
	}
}

Int64 HUREL::Compton::LahgiWrapper::GetListedListModeDataSize()
{
	return LahgiCppWrapper::instance().GetListedListModeDataSize();
}


void HUREL::Compton::LahgiWrapper::ResetListmodeData()
{
	WrapperLogger::Log::Info("C++CLR::UREL::Compton::LahgiWrapper", "Reset list mode data");
	LahgiCppWrapper::instance().ResetListedListModeData();
}

void HUREL::Compton::LahgiWrapper::GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetSpectrum(channelNumer);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetEcal(unsigned int channelNumer, double% ecalA, double% ecalB, double% ecalC)
{
	std::tuple<double, double, double> ecals = LahgiCppWrapper::instance().GetEcalValue(channelNumer);

	ecalA = std::get<0>(ecals);
	ecalB = std::get<1>(ecals);
	ecalC = std::get<2>(ecals);
}

void HUREL::Compton::LahgiWrapper::SetEcal(unsigned int channelNumer, double ecalA, double ecalB, double ecalC)
{
	std::tuple<double, double, double> ecals = std::make_tuple(static_cast<double>(ecalA), static_cast<double>(ecalB), static_cast<double>(ecalC));
	LahgiCppWrapper::instance().SetEcalValue(channelNumer, ecals);
}

void HUREL::Compton::LahgiWrapper::GetSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetSumSpectrum();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

//231100-GUI sbkwon : time
void HUREL::Compton::LahgiWrapper::GetSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetSumSpectrum(time);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetAbsorberSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetAbsorberSumSpectrum();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::SaveListModeData(System::String^ fileName)
{
	IntPtr ptrToNativeString = Marshal::StringToHGlobalAnsi(fileName);
	LahgiCppWrapper::instance().SaveListedListModeData(static_cast<char*>(ptrToNativeString.ToPointer()));
}

bool HUREL::Compton::LahgiWrapper::LoadListModeData(System::String^ filePath)
{
	IntPtr ptrToNativeString = Marshal::StringToHGlobalAnsi(filePath);
	return LahgiCppWrapper::instance().LoadListedListModeData(static_cast<char*>(ptrToNativeString.ToPointer()));
}

void HUREL::Compton::LahgiWrapper::GetScatterSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetScatterSumSpectrum();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetScatterSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetScatterSumSpectrum(time);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}

}

//250410
void HUREL::Compton::LahgiWrapper::GetSpectrumData(List<array<double>^>^% energyCount, unsigned int type, unsigned int ch)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetSpectrumData(type, ch);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}

}

//231123 sbkwon
void HUREL::Compton::LahgiWrapper::GetSpectrumByTime(unsigned int channelNumer, List<array<double>^>^% energyCount, unsigned int time)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetSpectrum(channelNumer, time);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

//240311 : 고장 검사 설정
void HUREL::Compton::LahgiWrapper::SetUseFD(bool set)
{
	LahgiCppWrapper::instance().SetUseFD(set);
}

//240228 : 고장검사 PMT Data Copy
void HUREL::Compton::LahgiWrapper::CopyPMTData()
{
	LahgiCppWrapper::instance().CopyPMTData();
}

//240228 채널별 게인 획득
void HUREL::Compton::LahgiWrapper::GetGainref(unsigned int channelNumer, List<double>^% corrMatRef)
{
	std::vector <double> gain = LahgiCppWrapper::instance().GetGainref(channelNumer);

	corrMatRef = gcnew List<double>();
	corrMatRef->Capacity = gain.size();
	for (size_t i = 0; i < gain.size(); i++)
	{
		corrMatRef->Add(gain[i]);
	}
}

//240228 : 고장검사 - 채널별 Bin Count를 반환한다.
void HUREL::Compton::LahgiWrapper::GetPMTEnergyData(unsigned int channelNumer, List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetPMTEnergyData(channelNumer);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

//240228 : 고장검사 - CorrMatIn을 이용하여 채널별 Bin Count를 반환한다.
void HUREL::Compton::LahgiWrapper::GetPMTEnergyData(unsigned int channelNumer, List<double>^ corrMatIn, List<array<double>^>^% energyCount)
{
	std::vector<double> vcorrMatIn;
	vcorrMatIn.reserve(corrMatIn->Count);
	for each (double range in corrMatIn)
	{
		vcorrMatIn.push_back(range);
	}

	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetPMTEnergyData(channelNumer, vcorrMatIn);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

//240228 : 고장검사 - usedPead의 Range를 이용하여 gain 계산한 값을 반환한다.
void HUREL::Compton::LahgiWrapper::GetPMTCorrMatIn(unsigned int channelNumer, List<int>^ usedPeak, List<double>^ range_bkg, List<double>^% corrMatIn)
{
	std::vector<int> vusedPeak;
	vusedPeak.reserve(usedPeak->Count);
	for each (int peak in usedPeak)
	{
		vusedPeak.push_back(peak);
	}

	std::vector<double> vrange_bkg;
	vrange_bkg.reserve(range_bkg->Count);
	for each (double range in range_bkg)
	{
		vrange_bkg.push_back(range);
	}

	std::vector <double> gain = LahgiCppWrapper::instance().GetPMTCorrMatIn(channelNumer, vusedPeak, vrange_bkg);

	corrMatIn = gcnew List<double>();
	corrMatIn->Capacity = gain.size();
	for (size_t i = 0; i < gain.size(); i++)
	{
		corrMatIn->Add(gain[i]);
	}
}

//240315
void HUREL::Compton::LahgiWrapper::GetPMTCorrMatInBeforGain(unsigned int channelNumer, List<int>^ usedPeak, List<double>^ range_bkg, List<double>^ corrMatIn, List<double>^% corrMatOut)
{
	std::vector<int> vusedPeak;
	vusedPeak.reserve(usedPeak->Count);
	for each (int peak in usedPeak)
	{
		vusedPeak.push_back(peak);
	}

	std::vector<double> vrange_bkg;
	vrange_bkg.reserve(range_bkg->Count);
	for each (double range in range_bkg)
	{
		vrange_bkg.push_back(range);
	}

	std::vector<double> vcorrMatIn;
	vcorrMatIn.reserve(corrMatIn->Count);
	for each (double range in corrMatIn)
	{
		vcorrMatIn.push_back(range);
	}

	std::vector <double> gain = LahgiCppWrapper::instance().GetPMTCorrMatInBeforGain(channelNumer, vusedPeak, vrange_bkg, vcorrMatIn);

	corrMatOut = gcnew List<double>();
	corrMatOut->Capacity = gain.size();
	for (size_t i = 0; i < gain.size(); i++)
	{
		corrMatOut->Add(gain[i]);
	}
}

void HUREL::Compton::LahgiWrapper::GetAbsorberSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetAbsorberSumSpectrum(time);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::ResetSpectrum(unsigned int channelNumber)
{
	LahgiCppWrapper::instance().RestEnergySpectrum(channelNumber);
}

//231113-1 sbkwon
void HUREL::Compton::LahgiWrapper::InitRadiationImage()
{
	LahgiCppWrapper::instance().InitRadiationImage();
}

Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>^ HUREL::Compton::LahgiWrapper::Get2dRadationImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion)
{
	auto data = LahgiCppWrapper::instance().GetRadiation2dImage(timeInMiliSeconds, s2M, det_W, resImprov, m2D, hFov, wFov, imgSize, minValuePortion);
	return gcnew Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>(gcnew sBitmapWrapper(std::get<0>(data)), gcnew sBitmapWrapper(std::get<1>(data)), gcnew sBitmapWrapper(std::get<2>(data)));
}

//231025-1 sbkwon
Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>^ HUREL::Compton::LahgiWrapper::Get2dRadationImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double minValuePortion)
{
	auto data = LahgiCppWrapper::instance().GetRadiation2dImage(timeInMiliSeconds, s2M, det_W, resImprov, m2D, minValuePortion);
	return gcnew Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>(gcnew sBitmapWrapper(std::get<0>(data)), gcnew sBitmapWrapper(std::get<1>(data)), gcnew sBitmapWrapper(std::get<2>(data)));
}

//231212 : 실외(plane), 241021 labeling
Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>^ HUREL::Compton::LahgiWrapper::GetRadation2dImageCount(int count, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion, int time, int maxValue, bool fullrange, bool labeling)
{
	if (labeling == false)
	{
		auto data = LahgiCppWrapper::instance().GetRadation2dImageCount(count, s2M, det_W, resImprov, m2D, hFov, wFov, imgSize, minValuePortion, time, maxValue, fullrange);
		return gcnew Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>(gcnew sBitmapWrapper(std::get<0>(data)), gcnew sBitmapWrapper(std::get<1>(data)), gcnew sBitmapWrapper(std::get<2>(data)));
	}
	else
	{
		auto data = LahgiCppWrapper::instance().GetRadation2dImageCountLabel(count, s2M, det_W, resImprov, m2D, hFov, wFov, imgSize, minValuePortion, time, maxValue, fullrange);
		return gcnew Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>(gcnew sBitmapWrapper(std::get<0>(data)), gcnew sBitmapWrapper(std::get<1>(data)), gcnew sBitmapWrapper(std::get<2>(data)));
	}
}

//231100-GUI sbkwon : 실내(pointcloud), 241021 labeling
Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>^ HUREL::Compton::LahgiWrapper::GetRadation2dImageCount(int count, double s2M, double det_W, double resImprov, double m2D, double minValuePortion, int time, int maxValue, bool labeling)
{
	if (labeling == false)
	{
		auto data = LahgiCppWrapper::instance().GetRadation2dImageCount(count, s2M, det_W, resImprov, m2D, minValuePortion, time, maxValue);
		return gcnew Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>(gcnew sBitmapWrapper(std::get<0>(data)), gcnew sBitmapWrapper(std::get<1>(data)), gcnew sBitmapWrapper(std::get<2>(data)));
	}
	else
	{
		auto data = LahgiCppWrapper::instance().GetRadation2dImageCountLabel(count, s2M, det_W, resImprov, m2D, minValuePortion, time, maxValue);
		return gcnew Tuple<sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>(gcnew sBitmapWrapper(std::get<0>(data)), gcnew sBitmapWrapper(std::get<1>(data)), gcnew sBitmapWrapper(std::get<2>(data)));
	}
}

sBitmapWrapper^ HUREL::Compton::LahgiWrapper::GetTransPoseRadiationImage(int timeInMiliSeconds, double minValuePortion, double resolution)
{
	return gcnew sBitmapWrapper(LahgiCppWrapper::instance().GetTransPoseRadiationImage(timeInMiliSeconds, minValuePortion, resolution));
}


void HUREL::Compton::LahgiWrapper::GetRealTimeReconImage(double time, eReconType reconType, int% width, int% height, int% stride, IntPtr% data)
{
	//cv::Mat color = cv::Mat();
	//switch (reconType)
	//{
	//case HUREL::Compton::eReconType::CODED:
	//{
	//	color = lahgiControlInstance->GetListModeImageCoded(time);
	//	break;
	//}
	//case HUREL::Compton::eReconType::COMPTON:
	//	color = lahgiControlInstance->GetListModeImageCompton(time);
	//	break;
	//case HUREL::Compton::eReconType::HYBRID:
	//	break;
	//default:
	//	color = lahgiControlInstance->GetListModeImageCompton(time);
	//	break;
	//}


	//if (color.cols > 1) {
	//	width = color.cols;
	//	height = color.rows;
	//	stride = color.step;
	//	void* ptr = static_cast<void*>(color.ptr());

	//	if (ptr == NULL)
	//	{
	//		return;
	//	}

	//	data = IntPtr(ptr);
	//}
}

sBitmapWrapper^ HUREL::Compton::LahgiWrapper::GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter)
{
	auto data = LahgiCppWrapper::instance().GetResponseImage(imgSize, pixelCount, timeInSeconds, isScatter);

	return gcnew sBitmapWrapper(data);
}


void HUREL::Compton::LahgiWrapper::Logging(std::string className, std::string msg)
{
	System::String^ classNameManage = gcnew System::String(className.c_str());
	System::String^ msgManage = gcnew System::String(msg.c_str());
	WrapperLogger::Log::Info(classNameManage, msgManage);
}

int HUREL::Compton::LahgiWrapper::GetSlamedPointCloudCount()
{
	return LahgiCppWrapper::instance().GetSlamedPointCloudCount();
}

//2404 : MLEM
bool HUREL::Compton::LahgiWrapper::LoadMLEMData(System::String^ PLYPath, System::String^ LMDPath, bool bLoad, int nSize)
{
	IntPtr ptrToNativesPLYPath = Marshal::StringToHGlobalAnsi(PLYPath);
	IntPtr ptrToNativesLMDPath = Marshal::StringToHGlobalAnsi(LMDPath);

	return LahgiCppWrapper::instance().LoadMLEMData(static_cast<char*>(ptrToNativesPLYPath.ToPointer()), static_cast<char*>(ptrToNativesLMDPath.ToPointer()), bLoad, nSize);
}

//240429 : MLEM List
bool HUREL::Compton::LahgiWrapper::CalMLEMList(System::String^ systemMPath, List<double>^ energy, List<double>^ EgateMin, List<double>^ EgateMax, double minValuePer)
{
	IntPtr ptrToNativesystemMPath = Marshal::StringToHGlobalAnsi(systemMPath);

	std::vector<double> vEnergy;
	vEnergy.reserve(energy->Count);
	for each (int peak in energy)
	{
		vEnergy.push_back(peak);
	}

	std::vector<double> vEgateMin;
	vEgateMin.reserve(EgateMin->Count);
	for each (int peak in EgateMin)
	{
		vEgateMin.push_back(peak);
	}

	std::vector<double> vEgateMax;
	vEgateMax.reserve(EgateMax->Count);
	for each (int peak in EgateMax)
	{
		vEgateMax.push_back(peak);
	}

	return LahgiCppWrapper::instance().CalMLEMList(static_cast<char*>(ptrToNativesystemMPath.ToPointer()), vEnergy, vEgateMin, vEgateMax, minValuePer);
}

//2404 : MLEM
bool HUREL::Compton::LahgiWrapper::CalMLEM(System::String^ systemMPath, double energy, double EgateMin, double EgateMax, double minValuePer)
{
	IntPtr ptrToNativesystemMPath = Marshal::StringToHGlobalAnsi(systemMPath);

	return LahgiCppWrapper::instance().CalMLEM(static_cast<char*>(ptrToNativesystemMPath.ToPointer()), energy, EgateMin, EgateMax, minValuePer);
}

//2404 : MLEM
void HUREL::Compton::LahgiWrapper::GetMLEMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, int nNo)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	std::vector<ReconPointCppWrapper> pose = LahgiCppWrapper::instance().GetMLEMPointCloud(nNo);


	int count = pose.size();

	vectors->Capacity = count;
	colors->Capacity = count;

	//Trace::WriteLine("SLAM points: " + count);
	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose[i].pointX, pose[i].pointY, pose[i].pointZ};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose[i].colorR, pose[i].colorG, pose[i].colorB, pose[i].colorA};
		colors->Add(colorVector);
	}
}

//250107 : 정밀영상 2D 영상 획득
bool HUREL::Compton::LahgiWrapper::Get2DMLEMData(System::String^ Path, int nNo)
{
	IntPtr ptrToNativesPath = Marshal::StringToHGlobalAnsi(Path);
	return LahgiCppWrapper::instance().Get2DMLEMData(static_cast<char*>(ptrToNativesPath.ToPointer()), nNo);;
}