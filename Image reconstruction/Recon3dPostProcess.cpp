#include "Recon3dPostProcess.h"
#include <cmath>

using namespace HUREL::Compton;
double HUREL::Compton::Recon3dPostProcess::normrnd(double mean, double stdDev)
{
	double u, v, s;
	do
	{
		u = ((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
		v = ((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
		s = u * u + v * v;
	} while (s >= 1 || s == 0);
	double mul = sqrt(-2.0 * log(s) / s);
	return mean + stdDev * u * mul;
}

double HUREL::Compton::Recon3dPostProcess::energyFWHM(double energy)
{
	return 0.08 * sqrt(energy);
}

double HUREL::Compton::Recon3dPostProcess::LinearFit(Eigen::MatrixXd& FitValue, Eigen::MatrixXd& EnergyFit, double Energy)
{
	double yValue = FitValue(0, 1) - FitValue(0, 0);
	double xValue = EnergyFit(0, 1) - EnergyFit(0, 0);
	double slope = yValue / xValue;
	return (slope * (Energy - EnergyFit(0, 1)) + FitValue(0, 1));
}

std::vector<ListModeData> HUREL::Compton::Recon3dPostProcess::GetListedCodedListModeData(std::vector<ListModeData>& lmData, Eigen::MatrixXd Egate)
{
	std::vector<ListModeData> CodedlmDataEff;
	for (int i = 0; i < lmData.size(); ++i)
	{
		if (lmData[i].Type == eInterationType::CODED)
		{
			if (lmData[i].Scatter.InteractionEnergy > Egate(0, 0) && lmData[i].Scatter.InteractionEnergy < Egate(0, 1))
			{
				CodedlmDataEff.push_back(lmData[i]);
			}
		}
	}
	return CodedlmDataEff;
}

std::vector<ListModeData> HUREL::Compton::Recon3dPostProcess::GetListedComptonListModeData(std::vector<ListModeData>& lmData, Eigen::MatrixXd Egate)
{
	std::vector<ListModeData> ComptonlmDataEff;
	for (int i = 0; i < lmData.size(); ++i)
	{
		if (lmData[i].Type == eInterationType::COMPTON)
		{
			if ((lmData[i].Scatter.InteractionEnergy + lmData[i].Absorber.InteractionEnergy) > Egate(0, 0) && (lmData[i].Scatter.InteractionEnergy + lmData[i].Absorber.InteractionEnergy) < Egate(0, 1))
			{
				if (lmData[i].Absorber.InteractionEnergy > 10 && lmData[i].Scatter.InteractionEnergy > 0)
				{
					ComptonlmDataEff.push_back(lmData[i]);
				}
			}
		}
	}
	return ComptonlmDataEff;
}

void HUREL::Compton::Recon3dPostProcess::removeRow(Eigen::MatrixXd& matrix, int rowToRemove)
{
	unsigned int numRows = matrix.rows() - 1;
	unsigned int numCols = matrix.cols();

	if (rowToRemove < numRows)
		matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

	matrix.conservativeResize(numRows, numCols);
}

void HUREL::Compton::Recon3dPostProcess::removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
	unsigned int numRows = matrix.rows();
	unsigned int numCols = matrix.cols() - 1;

	if (colToRemove < numCols)
		matrix.block(0, colToRemove, numRows, numCols - colToRemove) =
		matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

	matrix.conservativeResize(numRows, numCols);
}

double  HUREL::Compton::Recon3dPostProcess::sortposition(int lmposchk, std::vector<ListModeData>& lmDataEff)
{
	if (lmDataEff.size() == 0)
	{
		return 0;
	}
	if (lmposchk > lmDataEff.size())
	{
		return 0;
	}
	else
	{
		for (int i = lmposchk; i < lmDataEff.size(); ++i)
		{
			if (lmDataEff[i].InteractionTimeInMili != lmDataEff[i + 1].InteractionTimeInMili)
			{
				lmposchk = i;
				//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::Recon3dPostProcess", "End : " + std::to_string(lmposchk), eLoggerType::INFO);
				break;
			}
		}
		if (lmposchk != 0)
		{
			return lmposchk + 1;
		}
		else
			return 0;
	}

}

//240429 : transformation 이용 -
double  HUREL::Compton::Recon3dPostProcess::sortposition2(int lmposchk, std::vector<ListModeData>& lmDataEff)
{
	if (lmDataEff.size() == 0)
		return 0;

	if (lmposchk >= lmDataEff.size())
	{
		return 0;
	}
	else
	{
		int nEnd = 0;
		bool bFind = false;
		for (int i = lmposchk; i < lmDataEff.size() - 1; ++i)
		{
			Eigen::Matrix4d curTrans = lmDataEff[i].DetectorTransformation;
			Eigen::Matrix4d nextTrans = lmDataEff[i + 1].DetectorTransformation;

			int ncount = 0;
			for (int ii = 0; ii < 4; ii++)
			{
				for (int jj = 0; jj < 4; jj++)
				{
					if (abs(nextTrans(ii, jj) - curTrans(ii, jj)) <= 0.002)	//동일하다고 판단
						ncount++;
				}
			}

			if (ncount < 8)
			{
				nEnd = i;
				bFind = true;
				break;
			}
		}

		if (bFind == true)
		{
			return nEnd + 1;
		}
		else
			return lmDataEff.size();
	}

}

//240429 : transformation 이용 - 
double  HUREL::Compton::Recon3dPostProcess::sortposition3(int& lmposchk, std::vector<ListModeData>& lmDataEff, Eigen::Matrix4d& reftransformation)
{
	if (lmDataEff.size() == 0)
	{
		return 0;
	}
	if (lmposchk >= lmDataEff.size())
	{
		return 0;
	}
	else
	{
		int nEnd = 0;
		bool bFindStart = false;
		bool bFindEnd = false;
		//시작 지점 찾기
		for (int i = lmposchk; i < lmDataEff.size() - 1; ++i)
		{
			Eigen::Matrix4d curTrans = lmDataEff[i].DetectorTransformation;

			if (bFindStart == false)	//start point 찾기
			{
				int ncount = 0;
				for (int ii = 0; ii < 4; ii++)
				{
					for (int jj = 0; jj < 4; jj++)
					{
						if (abs(reftransformation(ii, jj) - curTrans(ii, jj)) <= 0.002)	//동일하다고 판단
							ncount++;
					}
				}

				if (ncount >= 8)
				{
					lmposchk = i;
					bFindStart = true;
					break;
				}
			}
		}

		if (bFindStart == true)
		{
			for (int i = lmposchk; i < lmDataEff.size(); ++i)
			{
				Eigen::Matrix4d curTrans = lmDataEff[i].DetectorTransformation;

				int ncount = 0;
				for (int ii = 0; ii < 4; ii++)
				{
					for (int jj = 0; jj < 4; jj++)
					{
						if (abs(reftransformation(ii, jj) - curTrans(ii, jj)) <= 0.002)	//동일하다고 판단
							ncount++;
					}
				}

				if (ncount < 8)
				{
					nEnd = i;
					bFindEnd = true;
					break;
				}
			}

			if (bFindEnd == true)
			{
				return nEnd + 1;
			}
			else
				return lmDataEff.size();
		}
		else
		{
			//동일한 시작지점이 없을 경우
			return 0;
		}
	}
}

//240429
double  HUREL::Compton::Recon3dPostProcess::sortposlmData(int Codedlmposchkend, int Codedlmposchkstart, int Comptonlmposchkstart, int Comptonlmposchkend,
	std::vector<ListModeData>& ComptonlmDataEff, std::vector<ListModeData>& CodedlmDataEff,
	std::vector<ListModeData>* outCodedlmDataEff, std::vector<ListModeData>* outComptonlmDataEff,
	Eigen::Matrix4d* outtransformation)
{
	std::vector<ListModeData>& posCodedlmDataEff = *outCodedlmDataEff;
	std::vector<ListModeData>& posComptonlmDataEff = *outComptonlmDataEff;
	Eigen::Matrix4d& transformation = *outtransformation;

	int casenum = 0;

	if (Comptonlmposchkend != 0)
	{
		for (int i = Comptonlmposchkstart; i < Comptonlmposchkend; ++i)
		{
			posComptonlmDataEff.push_back(ComptonlmDataEff[i]);
		}
	}
	if (Codedlmposchkend != 0)
	{
		for (int i = Codedlmposchkstart; i < Codedlmposchkend; ++i)
		{
			posCodedlmDataEff.push_back(CodedlmDataEff[i]);
		}
	}

	if (Comptonlmposchkend > 0 && Codedlmposchkend > 0)
	{
		if (posComptonlmDataEff.size() > 0 && posCodedlmDataEff.size() > 0)
		{
			transformation = posComptonlmDataEff[0].DetectorTransformation;
			casenum = 0;  //coded and compton image recon 
		}
		else if (posComptonlmDataEff.size() > 0 && posCodedlmDataEff.size() <= 0)
		{
			transformation = posComptonlmDataEff[0].DetectorTransformation;
			casenum = 2; //compton image recon only
		}
		else if (posComptonlmDataEff.size() <= 0 && posCodedlmDataEff.size() > 0)
		{
			transformation = posCodedlmDataEff[0].DetectorTransformation;
			casenum = 1; //coded image recon only
		}
		else
			casenum = 3;
	}
	else if (Codedlmposchkend > 0 && Comptonlmposchkend == 0)
	{
		transformation = posCodedlmDataEff[0].DetectorTransformation;
		casenum = 1; //coded image recon only
	}
	else if (Comptonlmposchkend > 0 && Codedlmposchkend == 0)
	{
		transformation = posComptonlmDataEff[0].DetectorTransformation;
		casenum = 2;
	}
	else
	{
		//std::cout << "Empty list mode" << std::endl;
		casenum = 3;
	}

	return casenum;
}

void HUREL::Compton::Recon3dPostProcess::sortposlmDataNEC(std::vector<ListModeData>& CodedlmDataEff, std::vector<ListModeData>& ComptonlmDataEff, std::vector<ListModeData>* outCodedlmDataEff, std::vector<ListModeData>* outComptonlmDataEff, Eigen::Matrix4d& transformation)
{
	std::vector<ListModeData>& posCodedlmDataNEC = *outCodedlmDataEff;
	std::vector<ListModeData>& posComptonlmDataNEC = *outComptonlmDataEff;
	for (size_t size = 0; size < CodedlmDataEff.size(); ++size)
	{
		if (CodedlmDataEff[size].DetectorTransformation == transformation)
		{
			posCodedlmDataNEC.push_back(CodedlmDataEff[size]);
		}
	}
	for (size_t size2 = 0; size2 < ComptonlmDataEff.size(); ++size2)
	{
		if (ComptonlmDataEff[size2].DetectorTransformation == transformation)
		{
			posComptonlmDataNEC.push_back(ComptonlmDataEff[size2]);
		}
	}
}

double HUREL::Compton::Recon3dPostProcess::imspacetranspose(HUREL::Compton::ReconPointCloud& reconPCFOVlim, 
	Eigen::Matrix4d* transformMatrixptr, HUREL::Compton::ReconPointCloud* outReconPCFOVlimtranposed)
{
	ReconPointCloud& reconPCFOVlimtranposed = *outReconPCFOVlimtranposed;
	Eigen::MatrixXd tranposematrix(4, 1);
	Eigen::MatrixXd transformation(4, 4);
	transformation = *transformMatrixptr;
	Eigen::MatrixXd tranposematrixpoint(4, 1);
	tranposematrixpoint(3, 0) = 1;
	for (size_t size = 0; size < reconPCFOVlim.points_.size(); ++size)
	{
		tranposematrixpoint(0, 0) = reconPCFOVlim.points_[size].x();
		tranposematrixpoint(1, 0) = reconPCFOVlim.points_[size].y();
		tranposematrixpoint(2, 0) = reconPCFOVlim.points_[size].z();
		tranposematrix = transformation.inverse() * tranposematrixpoint;
		reconPCFOVlimtranposed.points_[size].x() = tranposematrix(0, 0);
		reconPCFOVlimtranposed.points_[size].y() = tranposematrix(1, 0);
		reconPCFOVlimtranposed.points_[size].z() = tranposematrix(2, 0);
		reconPCFOVlimtranposed.colors_[size] = reconPCFOVlim.colors_[size];
		reconPCFOVlimtranposed.reconValues_[size] = reconPCFOVlim.reconValues_[size];
	}
	return 0;
}

double HUREL::Compton::Recon3dPostProcess::imspacetranspose2D(HUREL::Compton::ReconPointCloud& reconPCFOVlim, Eigen::Matrix4d* transformMatrixptr, HUREL::Compton::ReconPointCloud* outReconPCFOVlimtranposed)
{
	ReconPointCloud& reconPCFOVlimtranposed = *outReconPCFOVlimtranposed;
	Eigen::MatrixXd tranposematrix(4, 1);
	Eigen::MatrixXd transformation(4, 4);
	transformation = *transformMatrixptr;
	Eigen::MatrixXd tranposematrixpoint(4, 1);
	tranposematrixpoint(3, 0) = 1;
	for (size_t size = 0; size < reconPCFOVlim.points_.size(); ++size)
	{
		tranposematrixpoint(0, 0) = reconPCFOVlim.points_[size].x();
		tranposematrixpoint(1, 0) = reconPCFOVlim.points_[size].y();
		tranposematrixpoint(2, 0) = reconPCFOVlim.points_[size].z();

		tranposematrix = transformation.inverse() * tranposematrixpoint;
				
		//reconPCFOVlimtranposed.points_.push_back(Eigen::Vector3d(tranposematrix(0, 0), tranposematrix(1, 0), tranposematrix(2, 0)));
		//reconPCFOVlimtranposed.colors_.push_back(reconPCFOVlim.colors_[size]);
		//reconPCFOVlimtranposed.reconValues_.push_back(reconPCFOVlim.reconValues_[size]);

		reconPCFOVlimtranposed.points_[size].x() = tranposematrix(0, 0);
		reconPCFOVlimtranposed.points_[size].y() = tranposematrix(1, 0);
		reconPCFOVlimtranposed.points_[size].z() = tranposematrix(2, 0);
		reconPCFOVlimtranposed.colors_[size] = reconPCFOVlim.colors_[size];
		reconPCFOVlimtranposed.reconValues_[size] = reconPCFOVlim.reconValues_[size];
	}
	return 0;
}

//전체 point cloud 데이터에서 현재 포즈의 데이터만 추출
double HUREL::Compton::Recon3dPostProcess::imspacelim(HUREL::Compton::ReconPointCloud& reconPC, 
	Eigen::Matrix4d* transformMatrix, int azFOV, int polFOV,
	HUREL::Compton::ReconPointCloud* outReconPCFOVlim, Eigen::MatrixXd* outFovchk)
{
	ReconPointCloud& reconPCFOVlim = *outReconPCFOVlim;
	Eigen::MatrixXd& fovchk = *outFovchk;
	Eigen::Vector3d azvec(3); Eigen::Vector3d polarvec(3);
	Eigen::MatrixXd normvec(4, 1); normvec(0) = 0.; normvec(1) = 0.; normvec(2) = 1.; normvec(3) = 0.;
	Eigen::Vector3d normvec2(3); normvec2[0] = normvec(0); normvec2[1] = normvec(1); normvec2[2] = normvec(2);
	Eigen::Vector3d normvectransposed(3);
	Eigen::Vector3d deterctorpos(3);
	Eigen::Matrix4d transformation(4, 4); transformation = *transformMatrix;
	deterctorpos[0] = transformation(0, 3);
	deterctorpos[1] = transformation(1, 3);
	deterctorpos[2] = transformation(2, 3);
	Eigen::Matrix4d* transformptr = &transformation;

	//Recon3dPostProcess Post3drecon;
	double aziangle; double polarangle;

	//calculate azimutal angle
	normvec = transformation * normvec;
	normvectransposed[0] = normvec(0);
	normvectransposed[1] = normvec(1);
	normvectransposed[2] = normvec(2);
	normvectransposed = normvectransposed.normalized();

	HUREL::Compton::ReconPointCloud reconPCFOVlimtranposed = reconPC;

	//calculate polar angle
	imspacetranspose(reconPC, transformptr, &reconPCFOVlimtranposed);

	for (size_t size = 0; size < reconPC.points_.size(); ++size)
	{
		fovchk(0, size) = 0;
		//		azvec[0] = reconPC.points_[size].x() - deterctorpos(0); azvec[1] = 0; azvec[2] = reconPC.points_[size].z() - deterctorpos(2);
		azvec[0] = reconPCFOVlimtranposed.points_[size].x();
		azvec[1] = 0;
		azvec[2] = reconPCFOVlimtranposed.points_[size].z();
		azvec = azvec.normalized();
		aziangle = acos(azvec.dot(normvectransposed)) / EIGEN_PI * 180;

		polarvec[0] = 0;
		polarvec[1] = reconPCFOVlimtranposed.points_[size].y();
		polarvec[2] = reconPCFOVlimtranposed.points_[size].z();
		polarvec = polarvec.normalized();
		polarangle = acos(polarvec.dot(normvec2)) / EIGEN_PI * 180;

		if (aziangle <= azFOV && polarangle <= polFOV) //FOV 내인지 확인
		{
			fovchk(0, size) = 1; //FOV 내이면 index 1로 대입
			reconPCFOVlim.points_.push_back(reconPC.points_[size]);
			reconPCFOVlim.colors_.push_back(reconPC.colors_[size]);
			reconPCFOVlim.reconValues_.push_back(reconPC.reconValues_[size]);
		}
		else
			fovchk(0, size) = 0;
	}
	return 0;
}

//signal / background count 연산
void HUREL::Compton::Recon3dPostProcess::CodedNECCal(std::vector<ListModeData>& lmDataEff, int accumN, Eigen::MatrixXd& EgateNEC, Eigen::MatrixXd* outSignalmatrix, Eigen::MatrixXd* outBkgmatrix)
{
	double Energy;	int indexN; int indexN2;
	Eigen::MatrixXd EnergyBinMean = MatrixXd::Zero(300, 1);

	VectorXd EnergyMeanAxis;
	EnergyMeanAxis.setLinSpaced(300, 5, 2995);

	Eigen::MatrixXd EnergyFit(1, 2);
	Eigen::MatrixXd FitValue(1, 2);

	Eigen::MatrixXd& CodedNECSignal = *outSignalmatrix;
	Eigen::MatrixXd& CodedNECBackground = *outBkgmatrix;
	double bkgCount = 0;
	double totalCount = 0;

	for (size_t size1 = 0; size1 < lmDataEff.size(); ++size1)
	{
		Energy = lmDataEff[size1].Scatter.InteractionEnergy;
		indexN = floor(Energy / 10);
		EnergyBinMean(indexN, 0) = EnergyBinMean(indexN, 0) + 1;
	}

	indexN = floor(EgateNEC(0, 0) / 10);
	indexN2 = floor(EgateNEC(0, 1) / 10);

	EnergyFit(0, 0) = EnergyMeanAxis(indexN);
	EnergyFit(0, 1) = EnergyMeanAxis(indexN2);
	FitValue(0, 0) = EnergyBinMean(indexN, 0);
	FitValue(0, 1) = EnergyBinMean(indexN2, 0);

	for (size_t size2 = indexN; size2 < (indexN2 + 1); ++size2)
	{
		bkgCount = bkgCount + LinearFit(FitValue, EnergyFit, EnergyMeanAxis(size2));
		totalCount = totalCount + EnergyBinMean(size2);
	}

	CodedNECBackground(0, accumN) = bkgCount;
	CodedNECSignal(0, accumN) = totalCount - bkgCount;
}

//signal / background count 연산
void  HUREL::Compton::Recon3dPostProcess::ComptonNECCal(std::vector<ListModeData>& lmDataEff, int accumN, Eigen::MatrixXd& EgateNEC, Eigen::MatrixXd* outSignalmatrix, Eigen::MatrixXd* outBkgmatrix)
{
	double Energy;	int indexN; int indexN2;

	Eigen::MatrixXd EnergyBinMean = MatrixXd::Zero(300, 1);	//energy bin count
	VectorXd EnergyMeanAxis;

	EnergyMeanAxis.setLinSpaced(300, 5, 2995); // 300 : size(vector size), 5 : low(첫번째 요소 값), 2995 : high(마지막 요소 값)

	Eigen::MatrixXd EnergyFit(1, 2);
	Eigen::MatrixXd FitValue(1, 2);

	Eigen::MatrixXd& ComptonNECSignal = *outSignalmatrix;
	Eigen::MatrixXd& ComptonNECBackground = *outBkgmatrix;
	double bkgCount = 0;
	double totalCount = 0;

	//energy bin count
	for (size_t size1 = 0; size1 < lmDataEff.size(); ++size1)
	{
		Energy = lmDataEff[size1].Scatter.InteractionEnergy + lmDataEff[size1].Absorber.InteractionEnergy;
		indexN = floor(Energy / 10);
		EnergyBinMean(indexN, 0) = EnergyBinMean(indexN, 0) + 1;
	}

	indexN = floor(EgateNEC(0, 0) / 10);	//Egate Min 값에 해당하는 bin
	indexN2 = floor(EgateNEC(0, 1) / 10);	//Egate Max 값에 해당하는 bin

	EnergyFit(0, 0) = EnergyMeanAxis(indexN);
	EnergyFit(0, 1) = EnergyMeanAxis(indexN2);
	FitValue(0, 0) = EnergyBinMean(indexN, 0);
	FitValue(0, 1) = EnergyBinMean(indexN2, 0);

	for (size_t size2 = indexN; size2 < (indexN2 + 1); ++size2)
	{
		bkgCount = bkgCount + LinearFit(FitValue, EnergyFit, EnergyMeanAxis(size2));
		totalCount = totalCount + EnergyBinMean(size2);
	}

	ComptonNECBackground(0, accumN) = bkgCount;
	ComptonNECSignal(0, accumN) = totalCount - bkgCount;
}

double HUREL::Compton::Recon3dPostProcess::calEgate(double peakEnergy, Eigen::MatrixXd* outEgate)
{
	Eigen::MatrixXd& Egate = *outEgate;
	double stdCL; double stdCU;
	Recon3dPostProcess Post3drecon;
	if (peakEnergy <= 130)
	{
		stdCL = 7; stdCU = 10;
	}
	else
	{
		stdCL = 2; stdCU = 2;
	}
	Egate(0, 0) = floor((peakEnergy / 1000 - Post3drecon.energyFWHM(peakEnergy / 1000) / 2.35 * stdCL) * 1000);
	Egate(0, 1) = ceil((peakEnergy / 1000 + Post3drecon.energyFWHM(peakEnergy / 1000) / 2.35 * stdCU) * 1000);
	//std::cout << "Energy gate is: " << Egate << std::endl;
	return 0;
}