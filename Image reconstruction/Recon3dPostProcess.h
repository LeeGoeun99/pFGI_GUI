#pragma once

#include <open3d/geometry/PointCloud.h>
#include "ListModeData.h"
#include "ReconPointCloud.h"

namespace HUREL
{
	namespace Compton
	{
		class ReconPointCloud;
		class Recon3dPostProcess
		{
		private:

		public:

			double normrnd(double mean, double stdDev);
			double energyFWHM(double energy);
			std::vector<ListModeData> GetListedCodedListModeData(std::vector<ListModeData>& lmData, Eigen::MatrixXd Egate);
			std::vector<ListModeData> GetListedComptonListModeData(std::vector<ListModeData>& lmData, Eigen::MatrixXd Egate);
			void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);
			void removeRow(Eigen::MatrixXd& matrix, int rowToRemove);
			double sortposlmData(int Codedlmposchkend, int Codedlmposchkstart, int Comptonlmposchkstart, int Comptonlmposchkend,
				std::vector<ListModeData>& ComptonlmDataEff, std::vector<ListModeData>& CodedlmDataEff, std::vector<ListModeData>* outCodedlmDataEff, std::vector<ListModeData>* outComptonlmDataEff, Eigen::Matrix4d* outtransformation);
			void sortposlmDataNEC(std::vector<ListModeData>& CodedlmDataEff, std::vector<ListModeData>& ComptonlmDataEff, std::vector<ListModeData>* outCodedlmDataEff, std::vector<ListModeData>* outComptonlmDataEff, Eigen::Matrix4d& transformation);
			double sortposition(int lmposchk, std::vector<ListModeData>& lmDataEff);
			double sortposition2(int lmposchk, std::vector<ListModeData>& lmDataEff);
			double sortposition3(int& lmposchk, std::vector<ListModeData>& lmDataEff, Eigen::Matrix4d& Comtransformation);
			double imspacetranspose(HUREL::Compton::ReconPointCloud& reconPCFOVlim, Eigen::Matrix4d* transformation, HUREL::Compton::ReconPointCloud* outReconPCFOVlimtranposed);
			double imspacetranspose2D(HUREL::Compton::ReconPointCloud& reconPCFOVlim, Eigen::Matrix4d* transformation, HUREL::Compton::ReconPointCloud* outReconPCFOVlimtranposed);//240621
			double imspacelim(HUREL::Compton::ReconPointCloud& reconPC, Eigen::Matrix4d* transformMatrix, int azFOV, int polFOV, HUREL::Compton::ReconPointCloud* outReconPCFOVlim, Eigen::MatrixXd* outFovchk);
			void ComptonNECCal(std::vector<ListModeData>& lmDataEff, int accumN, Eigen::MatrixXd& EgateNEC, Eigen::MatrixXd* outSignalmatrix, Eigen::MatrixXd* outBkgmatrix);
			void CodedNECCal(std::vector<ListModeData>& lmDataEff, int accumN, Eigen::MatrixXd& EgateNEC, Eigen::MatrixXd* outSignalmatrix, Eigen::MatrixXd* outBkgmatrix);
			double LinearFit(Eigen::MatrixXd& FitValue, Eigen::MatrixXd& EnergyFit, double Energy);

			double calEgate(double peakEnergy, Eigen::MatrixXd* outEgate);

			/*
				open3d::geometry::PointCloud Reconstruct3dPostProcessing(std::vector<ListModeData>& ComptonlmDataEff, std::vector<ListModeData>& CodedlmDataEff, Eigen::MatrixXd& systemmatrix, open3d::geometry::PointCloud& pc, HUREL::Compton::ReconPointCloud* outReconPC);
				*/
		};
	}
}


