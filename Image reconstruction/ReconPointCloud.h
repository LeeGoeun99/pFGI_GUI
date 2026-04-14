#pragma once

#include <cmath>
#include <open3d/geometry/PointCloud.h>
#include <open3d/utility/Helper.h>
#include <unordered_map>

#include "ListModeData.h"
#include "RadiationImage.h"

//231020 sbkwon : acos() LUT
#define CAL_ACOS_PI_Size			2000001
#define CAL_ACOS_PI_Increase		0.000001
#define CAL_ACOS_PI_MUL				1000000
#define CAL_180_DEV_PI				57.295779513082320876798154814105		// 180 / Math.PI => 180/PI 

namespace HUREL
{

	namespace Compton
	{

		struct RGBA {
			double R;
			double G;
			double B;
			double A;
		} typedef RGBA_t;

		class RadiationImage;

		class ReconPointCloud : public open3d::geometry::PointCloud
		{
		private:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		public:

			/// ReconValue;		
			ReconPointCloud() : PointCloud() {};
			ReconPointCloud(open3d::geometry::PointCloud& pc);
			std::vector<double> reconValues_;
			double maxReoconValue = 0;

			void imspaceLim(open3d::geometry::PointCloud& totalPC, int azFOV, int polFOV, Eigen::Matrix4d transMatrix, open3d::geometry::PointCloud* outtransFOVPC, open3d::geometry::PointCloud* outtransPC, Eigen::MatrixXd* outFOVchk);//231025-1 sbkwon

			void CalculateReconPoint(ListModeData lmData, double(*calcFunc)(ListModeData&, Eigen::Vector3d&));
			void CalculateReconPointCoded(RadiationImage& lmImage);
			void CalculateReconPointCompton(RadiationImage& lmImage);
			void CalculateReconPointHybrid(RadiationImage& lmImage);

			//231020 sbkwon : acos LUT
			static double m_dAcosPiCalLUT[CAL_ACOS_PI_Size];
			static void CalAcos();
			static double GetCalAcos(double radian);

			//240227 lge
			static int seqrecon(double TotalEnergy, double ScatterEnergy, double AbsorberEnergy);
			// seqrecon2 with FOM calculation based on KN cross-section and photoelectric absorption probability
			static int seqrecon2(double TotalEnergy, double ScatterEnergy, double AbsorberEnergy);

			static double SimpleComptonBackprojection(ListModeData& lmData, Eigen::Vector3d& imgPoint);

			static double SimpleComptonBackprojectionUntransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint);
			static double SimpleComptonBackprojectionUntransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint, double* outComptonScatterAngle, double* outSigmacomptonScatteringAngle, Eigen::Vector3d* outScatterToAbsorberVector);
			static double SimpleComptonBackprojection(ListModeData& lmData, Eigen::Vector3d& imgPoint, double FOVchk);	//231020 sbkwon : �߰�
			static double SimpleComptonBackprojectionTransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint, double FOVchk);	//231106-2 sbkwon
			static double SqComptonBackprojection(ListModeData& lmData, Eigen::Vector3d& imgPoint);
			static double SqComptonBackprojectionTransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint);

			static double SimpleComptonBackprojectionSphere(ListModeData& lmData, Eigen::Vector3d& imgPoint);
			static double SqComptonBackprojectionSphere(ListModeData& lmData, Eigen::Vector3d& imgPoint);

			static RGBA_t ColorScaleJet(double v, double vmin, double vmax);

			std::shared_ptr<ReconPointCloud> VoxelDownSample(double voxel_size) const;
			std::tuple<double, double, double> cartesianToSpherical(double x, double y, double z);	//231025-1 sbkwon

			void ReconComptonMLEMIteration(int iterationtotalN, Eigen::MatrixXd SystemMatrix);
			void ReconCodedMLEMIteration(int iterationtotalN, Eigen::MatrixXd SystemMatrix);
			void ReconHybridMLEMIteration(int iterationtotalN, Eigen::MatrixXd CCSystemMatrix, Eigen::MatrixXd CASystemMatrix, int weight);

			void CalculateComptonMLEMSystemMatrix(std::vector<ListModeData> lmData, open3d::geometry::PointCloud& imagespace, Eigen::MatrixXd* SystemMatrix, double(*calcFunc)(ListModeData, Eigen::Vector3d));
			void CalculateCodedMLEMSystemMatrix(std::vector<ListModeData>& lmData, Eigen::MatrixXd& Geant4SystemMatrix, open3d::geometry::PointCloud& reconPCFOVlimtranposed, Eigen::Matrix4d& transformation, Eigen::MatrixXd* outposCASystemMatrix);
			double maxReconValue = 0;

			static double SimpleComptonBackprojection(ListModeData lmData, Eigen::Vector3d imgPoint);
			static double SimpleComptonMLEM(ListModeData lmData, Eigen::Vector3d imgPoint);
			static double SimpleComptonMLEM2D(ListModeData lmData, Eigen::Vector3d imgPoint);
		};
	}
}
