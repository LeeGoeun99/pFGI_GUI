#include "ReconPointCloud.h"

#include "open3d/geometry/Image.h"
#include <complex>
#include <cmath>

// helper classes for VoxelDownSample and VoxelDownSampleAndTrace
namespace {
	class AccumulatedReconPoint {
	public:

		AccumulatedReconPoint()
			: num_of_points_(0),
			point_(0.0, 0.0, 0.0),
			normal_(0.0, 0.0, 0.0),
			color_(0.0, 0.0, 0.0),
			reconValue_(0) {}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			void AddPoint(const HUREL::Compton::ReconPointCloud& cloud, int index) {
			point_ += cloud.points_[index];
			if (cloud.HasNormals()) {
				if (!std::isnan(cloud.normals_[index](0)) &&
					!std::isnan(cloud.normals_[index](1)) &&
					!std::isnan(cloud.normals_[index](2))) {
					normal_ += cloud.normals_[index];
				}
			}
			if (cloud.HasColors()) {
				color_ += cloud.colors_[index];
			}
			num_of_points_++;
			reconValue_ += cloud.reconValues_[index];
		}

		Eigen::Vector3d GetAveragePoint() const {
			return point_ / double(num_of_points_);
		}

		Eigen::Vector3d GetAverageNormal() const {
			// Call NormalizeNormals() afterwards if necessary
			return normal_ / double(num_of_points_);
		}

		Eigen::Vector3d GetAverageColor() const {
			return color_ / double(num_of_points_);
		}

		double GetReconValue() const {
			return reconValue_ / double(num_of_points_);
		}

	public:
		int num_of_points_;
		Eigen::Vector3d point_;
		Eigen::Vector3d normal_;
		Eigen::Vector3d color_;
		double reconValue_;
	};
};

//231025-1 sbkwon : �߰�
void HUREL::Compton::ReconPointCloud::imspaceLim(open3d::geometry::PointCloud& totalPC, int azFOV, int polFOV, Eigen::Matrix4d transMatrix, open3d::geometry::PointCloud* outtransFOVPC, open3d::geometry::PointCloud* outtransPC, Eigen::MatrixXd* outFOVchk)
{
	open3d::geometry::PointCloud& totalPCtrans = *outtransPC;
	totalPCtrans = totalPC;
	open3d::geometry::PointCloud& recontransPCFOVlim = *outtransFOVPC;
	Eigen::MatrixXd& fovchk = *outFOVchk;
	Eigen::Matrix4d t265toLACCPosTranslate;
	Eigen::Matrix4d t265toLACCPosTransformInv;
	t265toLACCPosTranslate << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;
	t265toLACCPosTransformInv = t265toLACCPosTranslate.inverse();
	totalPCtrans.Transform(t265toLACCPosTransformInv);

	for (size_t size = 0; size < totalPCtrans.points_.size(); size++)
	{
		Eigen::Vector3d& point_trans = totalPCtrans.points_[size];
		//std::cout << "point_trans is: " << point_trans <<std::endl;
		Eigen::Vector3d& color_trans = totalPCtrans.colors_[size];
		//auto [rho, az, pol] = cartesianToSpherical(point_trans.z(), point_trans.x(), point_trans.y());
		std::tuple<double, double, double> degree = cartesianToSpherical(point_trans.z(), point_trans.x(), point_trans.y());
		double rho = std::get<0>(degree);
		double az = std::get<1>(degree);	//231106-1 sbkwon
		double pol = std::get<2>(degree);
		//std::cout << "az is: " << az << "pol is: " << pol << std::endl;
		if (abs(az) <= azFOV && abs(pol) <= polFOV)
		{
			fovchk(0, size) = 1;
			recontransPCFOVlim.points_.push_back(point_trans);
			recontransPCFOVlim.colors_.push_back(color_trans);
		}
		else
		{
			fovchk(0, size) = 0;
		}
	}
}

//231025-1 sbkwon : �߰�
std::tuple<double, double, double> HUREL::Compton::ReconPointCloud::cartesianToSpherical(double x, double y, double z)
{
	double rho = std::sqrt(x * x + y * y + z * z);
	double az = std::atan2(y, x); //rad
	//double az = std::atan2(x, y);
	// double pol = std::acos(z/rho); //rad

	double pol = std::atan2(z, std::sqrt(x * x + y * y));

	return std::make_tuple(rho, RAD2DEG(az), RAD2DEG(pol));
}

//231020 sbkwon : �������� �ʱ�ȭ
double HUREL::Compton::ReconPointCloud::m_dAcosPiCalLUT[CAL_ACOS_PI_Size] = { 0.0, };

//231020 sbkwon : LUT Init
void HUREL::Compton::ReconPointCloud::CalAcos()
{
	static bool bInitacos = false;

	if (!bInitacos)
	{
		double a = -1.0;

		for (int i = 0; i < CAL_ACOS_PI_Size; i++)
		{
			m_dAcosPiCalLUT[i] = acos(a) /** DEV_PI_180*/; //radian	//230411

			if (i == (CAL_ACOS_PI_Size - 2))
				a = 1;
			else
				a += CAL_ACOS_PI_Increase;
		}
		bInitacos = true;
	}
}

//231020 sbkwon : acos(radian)
double HUREL::Compton::ReconPointCloud::GetCalAcos(double radian)
{
	if (radian < -1 || radian > 1)
		return FALSE;

	int nIndex = (int)((radian + 1) * CAL_ACOS_PI_MUL);

	return m_dAcosPiCalLUT[nIndex];
}

//240227 lge
int HUREL::Compton::ReconPointCloud::seqrecon(double TotalEnergy, double ScatterEnergy, double AbsorberEnergy)
{
	if (TotalEnergy <= 0.4)
	{
		if (ScatterEnergy <= AbsorberEnergy)
			if (AbsorberEnergy < 0.1)
				return 0;
			else
				return 1;
		else
			if (ScatterEnergy < 0.1)
				return 0;
			else
				return 2;
	}
	else
	{
		double ComptonEdge = TotalEnergy * (1 - TotalEnergy / (1 + (2 * TotalEnergy / 0.511)));
		if (ScatterEnergy >= ComptonEdge)
			if (ScatterEnergy < 0.1)
				return 0;
			else
				return 2;
		if (AbsorberEnergy >= ComptonEdge)
		{
			if (AbsorberEnergy < 0.1)
				return 0;
			else
				return 1;
		}

		if (ScatterEnergy < ComptonEdge && AbsorberEnergy < ComptonEdge)
		{
			if (ScatterEnergy > AbsorberEnergy)
				return 1;
			else
				return 2;
		}
	}
}

// seqrecon2 with FOM calculation based on KN cross-section and photoelectric absorption probability
// Helper function for photoelectric absorption probability lookup table
namespace {
	// Photoelectric absorption probability dataset: [Energy(MeV), photab, phottotal]
	const double photoelectricData[][3] = {
		{4.000E-02, 1.822E+01, 1.833E+01},
		{5.000E-02, 1.005E+01, 1.017E+01},
		{6.000E-02, 6.103E+00, 6.220E+00},
		{8.000E-02, 2.743E+00, 2.859E+00},
		{8.553E-02, 2.273E+00, 2.389E+00},
		{1.000E-01, 1.476E+00, 1.591E+00},
		{1.500E-01, 4.644E-01, 5.715E-01},
		{2.000E-01, 2.044E-01, 3.044E-01},
		{3.000E-01, 6.568E-02, 1.542E-01},
		{4.000E-01, 3.029E-02, 1.104E-01},
		{5.000E-01, 1.709E-02, 9.058E-02},
		{6.000E-01, 1.095E-02, 7.915E-02},
		{8.000E-01, 5.676E-03, 6.578E-02},
		{1.000E+00, 3.546E-03, 5.766E-02},
		{1.022E+00, 3.376E-03, 5.691E-02},
		{1.250E+00, 2.276E-03, 5.088E-02},
		{1.500E+00, 1.629E-03, 4.645E-02},
		{2.000E+00, 9.915E-04, 4.120E-02}
	};
	const int photoelectricDataSize = sizeof(photoelectricData) / sizeof(photoelectricData[0]);
	const double minEnergy = photoelectricData[0][0];
	const double maxEnergy = photoelectricData[photoelectricDataSize - 1][0];

	// Optimized binary search for linear interpolation (O(log n) instead of O(n))
	inline std::pair<double, double> GetPhotoelectricProb(double energy)
	{
		// Fast path for out-of-range values
		if (energy <= minEnergy)
		{
			return std::make_pair(photoelectricData[0][1], photoelectricData[0][2]);
		}
		if (energy >= maxEnergy)
		{
			return std::make_pair(photoelectricData[photoelectricDataSize - 1][1], photoelectricData[photoelectricDataSize - 1][2]);
		}

		// Binary search for the interval
		int left = 0;
		int right = photoelectricDataSize - 1;
		int mid = 0;

		while (left < right - 1)
		{
			mid = (left + right) >> 1; // Bit shift for division by 2
			if (energy < photoelectricData[mid][0])
			{
				right = mid;
			}
			else
			{
				left = mid;
			}
		}

		// Linear interpolation
		const double* data0 = photoelectricData[left];
		const double* data1 = photoelectricData[right];
		double x0 = data0[0];
		double x1 = data1[0];
		double inv_dx = 1.0 / (x1 - x0);
		double t = (energy - x0) * inv_dx;
		
		double photab = data0[1] + t * (data1[1] - data0[1]);
		double phottotal = data0[2] + t * (data1[2] - data0[2]);

		return std::make_pair(photab, phottotal);
	}
}

int HUREL::Compton::ReconPointCloud::seqrecon2(double TotalEnergy, double ScatterEnergy, double AbsorberEnergy)
{
	// Early return for invalid energies
	if (ScatterEnergy < 0.1 && AbsorberEnergy < 0.1)
		return 0;

	double E0 = TotalEnergy;
	double E1 = ScatterEnergy;
	double E2 = AbsorberEnergy;

	// Check if TotalEnergy > 0.4 (case1chk condition from MATLAB)
	bool case1chk = E0 > 0.4;

	// Calculate Compton edge (optimized: precompute denominator)
	double denom = 1.0 + (2.0 * E0 * 1.956947162426614); // 1/0.511 ≈ 1.956947162426614
	double Ecompton = E0 * (1.0 - E0 / denom);

	// Case 1: E1 > Ecompton -> return 1 (no switch)
	// Only apply when TotalEnergy > 0.4
	if (case1chk && E1 > Ecompton)
	{
		return (E1 < 0.1) ? 0 : 1;
	}

	// Case 2: E2 > Ecompton -> return 2 (switch)
	// Only apply when TotalEnergy > 0.4
	if (case1chk && E2 > Ecompton)
	{
		return (E2 < 0.1) ? 0 : 2;
	}

	// Case 3: E1 < Ecompton && E2 < Ecompton
	if (E1 < Ecompton && E2 < Ecompton)
	{
		// For low energy, exclude events where energies are too close
		if (E0 < 0.5)
		{
			if (std::abs(E1 - E2) < 0.05)
				return 0;
		}

		// Precompute common values
		double E0_inv = 1.0 / E0;
		double E1E0 = E1 * E0_inv;
		double E2E0 = E2 * E0_inv;
		double E0E1 = E0 / E1;
		double E0E2 = E0 / E2;

		// Calculate FOM1 (case 1->2, no switch)
		double cos1 = 1.0 - 0.511 * E1 * E0_inv / E2;
		if (cos1 < -1.0 || cos1 > 1.0)
			return 0;

		// Use direct acos instead of complex (faster and cos is already validated)
		double theta1 = std::acos(cos1);
		double sin1 = std::sin(theta1);
		if (sin1 < 1e-10)
			return 0;

		double sin1_sq = sin1 * sin1;
		double E2E0_sq = E2E0 * E2E0; // Use multiplication instead of pow
		double FOM1_1 = E2E0_sq * (E2E0 + E0E2 - sin1_sq);
		double sin1_inv = 1.0 / sin1;
		double E2_sq_inv = 1.0 / (E2 * E2);
		double FOM1_4 = 2.0 * EIGEN_PI * 0.511 * sin1_inv * E2_sq_inv;

		// Get photoelectric absorption probability for E2
		auto prob2 = GetPhotoelectricProb(E2);
		double phottotal_E2 = prob2.second;
		double photab_E2 = prob2.first;
		// Precompute exponential terms
		double exp_term_E2 = std::exp(-phottotal_E2 * 4.5875); // 3.67 * 1.25
		double exp_term2_E2 = std::exp(-phottotal_E2 * 0.9175); // 3.67 * 0.25
		double phottotal_E2_inv = 1.0 / phottotal_E2;
		double p2 = (1.0 - exp_term_E2) * exp_term2_E2 * photab_E2 * phottotal_E2_inv;

		double FOM1 = FOM1_1 * FOM1_4 * p2;

		// Calculate FOM2 (case 2->1, switch)
		double cos2 = 1.0 - 0.511 * E2 * E0_inv / E1;
		if (cos2 < -1.0 || cos2 > 1.0)
			return 0;

		// Use direct acos instead of complex (faster and cos is already validated)
		double theta2 = std::acos(cos2);
		double sin2 = std::sin(theta2);
		if (sin2 < 1e-10)
			return 0;

		double sin2_sq = sin2 * sin2;
		double E1E0_sq = E1E0 * E1E0; // Use multiplication instead of pow
		double FOM2_1 = E1E0_sq * (E1E0 + E0E1 - sin2_sq);
		double sin2_inv = 1.0 / sin2;
		double E1_sq_inv = 1.0 / (E1 * E1);
		double FOM2_4 = 2.0 * EIGEN_PI * 0.511 * sin2_inv * E1_sq_inv;

		// Get photoelectric absorption probability for E1
		auto prob1 = GetPhotoelectricProb(E1);
		double phottotal_E1 = prob1.second;
		double photab_E1 = prob1.first;
		// Precompute exponential terms
		double exp_term_E1 = std::exp(-phottotal_E1 * 4.5875); // 3.67 * 1.25
		double exp_term2_E1 = std::exp(-phottotal_E1 * 0.9175); // 3.67 * 0.25
		double phottotal_E1_inv = 1.0 / phottotal_E1;
		double p1 = (1.0 - exp_term_E1) * exp_term2_E1 * photab_E1 * phottotal_E1_inv;

		double FOM2 = FOM2_1 * FOM2_4 * p1;

		// Compare FOM1 and FOM2
		return (FOM1 > FOM2) ? 1 : 2; // no switch : switch
	}

	return 0;
}

HUREL::Compton::ReconPointCloud::ReconPointCloud(open3d::geometry::PointCloud& pc) :
	open3d::geometry::PointCloud(pc)
{
	size_t size = pc.points_.size();
	reconValues_ = std::vector<double>(size);

}

void HUREL::Compton::ReconPointCloud::CalculateReconPoint(ListModeData lmData, double(*calcFunc)(ListModeData&, Eigen::Vector3d&))
{

	size_t size = points_.size();
	maxReoconValue = -DBL_MAX;
#pragma omp parallel for
	for (int i = 0; i < size; ++i)
	{
#pragma omp atomic
		reconValues_[i] += calcFunc(lmData, points_[i]);
		if (reconValues_[i] > maxReoconValue)
		{
			maxReoconValue = reconValues_[i];
		}
	}
}

void HUREL::Compton::ReconPointCloud::CalculateReconPointCoded(RadiationImage& lmImage)
{
	size_t size = points_.size();
	maxReoconValue = -DBL_MAX;
#pragma omp parallel for
	for (int i = 0; i < size; ++i)
	{
#pragma omp atomic
		reconValues_[i] += lmImage.OverlayValue(points_[i], eRadiationImagingMode::CODED);
		if (reconValues_[i] > maxReoconValue)
		{
			maxReoconValue = reconValues_[i];
		}
	}
}

void HUREL::Compton::ReconPointCloud::CalculateReconPointCompton(RadiationImage& lmImage)
{
	size_t size = points_.size();
	maxReoconValue = -DBL_MAX;
#pragma omp parallel for
	for (int i = 0; i < size; ++i)
	{
#pragma omp atomic
		reconValues_[i] += lmImage.OverlayValue(points_[i], eRadiationImagingMode::COMPTON);
		if (reconValues_[i] > maxReoconValue)
		{
			maxReoconValue = reconValues_[i];
		}
	}
}

void HUREL::Compton::ReconPointCloud::CalculateReconPointHybrid(RadiationImage& lmImage)
{
	size_t size = points_.size();
	maxReoconValue = -DBL_MAX;
#pragma omp parallel for
	for (int i = 0; i < size; ++i)
	{
#pragma omp atomic
		reconValues_[i] += lmImage.OverlayValue(points_[i], eRadiationImagingMode::HYBRID);
		if (reconValues_[i] > maxReoconValue)
		{
			maxReoconValue = reconValues_[i];
		}
	}
}

double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojection(ListModeData& lmData, Eigen::Vector3d& imgPoint)
{
	double ScatterEnergy = lmData.Scatter.InteractionEnergy;
	double AbsorberEnergy = lmData.Absorber.InteractionEnergy;
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}
	double comptonCal = 1 - 511 * lmData.Scatter.InteractionEnergy / lmData.Absorber.InteractionEnergy / (lmData.Scatter.InteractionEnergy + lmData.Absorber.InteractionEnergy);
	if (comptonCal >= 1 || comptonCal <= -1)
	{
		//return 0;
	}

	double comptonScatteringAngle = acos(comptonCal) / EIGEN_PI * 180;
	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.TransformedInteractionPoint.head<3>());
	Eigen::Vector3d scatterToAbsorberVector = (lmData.Scatter.TransformedInteractionPoint.head<3>() - lmData.Absorber.TransformedInteractionPoint.head<3>());
	effectToScatterVector.normalize();
	scatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(scatterToAbsorberVector);
	double effectedAngle = acos(positionDotPord) / EIGEN_PI * 180;
	double sigmacomptonScatteringAngle = 511 / sin(comptonScatteringAngle) * sqrt(1 / pow(AbsorberEnergy, 2)) - 1 / pow(TotalEnergy, 2) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2);
	double BP_sig_thres = 2;

	if (abs(effectedAngle - comptonScatteringAngle) < BP_sig_thres * sigmacomptonScatteringAngle)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojectionUntransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint)
{

	double ScatterEnergy = lmData.Scatter.InteractionEnergy;
	double AbsorberEnergy = lmData.Absorber.InteractionEnergy;
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}
	double comptonCal = 1 - 511 * lmData.Scatter.InteractionEnergy / lmData.Absorber.InteractionEnergy / (lmData.Scatter.InteractionEnergy + lmData.Absorber.InteractionEnergy);
	if (comptonCal >= 1 || comptonCal <= -1)
	{
		//return 0;
	}

	double comptonScatteringAngle = acos(comptonCal) / EIGEN_PI * 180;
	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.RelativeInteractionPoint.head<3>());
	Eigen::Vector3d scatterToAbsorberVector = (lmData.Scatter.RelativeInteractionPoint.head<3>() - lmData.Absorber.RelativeInteractionPoint.head<3>());
	effectToScatterVector.normalize();
	scatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(scatterToAbsorberVector);
	double effectedAngle = acos(positionDotPord) / EIGEN_PI * 180;
	double sigmacomptonScatteringAngle = 511 / sin(comptonScatteringAngle) * sqrt(1 / pow(AbsorberEnergy, 2)) - 1 / pow(TotalEnergy, 2) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2);
	double BP_sig_thres = 2;

	if (abs(effectedAngle - comptonScatteringAngle) < BP_sig_thres * sigmacomptonScatteringAngle)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojectionUntransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint, double* outComptonScatterAngle, double* outSigmacomptonScatteringAngle, Eigen::Vector3d* outScatterToAbsorberVector)
{
	double BP_sig_thres = 2.5;

	if (isnan(*outSigmacomptonScatteringAngle) && isnan(*outComptonScatterAngle))
	{
		double ScatterEnergy = lmData.Scatter.InteractionEnergy;
		double AbsorberEnergy = lmData.Absorber.InteractionEnergy;
		double TotalEnergy = ScatterEnergy + AbsorberEnergy;
		if (lmData.Type != eInterationType::COMPTON)
		{
			return 0;
		}
		double comptonCal = 1 - 511 * ScatterEnergy / AbsorberEnergy / TotalEnergy;//231013 sbkwon
		//double comptonCal = 1 - 511 * lmData.Scatter.InteractionEnergy / lmData.Absorber.InteractionEnergy / (lmData.Scatter.InteractionEnergy + lmData.Absorber.InteractionEnergy);//����
		if (comptonCal >= 1 || comptonCal <= -1)
		{
			//return 0;
		}

		*outComptonScatterAngle = acos(comptonCal) / EIGEN_PI * 180;
		Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.RelativeInteractionPoint.head<3>());
		*outScatterToAbsorberVector = (lmData.Scatter.RelativeInteractionPoint.head<3>() - lmData.Absorber.RelativeInteractionPoint.head<3>());
		effectToScatterVector.normalize();
		outScatterToAbsorberVector->normalize();
		double positionDotPord = effectToScatterVector.dot(*outScatterToAbsorberVector);
		double effectedAngle = acos(positionDotPord) / EIGEN_PI * 180;
		*outSigmacomptonScatteringAngle = 511 / sin(*outComptonScatterAngle) * sqrt(1 / pow(AbsorberEnergy, 2)) - 1 / pow(TotalEnergy, 2) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2);

		if (abs(effectedAngle - *outComptonScatterAngle) < BP_sig_thres * *outSigmacomptonScatteringAngle)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.RelativeInteractionPoint.head<3>());
		effectToScatterVector.normalize();
		double positionDotPord = effectToScatterVector.dot(*outScatterToAbsorberVector);
		double effectedAngle = acos(positionDotPord) / EIGEN_PI * 180;
		if (abs(effectedAngle - *outComptonScatterAngle) < BP_sig_thres * *outSigmacomptonScatteringAngle)
		{
			return 1;
		}
		else
		{
			return 0;
		}

	}

}

//231020 sbkwon : �߰�
double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojection(ListModeData& lmData, Eigen::Vector3d& imgPoint, double FOVchk)
{
	double BP_sig_thres = 2;
	double ComptonScatterAngle;
	Eigen::Vector3d ScatterToAbsorberVector;
	double SigmacomptonScatteringAngle;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}

	CalAcos();

	if (FOVchk != 0)
	{
		double ScatterEnergy = (lmData.Scatter.InteractionEnergy) * 0.001;
		double AbsorberEnergy = (lmData.Absorber.InteractionEnergy) * 0.001;
		double TotalEnergy = ScatterEnergy + AbsorberEnergy;

		double value = 1 - 0.511 * ScatterEnergy / AbsorberEnergy / TotalEnergy;
		if (value >= 1 || value <= -1)
		{
			//return 0;
		}

		double ComptonScatterAngle = GetCalAcos(value);//acos(value); //rad
		Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.RelativeInteractionPoint.head<3>());
		ScatterToAbsorberVector = (lmData.Scatter.RelativeInteractionPoint.head<3>() - lmData.Absorber.RelativeInteractionPoint.head<3>());
		effectToScatterVector.normalize();
		ScatterToAbsorberVector.normalize();
		double positionDotPord = effectToScatterVector.dot(ScatterToAbsorberVector);
		double effectedAngle = GetCalAcos(positionDotPord);//acos(positionDotPord); //rad 
		SigmacomptonScatteringAngle = 0.511 / sin(ComptonScatterAngle) * sqrt((1 / pow(AbsorberEnergy, 2) - 1 / pow(TotalEnergy, 2)) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2));

		//if ((effectedAngle * CAL_180_DEV_PI) > 80)//(effectedAngle / EIGEN_PI * 180)
		//{
		//	return 0;
		//}
		//else
		//{
			if (abs(effectedAngle - ComptonScatterAngle) < BP_sig_thres * SigmacomptonScatteringAngle)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		//}
	}
	else
	{
		return 0;
	}
}

//240319 lge: �߰�
double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojectionSphere(ListModeData& lmData, Eigen::Vector3d& imgPoint)
{
	double BP_sig_thres = 3;
	double ComptonScatterAngle;
	Eigen::Vector3d ScatterToAbsorberVector;
	double SigmacomptonScatteringAngle;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}

	CalAcos();


	double ScatterEnergy = (lmData.Scatter.InteractionEnergy) * 0.001;
	double AbsorberEnergy = (lmData.Absorber.InteractionEnergy) * 0.001;
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;

	double value = 1 - 0.511 * ScatterEnergy / AbsorberEnergy / TotalEnergy;
	if (value >= 1 || value <= -1)
	{
		//return 0;
	}

	ComptonScatterAngle = GetCalAcos(value);//acos(value); //rad
	Eigen::Vector4d ScatterChangedPoint = Eigen::Vector4d(nan(""), nan(""), nan(""), 1);
	Eigen::Vector4d AbsorberChangedPoint = Eigen::Vector4d(nan(""), nan(""), nan(""), 1);
	ScatterChangedPoint(0) = lmData.Scatter.RelativeInteractionPoint(2);
	ScatterChangedPoint(1) = lmData.Scatter.RelativeInteractionPoint(0);
	ScatterChangedPoint(2) = lmData.Scatter.RelativeInteractionPoint(1);
	AbsorberChangedPoint(0) = lmData.Absorber.RelativeInteractionPoint(2);
	AbsorberChangedPoint(1) = lmData.Absorber.RelativeInteractionPoint(0);
	AbsorberChangedPoint(2) = lmData.Absorber.RelativeInteractionPoint(1);

	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - ScatterChangedPoint.head<3>());
	ScatterToAbsorberVector = (ScatterChangedPoint.head<3>() - AbsorberChangedPoint.head<3>());
	effectToScatterVector.normalize();
	ScatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(ScatterToAbsorberVector);
	double effectedAngle = GetCalAcos(positionDotPord);//acos(positionDotPord); //rad 
	SigmacomptonScatteringAngle = 0.511 / sin(ComptonScatterAngle) * sqrt((1 / pow(AbsorberEnergy, 2) - 1 / pow(TotalEnergy, 2)) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2));


	if (abs(effectedAngle - ComptonScatterAngle) < BP_sig_thres * SigmacomptonScatteringAngle)
	{
		return (1 / SigmacomptonScatteringAngle * exp(-0.5 * pow((pow((effectedAngle - ComptonScatterAngle), 2) / SigmacomptonScatteringAngle), 2)));
	}
	else
	{
		return 0;
	}


}

double HUREL::Compton::ReconPointCloud::SqComptonBackprojectionSphere(ListModeData& lmData, Eigen::Vector3d& imgPoint)
{
	double BP_sig_thres = 2;
	double ComptonScatterAngle;
	Eigen::Vector3d ScatterToAbsorberVector;
	double SigmacomptonScatteringAngle;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}

	double ScatterEnergy = (lmData.Scatter.InteractionEnergy) * 0.001;
	double AbsorberEnergy = (lmData.Absorber.InteractionEnergy) * 0.001;
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;


	//int casenumber = seqrecon(TotalEnergy, ScatterEnergy, AbsorberEnergy);
	int casenumber = seqrecon2(TotalEnergy, ScatterEnergy, AbsorberEnergy);
	if (casenumber == 0)
		return 0;

	Eigen::Vector4d ScatterChangedPoint = Eigen::Vector4d(nan(""), nan(""), nan(""), 1);
	Eigen::Vector4d AbsorberChangedPoint = Eigen::Vector4d(nan(""), nan(""), nan(""), 1);
	ScatterChangedPoint(0) = lmData.Scatter.RelativeInteractionPoint(2);
	ScatterChangedPoint(1) = lmData.Scatter.RelativeInteractionPoint(0);
	ScatterChangedPoint(2) = lmData.Scatter.RelativeInteractionPoint(1);
	AbsorberChangedPoint(0) = lmData.Absorber.RelativeInteractionPoint(2);
	AbsorberChangedPoint(1) = lmData.Absorber.RelativeInteractionPoint(0);
	AbsorberChangedPoint(2) = lmData.Absorber.RelativeInteractionPoint(1);

	Eigen:Vector3d SQScatterRelativePoint;
	Eigen::Vector3d SQAbsorberRelativePoint;
	double SQScatterEnergy;
	double SQAbsorberEnergy;

	if (casenumber == 2) // 1->2
	{
		SQScatterEnergy = ScatterEnergy;
		SQAbsorberEnergy = AbsorberEnergy;
		SQScatterRelativePoint = ScatterChangedPoint.head<3>();
		SQAbsorberRelativePoint = AbsorberChangedPoint.head<3>();
	}
	else if (casenumber == 1) //2->1
	{
		SQScatterEnergy = AbsorberEnergy;
		SQAbsorberEnergy = ScatterEnergy;
		SQScatterRelativePoint = AbsorberChangedPoint.head<3>();
		SQAbsorberRelativePoint = ScatterChangedPoint.head<3>();
	}
	else
		return 0;

	if (SQScatterEnergy < 0.01)
		return 0;

	if (SQAbsorberEnergy < 0.1)
		return 0;

	CalAcos();


	double value_sq = 1 - 0.511 * SQScatterEnergy / SQAbsorberEnergy / TotalEnergy;

	if (value_sq >= 1 || value_sq <= -1)
	{
		return 0;
	}

	ComptonScatterAngle = GetCalAcos(value_sq);//acos(value); //rad

	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - SQScatterRelativePoint);
	ScatterToAbsorberVector = (SQScatterRelativePoint - SQAbsorberRelativePoint);
	effectToScatterVector.normalize();
	ScatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(ScatterToAbsorberVector);
	double effectedAngle = GetCalAcos(positionDotPord);//acos(positionDotPord); //rad 
	SigmacomptonScatteringAngle = 0.511 / sin(ComptonScatterAngle) * sqrt((1 / pow(SQAbsorberEnergy, 2) - 1 / pow(TotalEnergy, 2)) * pow(0.08 / 2.35 * sqrt(SQAbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(SQScatterEnergy), 2));

	if (abs(effectedAngle - ComptonScatterAngle) < BP_sig_thres * SigmacomptonScatteringAngle)
	{
		return (1 / SigmacomptonScatteringAngle * exp(-0.5 * pow((pow((effectedAngle - ComptonScatterAngle), 2) / SigmacomptonScatteringAngle), 2)));

	}
	else
	{
		return 0;
	}
}

//231106-2 sbkwon
double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojectionTransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint, double FOVchk)
{
	double BP_sig_thres = 2;
	double ComptonScatterAngle;
	Eigen::Vector3d ScatterToAbsorberVector;
	double SigmacomptonScatteringAngle;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}

	CalAcos();

	if (FOVchk != 0)
	{
		double ScatterEnergy = (lmData.Scatter.InteractionEnergy) * 0.001;
		double AbsorberEnergy = (lmData.Absorber.InteractionEnergy) * 0.001;
		double TotalEnergy = ScatterEnergy + AbsorberEnergy;

		double value = 1 - 0.511 * ScatterEnergy / AbsorberEnergy / TotalEnergy;
		//if (value >= 1 || value <= -1)
		//{
		//	return 0;
		//}

		double ComptonScatterAngle = GetCalAcos(value);//acos(value); //rad
		Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.TransformedInteractionPoint.head<3>());
		ScatterToAbsorberVector = (lmData.Scatter.TransformedInteractionPoint.head<3>() - lmData.Absorber.TransformedInteractionPoint.head<3>());
		effectToScatterVector.normalize();
		ScatterToAbsorberVector.normalize();
		double positionDotPord = effectToScatterVector.dot(ScatterToAbsorberVector);
		double effectedAngle = GetCalAcos(positionDotPord);//acos(positionDotPord); //rad 
		SigmacomptonScatteringAngle = 0.511 / sin(ComptonScatterAngle) * sqrt((1 / pow(AbsorberEnergy, 2) - 1 / pow(TotalEnergy, 2)) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2));

		//if ((effectedAngle * CAL_180_DEV_PI) > 65)//(effectedAngle / EIGEN_PI * 180)
		//{
		//	return 0;
		//}
		//else
		//{
			if (abs(effectedAngle - ComptonScatterAngle) < BP_sig_thres * SigmacomptonScatteringAngle)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		//}
	}
	else
	{
		return 0;
	}
}

//20240227 lge
double HUREL::Compton::ReconPointCloud::SqComptonBackprojection(ListModeData& lmData, Eigen::Vector3d& imgPoint)
{
	double BP_sig_thres = 2;
	double ComptonScatterAngle;
	Eigen::Vector3d ScatterToAbsorberVector;
	double SigmacomptonScatteringAngle;

	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}

	double ScatterEnergy = (lmData.Scatter.InteractionEnergy) * 0.001; //MeV
	double AbsorberEnergy = (lmData.Absorber.InteractionEnergy) * 0.001; //MeV
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;
	int casenumber = seqrecon(TotalEnergy, ScatterEnergy, AbsorberEnergy);
	if (casenumber == 0)
		return 0;

	Eigen::Vector3d ScatterRelativePoint = lmData.Scatter.RelativeInteractionPoint.head<3>();
	Eigen::Vector3d AbsorberRelativePoint = lmData.Absorber.RelativeInteractionPoint.head<3>();
	Eigen:Vector3d SQScatterRelativePoint;
	Eigen::Vector3d SQAbsorberRelativePoint;
	double SQScatterEnergy;
	double SQAbsorberEnergy;

	if (casenumber == 1) // 1->2
	{
		SQScatterEnergy = ScatterEnergy;
		SQAbsorberEnergy = AbsorberEnergy;
		SQScatterRelativePoint = ScatterRelativePoint;
		SQAbsorberRelativePoint = AbsorberRelativePoint;
	}
	else if (casenumber == 2) //2->1
	{
		SQScatterEnergy = AbsorberEnergy;
		SQAbsorberEnergy = ScatterEnergy;
		SQScatterRelativePoint = AbsorberRelativePoint;
		SQAbsorberRelativePoint = ScatterRelativePoint;
	}
	else
		return 0;


	if (SQScatterEnergy < 0.01)
		return 0;

	if (SQAbsorberEnergy < 0.1)
		return 0;
	
	CalAcos();

	double value = 1 - 0.511 * SQScatterEnergy / SQAbsorberEnergy / TotalEnergy;

	if (value >= 1 || value <= -1)
	{
		return 0;
	}

	ComptonScatterAngle = GetCalAcos(value);//acos(value); //rad

	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - SQScatterRelativePoint);
	ScatterToAbsorberVector = (SQScatterRelativePoint - SQAbsorberRelativePoint);
	effectToScatterVector.normalize();
	ScatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(ScatterToAbsorberVector);
	double effectedAngle = GetCalAcos(positionDotPord);//acos(positionDotPord); //rad 
	SigmacomptonScatteringAngle = 0.511 / sin(ComptonScatterAngle) * sqrt((1 / pow(SQAbsorberEnergy, 2) - 1 / pow(TotalEnergy, 2)) * pow(0.08 / 2.35 * sqrt(SQAbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(SQScatterEnergy), 2));
	//if ((effectedAngle * CAL_180_DEV_PI) > 65)//(effectedAngle / EIGEN_PI * 180)
	//{
	//	return 0;
	//}
	//else
	//{
	if (abs(effectedAngle - ComptonScatterAngle) < BP_sig_thres * SigmacomptonScatteringAngle)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//20240227 lge
double HUREL::Compton::ReconPointCloud::SqComptonBackprojectionTransformed(ListModeData& lmData, Eigen::Vector3d& imgPoint)
{

	double BP_sig_thres = 2;
	double ComptonScatterAngle;
	Eigen::Vector3d ScatterToAbsorberVector;
	double SigmacomptonScatteringAngle;
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}

	double ScatterEnergy = (lmData.Scatter.InteractionEnergy) * 0.001; //MeV
	double AbsorberEnergy = (lmData.Absorber.InteractionEnergy) * 0.001; //MeV
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;
	int casenumber = seqrecon(TotalEnergy, ScatterEnergy, AbsorberEnergy);
	if (casenumber == 0)
		return 0;

	Eigen::Vector3d ScatterRelativePoint = lmData.Scatter.TransformedInteractionPoint.head<3>();
	Eigen::Vector3d AbsorberRelativePoint = lmData.Absorber.TransformedInteractionPoint.head<3>();
	Eigen:Vector3d SQScatterRelativePoint;
	Eigen::Vector3d SQAbsorberRelativePoint;
	double SQScatterEnergy;
	double SQAbsorberEnergy;

	if (casenumber == 1) // 1->2
	{
		SQScatterEnergy = ScatterEnergy;
		SQAbsorberEnergy = AbsorberEnergy;
		SQScatterRelativePoint = ScatterRelativePoint;
		SQAbsorberRelativePoint = AbsorberRelativePoint;
	}
	else if (casenumber == 2) //2->1
	{
		SQScatterEnergy = AbsorberEnergy;
		SQAbsorberEnergy = ScatterEnergy;
		SQScatterRelativePoint = AbsorberRelativePoint;
		SQAbsorberRelativePoint = ScatterRelativePoint;
	}
	else
		return 0;


	if (SQScatterEnergy < 0.01)
		return 0;

	if (SQAbsorberEnergy < 0.1)
		return 0;

	CalAcos();


	double value = 1 - 0.511 * SQScatterEnergy / SQAbsorberEnergy / TotalEnergy;

	if (value >= 1 || value <= -1)
	{
		return 0;
	}

	ComptonScatterAngle = GetCalAcos(value);//acos(value); //rad

	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - SQScatterRelativePoint);
	ScatterToAbsorberVector = (SQScatterRelativePoint - SQAbsorberRelativePoint);
	effectToScatterVector.normalize();
	ScatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(ScatterToAbsorberVector);
	double effectedAngle = GetCalAcos(positionDotPord);//acos(positionDotPord); //rad 
	SigmacomptonScatteringAngle = 0.511 / sin(ComptonScatterAngle) * sqrt((1 / pow(SQAbsorberEnergy, 2) - 1 / pow(TotalEnergy, 2)) * pow(0.08 / 2.35 * sqrt(SQAbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(SQScatterEnergy), 2));
	//if ((effectedAngle * CAL_180_DEV_PI) > 65)//(effectedAngle / EIGEN_PI * 180)
	//{
	//	return 0;
	//}
	//else
	//{
	if (abs(effectedAngle - ComptonScatterAngle) < BP_sig_thres * SigmacomptonScatteringAngle)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

HUREL::Compton::RGBA_t HUREL::Compton::ReconPointCloud::ColorScaleJet(double v, double vmin, double vmax)
{
	double dv;

	if (v < vmin)
	{
		v = vmin;
		RGBA_t rgba = { 0, 0, 0, 0.0001 };
		return rgba;
	}
	if (v > vmax)
	{
		v = vmax;
	}
	dv = vmax - vmin;
	if (dv == 0)
	{
		RGBA_t rgba = { 0, 0, 0, 0.0001 };
		return rgba;
	}
	double r = 1.0f, g = 1.0f, b = 1.0f;
	if (v < (vmin + 0.25 * dv))
	{
		r = 0;
		g = 4 * (v - vmin) / dv;
	}
	else if (v < (vmin + 0.5 * dv))
	{
		r = 0;
		b = 1 + 4 * (vmin + 0.25f * dv - v) / dv;
	}
	else if (v < (vmin + 0.75 * dv))
	{
		r = 4 * (v - vmin - 0.5f * dv) / dv;
		b = 0;
	}
	else
	{
		g = 1 + 4 * (vmin + 0.75f * dv - v) / dv;
		b = 0;
	}
	double a = 0.5;
	RGBA_t rgba_out = { r, g, b, a };
	return rgba_out;
}

std::shared_ptr<HUREL::Compton::ReconPointCloud> HUREL::Compton::ReconPointCloud::VoxelDownSample(double voxel_size) const
{
	auto output = std::make_shared<HUREL::Compton::ReconPointCloud>();
	if (voxel_size <= 0.0) {
		printf("[VoxelDownSample] voxel_size <= 0.");
	}
	Eigen::Vector3d voxel_size3 =
		Eigen::Vector3d(voxel_size, voxel_size, voxel_size);
	Eigen::Vector3d voxel_min_bound = GetMinBound() - voxel_size3 * 0.5;
	Eigen::Vector3d voxel_max_bound = GetMaxBound() + voxel_size3 * 0.5;
	if (voxel_size * std::numeric_limits<int>::max() <
		(voxel_max_bound - voxel_min_bound).maxCoeff()) {
		printf("[VoxelDownSample] voxel_size is too small.");
	}
	std::unordered_map<Eigen::Vector3i, AccumulatedReconPoint,
		open3d::utility::hash_eigen<Eigen::Vector3i>>
		voxelindex_to_accpoint;

	Eigen::Vector3d ref_coord;
	Eigen::Vector3i voxel_index;
	for (int i = 0; i < (int)points_.size(); i++) {
		ref_coord = (points_[i] - voxel_min_bound) / voxel_size;
		voxel_index << int(floor(ref_coord(0))), int(floor(ref_coord(1))),
			int(floor(ref_coord(2)));
		voxelindex_to_accpoint[voxel_index].AddPoint(*this, i);
	}
	bool has_normals = HasNormals();
	bool has_colors = HasColors();
	for (const auto& accpoint : voxelindex_to_accpoint) {
		output->points_.push_back(accpoint.second.GetAveragePoint());
		if (has_normals) {
			output->normals_.push_back(accpoint.second.GetAverageNormal());
		}
		if (has_colors) {
			output->colors_.push_back(accpoint.second.GetAverageColor());
		}
		output->reconValues_.push_back(accpoint.second.GetReconValue());
	}
	printf(
		"Pointcloud down sampled from {%d} points to {%d} points.",
		(int)points_.size(), (int)output->points_.size());
	return output;
}

//Coded ML-EM
void HUREL::Compton::ReconPointCloud::CalculateCodedMLEMSystemMatrix(std::vector<ListModeData>& lmData, Eigen::MatrixXd& Geant4SystemMatrix, open3d::geometry::PointCloud& reconPCFOVlimtranposed, Eigen::Matrix4d& transformation, Eigen::MatrixXd* outposCASystemMatrix)
{
	Eigen::MatrixXd PositionCount; PositionCount = MatrixXd::Zero(60, 60);
	Eigen::MatrixXd emptyCASystemMatrix;
	//Eigen::MatrixXd ZeroMatrix; ZeroMatrix = MatrixXd::Zero(1, 17161); //20250416
	//Eigen::MatrixXd sumMatrix; 	sumMatrix = MatrixXd::Zero(17161, 1); //20250416
	Eigen::MatrixXd ZeroMatrix; ZeroMatrix = MatrixXd::Zero(1, 14641);
	Eigen::MatrixXd sumMatrix; 	sumMatrix = MatrixXd::Zero(14641, 1);
	Eigen::MatrixXd tmptmpSystemMatrix; tmptmpSystemMatrix = MatrixXd::Zero(1, 3600);
	//Eigen::MatrixXd tmpSystemMatrix;	tmpSystemMatrix = MatrixXd::Zero(17161, 3600); //20250416
	//Eigen::MatrixXd SystemMatrix = MatrixXd::Zero(17161, 3600); //20250416
	Eigen::MatrixXd tmpSystemMatrix;	tmpSystemMatrix = MatrixXd::Zero(14641, 3600); 
	Eigen::MatrixXd SystemMatrix = MatrixXd::Zero(14641, 3600); 

	//float calValue = 0.0;
	int index = 0;

	VectorXd DetectorXpos; VectorXd DetectorYpos;
	DetectorXpos.setLinSpaced(61, -0.15, 0.15); DetectorYpos.setLinSpaced(61, -0.15, 0.15);

	for (size_t Xposition = 0; Xposition < DetectorXpos.rows() - 1; ++Xposition)
	{
		for (size_t Yposition = 0; Yposition < DetectorYpos.rows() - 1; ++Yposition)
		{
			//PositionCount(Yposition, Xposition) = 0;
			for (size_t eventnumber = 0; eventnumber < lmData.size(); ++eventnumber)
			{
				//std::cout << DetectorYpos(Yposition) << std::endl;
				if ((DetectorYpos(Yposition) <= lmData[eventnumber].Scatter.RelativeInteractionPoint(1)) && (DetectorYpos(Yposition + 1)) > lmData[eventnumber].Scatter.RelativeInteractionPoint(1))
				{
					if ((DetectorXpos(Xposition) <= lmData[eventnumber].Scatter.RelativeInteractionPoint(0)) && (DetectorXpos(Xposition + 1) > lmData[eventnumber].Scatter.RelativeInteractionPoint(0)))
					{
						PositionCount(Yposition, Xposition) += 1;
					}
				}
			}
		}
	}

	//PositionCount = PositionCount.transpose();
	index = 0;
	for (size_t Xposition = 0; Xposition < DetectorXpos.rows() - 1; ++Xposition)
	{
		for (size_t Yposition = 0; Yposition < DetectorYpos.rows() - 1; ++Yposition)
		{
			tmptmpSystemMatrix(0, index) = PositionCount(Yposition, Xposition);
			index = index + 1;
		}
	}
	for (size_t j = 0; j < 14641; ++j) //20250416
	{
		tmpSystemMatrix.row(j) = tmptmpSystemMatrix;
	}

	tmpSystemMatrix = tmpSystemMatrix.cwiseProduct(Geant4SystemMatrix); //��� ��
	//for (size_t k = 0; k < 17161; ++k)
	//{
	//	sumMatrix(k, 0) = tmpSystemMatrix.row(k).sum();
	//}
	size_t rownumber = 0;
	for (size_t r = 0; r < 121; ++r) //20250416
	{
		for (size_t c = 0; c < 121; ++c) //20250416
		{
			SystemMatrix.row(rownumber) = tmpSystemMatrix.row(r + 121 * c); //20250416
			rownumber += 1;
		}
	}
	//sphere SystemMatrix (SystemMatrix) to pcl SystemMatrix
	Eigen::Vector3d normvec(3); normvec[0] = 0.; normvec[1] = 0.; normvec[2] = 1.;
	Eigen::MatrixXd Xpos; Eigen::MatrixXd Ypos; Eigen::MatrixXd Zpos;
	Eigen::Vector3d azvec(3); Eigen::Vector3d polarvec(3);
	Eigen::MatrixXd& CASystemMatrix = *outposCASystemMatrix;

	double pcvecx;  double pcvecy; double pcvecz;
	double aziangle = 0; double polarangle = 0;
	double aziangledegree = 0; double polarangledegree = 0;
	VectorXf SystemMatrixAz; SystemMatrixAz.setLinSpaced(121, -60, 60); //20250416
	VectorXf SystemMatrixPol; SystemMatrixPol.setLinSpaced(121, -60, 60); //20250416
	int skip = 0;

	for (size_t i = 0; i < reconPCFOVlimtranposed.points_.size(); ++i)
	{
		skip = 0;
		for (size_t Azpos = 0; Azpos < SystemMatrixAz.rows(); ++Azpos)
		{
			aziangle = SystemMatrixAz(Azpos, 0);
			if (skip == 0)
			{
				for (size_t Polpos = 0; Polpos < SystemMatrixPol.rows(); ++Polpos)
				{
					polarangle = SystemMatrixPol(Polpos, 0);
					pcvecx = reconPCFOVlimtranposed.points_[i].x();
					pcvecy = reconPCFOVlimtranposed.points_[i].y();
					pcvecz = reconPCFOVlimtranposed.points_[i].z();
					azvec[0] = pcvecx; azvec[1] = 0; azvec[2] = pcvecz;
					azvec = azvec.normalized();
					polarvec[0] = 0; polarvec[1] = pcvecy; polarvec[2] = pcvecz;
					polarvec = polarvec.normalized();
					aziangledegree = acos(azvec.dot(normvec)) / EIGEN_PI * 180;
					polarangledegree = acos(polarvec.dot(normvec)) / EIGEN_PI * 180;
					if (azvec[0] <= 0)
					{
						aziangledegree = aziangledegree * (-1);
					}
					if (polarvec[1] <= 0)
					{
						polarangledegree = polarangledegree * (-1);
					}
					if (abs(aziangledegree - aziangle) <= 0.5 && abs(polarangledegree - polarangle) <= 0.5)
					{
						CASystemMatrix.row(i) += SystemMatrix.row(Azpos + 121 * Polpos); //20250416
						//						CASystemMatrix.row(i) += angCASystemMatrix.row(Polpos + 131 * Azpos);
						skip = 1;
						break;
					}
				}
			}
			else
				break;
		}
	}
	//for (size_t j = 0; j < reconPCFOVlimtranposed.points_.size(); ++j)
	//{
	//	reconValues_[j] = CASystemMatrix.row(j).sum();
	//	if (reconValues_[j] > maxReconValue)
	//	{
	//		maxReconValue = reconValues_[j];
	//	}
	//}
}

//Compton ML-EM
void HUREL::Compton::ReconPointCloud::CalculateComptonMLEMSystemMatrix(std::vector<ListModeData> lmData, open3d::geometry::PointCloud& imagespace, Eigen::MatrixXd* outSystemMatrix, double(*calcFunc)(ListModeData, Eigen::Vector3d))
{
	Eigen::MatrixXd& SystemMatrix = *outSystemMatrix;
	float calValue = 0.0;
	for (size_t eventnumber = 0; eventnumber < lmData.size(); ++eventnumber)
	{
		for (size_t i = 0; i < imagespace.points_.size(); ++i) //for every point cloud
		{
			calValue = static_cast<float>(calcFunc(lmData[eventnumber], imagespace.points_[i]));
			if (!isnan(calValue))
			{
				SystemMatrix(eventnumber, i) = calValue;
				//reconValues_[i] += calValue;
			}
			else
			{
				SystemMatrix(eventnumber, i) = 0;
			}
		}
	}
	/*
	for (size_t i = 0; i < points_.size(); ++i)
	{
		if (reconValues_[i] > maxReconValue)
		{
			maxReconValue = reconValues_[i];
		}
	}
	*/
}

void HUREL::Compton::ReconPointCloud::ReconComptonMLEMIteration(int iterationtotalN, Eigen::MatrixXd SystemMatrix)
{
	Eigen::MatrixXd EMimg(points_.size(), 1);
	Eigen::MatrixXd EMimgtmp(points_.size(), 1);
	Eigen::MatrixXd pinvcc(SystemMatrix.rows(), 1);
	Eigen::MatrixXd pcc; 		Eigen::MatrixXd bprjcc;
	EMimg = Eigen::MatrixXd::Ones(points_.size(), 1);
	double dZero = 0.0; double InfN = 1 / dZero;

	for (size_t i = 0; i < points_.size(); ++i)
	{
		if (isnan(points_[i][0] + points_[i][1] + points_[i][2]) == 0) // if NaN: 1 is printed out
		{
			EMimg(i, 0) = 1;
		}
		else
		{
			EMimg(i, 0) = 0;
		}
	}

	for (int i = 0; i < iterationtotalN; ++i)
	{
		pcc = SystemMatrix * EMimg;
		for (int j = 0; j < SystemMatrix.rows(); ++j)
		{
			pinvcc(j, 0) = 1 / pcc(j, 0);
			if (pinvcc(j, 0) == InfN)
			{
				pinvcc(j, 0) = 0;
			}
		}
		//bprjcc = SystemMatrix.transpose() * pinvcc;
		bprjcc = (SystemMatrix.colwise().sum()).transpose() * (SystemMatrix.transpose() * pinvcc);

		for (int k = 0; k < points_.size(); ++k)
		{
			EMimgtmp(k, 0) = EMimg(k, 0) * bprjcc(k, 0);
		}
		EMimg = EMimgtmp;
	}
	for (size_t j = 0; j < points_.size(); ++j)
	{
		reconValues_[j] = EMimg(j, 0);
		if (reconValues_[j] > maxReconValue)
		{
			maxReconValue = reconValues_[j];
		}
	}
}

void HUREL::Compton::ReconPointCloud::ReconCodedMLEMIteration(int iterationtotalN, Eigen::MatrixXd SystemMatrix)
{
	Eigen::MatrixXd EMimg(points_.size(), 1); Eigen::MatrixXd EMimgtmp(points_.size(), 1);
	Eigen::MatrixXd pca; Eigen::MatrixXd bprjca;
	Eigen::MatrixXd pinvca(SystemMatrix.cols(), 1);
	EMimg = Eigen::MatrixXd::Ones(points_.size(), 1);
	double dZero = 0.0; double InfN = 1 / dZero;
	for (size_t i = 0; i < points_.size(); ++i)
	{
		if (isnan(points_[i][0] + points_[i][1] + points_[i][2]) == 0) // if NaN: 1 is printed out
		{
			EMimg(i, 0) = 1;
		}
		else
		{
			EMimg(i, 0) = 0;
		}
	}

	for (int i = 0; i < iterationtotalN; ++i)
	{
		pca = SystemMatrix.transpose() * EMimg;
		for (int j = 0; j < SystemMatrix.cols(); ++j)
		{
			pinvca(j, 0) = 1 / pca(j, 0);
			if (pinvca(j, 0) == InfN)
			{
				pinvca(j, 0) = 0;
			}
		}
		bprjca = SystemMatrix * pinvca;
		//bprjca = (SystemMatrix.rowwise().sum()).cwiseProduct(SystemMatrix * pinvca);

		for (int k = 0; k < points_.size(); ++k)
		{
			EMimgtmp(k, 0) = EMimg(k, 0) * bprjca(k, 0);
		}
		EMimg = EMimgtmp;
	}
	for (size_t j = 0; j < points_.size(); ++j)
	{
		reconValues_[j] = EMimg(j, 0);
		if (reconValues_[j] > maxReconValue)
		{
			maxReconValue = reconValues_[j];
		}
	}
}
void HUREL::Compton::ReconPointCloud::ReconHybridMLEMIteration(int iterationtotalN, Eigen::MatrixXd CCSystemMatrix, Eigen::MatrixXd CASystemMatrix, int weight)
{
	Eigen::MatrixXd EMimg(points_.size(), 1); Eigen::MatrixXd EMimgtmp(points_.size(), 1); 	EMimg = Eigen::MatrixXd::Ones(points_.size(), 1);
	Eigen::MatrixXd pca; Eigen::MatrixXd bprjca; 	Eigen::MatrixXd pcc; Eigen::MatrixXd bprjcc;
	Eigen::MatrixXd pinvca(CASystemMatrix.cols(), 1); Eigen::MatrixXd pinvcc(CCSystemMatrix.rows(), 1);
	Eigen::MatrixXd CCSI;  CCSI = Eigen::MatrixXd::Ones(points_.size(), 1);
	Eigen::MatrixXd CASI;  CASI = Eigen::MatrixXd::Ones(points_.size(), 1);
	double dZero = 0.0; double InfN = 1 / dZero;
	for (size_t i = 0; i < points_.size(); ++i)
	{
		if (isnan(points_[i][0] + points_[i][1] + points_[i][2]) == 0) // if NaN: 1 is printed out
		{
			EMimg(i, 0) = 1;
		}
		else
		{
			EMimg(i, 0) = 0;
		}
	}

	for (int i = 0; i < iterationtotalN; ++i)
	{
		pcc = CCSystemMatrix * EMimg; 		pca = CASystemMatrix.transpose() * EMimg;
		for (int j = 0; j < CCSystemMatrix.rows(); ++j)
		{
			pinvcc(j, 0) = 1 / pcc(j, 0);
			if (pinvcc(j, 0) == InfN)
			{
				pinvcc(j, 0) = 0;
			}
		}
		for (int j = 0; j < CASystemMatrix.cols(); ++j)
		{
			pinvca(j, 0) = 1 / pca(j, 0);
			if (pinvca(j, 0) == InfN)
			{
				pinvca(j, 0) = 0;
			}
		}
		bprjcc = (CCSystemMatrix.colwise().sum()).transpose() * (CCSystemMatrix.transpose() * pinvcc);
		bprjca = (CASystemMatrix.rowwise().sum()).cwiseProduct(CASystemMatrix * pinvca);

		for (int k = 0; k < points_.size(); ++k)
		{
			EMimgtmp(k, 0) = EMimg(k, 0) * (bprjcc(k, 0) + bprjca(k, 0)) / (CASI(k, 0) + CCSI(k, 0) * weight);
		}
		EMimg = EMimgtmp;
	}
	for (size_t j = 0; j < points_.size(); ++j)
	{
		reconValues_[j] = EMimg(j, 0);
		if (reconValues_[j] > maxReconValue)
		{
			maxReconValue = reconValues_[j];
		}
	}
}

double HUREL::Compton::ReconPointCloud::SimpleComptonBackprojection(ListModeData lmData, Eigen::Vector3d imgPoint)
{
	double ScatterEnergy = lmData.Scatter.InteractionEnergy;
	double AbsorberEnergy = lmData.Absorber.InteractionEnergy;
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;

	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}
	double comptonCal = 1 - 511 * ScatterEnergy / AbsorberEnergy / TotalEnergy;
	if (comptonCal >= 1 || comptonCal <= -1)
	{
		return 0;
	}
	double complexChk = acos(comptonCal);
	typedef std::complex<double> dcomplex;
	dcomplex c(complexChk);

	if (c.imag() != 0)
	{
		return 0;
	}
	if (c.real() >= 65 * 3.1416 / 180)
	{
		return 0;
	}

	double comptonScatteringAngle = acos(comptonCal) / EIGEN_PI * 180; //degree
	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.TransformedInteractionPoint.head<3>());
	Eigen::Vector3d scatterToAbsorberVector = (lmData.Scatter.TransformedInteractionPoint.head<3>() - lmData.Absorber.TransformedInteractionPoint.head<3>());
	effectToScatterVector.normalize();
	scatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(scatterToAbsorberVector);
	double effectedAngle = acos(positionDotPord) / EIGEN_PI * 180;
	double sigmacomptonScatteringAngle = 511 / sin(comptonScatteringAngle) * sqrt(1 / pow(AbsorberEnergy, 2)) - 1 / pow(TotalEnergy, 2) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy), 2) + 1 / pow(TotalEnergy, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy), 2);
	double BP_sig_thres = 2;
	if (sigmacomptonScatteringAngle * 180 / 3.1416 < 1.5)
	{
		sigmacomptonScatteringAngle = 1.5 * 3.1416 / 180;
	}
	if (comptonScatteringAngle <= 65)
	{
		if (abs(effectedAngle - comptonScatteringAngle) < BP_sig_thres * sigmacomptonScatteringAngle)
		{
			return 1;
		}
	}
	else
	{
		return 0;
	}
}

double HUREL::Compton::ReconPointCloud::SimpleComptonMLEM(ListModeData lmData, Eigen::Vector3d imgPoint)
{
	if (isnan(imgPoint[0] + imgPoint[1] + imgPoint[2]) == 1)
	{
		return 0;
	}
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}
	double ScatterEnergy = lmData.Scatter.InteractionEnergy;
	double AbsorberEnergy = lmData.Absorber.InteractionEnergy;
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;
	float reconValue;

	double comptonCal = acos(1 - 511 * ScatterEnergy / AbsorberEnergy / TotalEnergy);
	typedef std::complex<double> dcomplex;
	dcomplex c(comptonCal);
	if (c.imag() != 0)
	{
		return 0.0;
	}
	if (c.real() > 65 * EIGEN_PI / 180) //check scatter angle 
	{
		return 0.0;
	}
	double comptonScatteringAngle = comptonCal / EIGEN_PI * 180; //degree
	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.RelativeInteractionPoint.head<3>());
	Eigen::Vector3d scatterToAbsorberVector = (lmData.Scatter.RelativeInteractionPoint.head<3>() - lmData.Absorber.RelativeInteractionPoint.head<3>());
	effectToScatterVector.normalize(); 	scatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(scatterToAbsorberVector);
	double effectedAngle = acos(positionDotPord);
	double sigmacomptonScatteringAngle = 0.511 / sin(comptonCal) * sqrt(((1 / pow(AbsorberEnergy / 1000, 2)) - 1 / pow(TotalEnergy / 1000, 2)) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy / 1000), 2) + 1 / pow(TotalEnergy / 1000, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy / 1000), 2));
	double Systemmatrixthres = 5;
	double KNcrosssection = pow(AbsorberEnergy / TotalEnergy, 2) * (TotalEnergy / AbsorberEnergy + AbsorberEnergy / TotalEnergy - pow(sin(comptonScatteringAngle), 2));

	if (sigmacomptonScatteringAngle * 180 / 3.1416 < 1.5)
	{
		sigmacomptonScatteringAngle = 1.5 * 3.1416 / 180;
	}
	if (abs(effectedAngle - comptonCal) < Systemmatrixthres * sigmacomptonScatteringAngle)
	{
		float reconValue = 1 * exp(-0.5 * pow(((effectedAngle - comptonCal) / sigmacomptonScatteringAngle), 2));
		return reconValue;
	}
	else
	{
		return 0.0;
	}
}

double HUREL::Compton::ReconPointCloud::SimpleComptonMLEM2D(ListModeData lmData, Eigen::Vector3d imgPoint)
{
	if (isnan(imgPoint[0] + imgPoint[1] + imgPoint[2]) == 1)
	{
		return 0;
	}
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}
	double ScatterEnergy = lmData.Scatter.InteractionEnergy;
	double AbsorberEnergy = lmData.Absorber.InteractionEnergy;
	double TotalEnergy = ScatterEnergy + AbsorberEnergy;
	float reconValue;

	double comptonCal = acos(1 - 511 * ScatterEnergy / AbsorberEnergy / TotalEnergy);
	typedef std::complex<double> dcomplex;
	dcomplex c(comptonCal);
	if (c.imag() != 0)
	{
		return 0.0;
	}
	if (c.real() > 65 * EIGEN_PI / 180) //check scatter angle 
	{
		return 0.0;
	}
	double comptonScatteringAngle = comptonCal / EIGEN_PI * 180; //degree
	Eigen::Vector3d effectToScatterVector = (imgPoint.head<3>() - lmData.Scatter.TransformedInteractionPoint.head<3>());
	Eigen::Vector3d scatterToAbsorberVector = (lmData.Scatter.TransformedInteractionPoint.head<3>() - lmData.Absorber.TransformedInteractionPoint.head<3>());
	effectToScatterVector.normalize(); 	scatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(scatterToAbsorberVector);
	double effectedAngle = acos(positionDotPord);
	double sigmacomptonScatteringAngle = 0.511 / sin(comptonCal) * sqrt(((1 / pow(AbsorberEnergy / 1000, 2)) - 1 / pow(TotalEnergy / 1000, 2)) * pow(0.08 / 2.35 * sqrt(AbsorberEnergy / 1000), 2) + 1 / pow(TotalEnergy / 1000, 4) * pow(0.08 / 2.35 * sqrt(ScatterEnergy / 1000), 2));
	double Systemmatrixthres = 5;
	double KNcrosssection = pow(AbsorberEnergy / TotalEnergy, 2) * (TotalEnergy / AbsorberEnergy + AbsorberEnergy / TotalEnergy - pow(sin(comptonScatteringAngle), 2));

	if (sigmacomptonScatteringAngle * 180 / 3.1416 < 1.5)
	{
		sigmacomptonScatteringAngle = 1.5 * 3.1416 / 180;
	}
	if (abs(effectedAngle - comptonCal) < Systemmatrixthres * sigmacomptonScatteringAngle)
	{
		float reconValue = 1 * exp(-0.5 * pow(((effectedAngle - comptonCal) / sigmacomptonScatteringAngle), 2));
		return reconValue;
	}
	else
	{
		return 0.0;
	}
}