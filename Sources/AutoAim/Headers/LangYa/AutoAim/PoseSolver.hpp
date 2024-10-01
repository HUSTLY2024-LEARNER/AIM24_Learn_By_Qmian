#pragma once

#include <vector>
#include <opencv2/core.hpp>

namespace LangYa:: inline AutoAim
{
	struct CameraIntrinsicsParameterPack
	{
		float FocalLength[2]{1331.1f, 1330.1f};
		float PrincipalPoint[2]{624.5817f, 526.3662f};
		float RadialDistortion[3]{-0.1059f, -0.3427f, 1.4125f};
		float TangentialDistortion[2]{0.0072f, 0};

		void GetCameraMatrix(cv::Mat& matrix) const;

		void GetDistortionCoefficients(cv::Mat& matrix) const;
	};

	struct ArmorTransform
	{
		std::vector<float> Rotation{};
		std::vector<float> Translation{};
	};

	struct PoseSolver
	{
		bool SolveLargeArmor(
			const std::vector<cv::Point2f>& orderedPoints,
			ArmorTransform& transform);
		bool SolveSmallArmor(
			const std::vector<cv::Point2f>& orderedPoints,
			ArmorTransform& transform);

		/*bool SolveOutpostArmor(
			const std::vector<cv::Point2f>& orderedPoints,
			ArmorTransform& transform
		);*/

		bool SolveArmor(
			const std::vector<cv::Point2f>& orderedPicturePoints,
			const std::vector<cv::Point3f>& worldPoints,
			ArmorTransform& transform);

		bool SolveArmor(const std::vector<cv::Point2f>& orderedPoints, ArmorTransform& transform, const bool isLarge);

		explicit PoseSolver(const CameraIntrinsicsParameterPack& parameter);

	private:
		cv::Mat CameraMatrix{};
		cv::Mat DistortionCoefficients{};
		cv::Mat RVec{};
		cv::Mat TVec{};
	};
}
