#pragma once

#include <LangYa/RoboMaster/UnitType.hpp>
#include <opencv2/opencv.hpp>

namespace LangYa:: inline ArmorDetectors
{
	struct Armor 
	{
		UnitType Type;
		UnitTeam Team;
	};

	/// @brief 传统视觉中的装甲板
	struct ArmorBlob
	{
		ArmorBlob() : corners(4)
		{
        }

        double confidence;
        cv::Rect rect;
        std::vector<cv::Point2f> corners;
        int _class;
        double angle;
        double x, y, z;
        bool is_big_armor;

		[[nodiscard]] double GetSquareDistance() const
		{
			return x * x + y * y + z * z;
		}

		friend bool operator<(const ArmorBlob& left, const ArmorBlob& right)
		{
			return left.GetSquareDistance() < right.GetSquareDistance();
		}
	};

	using ArmorBlobs = std::vector<ArmorBlob>;

	/// @brief 机器学习中的装甲板
	struct ArmorObject
	{
		enum ColorEntry {
			Blue = 0,
			Red = 1,
			Gray = 2,
			Others
		};

		cv::Rect2f Rectangular;
		int type;
		int color;
		float prob;
		std::vector<cv::Point2f> points;
		int area;
		cv::Point2f apex[4];

		ColorEntry ActualColor() const {
			switch (color / 2) {
			case 0:
				return Blue;
			case 1:
				return Red;
			case 2:
				return Gray;
			default:
				return Others;
			}
		}

		bool IsLarge() const {
			return color % 2 == 1;
		}
	};
}
