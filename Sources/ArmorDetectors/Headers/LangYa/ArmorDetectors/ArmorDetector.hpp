#pragma once

#include <LangYa/ArmorDetectors/Armor.hpp>
#include <LangYa/CodeHelper.hpp>
#include <opencv2/core.hpp>

namespace LangYa:: inline ArmorDetectors
{
	struct ArmorDetector
	{
		virtual ~ArmorDetector();

		virtual bool Detect(const cv::Mat& src, std::vector<ArmorObject>& objects) = 0;

		bool Process(const cv::Mat& argument, std::vector<ArmorObject>& result);
	};
}
