#pragma once

#include <vector>

#include "Armor.hpp"

namespace LangYa:: inline ArmorDetectors
{
	class ArmorCorrector
	{
	public:
		virtual ~ArmorCorrector() = default;
		virtual void Correct(const cv::Mat& frame, std::vector<ArmorObject>& armors) = 0;
	};
}
