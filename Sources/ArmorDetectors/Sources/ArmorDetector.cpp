#include <LangYa/ArmorDetectors/ArmorDetector.hpp>

namespace LangYa:: inline ArmorDetectors 
{
	ArmorDetector::~ArmorDetector() = default;

	bool ArmorDetector::Process(const cv::Mat& argument, std::vector<ArmorObject>& result)
	{
		return Detect(argument, result);
	}
}
