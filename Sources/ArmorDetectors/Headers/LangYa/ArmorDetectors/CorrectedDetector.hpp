#pragma once

#include "ArmorFilter.hpp"
#include "InferenceAPI.hpp"
#include "NumberClassifier.hpp"


namespace LangYa:: inline ArmorDetectors
{
	class CorrectedDetector final
	{

	public:
        // openvino装甲板检测
		OpenVINOArmorDetector Detector{};
        // SVM数字识别
		SVMArmorCorrector Corrector{};

		bool Detect(const cv::Mat& src, std::vector<ArmorObject>& objects)
		{
			const auto width = src.cols;
			const auto height = src.rows;
			const cv::Rect2i roi(width / 4, height / 4, width / 2, height / 2);
            // 若在ROI内未检测到
			if (!Detector.Detect(src(roi), objects))
			{
                // 整个图找
				if (!Detector.Detect(src, objects)) return false;
			}
			else
			{
                // ROI的左上坐标
				const auto tl = roi.tl();
				// roi 的后处理
				for (auto& armor : objects)
				{
					for (auto& point : armor.apex)
					{
                        // 加上ROI的左上坐标
						point.x += tl.x;
						point.y += tl.y;
					}

					for (auto& point : armor.points)
					{
						point.x += tl.x;
						point.y += tl.y;
					}
				}
			}

			Corrector.Correct(src, objects);
			return true;
		}
	};
}
