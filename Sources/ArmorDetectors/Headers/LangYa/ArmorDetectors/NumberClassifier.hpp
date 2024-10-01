#pragma once

#include <span>
#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <string>
#include <spdlog/spdlog.h>

#include "ArmorCorrector.hpp"

namespace LangYa:: inline ArmorDetectors
{
	[[nodiscard]] inline cv::Mat GetLUTTable(const float gamma, std::span<std::uint8_t, 256> gammaTableCache)
	{
		for (int i = 0; i < 256; ++i)
			gammaTableCache[i] = cv::saturate_cast<std::uint8_t>(
				std::pow(static_cast<float>(i / 255.0), gamma) * 255.0f
			);
		return {1, 256, CV_8UC1, gammaTableCache.data()};
	}

	class NumberClassifier
	{
	public:
		static constexpr auto DefaultGamma = 2.5f;

	private:
		std::array<std::uint8_t, 256> GammaTableCache{};
		cv::Mat LUTTable{GetLUTTable(DefaultGamma, GammaTableCache)};

		cv::Ptr<cv::ml::SVM> SVMModel;
		cv::HOGDescriptor HOG{
			{32, 32},
			{16, 16},
			{8, 8},
			{8, 8},
			16
		};

	public:
		[[nodiscard]] bool LoadModel(const std::string& model_path);

		[[nodiscard]] int Predict(const cv::Mat& frame, const std::vector<cv::Point2f>& corners) const;
	};

	class SVMArmorCorrector final : public ArmorCorrector
	{
	public:
		NumberClassifier Classifier{};

		void Correct(const cv::Mat& frame, std::vector<ArmorObject>& armors) override
		{
			for (auto& armor: armors)
			{
				static std::vector<cv::Point2f> temp_points(4);
				temp_points[0] = armor.apex[0];
				temp_points[1] = armor.apex[3];
				temp_points[2] = armor.apex[2];
				temp_points[3] = armor.apex[1];
				const auto before_type = armor.type;

				const auto mid_type = Classifier.Predict(frame, temp_points);

				if (mid_type != 0 || before_type == 6)
				{
					armor.type = mid_type;
				}
				
				spdlog::debug("SVMArmorCorrector> ov({}) svm({}) result({})",
					before_type, 
					mid_type, 
					armor.type);
			}
		}
	};
}
