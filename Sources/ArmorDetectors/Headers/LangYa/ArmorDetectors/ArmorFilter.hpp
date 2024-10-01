#pragma once

#include <LangYa/ArmorDetectors/Armor.hpp>
#include <spdlog/spdlog.h>

namespace LangYa:: inline ArmorDetectors {
	class ArmorFilter final {
		[[nodiscard]] static std::uint64_t FindClosestArmor(
			const std::vector<ArmorObject>& armors,
			const cv::Point2f& target) {
			std::uint64_t min_offset_index{};
			float min_offset{99999.9f};
			for (std::uint64_t i{0}; i < armors.size(); i++) {
				const auto& points = armors[i].apex;
				const auto center = (points[0] + points[1] + points[2] + points[3]) / 4;
				const auto offset_vector = center - target;
				const auto offset = offset_vector.dot(offset_vector);
				if (offset >= min_offset) continue;

				min_offset = offset;
				min_offset_index = i;
			}
            // 选出最近的装甲板
			return min_offset_index;
		}

	public:
		/// @brief 获取装甲板的 宽高比
		[[nodiscard]] static float GetArmorRatio(const cv::Point2f points[4]) {
			const auto width = std::max(
				std::abs(points[0].x - points[1].x),
				std::abs(points[1].x - points[2].x)
			);

			const auto height = std::max(
				std::abs(points[0].y - points[1].y),
				std::abs(points[1].y - points[2].y)
			);

			return width / height;
		}

		UnitTeam MyTeam{UnitTeam::Red};
		std::vector<int> BalanceUnitIDList{};

		[[nodiscard]] bool ShouldHitRed() const { return MyTeam != UnitTeam::Red; }

		[[nodiscard]] bool ShouldHitBlue() const { return MyTeam != UnitTeam::Blue; }

		[[nodiscard]] bool Filter(
			const std::vector<ArmorObject>& armors,
			ArmorObject& target,
			const bool hitOutpost,
			const bool hitSentry) const {
			std::vector<ArmorObject> filtered_armors{};
			for (const auto& armor : armors) {
				if (ShouldHitBlue() && armor.ActualColor() != ArmorObject::Blue) continue;
				if (ShouldHitRed() && armor.ActualColor() != ArmorObject::Red) continue;

				static constexpr auto error = 0;
				static constexpr auto sentry = 6;
				static constexpr auto outpost = 0;
				static constexpr auto base = 7;
				static constexpr auto engineer = 2;

				if (armor.ActualColor() == ArmorObject::Gray) continue;

				const auto sqrt_size = std::sqrt(armor.area);
				if (armor.type == engineer) {
					constexpr auto size_threshold = 24;
					//spdlog::info("工程装甲板大小：{}", sqrt_size);
					if (sqrt_size < size_threshold) {
						continue;
					}
				}
				else {
					constexpr auto size_threshold = 20;
					if (sqrt_size < size_threshold) {
						continue;;
					}
				}


				if (!hitSentry && armor.type == sentry) continue;
				if (!hitOutpost && (armor.type == outpost || armor.type == base)) continue;
				if (hitOutpost && (armor.type != outpost && armor.type != base)) continue;

				filtered_armors.push_back(armor);
			}

			if (filtered_armors.empty()) return false;

			static const cv::Point2f Center{640, 512};
			target = filtered_armors[FindClosestArmor(filtered_armors, Center)];

			return true;
		}
	};
}
