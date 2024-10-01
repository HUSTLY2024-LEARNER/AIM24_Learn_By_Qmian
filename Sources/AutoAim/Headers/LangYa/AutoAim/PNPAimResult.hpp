#pragma once

#include <ranges>
#include <span>
#include <spdlog/spdlog.h>

#include "Gimbals.hpp"

namespace LangYa:: inline AutoAim
{
	template <typename T>
	bool Contains(const T value, const T min, const T max)
	{
		return min <= value && value < max;
	}

	/// @brief 从PNP的相对位置计算出 yaw pitch
	struct PNPAimResult
	{
		float Distance{};
		AngleType DeltaYaw{};
		AngleType DeltaPitch{};

		/// @brief 从相对三维位置计算角度
		bool FromTranslation(const std::span<const float> relative)
		{
			// 此处距离的单位都是 m

			auto x = relative[0]; // 相对左右距离 相机面向对象 左负右正
			auto y = relative[1]; // 相对上下距离 相机面向对象 下负后正
			auto z = relative[2]; // 相对前后距离 相机面向对象 后负前正

			constexpr std::ranges::min_max_result<float> x_reasonable_range{-5, 5};
			constexpr std::ranges::min_max_result<float> y_reasonable_range{-5, 5};
			constexpr std::ranges::min_max_result<float> z_reasonable_range{0, 30};

			if (!Contains(x, x_reasonable_range.min, x_reasonable_range.max)
				|| !Contains(y, y_reasonable_range.min, y_reasonable_range.max)
				|| !Contains(z, z_reasonable_range.min, z_reasonable_range.max))
			{
				spdlog::warn("PNPAimResult> relative translation value not reasonable!");
				return false;
			}

			// 将 pnp 的结果从相机坐标系转换到云台坐标系
			constexpr auto z_offset = 0.0496f;
			constexpr auto x_offset = 0.005f;

			z += z_offset;
			x += x_offset;

			// 计算得到水平角和竖直角的相对值
			Distance = std::sqrt(x * x + z * z);

			DeltaYaw = AngleType{RadianType{std::atan2(-x, z)}};
			DeltaPitch = AngleType{RadianType{std::atan2(-y, Distance)}};

			return true;
		}
	};
}
