#pragma once

#include <chrono>
#include <numbers>
#include <spdlog/spdlog.h>

namespace LangYa:: inline AutoAim 
{
	/// @brief 二维三质点匀速旋转模型
	/// 自己向着前哨站的视角的中心的连线设为前哨站模型的 0 度，设为 x 轴正方向
	/// 面向前哨站，装甲板向右转为正角度，向左转为负
	///
	///	前哨站为原点，自己在前哨站x正半轴的位置
	/// 从自己的角度看，x减小为前，x增大为后，y减小为左，y增大为右
	///
	/// 时间分度为毫秒
	///
	class OutpostViewModel 
	{
	public:
		static constexpr double RoundPi = std::numbers::pi_v<double>;
		static constexpr double Round2Pi = 2 * RoundPi;
		static constexpr double DeltaPhase = Round2Pi / 3.0;
		static constexpr double ViewOffset = Round2Pi / 4.0;
		static constexpr double ViewLength = DeltaPhase;
		static constexpr double RotationRadius = 0.60f;

	private:
		double LastOmega{};		// 弧度 / ms
		double ViewMin{};
		double ViewMax{};

		void UpdateView() noexcept
		{
			if (std::fabs(Omega - LastOmega) < 0e-5) return;

			LastOmega = Omega;
			if (Omega > 0)
			{
				ViewMin = -ViewOffset + RoundPi;
				ViewMax = ViewMin + ViewLength;
			}
			else
			{
				ViewMax = ViewOffset + RoundPi;
				ViewMin = ViewMax - ViewLength;
			}
		}

		[[nodiscard]] bool ViewRadian(const double radian) const noexcept
		{
			return radian >= ViewMin && radian <= ViewMax;
		}

	public:

		double Omega{0};		// 弧度 / ms
		double Phase{0};		// 弧度
		std::chrono::steady_clock::time_point InitialTime{};

		void Reset() noexcept
		{
			Phase = 0;
			Omega = 0;
			InitialTime = std::chrono::steady_clock::now();
		}

		[[nodiscard]] double View(const std::chrono::steady_clock::time_point& currentTime) noexcept
		{
			UpdateView();

			const auto ms_count = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - InitialTime).count();
			const double radian = Omega * ms_count;

			if (const double radian1 = std::fmod(radian + DeltaPhase * 0, Round2Pi); 
				ViewRadian(radian1))
			{
				spdlog::info("Outpost> viewing 1({})", radian1);
				return radian1;
			}

			if (const double radian2 = std::fmod(radian + DeltaPhase * 1, Round2Pi); 
				ViewRadian(radian2)) {
				spdlog::info("Outpost> viewing 2({})", radian2);
				return radian2;
			}

			const double radian3 = std::fmod(radian + DeltaPhase * 2, Round2Pi);
			spdlog::info("Outpost> viewing 3({})", radian3);
			return radian3;
		}

		/// @brief 产生自己观测前哨站的角度
		///	@param distance 自己在x轴上距离原点的距离
		///	@param currentTime 当前时间
		///	@return 在当前位置，看向观测结果时，视角与x轴的夹角方向，向左为正，向右为负
		[[nodiscard]] double Produce(const double distance, const std::chrono::steady_clock::time_point& currentTime) noexcept
		{
			const auto radian = View(currentTime);
			const double x = RotationRadius * std::cos(radian);
			const double y = RotationRadius * std::sin(radian);

			// 自己在 (distance, 0)，获取 (x,y) 与 (distance, 0) 的夹角
			return std::atan2(distance - x, y);
		}
	};
}
