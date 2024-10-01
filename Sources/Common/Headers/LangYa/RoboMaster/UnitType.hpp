#pragma once

#include <cstdint>

namespace LangYa:: inline RoboMaster
{
	/// @brief 单位类型
	enum class UnitType : std::uint8_t
	{
		Unknown   = 0,
		Hero      = Unknown + 1,
		Engineer  = Hero + 1,
		Infantry1 = Engineer + 1,
		Infantry2 = Infantry1 + 1,
		Infantry3 = Infantry2 + 1,
		Drone     = Infantry3 + 1,
		Sentry    = Drone + 1,
		Dart	  = Sentry + 1,
		Radar     = Dart + 1
	};

	/// @brief 团队类型
	enum class UnitTeam : std::uint8_t
	{
		Unknown = 0,
		Red     = Unknown + 1,
		Blue    = Red + 1,
		Other   = Blue + 1
	};

	/// @brief 与串口协议中相同的单位类型数值
	enum class AllUnitType : std::uint8_t
	{
		Unknown    = 0,
		RedOffset  = 0,
		BlueOffset = 100,

#define sc_team_unit(team, type) team##type = team##Offset + static_cast<std::uint8_t>(UnitType::type)  // NOLINT(bugprone-macro-parentheses)

#define sc_red_unit(type) sc_team_unit(Red, type)
		sc_red_unit(Hero),
		sc_red_unit(Engineer),
		sc_red_unit(Infantry1),
		sc_red_unit(Infantry2),
		sc_red_unit(Infantry3),
		sc_red_unit(Drone),
		sc_red_unit(Sentry),
		sc_red_unit(Radar),
#undef sc_red_unit

#define sc_blue_unit(type) sc_team_unit(Blue, type)
		sc_blue_unit(Hero),
		sc_blue_unit(Engineer),
		sc_blue_unit(Infantry1),
		sc_blue_unit(Infantry2),
		sc_blue_unit(Infantry3),
		sc_blue_unit(Drone),
		sc_blue_unit(Sentry),
		sc_blue_unit(Radar),
#undef sc_blue_unit

#undef sc_team_unit
	};
}
