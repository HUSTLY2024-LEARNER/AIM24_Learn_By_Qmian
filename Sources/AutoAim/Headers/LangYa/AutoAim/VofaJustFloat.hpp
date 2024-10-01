#pragma once

#include <array>
#include <cstdint>
#include <initializer_list>

#define SC_JUST_FLOAT_ARRAY_NUMBERS 0x00, 0x00, 0x80, 0x7f

namespace LangYa:: inline CodeHelper
{
	inline constexpr std::array<std::uint8_t, 4> JustFloatTails = {SC_JUST_FLOAT_ARRAY_NUMBERS};

#pragma pack(push, 1)
	template <std::uint64_t TElementCount>
	struct JustFloats final
	{
		static_assert(TElementCount > 0, "Element count of just floats must be greater than 0");

		static constexpr auto ElementCount = TElementCount;

		std::array<float, TElementCount> Elements{};
		std::array<const std::uint8_t, sizeof(JustFloatTails)> Tail{SC_JUST_FLOAT_ARRAY_NUMBERS};

		JustFloats() = default;

		JustFloats(const std::initializer_list<float>& elements)
		{
			std::uint64_t i{0};
			for (const auto& element : elements)
			{
				Elements[i++] = element;
				if (i >= TElementCount) break;
			}
		}

		JustFloats& operator=(const JustFloats& other)
		{
			if (&other == this) return *this;

			Elements = other.Elements;
			return *this;
		}
	};
#pragma pack(pop)
}
