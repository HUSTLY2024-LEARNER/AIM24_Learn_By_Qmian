#pragma once

#include <vector>

namespace LangYa:: inline AutoAim
{
	class IDBuffer final
	{
	public:
		static constexpr auto MaxIDCount = 3;

	private:
		std::vector<int> Buffer{};

	public:
		IDBuffer() noexcept;

		void Push(int id) noexcept;

		/// @return 是否稳定识别
		bool Correct(int& id) noexcept;
	};
}
