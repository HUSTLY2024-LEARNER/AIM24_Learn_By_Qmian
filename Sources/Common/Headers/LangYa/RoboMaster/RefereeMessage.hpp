#pragma once

#include <cstdint>

namespace LangYa:: inline RoboMaster 
{
	//TODO 大疆的裁判系统
	struct FrameHeader 
	{
		std::uint8_t Head{0xA5};
	};
}
