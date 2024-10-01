#pragma once

#include <opencv2/core.hpp>
#include <span>

namespace LangYa:: inline OpenCVUtils 
{
	inline std::span<const std::uint8_t> ToSpan(const cv::Mat& frame)
	{
		return {
			frame.data,
			static_cast<std::size_t>(frame.cols) * frame.rows * frame.channels()
		};
	}
}
