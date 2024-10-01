#pragma once

#include <opencv2/core.hpp>
#include <span>
// 从OpenCV的 cv::Mat 对象创建一个 std::span 对象，这样可以方便地访问和操作图像数据
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
