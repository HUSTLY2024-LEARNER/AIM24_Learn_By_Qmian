#include <LangYa/AutoAim/IDBuffer.hpp>

namespace LangYa:: inline AutoAim
{
	IDBuffer::IDBuffer() noexcept
	{
		Buffer.reserve(3);
	}

	void IDBuffer::Push(const int id) noexcept
	{
		if (Buffer.size() < MaxIDCount)
			Buffer.push_back(id);
	}

	bool IDBuffer::Correct(int& id) noexcept
	{
		// 当前为空，认为新的输入是稳定的
		if (Buffer.empty())
		{
			Buffer.push_back(id);
			return true;
		}

		// 当前不为空，检查输入和缓冲区的值是否一致
		if (Buffer[0] == id)
		{
			// 一致则提高置信度
			Push(id);
			return true;
		}

		// 不一致则降低置信度
		Buffer.pop_back();
		if (Buffer.empty())
		{
			// 置信度够低，采用新值
			Buffer.push_back(id);
			return false;
		}

		// 置信度不够低，采用缓冲区的值
		id = Buffer[0];
		return false;
	}
}
