#include <Sango/Assistant/TaskArchitect.hpp>

namespace Sango :: inline Assistant 
{
	bool TaskDone::IsDone() const noexcept
	{
		return Done.load(std::memory_order_relaxed);
	}

	void TaskDone::Interrupt() noexcept
	{
		Done.store(true, std::memory_order_relaxed);
	}
}
