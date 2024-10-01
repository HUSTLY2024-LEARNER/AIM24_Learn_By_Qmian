#pragma once

#include <atomic>
#include <taskflow/taskflow.hpp>

namespace Sango:: inline Assistant
{

#pragma region concepts

	/// @brief 任务状态，要求内部有一个判断任务是否完成的方法和一个中断任务的方法
	struct TaskStatus
	{
		/// @brief 用于判断任务是否已经执行完毕，要求判断的过程不对任务产生任何影响
		///	@return 只要任务执行完毕，即使是异常退出，都会返回 @c true ，其他情况会是 @c false
		[[nodiscard]] virtual bool IsDone() const noexcept = 0;

		/// @brief 要求中断任务，执行后可通过 @c IsDone 函数判断任务是否已经中断
		///	任务一般通过在循环中检查此函数控制的一个标志量来提前中断任务，所以此函数不能保证立即中断任务
		virtual void Interrupt() noexcept = 0;
	};

	/// @brief 满足 @c TaskStatus 定义的概念的类型
	template <typename TObject>
	concept IsTaskStatus = requires(const TObject& readonlyObject, TObject& mutableObject)
	{
		{ readonlyObject.IsDone() } -> std::convertible_to<bool>;
		{ mutableObject.Interrupt() } -> std::same_as<void>;
	};

	/// @brief 针对特别任务的架构师，负责判断任务的可执行状态，规划任务到工作流，观测任务状态
	struct TaskArchitect : virtual TaskStatus
	{
		/// @brief 用于检查任务的依赖状态，判断是否满足依赖，可以执行
		/// @return 如果任务可以执行，返回 @c true ，否则返回 @c false
		[[nodiscard]] virtual bool Check() noexcept = 0;

		/// @brief 规划任务到工作流，将任务的执行过程添加到工作流中
		///	@param taskflow 用于规划任务的工作流
		virtual void Plan(tf::FlowBuilder& taskflow) noexcept = 0;
	};

	/// @brief 满足 @c TaskArchitect 定义的概念的类型
	template <typename TObject>
	concept IsTaskArchitect = requires(const TObject& readonlyObject, TObject& mutableObject, tf::FlowBuilder& taskflow)
	{
		{ mutableObject.Check() } -> std::convertible_to<bool>;
		{ mutableObject.Plan(taskflow) };
		{ mutableObject.Monitor() } -> IsTaskStatus;
		{ readonlyObject.Monitor() } -> IsTaskStatus;
	};

#pragma endregion

#pragma region implementations

	/// @brief 最简单的任务状态实现，仅维持一个 @c std::atomic_bool ，用于多线程中存储任务状态
	struct TaskDone final : TaskStatus
	{
		/// @brief 在任务没有执行时，此值默认为 @c false ，当任务执行完毕或异常结束时，此值为 @c true
		std::atomic_bool Done{false};

		/// 实现 @c TaskStatus::IsDone
		[[nodiscard]] bool IsDone() const noexcept override;

		/// 实现 @c TaskStatus::Interrupt
		void Interrupt() noexcept override;
	};

#pragma endregion

}
