#pragma once

#include <Cango/ByteCommunication/BoostImplementations/BoostRWerProvider.hpp>
#include <Cango/ByteCommunication/Core/TypedMessage.hpp>
#include <Cango/CommonUtils/AsyncItemPool.hpp>
#include <Cango/TaskDesign/FormattableObject.hpp>
#include <Cango/CommonUtils/ObjectOwnership.hpp>
#include <spdlog/logger.h>
#include <numbers>

namespace LangYa :: inline Gimbals :: inline Messages {
	using AngleType = float;
	using Angle100Type = std::int16_t;
	using RadianType = float;

	inline RadianType ToRadian(const AngleType angle) noexcept { return angle * std::numbers::pi_v<float> / 180.0f; }

	inline AngleType ToAngle(const RadianType radian) noexcept { return radian * 180.0f / std::numbers::pi_v<float>; }

#pragma pack(push, 1)
	struct GimbalAnglesType {
		AngleType Yaw;
		Angle100Type Pitch;
	};

	struct VelocityType {
		std::int8_t X;
		std::int8_t Y;
	};

	struct UWBPositionType {
		std::int16_t X;
		std::int16_t Y;
	};

	struct FireCodeType {
		/// @brief 开火状态，所有位反转表示开火 0b00 <-> 0b11
		std::uint8_t FireStatus : 2 = 0;

		/// @brief 摩擦轮状态，暂时无用
		std::uint8_t FrictionStatus : 2 = 0;

		/// @brief 开火频率，暂时无用
		std::uint8_t Frequency : 2 = 0;

		/// @brief 小陀螺状态，共四档，0 表示无速，1 表示低速， 2 表示中速， 3 表示高速
		std::uint8_t Rotate : 2 = 0;

		/// @brief 翻转开火标志位
		void FlipFireStatus() noexcept { FireStatus = FireStatus == 0 ? 0b11 : 0b00; }
	};

	/// @note 这里类型要求使用 @c std::uint16_t ，是因为在非 g++ 编译器上，此结构体的大小可能不符合预期
	struct GameCodeType {
		std::uint16_t IsGameBegin        : 1 = 0;
		std::uint16_t HeroPrecaution     : 1 = 0;
		std::uint16_t IsMyTeamRed        : 1 = 0;
		std::uint16_t EnemyOutpostHealth : 6 = 60;
		std::uint16_t SelfOutpostHealth  : 6 = 60;
		std::uint16_t IsReturnedHome     : 1 = 0;
	};

	struct GimbalData {
		static constexpr auto TypeID = 0;

		GimbalAnglesType GimbalAngles;
		std::uint16_t Unused;
		VelocityType Velocity;
		FireCodeType FireCode;
		std::uint8_t Reserved;
	};

	struct GameData {
		static constexpr auto TypeID = 1;

		GameCodeType GameCode{};
		std::uint16_t AmmoLeft{};
		std::uint16_t TimeLeft{};
		std::uint16_t SelfHealth{};
		UWBPositionType UWBPosition{};
	};

	namespace GimbalSensorVerification {
		constexpr auto DataSize = sizeof(GimbalData);
		static_assert(sizeof(GimbalData) == sizeof(GameData), "inconsistent size of messages");
	}

	using GimbalSensorData = Cango::TypedMessage<GimbalSensorVerification::DataSize>;

	struct GimbalControlData {
		std::uint8_t HeadFlag{'!'};
		VelocityType Velocity;
		GimbalAnglesType GimbalAngles;
		FireCodeType FireCode;
		std::uint8_t Tail{0};
	};
#pragma pack(pop)
}

namespace LangYa :: inline Gimbals :: inline Actors {
	class GimbalMessageDispatcher {
		Cango::ObjectUser<Cango::AsyncItemPool<GimbalData>> GimbalDataPool{};
		Cango::ObjectUser<Cango::AsyncItemPool<GameData>> GameDataPool{};
		Cango::ObjectUser<spdlog::logger> Logger{};

		struct Configurations {
			struct ActorsType {
				Cango::ObjectUser<Cango::AsyncItemPool<GimbalData>>& GimbalDataPool;
				Cango::ObjectUser<Cango::AsyncItemPool<GameData>>& GameDataPool;
				Cango::ObjectUser<spdlog::logger>& Logger;
			} Actors;
		};

	public:
		Configurations Configure() noexcept { return {.Actors = {GimbalDataPool, GameDataPool, Logger}}; }

		[[nodiscard]] bool IsFunctional() const noexcept {
			return GimbalDataPool != nullptr && GameDataPool != nullptr;
		}

		using ItemType = GimbalSensorData;

		void SetItem(const GimbalSensorData& item) const noexcept {
			switch (item.Type) {
			case GimbalData::TypeID:
				GimbalDataPool->SetItem(item.GetDataAs<GimbalData>());
				break;
			case GameData::TypeID:
				GameDataPool->SetItem(item.GetDataAs<GameData>());
				break;
			default:
				if (Logger != nullptr) { Logger->warn("接收到了未知的云台消息(类型({})字节({}))", item.Type, Format(item)); }
				break;
			}
		}
	};
}

namespace LangYa :: inline Gimbals :: inline Tasks {
	template <Cango::IsRWerProvider TProvider>
	using EasyGimbalCommunicationTask = Cango::CommunicationTask<
		TProvider,
		Cango::EasyDeliveryTaskMonitor,
		Cango::TailZeroVerifier,
		GimbalMessageDispatcher,
		Cango::AsyncItemPool<GimbalControlData>,
		Cango::EasyDeliveryTaskMonitor,
		Cango::EasyDeliveryTaskMonitor>;

	struct EasyGimbalCommunicationTaskDispatcherPoolsAndMonitors {
		Cango::ObjectOwner<GimbalMessageDispatcher> Dispatcher{std::make_shared<GimbalMessageDispatcher>()};
		Cango::ObjectOwner<Cango::AsyncItemPool<GimbalData>> GimbalDataPool{
			std::make_shared<Cango::AsyncItemPool<GimbalData>>()
		};
		Cango::ObjectOwner<Cango::AsyncItemPool<GameData>> GameDataPool{
			std::make_shared<Cango::AsyncItemPool<GameData>>()
		};
		Cango::ObjectOwner<Cango::AsyncItemPool<GimbalControlData>> GimbalControlPool{
			std::make_shared<Cango::AsyncItemPool<GimbalControlData>>()
		};
		Cango::ObjectOwner<Cango::EasyDeliveryTaskMonitor> ProviderMonitor{
			std::make_shared<Cango::EasyDeliveryTaskMonitor>()
		};
		Cango::ObjectOwner<Cango::EasyDeliveryTaskMonitor> ReaderMonitor{
			std::make_shared<Cango::EasyDeliveryTaskMonitor>()
		};
		Cango::ObjectOwner<Cango::EasyDeliveryTaskMonitor> WriterMonitor{
			std::make_shared<Cango::EasyDeliveryTaskMonitor>()
		};

		EasyGimbalCommunicationTaskDispatcherPoolsAndMonitors() noexcept {
			auto&& config = Dispatcher->Configure();
			const auto actors = config.Actors;
			actors.GameDataPool = GameDataPool;
			actors.GimbalDataPool = GimbalDataPool;
		}

		template <Cango::IsRWerProvider TProvider>
		void Apply(EasyGimbalCommunicationTask<TProvider>& task) {
			auto&& config = task.Configure();
			auto& actors = config.Actors;

			WriterMonitor->ExceptionHandler = []{};
			actors.ReaderMessageDestination = Dispatcher;
			actors.WriterMessageSource = GimbalControlPool;
			actors.ProviderMonitor = ProviderMonitor;
			actors.ReaderMonitor = ReaderMonitor;
			actors.WriterMonitor = WriterMonitor;
		}
	};

	using AutoAimGimbalCommunicationTask = EasyGimbalCommunicationTask<Cango::CangoSerialPortRWerProvider>;

	struct GimbalCheatsheet {
		AutoAimGimbalCommunicationTask Task{};
		EasyGimbalCommunicationTaskDispatcherPoolsAndMonitors Utils{};
		Cango::ObjectOwner<boost::asio::io_context> IOContext{std::make_shared<boost::asio::io_context>()};
		Cango::ObjectOwner<Cango::CangoSerialPortRWerProvider> Provider{
			std::make_shared<Cango::CangoSerialPortRWerProvider>()
		};

		GimbalCheatsheet() noexcept {
			auto&& provider_config = Provider->Configure();
			provider_config.Actors.IOContext = IOContext;

			Utils.Apply(Task);
			auto&& task_config = Task.Configure();
			task_config.Actors.Provider = Provider;
		}
	};
}
