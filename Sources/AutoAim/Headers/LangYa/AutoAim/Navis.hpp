#pragma once

#include <Cango/ByteCommunication/BoostImplementations.hpp>

#include "Gimbals.hpp"

namespace LangYa:: inline Navis :: inline Messages {
#pragma pack(push, 1)
	struct LocatorMessage final {
		struct RotationType {
			float W;
			float X;
			float Y;
			float Z;
		};

		RotationType Rotation{};

		struct LocationType {
			float X;
			float Y;
			float Z;
		};

		LocationType Location{};
		std::int8_t Neighbor{};
	};

	struct NaviCommandMessage {
		std::uint8_t Head{'!'};
		std::uint8_t DestinationID{};
		std::uint8_t CRC{0};
	};

	struct NaviControlMessage {
		std::uint8_t Head{'!'};
		std::uint8_t LocationID{0};
		VelocityType Velocity{};
		LocatorMessage Locator{};
		std::uint8_t CRC{0};
	};
#pragma pack(pop)
}

namespace LangYa:: inline Navis :: inline Tasks {
	using EasyNaviServerTask = Cango::EasyCommunicationTask<
		Cango::BoostTCPSocketRWerProvider,
		NaviControlMessage,
		NaviCommandMessage>;

	using NaviServerCheatsheet = Cango::EasyBoostTCPSocketRWerCommunicationTaskCheatsheet<
		NaviControlMessage,
		NaviCommandMessage>;

	using EasyNaviClientTask = Cango::EasyCommunicationTask<
		Cango::CangoTCPSocketRWerProvider,
		NaviCommandMessage,
		NaviControlMessage>;

	struct NaviClientCheatsheet {
		EasyNaviClientTask Task{};
		Cango::EasyCommunicationTaskPoolsAndMonitors<NaviCommandMessage, NaviControlMessage> Utils{};
		Cango::ObjectOwner<Cango::CangoTCPSocketRWerProvider> Provider{
			std::make_shared<Cango::CangoTCPSocketRWerProvider>()
		};
		Cango::ObjectOwner<boost::asio::io_context> IOContext{std::make_shared<boost::asio::io_context>()};

		NaviClientCheatsheet() {
			auto&& provider_config = Provider->Configure();
			provider_config.Actors.IOContext = IOContext;

			Utils.Apply(Task);
			auto&& task_config = Task.Configure();
			task_config.Actors.Provider = Provider;
		}
	};
}
