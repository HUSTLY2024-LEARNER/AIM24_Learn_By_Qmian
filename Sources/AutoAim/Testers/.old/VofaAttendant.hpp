#pragma once

#include <LangYa/Network/IPv4TCPServer.hpp>
#include <LangYa/Network/AsyncWriterHostService.hpp>
#include <LangYa/Network/VofaJustFloat.hpp>

namespace LangYa:: inline AutoAim
{
	/// @brief Vofa 软件 TCP 客户端接待器，用于向连接到程序的 Vofa 客户端发送数据
	///	在新的客户端连接时，启动异步写入服务
	template<std::size_t TElementCount>
	class VofaAttendant final : public IPv4TCPServer::AttendantType
	{
	public:
		AsyncWriterHostService<JustFloats<TElementCount>, IPv4TCPSocket> WriterHostService{};

		void SetItem(const std::shared_ptr<IPv4TCPSocket>& item) override
		{
			std::string name = "VofaAttendant";
			spdlog::info("{}> found new connection({})", name, ToString(*item));
			WriterHostService.Dependencies.Writer = item;
			AsyncWriterHostService<JustFloats<TElementCount>, IPv4TCPSocket>::Run(WriterHostService, name);
		}
	};
}
