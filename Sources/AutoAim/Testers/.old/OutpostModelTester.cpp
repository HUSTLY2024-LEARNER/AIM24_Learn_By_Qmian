#include <LangYa/AutoAim/OutpostModel.hpp>
#include <LangYa/Network.hpp>
#include <LangYa/CodeHelper.hpp>
#include <LangYa/Messages.hpp>

using namespace LangYa;

int main() {
	if (!InitializeGlobalLogger("outpost")) return -1;

	using OutpostData = JustFloats<3>;
	const auto outpost_data_pool = std::make_shared<AsyncItemPool<OutpostData>>();

	using endpoint_t = boost::asio::ip::tcp::endpoint;

	boost::asio::io_context navi_acceptor_io_context{};
	const auto acceptor = std::make_shared<TCPAcceptorWrapper>();
	const endpoint_t endpoint{boost::asio::ip::address_v4::from_string("172.22.36.70"), 8989};
	acceptor->Device = MakeTCPAcceptor(navi_acceptor_io_context, endpoint);

	const auto destination = std::make_shared<EmptyDestinationNode<NaviCommandMessage>>();

	CommunicationArchitect<TCPAcceptorWrapper, NaviCommandMessage, OutpostData> navi_server{};
	{
		auto&& config = navi_server.Configure();
		config.ReaderDestination = destination;
		config.WriterSource = outpost_data_pool;
		config.Acceptor = acceptor;
	}
	if (!navi_server.Check()) return -1;

	auto outpost_task = [outpost_data_pool] {
		OutpostViewModel outpost{};
		outpost.Reset();
		outpost.Omega = 0.001;
		outpost.Phase = 0.0f;
		OutpostData data;

		while (true) {
			std::this_thread::sleep_for(std::chrono::milliseconds{100});
			const auto radian = outpost.Produce(3.0f, std::chrono::steady_clock::now());
			spdlog::info("Outpost> {}", radian);
			data.Elements[0] = radian;
			outpost_data_pool->SetItem(data);
		}
	};

	ThreadGroup group{};
	group << navi_server << outpost_task;
	Join(group);

	return 0;
}
