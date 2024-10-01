#include <LangYa/AutoAim.hpp>
#include <LangYa/CodeHelper.hpp>
#include <LangYa/Network.hpp>
#include <LangYa/Messages.hpp>

using namespace LangYa;

namespace {
	class NaviTransmitter final : public DestinationNode<NaviControlMessage> {
		GimbalControlMessage Message{};

	public:
		Dependency<DestinationNode<GimbalControlMessage>> GimbalDestination{};

		void SetItem(const NaviControlMessage& item) noexcept override {
			Message.Velocity = item.Velocity;
			GimbalDestination.GetObject().SetItem(Message);
		}
	};
}

int main() {
	if (!InitializeGlobalLogger("navi2serial")) return -1;

	const auto gimbal_control_message_pool = AsyncItemPool<GimbalControlMessage>::MakeShared();
	const auto navi_command_message_pool = AsyncItemPool<NaviCommandMessage>::MakeShared();

	const auto transmitter = std::make_shared<NaviTransmitter>();
	transmitter->GimbalDestination = gimbal_control_message_pool;

	AboutTCPSocket::ServerPlanner<NaviControlMessage, NaviCommandMessage> navi_server{};
	boost::asio::io_context navi_acceptor_io_context{};
	{
		auto navi = navi_server.Configure();
		navi.ReaderDestination = transmitter;
		navi.WriterSource = navi_command_message_pool;

		const auto acceptor = std::make_shared<TCPAcceptorWrapper>();
		acceptor->Device = MakeTCPAcceptor(
			navi_acceptor_io_context, boost::asio::ip::tcp::endpoint{
				boost::asio::ip::address_v4::from_string("127.0.0.1"), 8989
			});
		navi.Acceptor = acceptor;
	}
	if (!navi_server.Check()) return -1;

	GimbalDriver gimbal_driver{};
	{
		auto gimbal = gimbal_driver.Configure();
		gimbal.ReaderDestination = std::make_shared<EmptyDestinationNode<SerialPortMessage>>();
		gimbal.WriterSource = gimbal_control_message_pool;

		const auto maker = std::make_shared<SerialPortMaker>();
		maker->Parameters.DeviceNames = {
			"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"
		};
		maker->Parameters.BaudRate = boost::asio::serial_port::baud_rate{115200};
		gimbal.Acceptor = maker;

		gimbal.ReaderMinInterval = 0ms;
		gimbal.WriterMinInterval = 4ms;
	}
	if (!gimbal_driver.Check()) return -1;

	ThreadGroup group{};
	group << navi_server << gimbal_driver;
	Join(group);

	return 0;
}
