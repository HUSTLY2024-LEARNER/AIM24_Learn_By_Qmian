#include <LangYa/AutoAim/GimbalDriver.hpp>
#include <LangYa/CodeHelper.hpp>

using namespace LangYa;

// 此程序的可配置项
struct Configurations {
	SerialPortMakeParameters SerialPortConfig{};
	float DeltaAngle{};
	float Accuracy{};

	void WriteToPTree(boost::property_tree::ptree& tree) const {
		tree.put("SerialPortConfig.BaudRate", SerialPortConfig.BaudRate.value());
		tree.put("SerialPortConfig.Devices", FormatCombine(std::span{SerialPortConfig.DeviceNames}));
		tree.put("DeltaAngle", DeltaAngle);
		tree.put("Accuracy", Accuracy);
	}

	bool ReadFromPTree(const boost::property_tree::ptree& tree) {
		std::uint32_t baud_rate{};
		std::string devices{};

		const bool all_succeeded = DefaultConfigure(tree, "SerialPortConfig.BaudRate", baud_rate)
			&& DefaultConfigure(tree, "SerialPortConfig.Devices", devices)
			&& DefaultConfigure(tree, "DeltaAngle", DeltaAngle)
			&& DefaultConfigure(tree, "Accuracy", Accuracy);

		SerialPortConfig.BaudRate = boost::asio::serial_port::baud_rate{baud_rate};
		FormatSplit(devices, "|,", SerialPortConfig.DeviceNames);
		return all_succeeded;
	}
};

bool GenerateDefaultConfig(Configurations& configurations) {
	boost::property_tree::ptree config{};
	configurations.SerialPortConfig.DeviceNames = {""};
	configurations.SerialPortConfig.BaudRate = boost::asio::serial_port::baud_rate{115200};
	configurations.DeltaAngle = 5.0f;
	configurations.Accuracy = 0.5f;
	configurations.WriteToPTree(config);
	return DefaultSave(config, "config.ini");
}

int main() {
	using namespace LangYa;
	using namespace std::chrono_literals;

	if (!InitializeGlobalLogger("gimbal_control")) return -1;

	// 如果没找到配置文件，则生成默认配置文件
	Configurations configurations{};
	if (boost::property_tree::ptree tree{};
		!DefaultLoad(tree, "config.ini")
		&& !GenerateDefaultConfig(configurations)) {
		spdlog::error("Failed to generate default config");
		return -1;
	}

	// 创建异步数据池
	const auto gimbal_data_pool = std::make_shared<Cango::AsyncItemPool<GimbalData>>();
	const auto gimbal_control_pool = std::make_shared<Cango::AsyncItemPool<GimbalControlMessage>>();

	const auto dispatcher = std::make_shared<SerialPortMessageDispatcher>();
	dispatcher->GimbalDataDestination = gimbal_data_pool;
	dispatcher->GameDataDestination = std::make_shared<EmptyDestinationNode<GameData>>();

	const auto maker = std::make_shared<SerialPortMaker>();
	maker->Parameters = configurations.SerialPortConfig;
	maker->Parameters.DeviceNames.emplace_back("/dev/ttyACM0");

	// 配置云台驱动服务
	GimbalDriver gimbal_driver{};
	{
		auto&& config = gimbal_driver.Configure();
		config.ReaderDestination = dispatcher;
		config.Acceptor = maker;
		config.WriterSource = gimbal_control_pool;
		config.ReaderMinInterval = 0ms;
		config.WriterMinInterval = 4ms;
		config.SingleDeviceMode = true;
	}

	auto task = [configurations, &gimbal_driver, gimbal_data_pool, gimbal_control_pool] {
		const std::string name{"ControlTester"};

		// 从当前角度旋转DeltaAngle，当旋转后云台返回角度误差小于Accuracy时停止，期间有计时，最多 5s 后超时停止

		GimbalData gimbal_data{};
		spdlog::info("{}> wait for gimbal yaw", name);
		while (!gimbal_data_pool->GetItem(gimbal_data)) std::this_thread::sleep_for(1ms);

		GimbalControlMessage gimbal_control{};
		gimbal_control.GimbalAngles = gimbal_data.GimbalAngles;
		gimbal_control.FireCode.Rotate = 0;

		// 循环写入，防止哨兵不接收到云台消息后直接小陀螺
		spdlog::info("{}> wait 5s for begin", name);
		for (auto i = 0; i < 1000; i++) {
			std::this_thread::sleep_for(5ms);
			gimbal_control_pool->SetItem(gimbal_control);
		}

		spdlog::info("{}> get gimbal yaw: {}", name, gimbal_data.GimbalAngles.Yaw.Value);
		const auto delta_target = 10;
		const auto target_accuracy = configurations.Accuracy;

		gimbal_control.GimbalAngles.Yaw.Value = gimbal_data.GimbalAngles.Yaw.Value + delta_target;
		const auto old_yaw = gimbal_data.GimbalAngles.Yaw;
		const auto target_yaw = gimbal_control.GimbalAngles.Yaw;
		
		spdlog::info(
			"{}> control from old_yaw: {} to target_yaw: {}",
			name, 
			old_yaw.Value,
			target_yaw.Value);
		const auto begin_time = std::chrono::steady_clock::now();
		auto last_yaw = gimbal_data.GimbalAngles.Yaw.Value;
		auto end_time = begin_time;
		while (end_time - begin_time < 5s) {
			gimbal_control_pool->SetItem(gimbal_control);
			end_time = std::chrono::steady_clock::now();
			if (gimbal_data_pool->GetItem(gimbal_data)) {
				if (std::abs(gimbal_data.GimbalAngles.Yaw.Value - last_yaw) >= 0.1f) {
					last_yaw = gimbal_data.GimbalAngles.Yaw.Value;
					spdlog::info("{}> yaw changed to {}", name, last_yaw);	
				}
			}
			if (!gimbal_data_pool->GetItem(gimbal_data)) continue;
			if (const auto delta_yaw = target_yaw.Value - gimbal_data.GimbalAngles.Yaw.Value; std::abs(
					delta_yaw) <
				target_accuracy) {
				spdlog::info("{}> rotated to target", name);
				break;
			}
		}
		spdlog::info(
			"{}> finished with current yaw: {}", name, gimbal_data.GimbalAngles.Yaw.Value);

		const auto delta_time = end_time - begin_time;
		spdlog::info(
			"{}> test end, time: {}ms>",
			name,
			std::chrono::duration_cast<std::chrono::milliseconds>(delta_time).count());

		// 循环写入，防止哨兵不接收到云台消息后直接小陀螺
		spdlog::info("{}> wait 5s for exit", name);
		for (auto i = 0; i < 1000; i++) {
			std::this_thread::sleep_for(5ms);
			gimbal_control_pool->SetItem(gimbal_control);
		}
		gimbal_driver.Interrupt();
	};

	ThreadGroup group{};
	group << gimbal_driver << task;
	Join(group);

	return 0;
}
