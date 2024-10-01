//#define SC_MDB

#include <LangYa/AutoAim.hpp>

#include <Cango/CommonUtils.hpp>
#include <Cango/TaskDesign/TaskExecution.hpp>
#include <GalaxySDK.hpp>
#include <LangYa/ArmorDetectors.hpp>


using namespace LangYa;

namespace {
	class Application {
		std::string Name{"autoaim"};
		Cango::ObjectOwner<CorrectedDetector> Detector{std::make_shared<CorrectedDetector>()};// 检测
		Cango::ObjectOwner<ArmorFilter> Filter{std::make_shared<ArmorFilter>()};// 装甲板筛选
		CameraIntrinsicsParameterPack CameraIntrinsics{};// 初始化相机内参
		Cango::ObjectOwner<PoseSolver> Solver{std::make_shared<PoseSolver>(CameraIntrinsics)};// 解算
		Cango::ObjectOwner<Predictor> ArmorPredictor{std::make_shared<Predictor>()};// 预测
		Cango::GalaxySDK::GxCameraCheatsheet CameraCheatsheet{};// 管理相机相关的资源
		GimbalCheatsheet Gimbals{};// 云台
		NaviServerCheatsheet Navis{};// 导航
		MainService Mains{};

		void ConfigureCamera() noexcept {
			auto& sheet = CameraCheatsheet;

			const auto default_logger = spdlog::default_logger();

            // 捕获图像
			auto& provider = *sheet.Provider;
			{
				auto&& config = provider.Configure();// 相机配置

				const auto actors = config.Actors;
				actors.Logger = default_logger;
				actors.CameraLogger = default_logger;

				const auto options = config.Options;

				auto& op = options.OpenParameter;
				op.UseOrder = false;
				op.Identifier.Type = Cango::GalaxySDK::DeviceIdentifier::SerialNumber;
				op.Identifier.Content = "KE0200060396";

				auto& cp = options.ConfigureParameter;
				cp.AutoExposure.Value = GX_EXPOSURE_AUTO_OFF;
				cp.ExposureTime.Value = 4000;
				cp.AutoGain.Value = GX_GAIN_AUTO_OFF;
				cp.Gain.Value = 16.0f;
				cp.RedBalanceRatio.Value = 1.7148f;
				cp.GreenBalanceRatio.Value = 1;
				cp.BlueBalanceRatio.Value = 1.9414;
			}

            // 处理
			auto& consumer = *sheet.Consumer;
			{
				auto&& config = consumer.Configure();
				const auto options = config.Options;
				options.MinInterval = 0ms;
			}

            // 交付逻辑
			auto& provider_task = sheet.Task;
			{
				auto&& config = provider_task.Configure();
				const auto options = config.Options;
				options.MinInterval = 5s;
			}
		}

        // 云台&串口配置
		void ConfigureGimbal() noexcept {
			{
				const auto default_logger = spdlog::default_logger();

				auto&& config = Gimbals.Provider->Configure();
				const auto actors = config.Actors;
				actors.Logger = default_logger;
				actors.RWerLogger = default_logger;

				const auto options = config.Options;
				options.Ports = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"};
				options.BaudRate = boost::asio::serial_port::baud_rate{115200};
			}
			{
				auto&& config = Gimbals.Task.Configure();
				const auto options = config.Options;
				options.ProviderMinInterval = 5s;
				options.ReaderMinInterval = 0ms;
				options.WriterMinInterval = 4ms;
			}
			{
				const auto& utils = Gimbals.Utils;

				const auto& reader_config = utils.ReaderMonitor;
				reader_config->Counter.MaxCount = 10;

				const auto& writer_config = utils.WriterMonitor;
				writer_config->NormalHandler = Cango::EasyDeliveryTaskMonitor::EmptyHandler;
				writer_config->ExceptionHandler = Cango::EasyDeliveryTaskMonitor::EmptyHandler;
			}
		}

        // 导航
		void ConfigureNavi() noexcept {
			{
				const auto default_logger = spdlog::default_logger();

				const auto actors = Navis.Provider->Configure().Actors;
				actors.Logger = default_logger;
				actors.ClientLogger = default_logger;

				const auto options = Navis.Provider->Configure().Options;
				options.LocalEndpoint = boost::asio::ip::tcp::endpoint{
					boost::asio::ip::address_v4::from_string("0.0.0.0"), 8989 //TODO 不用全局地址
				};
			}
			{
				const auto options = Navis.Task.Configure().Options;
				options.ProviderMinInterval = 3s;
				options.ReaderMinInterval = 10ms;
				options.WriterMinInterval = 10ms;
			}
			{
				const auto& utils = Navis.Utils;

				const auto& reader_config = utils.ReaderMonitor;
				reader_config->Counter.MaxCount = 10;

				const auto& writer_config = utils.WriterMonitor;
				writer_config->NormalHandler = Cango::EasyDeliveryTaskMonitor::EmptyHandler;
				writer_config->ExceptionHandler = Cango::EasyDeliveryTaskMonitor::EmptyHandler;
			}
		}

		void ConfigureMain() {
			Mains.Detector = Detector;
			Mains.Filter = Filter;
			Mains.Solver = Solver;
			Mains.TargetPredictor = ArmorPredictor;
			Mains.GameDataSource = Gimbals.Utils.GameDataPool;
			Mains.GimbalDataSource = Gimbals.Utils.GimbalDataPool;
			Mains.GimbalControlDataDestination = Gimbals.Utils.GimbalControlPool;
			Mains.PictureSource = CameraCheatsheet.ImagePool;
			Mains.NaviCommandMessageDestination = Navis.Utils.WriterMessagePool;
			Mains.NaviControlMessageSource = Navis.Utils.ReaderMessagePool;
		}

	public:
		bool Initialize() {
			if (!Cango::InitializeGlobalLogger(Name)) return false;
			if (!Cango::GalaxySDK::Initialize(*spdlog::default_logger())) return false;
			if (!Detector->Corrector.Classifier.LoadModel("classifier.svm")) return false;
			if (!Detector->Detector.LoadModel("armor_detector_model.xml")) return false;

			ConfigureCamera();
			ConfigureGimbal();
			ConfigureNavi();
			ConfigureMain();

			return true;
		}

		int Main() {
			using namespace Cango;
			ThreadVector threads{};
			threads << Gimbals.Task << CameraCheatsheet.Task << Mains << Navis.Task
#ifdef SC_MDB
				<< [this] {
					GameData data{};
					auto& pool = *Gimbals.Utils.GameDataPool;
					spdlog::info("FakeData> wait 5s");
					std::this_thread::sleep_for(5s);
					static constexpr std::uint8_t isMyTeamRed{0};

					static auto new_fake = [&pool](
						const GameData& d,
						const std::string_view message,
						const std::chrono::seconds waitDuration) {
						static std::uint64_t index{0};
						pool.SetItem(d);
						spdlog::warn("FakeData({})> {}", index++, message);
						std::this_thread::sleep_for(waitDuration);
					};

					auto huge_disadvantage = [&data] {
						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 60;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "比赛开始", 30s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 31;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "己方前哨剩余775血", 10s);

						{
							data.GameCode.EnemyOutpostHealth = 0;
							data.GameCode.SelfOutpostHealth = 60;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 100;
							data.TimeLeft = 360;
						}
						new_fake(data, "敌方前哨被摧毁", 10s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 7;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "己方前哨剩余175血", 10s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 0;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "己方前哨被摧毀", 10s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 0;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 150;
							data.AmmoLeft = 300;
						}
						new_fake(data, "血量为 150", 50s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 0;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "血量为 400", 20s);
					};
					auto maybe_a_round = [&data] {
						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 60;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 0;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
							data.TimeLeft = 420;
						}
						new_fake(data, "比赛未开始", 10s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 60;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
							data.TimeLeft = 420;
						}
						new_fake(data, "比赛开始", 30s);

						{
							data.GameCode.EnemyOutpostHealth = 0;
							data.GameCode.SelfOutpostHealth = 60;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 0;
							data.TimeLeft = 360;
						}
						new_fake(data, "敌方前哨被摧毁", 10s);

						{
							data.GameCode.EnemyOutpostHealth = 0;
							data.GameCode.SelfOutpostHealth = 60;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 0;
							data.TimeLeft = 360;
						}
						new_fake(data, "等待补弹", 20s);

						{
							data.GameCode.EnemyOutpostHealth = 0;
							data.GameCode.SelfOutpostHealth = 40;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 100;
							data.TimeLeft = 360;
						}
						new_fake(data, "己方前哨 40 * 25", 10s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 20;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
							data.TimeLeft = 200;
						}
						new_fake(data, "己方前哨 20 * 25", 10s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 5;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
							data.TimeLeft = 200;
						}
						new_fake(data, "己方前哨 5 * 25", 10s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 0;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
							data.TimeLeft = 200;
						}
						new_fake(data, "己方前哨爆炸", 30s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 0;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 100;
							data.AmmoLeft = 300;
							data.TimeLeft = 100;
						}
						new_fake(data, "血量较低(hp=100)", 20s);
					};
					auto test_precaution = [&data] {
						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 60;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "比赛开始", 30s);

						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 0;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "己方前哨爆炸", 1s);
						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 0;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "己方前哨爆炸", 1s);
						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 0;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "己方前哨爆炸", 1s);
						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 0;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "己方前哨爆炸", 1s);
					};
					auto anti_op = [&data] {
						{
							data.GameCode.EnemyOutpostHealth = 60;
							data.GameCode.SelfOutpostHealth = 60;
							data.GameCode.IsMyTeamRed = isMyTeamRed;
							data.GameCode.IsGameBegin = 1;
							data.SelfHealth = 400;
							data.AmmoLeft = 300;
						}
						new_fake(data, "比赛开始", 30s);
					};

					//huge_disadvantage();
					maybe_a_round();
					//test_precaution();
					//anti_op();
				}

#endif
				;
			JoinThreads(threads);
			return 0;
		}

		~Application() {
			Cango::GalaxySDK::Close(*spdlog::default_logger());
			spdlog::drop_all();
		}
	};
}

int main() {
	Application app{};
	if (!app.Initialize()) return 1;
	return app.Main();
}
