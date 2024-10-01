#include <LangYa/ArmorDetectors.hpp>
#include <LangYa/AutoAim.hpp>
#include <LangYa/Camera.hpp>
#include <LangYa/CodeHelper.hpp>
#include <LangYa/Network.hpp>
#include <LangYa/RoboMaster.hpp>

using namespace LangYa;

int main() {
	if (!InitializeGlobalLogger("autoaim")) return -1;

	const auto detector = std::make_shared<CorrectedDetector>();
	if (!detector->Corrector.Classifier.LoadModel("classifier.svm")) return -1;
	if (!detector->Detector.LoadModel("armor_detector_model.xml")) return -1;

	const auto filter = std::make_shared<ArmorFilter>();
	filter->MyTeam = UnitTeam::Red;
	filter->BalanceUnitIDList = {};

	CameraIntrinsicsParameterPack camera_intrinsics_parameter_pack{};
	const auto solver = std::make_shared<PoseSolver>(camera_intrinsics_parameter_pack);

	const auto predictor = std::make_shared<Predictor>();

	DaHengParameterStorage camera_parameter_storage{};
	if (!static_cast<ParseFromStream&>(camera_parameter_storage)
		.Parse("1280 1024 640 480 1 6000 0 0 0 2 65536 200 0 1.2656 1 1.7461"))
		return -1;

	const auto game_data_pool = std::make_shared<AsyncItemPool<GameData>>();
	const auto gimbal_data_pool = AsyncItemPool<EasyMessageToAutoAim>::MakeShared();
	const auto gimbal_control_message_pool = AsyncItemPool<EasyMessageFromAutoAim>::MakeShared();
	const auto timed_picture_pool = AsyncItemPool<TimedPicture>::MakeShared();
	const auto debugger_frame_pool = AsyncItemPool<cv::Mat>::MakeShared();

	const auto dispatcher = std::make_shared<DroneSerialPortMessageDispatcher>();
	dispatcher->GimbalDataDestination = gimbal_data_pool;

	const auto gimbal_driver_maker = std::make_shared<SerialPortMaker>();
	gimbal_driver_maker->Parameters.DeviceNames = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"};
	gimbal_driver_maker->Parameters.BaudRate = boost::asio::serial_port::baud_rate{115200};

	EasyGimbalDriver gimbal_driver{};
	{
		auto&& config = gimbal_driver.Configure();
		config.ReaderDestination = dispatcher;
		config.WriterSource = gimbal_control_message_pool;
		config.Acceptor = gimbal_driver_maker;
		config.ReaderMinInterval = 0ms;
		config.WriterMinInterval = 4ms;
		config.SingleDeviceMode = true;
	}
	if (!gimbal_driver.Check()) return -1;

	DaHengCameraService camera_service{};
	{
		camera_service.PictureDestination = timed_picture_pool;
		camera_service.CameraSerialNumber = "KE0200060400";
		camera_service.CameraParameters = camera_parameter_storage;
		camera_service.AcquisitionInterval = 0ms;
	}

	boost::asio::io_context navi_acceptor_io_context{};
	const auto navi_acceptor = std::make_shared<TCPAcceptorWrapper>();
	navi_acceptor->Device = MakeTCPAcceptor(
		navi_acceptor_io_context, boost::asio::ip::tcp::endpoint{
			boost::asio::ip::address_v4::from_string("127.0.0.1"), 8989
		});

	DroneService main_service{};
	{
		main_service.Detector = detector;
		main_service.Filter = filter;
		main_service.Solver = solver;
		main_service.TargetPredictor = predictor;
		main_service.GimbalDataSource = gimbal_data_pool;
		main_service.GimbalControlMessageDestination = gimbal_control_message_pool;
		main_service.TimedPictureSource = timed_picture_pool;
	}

	ThreadGroup group{};
	group << gimbal_driver << camera_service  << main_service;
	Join(group);

	return 0;
}
