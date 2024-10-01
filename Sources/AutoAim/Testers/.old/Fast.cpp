int main(){}

#include <Eigen/Core>
#include <LangYa/ArmorDetectors.hpp>
#include <LangYa/AutoAim.hpp>
#include <LangYa/Network.hpp>
#include <LangYa/CodeHelper/SineWave.hpp>
#include <LangYa/CodeHelper/CollectiveLogger.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>

using namespace LangYa;
using namespace std::chrono_literals;

inline constexpr auto DetectorModulePath = "armor_detector_model.xml";
inline constexpr char CameraSerialNumber[] = {"KE0200060396"};
inline constexpr auto CameraParameterString = "1280 1024 640 480 1 8000 0 12 0 2 65536 200 0 1.5859 1 1.9531";

constexpr auto BalanceUnitID = 10;

auto ArmorDetectorHandle = std::make_shared<OpenVINOArmorDetector>();
auto PicturePoolHandle = std::make_shared<AsyncItemPool<TimedPicture>>();
const SerialPortMakeParameters serial_port_parameter{115200, "/dev/ttyACM0|/dev/ttyACM1"};
const auto running_handle = std::make_shared<std::atomic_bool>(true);
auto NaviCommandMessagePoolHandle = std::make_shared<AsyncItemPool<NaviCommandMessage>>();
auto NaviControlMessagePoolHandle = std::make_shared<AsyncItemPool<NaviControlMessage>>();
const auto gimbal_message_pool_handle = AsyncItemPool<GimbalData>::MakeShared();
const auto gimbal_control_message_pool_handle = AsyncItemPool<GimbalControlMessage>::MakeShared();
const auto debugger_image_pool = AsyncItemPool<cv::Mat>::MakeShared();

std::atomic_bool outpost_destroyed{false};

bool CheckHigh(const cv::Point2f points[4], const float pitch)
{
	static const cv::Point2f Center{640, 512};
	const auto center = (points[0] + points[1] + points[2] + points[3]) / 4;
	const auto offset = center - Center;

	// 装甲板中心距离图像中心的竖直偏移，向上为正
	const auto high = -offset.y;

	// 当前云台角对于图像中心的限位，当上文的偏移大于此值时，认为装甲板过高
	const auto pitch_gain = pitch > 0 ? (pitch * -13.3f) : pitch * -20.0f;

	return high <= pitch_gain;
}

std::atomic game_start = false;
std::atomic_bool iam_red = false;
std::atomic recovered = false;
std::atomic_bool force_run = false;
std::chrono::steady_clock::time_point game_start_time{};

struct GlobalGameStatusData
{
};

class GameCodeStandardize final : public DestinationNode<GameData>
{
public:
	Dependency<DestinationNode<GlobalGameStatusData>> GameStatusDestination{};
	GlobalGameStatusData Data{};

	std::uint16_t last_time_left = 0;
	static constexpr auto my_max_health = 400;

	void SetItem(const GameData& item) override
	{
		if (!game_start)
		{
			const auto time_diff = std::abs(item.TimeLeft - last_time_left);
			if ((time_diff > 200 && last_time_left < 15 && item.TimeLeft > 200)
				|| item.SelfHealth < my_max_health
			)
			{
				game_start = true;
				game_start_time = std::chrono::steady_clock::now();
			}
		}

		if (!outpost_destroyed)
		{
			if (item.GameStatus.SelfOutpostHealth <= 200) outpost_destroyed = true;
		}

		GameStatusDestination.GetObject().SetItem(Data);
	}
};

float PitchDiff(const float distance)
{
	// const expresion 
	constexpr auto ratio = 0.64f;
	constexpr auto ratio2 = 0.04f;

	return std::sqrt(distance) * ratio + distance * ratio2;
}

std::atomic_uint8_t DestinationID{0};

void MovementTask()
{
	while (!game_start) std::this_thread::sleep_for(1s);

	while (true)
	{
		if (outpost_destroyed)
		{
			switch (DestinationID)
			{
			case 2:
				DestinationID = 0;
				std::this_thread::sleep_for(30s);
				break;

			case 0:
				DestinationID = 3;
				std::this_thread::sleep_for(30s);
				break;

			case 3:
				DestinationID = 2;
				std::this_thread::sleep_for(40s);
				break;

			default:
				DestinationID = 0;
				std::this_thread::sleep_for(30s);
				break;
			}
		}
		else
		{
			switch (DestinationID)
			{
			case 2:
				DestinationID = 10;
				std::this_thread::sleep_for(40s);
				break;

			case 10:
				DestinationID = 0;
				std::this_thread::sleep_for(40s);
				break;

			case 0:
				DestinationID = 16;
				std::this_thread::sleep_for(20s);
				break;

			case 16:
				DestinationID = 2;
				std::this_thread::sleep_for(40s);
				break;

			default:
				DestinationID = 0;
				std::this_thread::sleep_for(20s);
				break;
			}
		}
	}
}

std::atomic_bool EnemyBack{false};

void DetectBackTask()
{
	OpenVINOArmorDetector detector{};
	detector.LoadModel(DetectorModulePath);

	ArmorFilter filter{};
	cv::VideoCapture capture0{0, cv::CAP_ANY};
	cv::VideoCapture capture1{2, cv::CAP_ANY};

	auto set_capture = [](cv::VideoCapture& capture)
	{
		capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
		capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
		capture.set(cv::CAP_PROP_FRAME_WIDTH, 1600); // 最高 30fps
		capture.set(cv::CAP_PROP_EXPOSURE, -7);
		capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	};
	set_capture(capture0);
	set_capture(capture1);

	while (true)
	{
		std::this_thread::sleep_for(30ms);

		cv::Mat frame{};
		std::vector<ArmorObject> armors;
		ArmorObject target;

		capture0 >> frame;
		cv::waitKey(1);
		detector.Detect(frame, armors);
		if (filter.Filter(armors, target))
		{
			spdlog::info("DetectBack> at capture0");
			EnemyBack = true;
			continue;
		}

		capture1 >> frame;
		cv::waitKey(1);
		armors.clear();
		detector.Detect(frame, armors);
		if (filter.Filter(armors, target))
		{
			spdlog::info("DetectBack> at capture1");
			EnemyBack = true;
			continue;
		}

		EnemyBack = false;
	}
}

void AimTask()
{
	std::this_thread::sleep_for(1s);

	TimedPicture camera_picture{};
	GimbalData gimbal_message{};
	GimbalControlMessage gimbal_control_message{};
	gimbal_control_message.HeadFlag = '!';
	gimbal_control_message.CRC = 0;

	NaviCommandMessage command_message{};
	NaviControlMessage navi_control_message{};

	std::chrono::steady_clock::time_point last_found_time{};
	cv::Point2f last_found_view{};
	PoseSolver solver{CameraIntrinsicsParameterPack{}};
	Predictor predictor{};
	ArmorFilter armor_filter{};
	IDBuffer id_buffer{};

	// ROI 给点
	cv::Point2f top_left_offset{320, 320};
	cv::Rect2f roi{top_left_offset, cv::Size2f{640, 480}};

	PNPAimResult pnp_result{};

	spdlog::info("AutoAim> acquiring yaw pitch from gimbal");
	while (!gimbal_message_pool_handle->GetItem(gimbal_message))
	{
		std::this_thread::sleep_for(1s);
		spdlog::warn("AutoAim> waiting for yaw pitch from gimbal");
	}

	last_found_view.x = gimbal_message.GimbalAngles.Yaw.Value;
	last_found_view.y = 0;

	gimbal_control_message.GimbalAngles.Yaw.Value = gimbal_message.GimbalAngles.Yaw.Value;
	gimbal_control_message.GimbalAngles.Pitch.Value = 0;
	gimbal_control_message.Velocity.X = 0;
	gimbal_control_message.Velocity.Y = 0;


	command_message.DestinationID = 1;
	NaviCommandMessagePoolHandle->SetItem(command_message);

	last_found_view.y = 0;
	spdlog::info("AutoAim> waiting for game begin");
	while (!game_start)
	{
		std::this_thread::sleep_for(25ms);
		if (!gimbal_message_pool_handle->GetItem(gimbal_message)) continue;
		last_found_view.x = gimbal_control_message.GimbalAngles.Yaw.Value;

		command_message.DestinationID = 0;
		NaviCommandMessagePoolHandle->SetItem(command_message);

		if (NaviControlMessagePoolHandle->GetItem(navi_control_message))
		{
			gimbal_control_message.Velocity = navi_control_message.Velocity;
		}

		gimbal_control_message.GimbalAngles.Yaw.Value = last_found_view.x;
		gimbal_control_message.GimbalAngles.Pitch.Value = 0.0f;
		gimbal_control_message_pool_handle->SetItem(gimbal_control_message);
	}

	game_start_time = std::chrono::steady_clock::now();

	spdlog::info("AutoAim> unblocked for game begin!");

	constexpr auto x_offset = 0.005f;
	constexpr auto y_offset = 0.0f;
	constexpr auto z_offset = 0.0496f;
	Sophus::SE3<double> camera_to_gimbal
	{
		Eigen::Matrix3d::Identity(),
		Eigen::Vector3d(x_offset, y_offset, z_offset)
	};

	std::atomic_bool done{false};
	gimbal_control_message.FireCode.FireStatus = 0;
	gimbal_control_message.FireCode.FrictionStatus = 0;

	gimbal_control_message_pool_handle->SetItem(gimbal_control_message);

	auto last_enemy_back = std::chrono::steady_clock::now();

	SineWave pitch_wave{8, 0, 1s, std::chrono::steady_clock::now()};

	SineWave some_wave{100, 0, 1s, std::chrono::steady_clock::now()};
	std::this_thread::sleep_for(357ms);
	SineWave some_wave2{100, 0, 1s, std::chrono::steady_clock::now()};

	auto last_random_velocity_time = std::chrono::steady_clock::now();

	while (!done)
	{
		std::this_thread::sleep_for(1ms);

		if (!PicturePoolHandle->GetItem(camera_picture)) continue;
		if (!gimbal_message_pool_handle->GetItem(gimbal_message)) continue;

		const auto current_time = std::chrono::steady_clock::now();
		float gimbal_yaw = gimbal_message.GimbalAngles.Yaw.Value;
		float gimbal_pitch = AngleType{gimbal_message.GimbalAngles.Pitch}.Value;
		//gimbal_control_message.GimbalAngles = gimbal_message.GimbalAngles;

		//spdlog::info("AutoAim> Received yaw({}), pitch({})",gimbal_yaw, gimbal_pitch);

		std::vector<ArmorObject> armors{};
		cv::Mat canvas = camera_picture.Frame.clone();
		circle(canvas, {1280 / 2, 1024 / 2}, 4, {0, 255, 0}, 2);

		if (!ArmorDetectorHandle->Detect(camera_picture.Frame(roi), armors))
		{
			ArmorDetectorHandle->Detect(camera_picture.Frame, armors);
		}
		else
		{
			// roi 的后处理
			for (auto& armor : armors)
			{
				for (auto& point : armor.apex)
				{
					point.x += top_left_offset.x;
					point.y += top_left_offset.y;
				}

				for (auto& point : armor.points)
				{
					point.x += top_left_offset.x;
					point.y += top_left_offset.y;
				}
			}
		}

		for (auto& armor : armors)
		{
			line(canvas, armor.points[0], armor.points[2], {255, 0, 0}, 2);
			line(canvas, armor.points[1], armor.points[3], {255, 0, 0}, 2);
		}
		debugger_image_pool->SetItem(canvas);

		if (NaviControlMessagePoolHandle->GetItem(navi_control_message))
		{
			gimbal_control_message.Velocity = navi_control_message.Velocity;
			spdlog::info("NaviAttendant> neighbor id: {}", navi_control_message.Locator.Neighbor);

			const bool locator_failed = navi_control_message.Locator.Neighbor == -1;
			const bool actually_no_speed = std::pow(navi_control_message.Velocity.X, 2) +
				std::pow(navi_control_message.Velocity.X, 2) <= 4;

			gimbal_control_message.FireCode.Rotate = locator_failed || actually_no_speed ? 3 : 0;

			if (actually_no_speed && navi_control_message.Locator.Neighbor == 0)
			{
				const auto delta_t = last_random_velocity_time - current_time;
				if (delta_t > 1s && delta_t < 2s)
				{
					gimbal_control_message.Velocity.X = some_wave.Produce(current_time);
					gimbal_control_message.Velocity.Y = some_wave2.Produce(current_time);
				}
				else if (delta_t > 4s)
				{
					last_random_velocity_time = current_time;
				}
			}
		}

		ArmorObject selected_armor{};
		if (!armor_filter.Filter(armors, selected_armor))
		{
			if (EnemyBack)
			{
				spdlog::info("AimTask> EnemyBack");

				if (current_time - last_enemy_back > 2s)
				{
					last_enemy_back = current_time;
					gimbal_control_message.GimbalAngles.Yaw.Value = gimbal_message.GimbalAngles.Yaw.Value - 180.0f;
				}
			}

			const auto last_destination = command_message.DestinationID;
			command_message.DestinationID = DestinationID;
			if (command_message.DestinationID != last_destination)
				spdlog::info("Main> set destination node({})", command_message.DestinationID);
			NaviCommandMessagePoolHandle->SetItem(command_message);

			if (current_time - last_found_time > 2s)
			{
				/*gimbal_control_message.GimbalAngles.Pitch.Value = static_cast<std::int16_t>((-8.0f + pitch_wave.Produce()) * 100.0f);
				if (current_time - last_enemy_back > 2s)
					gimbal_control_message.GimbalAngles.Yaw.Value = gimbal_yaw + 10.0f;*/
			}
			gimbal_control_message_pool_handle->SetItem(gimbal_control_message);
			continue;
		}

		last_found_time = current_time;
		bool stable_recognized = id_buffer.Correct(selected_armor.type);
		spdlog::info("AutoAim> armor type({})", selected_armor.type);

		const float armor_ratio = ArmorFilter::GetArmorRatio(selected_armor.apex);
		spdlog::info("AutoAim> armor_ratio: ({})", armor_ratio);

		bool is_large = armor_ratio > 3.0f || selected_armor.type == 1 || selected_armor.type == BalanceUnitID;
		spdlog::info("AutoAim> is large({})", is_large);

		std::vector<cv::Point2f> pnp_points{};
		std::ranges::copy(selected_armor.apex, std::back_inserter(pnp_points));

		ArmorTransform transform{};
		solver.SolveArmor(pnp_points, transform, is_large);

		if (!pnp_result.FromTranslation(transform.Translation)) continue;

		spdlog::info("PNPResult> {} {}", pnp_result.DeltaYaw.Value, pnp_result.DeltaPitch.Value);

		gimbal_control_message.GimbalAngles.Yaw.Value = gimbal_yaw + pnp_result.DeltaYaw.Value;

		// fix for outpost armor
		/*if (selected_armor.type == 6)
		{
		    gimbal_control_message.GimbalAngles.Yaw.Value += 10.0f - pnp_result.Distance * 0.2f;
		}*/

		gimbal_control_message.GimbalAngles.Pitch = Angle100Type{
			AngleType{gimbal_pitch + pnp_result.DeltaPitch.Value * 0.56f + PitchDiff(pnp_result.Distance)}
		};
		gimbal_control_message.GimbalAngles.Pitch.Value -= 200;

		gimbal_control_message.FireCode.FlipFireStatus();
		gimbal_control_message_pool_handle->SetItem(gimbal_control_message);

		/*auto temp = transform.RVec.ptr<double>(0)[1];
		transform.RVec.ptr<double>(0)[1] = transform.RVec.ptr<double>(0)[2];
		transform.RVec.ptr<double>(0)[2] = temp;
		temp = transform.TVec.ptr<double>(0)[1];
		transform.TVec.ptr<double>(0)[1] = transform.TVec.ptr<double>(0)[2];
		transform.TVec.ptr<double>(0)[2] = temp;

		cv::Mat armor_rotation;
		Rodrigues(transform.RVec, armor_rotation);
		Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
		cv2eigen(armor_rotation, rotation_matrix);

		Sophus::SO3<double> rotate(rotation_matrix);

		Eigen::Vector3d translate(transform.TVec.ptr<double>(0)[0], transform.TVec.ptr<double>(0)[1],
		                          -transform.TVec.ptr<double>(0)[2]);
		Sophus::SE3<double> armor_to_camera{rotate.inverse(), translate};

		constexpr auto shoot_right_offset = 0.0f;
		constexpr auto shoot_up_offset = 0.0f;

		float rcv_yaw = (gimbal_yaw + shoot_right_offset / 100.f) * M_PI / 180.0f;
		float rcv_pitch = (gimbal_pitch + shoot_up_offset / 100.0f) * M_PI / 180.0f;

		Eigen::Matrix3d rotation_matrix3;
		rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(rcv_yaw, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(rcv_pitch, Eigen::Vector3d::UnitX());
		Sophus::SE3 gimbal_to_world{
			rotation_matrix3,
			Eigen::Vector3d(0, 0, 0)
		};
		auto camera_to_world = gimbal_to_world * camera_to_gimbal;

		auto armor_pose = camera_to_world * armor_to_camera;

		/*cv::Point3f target_pnp_point{
			static_cast<float>(armor_pose.translation().x()),
			static_cast<float>(armor_pose.translation().y()),
			static_cast<float>(armor_pose.translation().z())
		};#1#

		float delta_t = static_cast<float>(
			std::chrono::duration_cast<std::chrono::microseconds>(
				camera_picture.Time - last_found_time
			).count()
		) / 1000.0f;
		spdlog::info("AutoAim> delta t({})", delta_t);


		cv::Point3f predict_position{};
		float delay_s = 0;

		if (!stable_recognized)
			predictor.resetPredictor();
		//delta_t *= 2.0f;
		const auto predict_result = predictor.Predict(
			armor_pose,
			armor_pose,
			false,
			1,
			gimbal_yaw,
			gimbal_pitch,
			delta_t,
			predict_position,
			delay_s
		);

		float target_yaw = static_cast<float>(predict_result.yaw +
			std::round((gimbal_yaw - predict_result.yaw) / 360.0f) *
			360.0) /* - 2.5 #1#;
		float target_pitch = gimbal_pitch + (pnp_result.DeltaPitch.Value + PitchDiff(
			pnp_result.Distance * (is_large ? 0.91f : 1.0f))) * 0.53f;

		if (target_pitch < -180.0f) target_pitch += 360.0f;
		if (target_pitch > 180.0f) target_pitch -= 360.0f;

		last_found_time = camera_picture.Time;
		last_found_view.x = target_yaw;
		last_found_view.y = target_pitch;

		gimbal_control_message.GimbalAngles.Yaw.Value = target_yaw;
		gimbal_control_message.GimbalAngles.Pitch = Angle100Type{AngleType{target_pitch}};
		spdlog::info("Aim> send {} {}",
		             gimbal_control_message.GimbalAngles.Yaw.Value,
		             gimbal_control_message.GimbalAngles.Pitch.Value
		);

		gimbal_control_message_pool_handle->SetItem(gimbal_control_message);*/
	}
}

int main()
{
	if (!InitializeGlobalLogger("autoaim")) return -1;
	spdlog::info("Main> loading resources");

	spdlog::info("Main> configuring camera");
	DaHengParameterStorage CameraParameterPack{};
	if (!static_cast<ParseFromStream&>(CameraParameterPack)
		.Parse(CameraParameterString))
		return -1;

	spdlog::info("Main> configuring detector");
	if (!ArmorDetectorHandle->LoadModel(DetectorModulePath)) return -1;

	const auto game_status_pool = AsyncItemPool<GlobalGameStatusData>::MakeShared();

	const auto game_code_standardize = std::make_shared<GameCodeStandardize>();
	game_code_standardize->GameStatusDestination = game_status_pool;

	SerialPortIOHostService serial_port_host_service{};
	{
		serial_port_host_service.DispatcherPointer->GameDataDestination = game_code_standardize;
		serial_port_host_service.DispatcherPointer->GimbalDataDestination = gimbal_message_pool_handle;
		serial_port_host_service.IOHostService.FinderHostService.Finder = std::make_shared<
			SerialPortDevice::DefaultFinder>(serial_port_parameter);
		auto& io_able_host_service = serial_port_host_service.IOHostService.DeviceDestinationPointer->IOAbleHostService;
		io_able_host_service.WriterHostService.Source = gimbal_control_message_pool_handle;
		io_able_host_service.ReaderHostService.MinInterval = 0ms;
		io_able_host_service.WriterHostService.MinInterval = 0ms;
	}

	CameraService camera_service{};
	{
		auto& dependencies = camera_service.Dependencies;
		dependencies.PictureDestination = PicturePoolHandle;

		auto& configurations = camera_service.Configurations;
		configurations.CameraSerialNumber = CameraSerialNumber;
		configurations.CameraParameters = CameraParameterPack;
	}

	IPv4TCPServer navi_server{};
	{
		const auto attendant_handle = std::make_shared<NaviAttendant>();
		auto& host_service = attendant_handle->HostService;
		host_service.WriterHostService.Source = NaviCommandMessagePoolHandle;
		host_service.ReaderHostService.Destination = NaviControlMessagePoolHandle;
		navi_server.Dependencies.Attendant = attendant_handle;
		navi_server.Configurations.LocalEndpoint = IPv4Endpoint{"127.0.0.1:8989"};
	}

	IPv4TCPServer debugger_server{};
	{
		const auto attendant_handle = std::make_shared<DebuggerAttendant>();
		attendant_handle->FramePool = debugger_image_pool;
		attendant_handle->TransmitterPointer->DebuggerDataDestination = std::make_shared<EmptyDestinationNode<
			DebuggerAttendant::DebuggerData>>();
		debugger_server.Dependencies.Attendant = attendant_handle;
		debugger_server.Configurations.LocalEndpoint = IPv4Endpoint{"192.168.137.4:8989"};
	}

	ServiceList{}
		.Add(serial_port_host_service)
		.Add(camera_service)
		.Add(navi_server)
		.Add(AimTask)
		.Add(MovementTask)
		.Add(DetectBackTask)
		.Add(debugger_server)
		//.Add(GameStatusTask)
		.Join();

	spdlog::info("Main> wait 1s for all thread to exit");
	std::this_thread::sleep_for(1s);
	spdlog::info("Main> releasing all resource");

	ArmorDetectorHandle = nullptr;
	PicturePoolHandle = nullptr;
	NaviControlMessagePoolHandle = nullptr;

	return 0;
}
