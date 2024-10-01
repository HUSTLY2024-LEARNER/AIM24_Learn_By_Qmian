#pragma once

//#define SAFE_MODE

// ReSharper disable once CppUnusedIncludeDirective
#include <Eigen/Dense>

#include <LangYa/ArmorDetectors.hpp>
#include <LangYa/Camera.hpp>
#include <LangYa/CodeHelper.hpp>
#include <LangYa/Messages.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>

#include "GimbalDriver.hpp"
#include "PNPAimResult.hpp"
#include "PoseSolver.hpp"
#include "Predictor.hpp"

namespace LangYa:: inline AutoAim {
	using namespace std::chrono_literals;

	class DroneService final : public NamedModule, public TaskArchitect {
#pragma region working
		static bool GetTimedPicture(
			const std::string& name,
			TimedPicture& picture,
			SourceNode<TimedPicture>& timedPictureSource) {
			if (!timedPictureSource.GetItem(picture)) {
				spdlog::warn("{}> cannot get picture", name);
				std::this_thread::sleep_for(1ms);
				return false;
			}
			return true;
		}

		static bool GetGimbalData(
			const std::string& name,
			EasyMessageToAutoAim& gimbalData,
			SourceNode<EasyMessageToAutoAim>& gimbalDataSource) {
			if (!gimbalDataSource.GetItem(gimbalData)) {
				spdlog::warn("{}> cannot get gimbal data", name);
				return false;
			}
			return true;
		}

		static bool GetTarget(
			ArmorDetector& detector,
			ArmorFilter& filter,
			PoseSolver& solver,
			const cv::Mat& frame,
			ArmorObject& targetArmor,
			ArmorTransform& targetTransform,
			PNPAimResult& aimResult, bool hitOutpost) {
			std::vector<ArmorObject> detected_armors{};
			if (!detector.Detect(frame, detected_armors)) return false;

			if (!filter.Filter(detected_armors, targetArmor, hitOutpost)) return false;

			std::vector<cv::Point2f> pnp_points{};
			std::ranges::copy(targetArmor.apex, std::back_inserter(pnp_points));

			if (!(targetArmor.IsLarge
				      ? solver.SolveLargeArmor(pnp_points, targetTransform)
				      : solver.SolveSmallArmor(pnp_points, targetTransform)))
				return false;

			return aimResult.FromTranslation(targetTransform.Translation);
		}

		static void PredictTarget(
			const EasyMessageToAutoAim& gimbalData,
			const ArmorTransform& targetTransform,
			const bool stableRecognized,
			const std::chrono::steady_clock::time_point& currentTime,
			EasyMessageFromAutoAim& gimbalControl,
			Predictor& predictor,
			std::chrono::steady_clock::time_point& lastFoundTime,
			const bool noPredict) {

			const cv::Vec3d correct_rotation{
				targetTransform.Rotation[0], targetTransform.Rotation[2], targetTransform.Rotation[1]
			};
			cv::Vec3d correct_translation{
				targetTransform.Translation[0], targetTransform.Translation[2], targetTransform.Translation[1]
			};

			cv::Mat armor_rotation{};
			Rodrigues(correct_rotation, armor_rotation);
			Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
			cv2eigen(armor_rotation, rotation_matrix);
			Sophus::SO3<double> rotate{rotation_matrix};
			Eigen::Vector3d translate{correct_translation[0], correct_translation[1], -correct_translation[2]};
			Sophus::SE3<double> armor_to_camera{rotate.inverse(), translate};

			static const Angle100Type pitch_offset{-60};
			static const AngleType yaw_offset{-0.7f};
			AngleType current_yaw = gimbalData.Yaw;
			current_yaw.Value += yaw_offset.Value;
			Angle100Type current_pitch = gimbalData.Pitch;
			current_pitch.Value += pitch_offset.Value;

			RadianType gimbal_yaw{current_yaw};
			RadianType gimbal_pitch{current_pitch};

			Eigen::Matrix3d rotation_matrix3;
			rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(gimbal_yaw.Value, Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(gimbal_pitch.Value, Eigen::Vector3d::UnitX());
			Sophus::SE3 gimbal_to_world{
				rotation_matrix3,
				Eigen::Vector3d(0, 0, 0)
			};
			static constexpr auto x_offset = 0.005f;
			static constexpr auto y_offset = 0.0f;
			static constexpr auto z_offset = 0.0496f;
			static const Sophus::SE3<double> camera_to_gimbal
			{
				Eigen::Matrix3d::Identity(),
				Eigen::Vector3d(x_offset, y_offset, z_offset)
			};

			const auto camera_to_world = gimbal_to_world * camera_to_gimbal;
			const auto armor_pose = camera_to_world * armor_to_camera;

			float delta_t = static_cast<float>(
				std::chrono::duration_cast<std::chrono::microseconds>(
					currentTime - lastFoundTime
				).count()
			) / 1000.0f;
			lastFoundTime = currentTime;

			cv::Point3f predict_position{};
			float delay_s = 0;

			if (!stableRecognized) predictor.resetPredictor();

			const auto predict_result = predictor.Predict(
				armor_pose,
				armor_pose,
				false,
				1,
				gimbal_yaw.Value,
				gimbal_pitch.Value,
				delta_t,
				predict_position,
				delay_s,
				noPredict
			);

			AngleType target_yaw{
				predict_result.yaw +
				std::round((gimbalData.Yaw.Value - predict_result.yaw) / 360.0f) *
				360.0f
			};

			const Angle100Type target_pitch{
				AngleType{
					predict_result.pitch
				}
			};

			if (noPredict) {
				target_yaw.Value -= 1.1f; //std::sqrt(std::abs(targetTransform.Translation[2]));
			}

			gimbalControl.Content.GimbalAngles.Yaw = target_yaw;
			gimbalControl.Content.GimbalAngles.Pitch = target_pitch;
		}

		void GameLoop(const std::string& name) const {
			auto& detector = Detector.GetObject();
			auto& solver = Solver.GetObject();
			auto& filter = Filter.GetObject();
			auto& predictor = TargetPredictor.GetObject();
			auto& timed_picture_source = TimedPictureSource.GetObject();
			auto& gimbal_data_source = GimbalDataSource.GetObject();
			auto& gimbal_control_destination = GimbalControlMessageDestination.GetObject();

			EasyMessageFromAutoAim gimbal_control_message{};
			{
				EasyMessageToAutoAim gimbal_data{};
				while (!GetGimbalData(name, gimbal_data, gimbal_data_source)) {
					spdlog::trace("{}> cannot get first in game gimbal data");
					std::this_thread::sleep_for(100ms);
				}
				gimbal_control_message.Content.GimbalAngles = gimbal_data;
			}

			int last_target_type{};
			const auto my_team = filter.MyTeam;
			std::chrono::steady_clock::time_point last_enemy_found_time{std::chrono::steady_clock::now()};
			EasyMessageToAutoAim gimbal_data{};
			TimedPicture picture{};

			while (!Done) {
				const auto current_time = std::chrono::steady_clock::now();
				//spdlog::debug("{}> send gimbal control({})", name, ToString(gimbal_control_message));
				gimbal_control_destination.SetItem(gimbal_control_message);

				if (!GetTimedPicture(name, picture, timed_picture_source)) continue;

				if (!GetGimbalData(name, gimbal_data, gimbal_data_source)) continue;

				ArmorObject target_armor{};
				ArmorTransform target_transform{};
				PNPAimResult aim_result{};
				if (!GetTarget(
					detector,
					filter,
					solver,
					picture.Frame,
					target_armor,
					target_transform,
					aim_result, false))
					continue;

				GetGimbalData(name, gimbal_data, gimbal_data_source);

				bool stable_recognized = last_target_type == target_armor.type;
				last_target_type = target_armor.type;
				gimbal_control_message.Content.EnemyID = target_armor.type;
				PredictTarget(
					gimbal_data,
					target_transform,
					stable_recognized,
					picture.Time,
					gimbal_control_message,
					predictor,
					last_enemy_found_time,
					target_armor.type == 0 || target_armor.type == 7
				);
			}
		}

#pragma endregion

	public:
		static constexpr char Name[] = "MainService";

		Dependency<ArmorDetector> Detector{};
		Dependency<ArmorFilter> Filter{};
		Dependency<PoseSolver> Solver{};
		Dependency<Predictor> TargetPredictor{};
		Dependency<SourceNode<EasyMessageToAutoAim>> GimbalDataSource{};
		Dependency<DestinationNode<EasyMessageFromAutoAim>> GimbalControlMessageDestination{};
		Dependency<SourceNode<TimedPicture>> TimedPictureSource{};
		TaskDone DoneStatus{};

		std::atomic_bool Done{false};

		bool Check() noexcept override {
			return Detector.Check(Name, "Detector")
				&& Filter.Check(Name, "Filter")
				&& Solver.Check(Name, "Solver")
				&& TargetPredictor.Check(Name, "TargetPredictor")
				&& GimbalDataSource.Check(Name, "GimbalDataSource")
				&& GimbalControlMessageDestination.Check(Name, "GimbalControlMessageDestination")
				&& TimedPictureSource.Check(Name, "TimedPictureSource");
		}

		void Execute() noexcept override {
			const std::string name{Name};
			GameLoop(name);
		}

		[[nodiscard]] bool IsDone() const noexcept override { return DoneStatus.IsDone(); }
		void Interrupt() noexcept override { DoneStatus.Interrupt(); }
		void Reset() noexcept override { DoneStatus.Reset(); }
		std::ostream& GetName(std::ostream& stream) const noexcept override { return stream << "MainService"; }
		std::ostream& GetBrief(std::ostream& stream) const noexcept override { return stream; }
	};
}
