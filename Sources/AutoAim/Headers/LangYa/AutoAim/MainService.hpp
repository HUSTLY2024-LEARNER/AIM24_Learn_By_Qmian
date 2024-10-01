#pragma once

//#define SAFE_MODE

// ReSharper disable once CppUnusedIncludeDirective
#include <Eigen/Dense>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <Cango/CommonUtils/CallRateCounterX.hpp>
#include <LangYa/ArmorDetectors.hpp>
#include <LangYa/RoboMaster/UnitType.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>

#include "Gimbals.hpp"
#include "Navis.hpp"
#include "PNPAimResult.hpp"
#include "PoseSolver.hpp"
#include "Predictor.hpp"
#include "SineWave.hpp"
#include "VofaJustFloat.hpp"
#include "Cango/CommonUtils/CounterX.hpp"

namespace LangYa:: inline AutoAim {
	using namespace std::chrono_literals;

	class MainService final {
		static auto value_cfg(auto value, const std::int8_t cfg) { return cfg > 0 ? value : (cfg == 0 ? 0 : -value); }

#pragma region preparation

		/// @brief 与团队有关的位置 id
		struct TeamedLocation {
			static constexpr auto LocationCount = 9;

			std::uint8_t ID;

			std::uint8_t operator()(const UnitTeam team) const {
				return team == UnitTeam::Blue ? ID + LocationCount : ID;
			}
		};

		// ReSharper disable once CppRedundantZeroInitializerInAggregateInitialization
		static constexpr TeamedLocation Home{0};
		static constexpr TeamedLocation Outpost{1};
		static constexpr TeamedLocation High{2};
		static constexpr TeamedLocation Recovery{3};
		static constexpr TeamedLocation Fly{4};
		static constexpr TeamedLocation CoverShoot{5};
		static constexpr TeamedLocation Resource{7};
		static constexpr TeamedLocation EasyOres{8};

		static void WaitForFirstGimbalData(
			const std::string& name,
			const std::atomic_bool& interrupt,
			Cango::AsyncItemPool<GimbalData>& gimbalDataSource,
			GimbalData& firstGimbalData) {
			spdlog::trace("等待第一个云台数据包");
			while (true) {
				std::this_thread::sleep_for(1s);

				if (interrupt) {
					spdlog::trace("中止等待云台数据包", name);
					break;
				}

				if (gimbalDataSource.GetItem(firstGimbalData)) {
					spdlog::trace("取得第一个云台数据包", name);
					break;
				}
			}
			spdlog::debug("结束等待第一个云台数据包", name);
		}

		/// @brief 检查比赛是否开始
		[[nodiscard]] static bool CheckGameBegin(
			Cango::AsyncItemPool<GameData>& gameDataSource,
			GameData& gameData) {
			if (gameDataSource.GetItem(gameData))
				return gameData.GameCode.IsGameBegin;
			return false;
		}

		static void WaitForGameBegin(
			const std::string& name,
			const std::atomic_bool& interrupt,
			const AngleType& firstGimbalYaw,
			UnitTeam& team,
			Cango::AsyncItemPool<NaviControlMessage>& naviControlSource,
			Cango::AsyncItemPool<NaviCommandMessage>& naviCommandDestination,
			Cango::AsyncItemPool<GimbalControlData>& gimbalControlDestination,
			GameData& gameData,
			Cango::AsyncItemPool<GameData>& gameDataSource) {
			GimbalControlData gimbal_control_message{};

			// 设置云台角度
			gimbal_control_message.GimbalAngles.Yaw = firstGimbalYaw;
			gimbal_control_message.GimbalAngles.Pitch = Angle100Type{0};

			NaviCommandMessage navi_command_message{};
			NaviControlMessage navi_control_message{};


			boost::filesystem::path waiting_for_begin{"./.waiting_for_begin"};
			spdlog::trace("{}> begin waiting for game", name);
			{
				boost::filesystem::fstream flag_file{waiting_for_begin, boost::filesystem::fstream::out};
				flag_file << "1\n";
				flag_file.flush();
				flag_file.close();
			}

#ifdef SC_MDB
			JustFloats<9> vofa_message{};
#endif

			auto last_navi_command_time = std::chrono::steady_clock::now();
			boost::system::error_code ec{};
			while (exists(waiting_for_begin, ec)) {
				std::this_thread::sleep_for(std::chrono::milliseconds{10});
				const auto now = std::chrono::steady_clock::now();

				// 发送导航目的地指令
				team = gameData.GameCode.IsMyTeamRed ? UnitTeam::Red : UnitTeam::Blue;
				navi_command_message.DestinationID = Home(team);
				naviCommandDestination.SetItem(navi_command_message);

				// 尝试获取导航速度，并设置到云台
				if (naviControlSource.GetItem(navi_control_message)) {
					gimbal_control_message.Velocity = navi_control_message.Velocity;
					last_navi_command_time = now;
				}
				else {
					if (now - last_navi_command_time > 3s) {
						gimbal_control_message.Velocity.X = 0;
						gimbal_control_message.Velocity.Y = 0;
					}
				}

#ifdef SC_MDB
				vofa_message.Elements[0] = gimbal_control_message.GimbalAngles.Pitch / 100.0f;
				vofa_message.Elements[1] = gimbal_control_message.GimbalAngles.Pitch / 100.0f;
				vofa_message.Elements[2] = gimbal_control_message.GimbalAngles.Yaw;
				vofa_message.Elements[3] = gimbal_control_message.GimbalAngles.Yaw;
				vofa_message.Elements[5] = gimbal_control_message.Velocity.X;
				vofa_message.Elements[6] = gimbal_control_message.Velocity.Y;
				vofa_message.Elements[7] = gimbal_control_message.FireCode.FireStatus;
				vofa_message.Elements[8] = navi_command_message.DestinationID;
				vofa.Utils.WriterMessagePool->SetItem(vofa_message);
#endif

				// 发送云台控制信息
				gimbalControlDestination.SetItem(gimbal_control_message);

				// 检查额外的强制中断
				if (interrupt) {
					spdlog::trace("{}> 中断等待比赛开始", name);
					break;
				}

				// 检查比赛是否开始
				if (CheckGameBegin(gameDataSource, gameData)) {
					spdlog::trace("{}> 比赛开始", name);
					break;
				}
			}
			spdlog::debug("{}> stop waiting for game", name);
		}

		/// @brief 在比赛开始之前的处理
		/// 首先等待第一个云台角度，将设置 yaw 轴为第一次接收到的云台角，pitch 为 0°
		///	接下来等待比赛开始，在等待时设置导航点为初始点，并应用导航速度
		void WaitBeforeGame(
			const std::string& name,
			UnitTeam& team,
			GimbalData& firstGimbalData,
			GameData& gameData) const {
			spdlog::trace("{}> waiting before game", name);

			auto& filter = *Filter.lock();
			auto& gimbal_data_source = *GimbalDataSource.lock();
			auto& game_data_source = *GameDataSource.lock();
			auto& gimbal_control_destination = *GimbalControlDataDestination.lock();
			auto& navi_control_source = *NaviControlMessageSource.lock();
			auto& navi_command_destination = *NaviCommandMessageDestination.lock();
			const auto& interrupt = Done;

			WaitForFirstGimbalData(name, interrupt, gimbal_data_source, firstGimbalData);
			WaitForGameBegin(
				name,
				interrupt,
				firstGimbalData.GimbalAngles.Yaw,
				team,
				navi_control_source,
				navi_command_destination,
				gimbal_control_destination,
				gameData,
				game_data_source
			);
			filter.MyTeam = gameData.GameCode.IsMyTeamRed ? UnitTeam::Red : UnitTeam::Blue;

			spdlog::debug("{}> stop waiting before game", name);
		}

#pragma endregion

#pragma region working

		static bool GetTimedPicture(
			const std::string& name,
			cv::Mat& picture,
			Cango::AsyncItemPool<cv::Mat>& timedPictureSource) {
			if (!timedPictureSource.GetItem(picture)) {
				spdlog::warn("{}> cannot get picture", name);
				return false;
			}
			return true;
		}

		static bool GetGimbalData(
			const std::string& name,
			GimbalData& gimbalData,
			Cango::AsyncItemPool<GimbalData>& gimbalDataSource) {
			if (!gimbalDataSource.GetItem(gimbalData)) {
				spdlog::warn("{}> cannot get gimbal data", name);
				return false;
			}
			return true;
		}

		static void LogNaviLocatorLocation(const Eigen::Vector3f& location) {
			spdlog::info("LocatorMessage> {} {}", location.x(), location.y());
		}

		static void SetNaviControl(
			Cango::AsyncItemPool<NaviControlMessage>& naviControlSource,
			GimbalControlData& gimbalControl) {
			NaviControlMessage navi_control_message{};
			if (!naviControlSource.GetItem(navi_control_message)) return;

			//LogNaviLocatorLocation(navi_control_message.Locator.Location);

			// 应用导航速度
			gimbalControl.Velocity = navi_control_message.Velocity;

			// 检查小陀螺状态
			const auto navi_speed_vector_length = std::sqrt(
				std::pow(static_cast<float>(navi_control_message.Velocity.X), 2) +
				std::pow(static_cast<float>(navi_control_message.Velocity.Y), 2)
			);

#ifndef SC_NAVI_SPEED_ROTATE
			if (navi_speed_vector_length > 50) { gimbalControl.FireCode.Rotate = 0; } //TODO 小陀螺速度
			else if (navi_speed_vector_length > 35) { gimbalControl.FireCode.Rotate = 1; }
			else if (navi_speed_vector_length > 15) { gimbalControl.FireCode.Rotate = 2; }
			else { gimbalControl.FireCode.Rotate = 3; }
#else
			gimbalControl.FireCode.Rotate = navi_speed_vector_length > 25 ? 0 : 3;
#endif

			/*//TODO 设置禁用小陀螺

			gimbalControl.FireCode.Rotate = 0;*/

			//if (gimbalControl.FireCode.Rotate > 0) spdlog::error("TEST");
		}

		static bool GetTarget(
			CorrectedDetector& detector,
			ArmorFilter& filter,
			PoseSolver& solver,
			const cv::Mat& frame,
			ArmorObject& targetArmor,
			ArmorTransform& targetTransform,
			PNPAimResult& aimResult,
			const bool hitOutpost,
			const bool hitSentry) {

			static std::vector<ArmorObject> detected_armors{};
			detected_armors.clear();
			if (!detector.Detect(frame, detected_armors)) return false;

			if (!filter.Filter(detected_armors, targetArmor, hitOutpost, hitSentry)) return false;

			static std::vector<cv::Point2f> pnp_points{};
			pnp_points.clear();
			std::ranges::copy(targetArmor.apex, std::back_inserter(pnp_points));

			if (!(targetArmor.IsLarge()
				      ? solver.SolveLargeArmor(pnp_points, targetTransform)
				      : solver.SolveSmallArmor(pnp_points, targetTransform)))
				return false;

			return aimResult.FromTranslation(targetTransform.Translation);
		}

		static void PredictTarget(
			const GimbalData& gimbalData,
			const ArmorTransform& targetTransform,
			const bool stableRecognized,
			const std::chrono::steady_clock::time_point& currentTime,
			GimbalControlData& gimbalControl,
			Predictor& predictor,
			std::chrono::steady_clock::time_point& lastFoundTime,
			const bool noPredict,
			bool& lock,
			std::int8_t& toLeft) {
			//static Cango::Counter fire_counter{0, 3};
			//if (fire_counter.Count())
			//{
			//	fire_counter.Reset();
			//	gimbalControl.FireCode.FlipFireStatus();
			//}

			//gimbalControl.FireCode.FlipFireStatus();

			if (!noPredict) { gimbalControl.FireCode.FlipFireStatus(); }

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

			static constexpr Angle100Type pitch_offset{-60};
			static constexpr AngleType yaw_offset{-0.3f};
			AngleType current_yaw = gimbalData.GimbalAngles.Yaw;
			current_yaw += yaw_offset;
			Angle100Type current_pitch = gimbalData.GimbalAngles.Pitch;
			current_pitch += pitch_offset;

			RadianType gimbal_yaw{ToRadian(current_yaw)};
			RadianType gimbal_pitch{ToRadian(static_cast<float>(current_pitch) / 100.0f)};

			Eigen::Matrix3d rotation_matrix3;
			rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitX());
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
				gimbal_yaw,
				gimbal_pitch,
				delta_t,
				predict_position,
				delay_s,
				noPredict);

			spdlog::info("to_left: {}", toLeft);

			AngleType target_yaw{
				predict_result.yaw +
				std::round((gimbalData.GimbalAngles.Yaw - predict_result.yaw) / 360.0f) *
				360.0f
			};

			const Angle100Type target_pitch{static_cast<std::int16_t>(predict_result.pitch * 100.0f)};

			gimbalControl.GimbalAngles.Yaw = target_yaw;
			gimbalControl.GimbalAngles.Pitch = target_pitch;

			if (noPredict) {
				static Cango::Counter16 no_move{0, 6};
				static Cango::Counter16 counter{0, 8};
				static Cango::Counter16 left_c{0, 16};
				static Cango::Counter16 right_c{0, 16};
				static auto last_target_yaw = 0.0f;
				auto delta_yaw = gimbalControl.GimbalAngles.Yaw - gimbalData.GimbalAngles.Yaw;
				auto delta_yaw2 = gimbalControl.GimbalAngles.Yaw - last_target_yaw;
				last_target_yaw = target_yaw;

				if (delta_yaw2 > 0.04f) {
					(void)left_c.Count();
					//spdlog::info("AAAA");
				}
				else if (delta_yaw2 < -0.04f) {
					(void)right_c.Count();
					//spdlog::info("BBBB");
				}

				if (left_c.IsReached()) {
					toLeft = 1;
					right_c.Reset();
				}
				else if (right_c.IsReached()) {
					toLeft = -1;
					left_c.Reset();
				}
				else { toLeft = 0; }

				const auto abs_yaw = std::abs(delta_yaw);
				if (abs_yaw < 1.0f) {
					if (abs_yaw < 0.3f) {
						(void)no_move.Count();

						if (lock) {
							left_c.Reset();
							right_c.Reset();
						}
					}
					else { no_move.Reset(); }

					if (counter.Count()) {
						spdlog::warn("enough for an armor");
						counter.Reset();
						if (!no_move.IsReached()) {
							spdlog::warn("begin lock");
							gimbalControl.GimbalAngles.Yaw += value_cfg(-2.8f, toLeft);
							lock = true;
						}
					}
				}
				else {
					counter.Reset();
					no_move.Reset();
				}
			}
		}

		static void GetDestination(
			const std::string& name,
			const GameData& gameData,
			const UnitTeam& myTeam,
			std::uint8_t& destinationID,
			std::chrono::steady_clock::time_point& lastCommandTime,
			std::chrono::seconds& commandInterval,
			bool& lowOutpost,
			const std::chrono::steady_clock::time_point& beginTime) {
			const auto current_time = std::chrono::steady_clock::now();

			constexpr auto health_bag_value{25};
			constexpr auto health_bag_low{1 / health_bag_value};

			const auto time_passed_s = std::chrono::duration_cast<std::chrono::seconds>(current_time - beginTime).count();
			constexpr auto total_seconds = 7 * 60;
			const auto before_sentry_early_time = time_passed_s > 300;	// 前期一段时间哨兵回家计时较长
			constexpr auto outpost_low_health_threshold = 240;			// 触发中场阶段的前哨血量阈值
			constexpr auto op_low_hp_bag_threshold = outpost_low_health_threshold / health_bag_value;

			static bool last_low_ammo{false};
			static bool last_low_outpost{false};
			static bool last_very_low_outpost{false};
			static auto last_recover_time = std::chrono::steady_clock::now();
			static auto last_precaution_time = std::chrono::steady_clock::now();
			const auto now = std::chrono::steady_clock::now();
			const auto time_left_s = gameData.TimeLeft;

			const bool very_low_outpost = (gameData.GameCode.SelfOutpostHealth <= health_bag_low) ;
			if (very_low_outpost && !last_very_low_outpost) {
				last_very_low_outpost = true;
				spdlog::trace("{}> triggered: very low outpost interrupt", name);
				destinationID = Home(myTeam);
				lastCommandTime = current_time;
				commandInterval = 30s;

				// 设置低前哨血量、低弹丸中断为已触发
				last_low_outpost = true;
				last_low_ammo = true;
			}

			const bool low_ammo = gameData.AmmoLeft < 30;

			//TODO 修改比赛时间记录的方式 这个裁判系统的数据会有延时

			//const bool low_outpost = gameData.GameCode.SelfOutpostHealth < op_low_hp_bag_threshold || (60 < time_left_s && time_left_s < 270) || last_low_outpost;
			const bool low_outpost = gameData.GameCode.SelfOutpostHealth < op_low_hp_bag_threshold || time_passed_s > (total_seconds - 270) || last_low_outpost;
			//const bool low_outpost = gameData.GameCode.SelfOutpostHealth < op_low_hp_bag_threshold || time_left_s < 270 || last_low_outpost;
			lowOutpost = low_outpost;
			if (low_outpost && !last_low_outpost) {
				last_low_outpost = true;
				spdlog::trace("{}> triggered: low outpost interrupt", name);
				destinationID = Outpost(myTeam);
				lastCommandTime = current_time;
				commandInterval = 20s;
			}

			// 指令时间检测，防止高速切换指令
			if (current_time - lastCommandTime < commandInterval) return;
			lastCommandTime = current_time;

			const auto self_team = myTeam;
			const auto enemy_team = myTeam == UnitTeam::Red ? UnitTeam::Blue : UnitTeam::Red;

			const bool home_guard = low_ammo || very_low_outpost;

#ifdef SAFE_MODE
			constexpr bool far_attack = false;
			const bool mid_attack = !low_ammo && !very_low_outpost;
#else
			const bool far_attack = !low_ammo && !low_outpost && !very_low_outpost;
			const bool mid_attack = !low_ammo && !very_low_outpost && low_outpost;
#endif

			if (low_ammo && !last_very_low_outpost && (now - last_recover_time > 30s)) {
				destinationID = Recovery(self_team);
				commandInterval = 20s;
				last_recover_time = now;
				return;
			}

			if (home_guard) {
				if ((low_ammo || gameData.SelfHealth < 200) && now - last_recover_time > 30s) {
					destinationID = Recovery(self_team);
					commandInterval = 15s;
					last_recover_time = now;
				}
				else if (gameData.GameCode.HeroPrecaution && now - last_precaution_time > 30s) {
					destinationID = High(self_team);
					commandInterval = 20s;
					last_precaution_time = now;
				}
				else {
					destinationID = Home(self_team);
					commandInterval = 1s;
				}
			}

			if (far_attack) {
				if (destinationID == Home(self_team)) {
					destinationID = EasyOres(enemy_team);
					commandInterval = 15s;
				}
				else if (destinationID == EasyOres(enemy_team)) {
					destinationID = Fly(enemy_team);
					commandInterval = 1s;
				}
				else if (destinationID == Fly(enemy_team)) {
					destinationID = EasyOres(enemy_team);
					commandInterval = 15s;
				}
				else {
					destinationID = Home(self_team);
					commandInterval = 1s;
				}
			}

			if (mid_attack) {
				switch (std::chrono::steady_clock::now().time_since_epoch().count() % 4) {
				case 0:
					destinationID = Outpost(enemy_team);
					commandInterval = 30s;
					break;

				case 1:
					destinationID = Outpost(self_team);
					commandInterval = 20s;
					break;

				case 2:
					destinationID = Resource(enemy_team);
					commandInterval = 30s;
					break;

				case 3:
					destinationID = Outpost(self_team);
					commandInterval = 20s;
					break;
				}
			}
		}

		static void RotateGimbal(
			const SineWave& pitchWave,
			const AngleType& gimbalYaw,
			const bool highPitch,
			const std::chrono::steady_clock::time_point& currentTime,
			const std::chrono::steady_clock::time_point& lastEnemyFoundTime,
			GimbalControlData& gimbalControl) {
			static constexpr std::chrono::seconds wait_time{2};
			static constexpr auto delta_yaw = 12.0f;

			if (const bool lost_enemy = currentTime - lastEnemyFoundTime > wait_time; !lost_enemy) return;

			if (highPitch) {
				static std::chrono::steady_clock::time_point last_rotate_time = std::chrono::steady_clock::now();
				static constexpr auto interval = 400ms;
				const auto current_time = std::chrono::steady_clock::now();
				if (current_time - last_rotate_time >= interval) {
					gimbalControl.GimbalAngles.Yaw = gimbalYaw + 30.0f;
					last_rotate_time = current_time;
				}
			}
			else { gimbalControl.GimbalAngles.Yaw = gimbalYaw + delta_yaw; }


			if (highPitch) gimbalControl.GimbalAngles.Pitch = Angle100Type{1000};
			else
				gimbalControl.GimbalAngles.Pitch = Angle100Type{
					static_cast<std::int16_t>(-1.0f + pitchWave.Produce(currentTime) * 100.0f)
				};
		}

		static void UpdateAndLogGameData(
			const std::string& name,
			GameData& gameData,
			Cango::AsyncItemPool<GameData>& gameDataSource) {
			static UWBPositionType last_uwb = gameData.UWBPosition;
			if (!gameDataSource.GetItem(gameData)) return;

			if (last_uwb.X != gameData.UWBPosition.X
				|| last_uwb.Y != gameData.UWBPosition.Y) {
				spdlog::info(
					"{}> updated: UWBData {} {}",
					name,
					gameData.UWBPosition.X,
					gameData.UWBPosition.Y
				);
			}
			last_uwb = gameData.UWBPosition;
		}


		void GameLoop(const std::string& name, GameData& gameData) const {
			auto [d_user, detector] = Cango::Acquire(Detector);
			auto [s_user, solver] = Cango::Acquire(Solver);
			auto [f_user, filter] = Cango::Acquire(Filter);
			auto [p_user, predictor] = Cango::Acquire(TargetPredictor);
			auto [ps_user, timed_picture_source] = Acquire(PictureSource);
			auto [g_user, gimbal_data_source] = Acquire(GimbalDataSource);
			auto [gd_user, game_data_source] = Acquire(GameDataSource);
			auto [gc_user, gimbal_control_destination] = Acquire(GimbalControlDataDestination);
			auto [n_user, navi_control_source] = Acquire(NaviControlMessageSource);
			auto [nc_user, navi_command_destination] = Acquire(NaviCommandMessageDestination);

			GimbalControlData gimbal_control_message{};
			{
				GimbalData gimbal_data{};
				while (!GetGimbalData(name, gimbal_data, gimbal_data_source)) { std::this_thread::sleep_for(100ms); }
				gimbal_control_message.GimbalAngles = gimbal_data.GimbalAngles;
			}

			int last_target_type{};
			const auto my_team = filter.MyTeam;
			std::chrono::steady_clock::time_point last_enemy_found_time{std::chrono::steady_clock::now()};
			std::chrono::steady_clock::time_point last_command_time{std::chrono::steady_clock::now()};
			std::chrono::seconds command_interval{1};
			SineWave pitch_wave{14.0f, 0.0f, 2500ms, std::chrono::steady_clock::now()};
			NaviCommandMessage navi_command_message{};
			navi_command_message.DestinationID = Home(my_team);
			GimbalData gimbal_data{};
			cv::Mat picture{};
			Cango::IntervalSleeper sleeper{16ms};

			constexpr auto anti_op = true;
			bool low_outpost = false;

			
			constexpr std::chrono::seconds anti_op_timeout{50};
			navi_command_message.DestinationID = anti_op ? Fly(my_team) : Home(my_team);
			command_interval = anti_op ? anti_op_timeout : 0s;
			const auto begin_time = std::chrono::steady_clock::now();
			bool lock = false;
			std::int8_t to_left{};
			std::int8_t lock_to_left{};
			AngleType lock_yaw{};
			Cango::Counter locker{0, 55};
#ifdef SC_MDB
			JustFloats<9> vofa_message{};
#endif

			ArmorObject target_armor{};
			ArmorTransform target_transform{};
			PNPAimResult aim_result{};


			while (!Done) {
				if (anti_op) {
					if (lock && locker.Current == 0 && to_left != 0) {
						lock_to_left = to_left;
						spdlog::warn("i begin locker");
						lock_yaw = gimbal_control_message.GimbalAngles.Yaw + 0.99f * (to_left == -1
							? 2.0f
							: (to_left == 1)
							? -1
							: 0);
						(void)locker.Count();
					}

					if (locker.Current > 0) {
						if (locker.Current > 18 && locker.Current < 45) { gimbal_control_message.FireCode.FlipFireStatus(); }

						spdlog::warn("continue locker({})", (int)lock_to_left);
						const auto yaw_feed = locker.Current * locker.Current * (lock_to_left == 1 ? 0.003f : 0.003f);
						spdlog::info("feed: {}", yaw_feed);
						gimbal_control_message.GimbalAngles.Yaw = lock_yaw + value_cfg(yaw_feed, lock_to_left);

						if (locker.Count()) {
							locker.Reset();
							lock = false;
						}
					}
				}

				const auto current_time = std::chrono::steady_clock::now();
				bool at_beginning = (current_time - begin_time < anti_op_timeout) && gameData.GameCode.EnemyOutpostHealth > 0;
				if (at_beginning) gimbal_control_message.FireCode.Rotate = 0;
				//gimbal_control_message.FireCode.FireStatus = 0;  //TODO 禁用开火

				/*spdlog::info("pitch: {}", gimbal_data.GimbalAngles.Pitch);*/

#ifdef SC_MDB
				vofa_message.Elements[0] = gimbal_data.GimbalAngles.Pitch / 100.0f;
				vofa_message.Elements[1] = gimbal_control_message.GimbalAngles.Pitch / 100.0f;
				vofa_message.Elements[2] = gimbal_data.GimbalAngles.Yaw;
				vofa_message.Elements[3] = gimbal_control_message.GimbalAngles.Yaw;
				vofa_message.Elements[4] = gimbal_control_message.GimbalAngles.Yaw - gimbal_data.GimbalAngles.Yaw; 
				vofa_message.Elements[5] = gimbal_control_message.Velocity.X;
				vofa_message.Elements[6] = gimbal_control_message.Velocity.Y;
				vofa_message.Elements[7] = gimbal_control_message.FireCode.FireStatus;
				vofa_message.Elements[8] = navi_command_message.DestinationID;
				vofa.Utils.WriterMessagePool->SetItem(vofa_message);
#endif

				gimbal_control_destination.SetItem(gimbal_control_message);

				static Cango::CallRateCounter32 rate_counter{};
				static std::chrono::steady_clock::time_point last_rate_time{};
				const auto rate = rate_counter.Call();
				if (current_time - last_rate_time > 2s) {
					spdlog::info("rating: {}", rate);
					last_rate_time = current_time;
				}

				sleeper.Sleep();

				if (!GetTimedPicture(name, picture, timed_picture_source)) continue;
				GetGimbalData(name, gimbal_data, gimbal_data_source);

				UpdateAndLogGameData(name, gameData, game_data_source);

				if (!anti_op || !at_beginning)
					GetDestination(
						name,
						gameData,
						my_team,
						navi_command_message.DestinationID,
						last_command_time,
						command_interval,
						low_outpost, 
						begin_time);
				navi_command_destination.SetItem(navi_command_message);

				SetNaviControl(navi_control_source, gimbal_control_message);

				RotateGimbal(
					pitch_wave,
					gimbal_data.GimbalAngles.Yaw,
					anti_op && at_beginning,
					current_time,
					last_enemy_found_time,
					gimbal_control_message
				);

				if (!GetTarget(
					detector,
					filter,
					solver,
					picture,
					target_armor,
					target_transform,
					aim_result,
					anti_op && at_beginning,
					gameData.GameCode.EnemyOutpostHealth == 0))
					continue;


				bool stable_recognized = last_target_type == target_armor.type;
				last_target_type = target_armor.type;
				spdlog::info("type: {}", last_target_type);

				const bool hit_outpost = anti_op && at_beginning && (last_target_type == 0 || last_target_type == 7);
				PredictTarget(
					gimbal_data,
					target_transform,
					stable_recognized,
					current_time,
					gimbal_control_message,
					predictor,
					last_enemy_found_time,
					hit_outpost,
					lock,
					to_left
				);
			}
		}

#ifdef SC_MDB
		inline static Cango::EasyCangoUDPSocketRWerCommunicationTaskCheatsheet<JustFloats<9>, JustFloats<9>> vofa{};
#endif

#pragma endregion

	public:
		static constexpr char Name[] = "MainService";

		Cango::Credential<CorrectedDetector> Detector{};
		Cango::Credential<ArmorFilter> Filter{};
		Cango::Credential<PoseSolver> Solver{};
		Cango::Credential<Predictor> TargetPredictor{};
		Cango::Credential<Cango::AsyncItemPool<GimbalData>> GimbalDataSource{};
		Cango::Credential<Cango::AsyncItemPool<GameData>> GameDataSource{};
		Cango::Credential<Cango::AsyncItemPool<GimbalControlData>> GimbalControlDataDestination{};
		Cango::Credential<Cango::AsyncItemPool<NaviCommandMessage>> NaviCommandMessageDestination{};
		Cango::Credential<Cango::AsyncItemPool<NaviControlMessage>> NaviControlMessageSource{};
		Cango::Credential<Cango::AsyncItemPool<cv::Mat>> PictureSource{};
		Cango::EasyDeliveryTaskMonitor DoneStatus{};

		std::atomic_bool Done{false};

		[[nodiscard]] bool IsFunctional() const noexcept {
			return ValidateAll(
				Detector,
				Filter,
				Solver,
				TargetPredictor,
				GimbalDataSource,
				GameDataSource,
				GimbalControlDataDestination,
				NaviCommandMessageDestination,
				NaviControlMessageSource,
				PictureSource);
		}

		void Execute() const noexcept {
			const std::string name{Name};
			GimbalData first_gimbal_data{};
			GameData game_data{};

#ifdef SC_MDB
			{
				auto&& c = vofa.Provider->Configure();
				c.Options.LocalEndpoint = boost::asio::ip::udp::endpoint{
					boost::asio::ip::make_address("0.0.0.0"), 8982
				};
				c.Options.RemoteEndpoint = boost::asio::ip::udp::endpoint{
					boost::asio::ip::make_address("192.168.137.1"), 8983
				};

				auto&& t = vofa.Task.Configure();
				t.Options.ReaderMinInterval = 10s;
				t.Options.WriterMinInterval = 10ms;

				auto& utils = vofa.Utils;
				utils.WriterMonitor->ExceptionHandler = Cango::EasyDeliveryTaskMonitor::EmptyHandler;
				utils.WriterMonitor->NormalHandler = Cango::EasyDeliveryTaskMonitor::EmptyHandler;
			}
			std::thread vofa_thread{[] { vofa.Task.Execute(); }};
			vofa_thread.detach();
#endif

			WaitBeforeGame(name, Filter.lock()->MyTeam, first_gimbal_data, game_data);
			GameLoop(name, game_data);
		}

		[[nodiscard]] bool IsDone() const noexcept { return DoneStatus.IsDone(); }
		void Interrupt() noexcept { DoneStatus.Interrupt(); }
		void Reset() noexcept { DoneStatus.Reset(); }
	};
}
