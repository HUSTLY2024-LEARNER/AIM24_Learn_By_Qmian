#include <LangYa/AutoAim/Predictor.hpp>

namespace LangYa:: inline AutoAim {
	constexpr auto GRAVITY = 9.79338;

	Predictor::Predictor() {
		vehicle_tracker = new VehicleTracking();
		last_pose_vec = {0, 0, 0};
		EKFilter.UseEKF = &UseEKF;
	}

	Predictor::~Predictor() { delete vehicle_tracker; }

	//        NOT_GET_TARGET = 0,       // 未获得目标
	//    CONTINOUS_GET_TARGET = 1, // 连续获得目标
	//    LOST_BUMP = 2,            // 缓冲阶段
	//    DETECT_BUMP = 3           // 进入连续识别状态的缓冲

	PredictedAngleType Predictor::Predict(
		const Sophus::SE3<double>& armor_pose,
		const Sophus::SE3<double>& armor_pose_sec, bool is_get_second_armor,
		int detect_mode, float gimbal_yaw, float gimbal_pitch, float frame_delta_t,
		cv::Point3f& predict_translation, float& test_delay, bool no_predict) {
		static float t_raw = 0.0f;
		static float t_predict = 0.0f;

		// 先根据当前云台pitch和目标距离解算大致击打时间
		const float pitch_gimbal = gimbal_pitch * 3.14159f / 180.0f;
		const float yaw_gimbal = gimbal_yaw * 3.14159f / 180.0f;

		constexpr auto shoot_delay = 0.10f;

		setShootSpeed(0, 27.2f);
		shootAngleTime_now = ballistic_equation(pitch_gimbal, armor_pose.translation());
		const auto shoot_now = shootAngleTime_now;
		if (no_predict) return shoot_now;
		//return shootAngleTime_now;

		const float shootTime = shootAngleTime_now.time / 1000;

		t_raw += frame_delta_t / 1000.0f;
		t_predict = t_raw + shootTime / 1000 + shoot_delay;

		Eigen::Vector3d predict_point;
		Eigen::Vector3d predict_speed;
		Eigen::Vector3d filte_point;

		Sophus::SE3<double> tracked_armor_pose;
		if (is_get_second_armor) {
			tracked_armor_pose = ((armor_pose.translation() - last_pose_vec).norm() < (armor_pose_sec.translation() -
				                     last_pose_vec).norm())
				                     ? armor_pose
				                     : armor_pose_sec;
		}
		else { tracked_armor_pose = armor_pose; }

		// EKF球面模型测试
		filte_point = EKFilter.runKalman(tracked_armor_pose.translation(), frame_delta_t);

		last_pose_vec = filte_point;

		predict_point = EKFilter.predict(shootTime + shoot_delay);
		double chi_value = EKFilter.detect_param;
		test_delay = shootTime + shoot_delay;

		predict_speed = EKFilter.getSpeed();
		Eigen::Vector3d pyd_pre = EKFilter.getPYD();

		const auto prediction1 = ballistic_equation(pitch_gimbal, predict_point);


		// 整车预测解算 // LOST_BUMP状态，进行无量测更新 // CONTINOUS_GET_TARGET状态，正常更新

		predict_hit_point = vehicle_tracker->predictVehicleState(armor_pose, armor_pose_sec, is_get_second_armor,
		                                                         detect_mode, shootTime, frame_delta_t, yaw_gimbal);
		const auto prediction2 = ballistic_equation(pitch_gimbal, predict_hit_point);

		/*spdlog::info("vt: 0({}) 2({})",
			vehicle_tracker->vehicle_state(3),
			vehicle_tracker->vehicle_state(4),
			vehicle_tracker->vehicle_state(5),
			vehicle_tracker->vehicle_state(6),
			vehicle_tracker->vehicle_state(7),
			vehicle_tracker->vehicle_state(8)
		);*/

		predict_translation.x = predict_hit_point.x();
		predict_translation.y = predict_hit_point.y();
		predict_translation.z = predict_hit_point.z();

		if (no_predict || UseEKF) { spdlog::debug("Predictor> tracking by movement"); }
		else { spdlog::warn("Predictor> tracking by rotation"); }

		const auto param = EKFilter.detect_param;
		if (param > 4.0)
			spdlog::debug("Predictor> detect param({}) > 4.0", param);
		else
			spdlog::debug("Predictor> detect param({}) < 4.0", param);

		return no_predict ? shoot_now : (UseEKF ? prediction1 : prediction2);
	}

	void Predictor::resetPredictor() {
		EKFilter.resetKalman();
		vehicle_tracker->resetTracker();
	}

	// 根据物理方程来计算设定pitch和yaw
	PredictedAngleType Predictor::ballistic_equation(float gim_pitch, const Eigen::Vector3d& armor_Position) {
		PredictedAngleType shootAngleTime_;
		// 先计算yaw轴角度
		shootAngleTime_.yaw = -atan2(armor_Position[0], armor_Position[1]);
		shootAngleTime_.distance = std::sqrt(
			static_cast<float>(armor_Position[0] * armor_Position[0] + armor_Position[1] * armor_Position[1]));
		// armor 的位置进行了一定的旋转
		auto armor_new_position = Eigen::Vector3d(0, shootAngleTime_.distance, armor_Position[2]);
		// 计算pitch轴的初始角度
		shootAngleTime_.pitch = atan2(armor_new_position[2], armor_new_position[1]);

		float err = 100;

		float y_temp = armor_new_position[2];
		float dy, a, y_actual;
		for (int i = 0; i < 100; i++) {
			a = atan2(y_temp, shootAngleTime_.distance);
			y_actual = BulletModel(shootAngleTime_.distance, ShootSpeed, a);
			dy = armor_new_position[2] - y_actual;
			y_temp = y_temp + dy;
			if (fabs(dy) < 0.001) { break; }
		}

		shootAngleTime_.pitch = atan2(y_temp, shootAngleTime_.distance);

		shootAngleTime_.time = abs(shootAngleTime_.distance / (ShootSpeed * cos(shootAngleTime_.pitch)) * 1000);

		shootAngleTime_.pitch = (shootAngleTime_.pitch) / 3.1415926 * 180.0;

		shootAngleTime_.yaw = shootAngleTime_.yaw / 3.1415926 * 180.0f;

		return shootAngleTime_;
	}

	float Predictor::BulletModel(float x, float v, float angle) {
		constexpr auto k_wind = 0.0001f;

		float init_k_ = k_wind;
		float t, y;
		t = (exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)); // k系数等待测量
		y = static_cast<float>(v * sin(angle) * t - GRAVITY * t * t / 2);
		return y;
	}

	void Predictor::setShootSpeed(const char& flag, const float& shoot_speed) {
		/*switch (flag)
		{
		case 0:
		    ShootSpeed = 13.5f;
		    break;
		case 1:
		    ShootSpeed = 17.0f;
		    break;
		case 2:
		    ShootSpeed = 27.0f;
		    break;
		default:
		    ShootSpeed = 27.0f;
		    break;
		}*/
		ShootSpeed = shoot_speed;
	}

	float Predictor::calShootTime(const Eigen::Vector3d& armor_trans, float gimbal_pitch) // 根据锁给的三维坐标直接计算打击时间
	{
		float pitch_now = gimbal_pitch * 3.14159f / 180.0f;
		shootAngleTime_now = ballistic_equation(pitch_now, armor_trans); // 当前实际位置击打所需时间
		float shootTime = shootAngleTime_now.time / 1000;                // 需要测试各种延时(s)
		return shootTime;
	}
}
