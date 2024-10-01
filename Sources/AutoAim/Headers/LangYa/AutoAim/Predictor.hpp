#pragma once

#include <LangYa/AutoAim/NormalEKF.hpp>
#include <LangYa/AutoAim/VehicleTracking.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <vector>
#include <spdlog/spdlog.h>

namespace LangYa:: inline AutoAim
{
    struct PredictedAngleType
    {
    	float pitch; // rad
		float yaw;   // rad
		float time;  // 击打弹道时间(ms)
		float distance;
	};

    struct Predictor
    {
        bool UseEKF = false;

        Predictor();
        ~Predictor();
        PredictedAngleType Predict(const Sophus::SE3<double> &armor_pose, 
            const Sophus::SE3<double> &armor_pose_sec, 
            bool is_get_second_armor, int detect_mode, 
            float gimbal_yaw, float gimbal_pitch, 
            float frame_delta_t,
            cv::Point3f& predict_translation, 
            float& test_delay,
            bool no_predict
        );

    	void resetPredictor();
        std::vector<Eigen::Vector3d> getArmorSerial() { return vehicle_tracker->getArmorSerial(); }
        Eigen::Vector3d getPredictPoint() { return predict_hit_point; }
        bool getArmorSwitch() { return vehicle_tracker->armor_switch; }
        float calShootTime(const Eigen::Vector3d &armor_trans, float gimbal_pitch);

    private:
        void setShootSpeed(const char &flag, const float &shoot_speed);
        PredictedAngleType ballistic_equation(float gim_pitch, const Eigen::Vector3d &armor_Position);
        float BulletModel(float x, float v, float angle);
        float ShootSpeed;
        PredictedAngleType shootAngleTime_now;
        PredictedAngleType shootAngleTime_pre;
        VehicleTracking *vehicle_tracker;

        Eigen::Vector3d predict_hit_point;

        NormalEKF EKFilter{};

        Eigen::Vector3d last_pose_vec;
    };
}
