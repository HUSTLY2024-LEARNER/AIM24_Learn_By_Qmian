#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <LangYa/AutoAim/ExtendedKalman.hpp>
#include <vector>

namespace LangYa:: inline AutoAim
{
    struct VehicleTracking
    {
        // 状态量如下 x_c, v_x, y_c, v_y, z_c, v_z, yaw, v_yaw, r
        static constexpr auto VehicleStateDimension = 9;

        // 测量量如下 x, y, z, yaw
        static constexpr auto VehicleMeasurementDimension = 4;


        VehicleTracking();
        ~VehicleTracking();
        Eigen::Vector3d predictVehicleState(const Sophus::SE3<double> &armor_pose, const Sophus::SE3<double> &armor_pose_sec, bool is_get_second_armor, const int &detect_mode, float shoot_time, float frame_delta_t, float yaw_gimbal);
        void resetTracker();
        std::vector<Eigen::Vector3d> getArmorSerial() { return armor_serial; }
        std::vector<float> speed_vector = std::vector<float>(3);
        bool armor_switch = false;

        Eigen::Vector<double, VehicleStateDimension> vehicle_state;

    private:
        ExtendedKalman<double, VehicleStateDimension, VehicleMeasurementDimension> *extended_kalman_filter;

        void setUpdateTime(const double &delta_t);

        Eigen::Vector3d getPredictPoint(const Eigen::Matrix<double, 9, 1>& state, const float &shootTime, float yawAngle);

        Eigen::Vector4d PoseToMeasurement(const Sophus::SE3<double> &pose);

        void setQandRMatrix() const;

        void getVehicleState(Eigen::Vector4d &measure);

        static Eigen::Vector3d getArmorPositionFromState(Eigen::Matrix<double, 9, 1>);

        void handleArmorJump(Eigen::Vector4d &measure);

        void rebootKalman(bool, Eigen::Vector4d, Eigen::Vector4d);

        void setTransitionMatrix() const;
        void setObservationMatrix() const;

        double update_time;
        bool is_kalman_init;

        std::vector<Eigen::Vector3d> armor_serial;
        float yaw_send;
        double last_z, last_r, last_yaw;
    };

}
