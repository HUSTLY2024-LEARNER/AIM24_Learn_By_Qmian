#include <LangYa/AutoAim/NormalEKF.hpp>
#include <spdlog/spdlog.h>
#include <cmath>

#include "Cango/CommonUtils/CounterX.hpp"

namespace LangYa:: inline AutoAim
{
    NormalEKF::NormalEKF()
    {
        is_kalman_init = false;
        posteriori_pose = Eigen::Vector3d::Zero();
        posteriori_speed = Eigen::Vector3d::Zero();
        ProcessNoise = Eigen::Matrix3d::Identity();
        process_noise_matrix = Eigen::Matrix<double, 6, 3>::Zero();
    }

    void NormalEKF::rebootKalman(const Eigen::Vector3d &new_armor_pose)
    {
        for (int i = 0; i < 3; i++)
        {
            KalmanFilter.posteriori_state_estimate[i * 2] = new_armor_pose[i];
            KalmanFilter.posteriori_state_estimate[i * 2 + 1] = 0;

            posteriori_pose[i] = KalmanFilter.posteriori_state_estimate[i * 2];
            posteriori_speed[i] = KalmanFilter.posteriori_state_estimate[i * 2 + 1];
        }
        KalmanFilter.error_cov_post = Eigen::Matrix<double, 6, 6>::Identity();

        resetTransitionMatrix();
    }
    void NormalEKF::resetTransitionMatrix()
    {
        // x,x_v,y,y_v,z,z_v
        KalmanFilter.transition_matrix = Eigen::Matrix<double, 6, 6>::Identity();
    }
    void NormalEKF::setUpdateTime(const double &delta_t)
    {
        if (std::fabs(static_cast<float>(delta_t)) < 1e-4) // 防止时间差为0
        {
            update_time = 8.0 / 1000.0;
        }
        else
        {
            update_time = delta_t / 1000.0;
        }
    }

    Eigen::Vector3d NormalEKF::runKalman(const Eigen::Vector3d &new_armor_pose, const double &delta_t) // 量测有效更新
    {
        if (!is_kalman_init)
        {
            // set signal values
            is_kalman_init = true;

            // reset kalman
            rebootKalman(new_armor_pose);

            // return values
            return new_armor_pose;
        }
        else
        {
            // set update time
            setUpdateTime(delta_t);

            // update transition matrix
            setTransitionMatrix();

            return correct(new_armor_pose);
        }
    }

	void NormalEKF::setTransitionMatrix()
    {
        Eigen::Matrix2d transition;
        transition << 1, update_time, 0, 1;
        for (int i = 0; i < 3; i++)
        {
            KalmanFilter.transition_matrix.block<2, 2>(i * 2, i * 2) = transition;
        }
    }

    Eigen::Vector3d NormalEKF::correct(const Eigen::Vector3d &armor_pose)
    {
        setProcessNoise();

        setMeasurementNoise(armor_pose); // 设置测量噪声
        Eigen::Vector3d pyd = measure(armor_pose);

        // std::fstream ss("../pyd.txt", std::ios::app);
        // ss << pyd << std::endl;
        // ss.close();
        // std::cout << "update_time:" << update_time << std::endl;
        
        static std::chrono::steady_clock::time_point last_error_time{};


        if (update_time > 0.3) // 大于0.2s没有观测到数据，选择重启卡尔曼滤波
        {
            spdlog::warn("NormalEKF> reset due to time");
            rebootKalman(armor_pose);
			last_error_time = std::chrono::steady_clock::now() - std::chrono::seconds{2};
            if (UseEKF != nullptr) *UseEKF = true;
            return armor_pose;
        }

        KalmanFilter.predict(xyz_to_pyd, pyd); // 量测有效更新

        // 检验

        detect_param = KalmanFilter.ChiSquaredTest();

        /********************反小陀螺状态检测开始*******************/

        const auto current_time = std::chrono::steady_clock::now();
        
        if (detect_param > VERIFY_THRESH) // 检验失败
        {
            spdlog::warn("NormalEKF> reset due to verify");
            rebootKalman(armor_pose);
        	last_error_time = current_time;
            return armor_pose;
        }
        

        if (UseEKF != nullptr) *UseEKF = current_time - last_error_time > std::chrono::seconds{1};

        KalmanFilter.update();

        for (int i = 0; i < 3; i++)
        {
            // update armor status and return
            posteriori_pose[i] = KalmanFilter.posteriori_state_estimate[i * 2];
            posteriori_speed[i] = KalmanFilter.posteriori_state_estimate[i * 2 + 1];
        }
        return posteriori_pose;
    }

    Eigen::Vector3d NormalEKF::measure(const Eigen::Vector3d &armor_pose)
    {
        pyd[2] = armor_pose.norm();
        pyd[0] = ceres::atan2(armor_pose[2], std::sqrt(armor_pose[0] * armor_pose[0] + armor_pose[1] * armor_pose[1])); // pitch
        pyd[1] = ceres::atan2(armor_pose[0], armor_pose[1]);
        return pyd;
    }
    void NormalEKF::setMeasurementNoise(const Eigen::Vector3d &armor_pose)
    {
        // pitch,yaw,distance的噪声
        double measurement_noise_pose_pitch = 0.01;
        double measurement_noise_pose_yaw = 0.01;

        double distance = armor_pose.norm();
        double measurement_noise_pose_distance;

        if (distance < 1.5) // 统计方法计算，分段线性
        {
            measurement_noise_pose_distance = std::pow(distance * 0.01, 2);
        }
        else if (distance < 4.5)
        {
            measurement_noise_pose_distance = std::pow(0.015 + 0.058 * (distance - 1.5), 2);
        }
        else
        {
            measurement_noise_pose_distance = std::pow(0.189 + 0.03 * (distance - 4.5), 2);
        }

        KalmanFilter.measurement_noise_cov.diagonal() << measurement_noise_pose_pitch,
            measurement_noise_pose_yaw,
            measurement_noise_pose_distance; // 3个轴的测量噪声，感觉三个轴的噪声需要根据PNP距离来计算
    }

    void NormalEKF::setProcessNoise()
    {
        Eigen::Matrix<double, 2, 1> process_noice_vec;
        process_noice_vec << 0.5 * update_time * update_time, update_time;
        for (int i = 0; i < 3; i++)
        {
            process_noise_matrix.block<2, 1>(2 * i, i) = process_noice_vec;
        }

        constexpr auto process_noise_pose_x = 100;
        constexpr auto process_noise_pose_y = 100;
        constexpr auto process_noise_pose_z = 20;

        ProcessNoise.diagonal() << 
            process_noise_pose_x,
            process_noise_pose_y,
            process_noise_pose_z; // 3个轴的过程噪声

        KalmanFilter.process_noise_cov = process_noise_matrix * ProcessNoise * process_noise_matrix.transpose();
    }
    void NormalEKF::setProcessNoise(const double x, const double y, const double z)
    {
        Eigen::Matrix<double, 2, 1> process_noise_vec;
        process_noise_vec << 0.5 * update_time * update_time, update_time;
        for (int i = 0; i < 3; i++)
        {
            process_noise_matrix.block<2, 1>(2 * i, i) = process_noise_vec;
        }
        ProcessNoise.diagonal() << x, y, z;
    	KalmanFilter.process_noise_cov = process_noise_matrix * ProcessNoise * process_noise_matrix.transpose();
    }
    Eigen::Vector3d NormalEKF::predict(const double &predict_t)
    {
        return posteriori_pose + posteriori_speed * predict_t;
    }

}
