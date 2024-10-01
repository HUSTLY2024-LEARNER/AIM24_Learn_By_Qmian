#include <LangYa/AutoAim/VehicleTracking.hpp>
namespace LangYa:: inline AutoAim
{

#define PI 3.1415926

    VehicleTracking::VehicleTracking()
    {
        extended_kalman_filter = new ExtendedKalman<double, 9, 4>();
        is_kalman_init = false;
        setObservationMatrix();
    }

    VehicleTracking::~VehicleTracking()
    {
        delete extended_kalman_filter;
    }

    Eigen::Vector3d VehicleTracking::predictVehicleState(const Sophus::SE3<double> &armor_pose, const Sophus::SE3<double> &armor_pose_sec, bool is_get_second_armor, const int &detect_mode, float shoot_time, float frame_delta_t, float yaw_gimbal)
    {
    	armor_switch = false;

        auto measure = PoseToMeasurement(armor_pose);

        Eigen::Vector4d measure_sec;
        if (is_get_second_armor)
        {
            measure_sec = PoseToMeasurement(armor_pose_sec);
        }
        yaw_send = static_cast<float>(measure[3]);

        setUpdateTime(frame_delta_t);
        if (!is_kalman_init)
        {
            is_kalman_init = true;
            rebootKalman(is_get_second_armor, measure, measure_sec);
            setQandRMatrix();
        }
        else
        {
            setTransitionMatrix();
            setQandRMatrix();
            const auto prediction = extended_kalman_filter->predict();
            vehicle_state = prediction;

            if (detect_mode != 2)
            {
	            Eigen::Vector4d tracked_measure;
	            double min_position_diff;
                if (is_get_second_armor)
                {
	                const Eigen::Vector3d predicted_position = getArmorPositionFromState(prediction);
	                const Eigen::Vector3d position_vec_1(measure[0], measure[1], measure[2]);
	                const Eigen::Vector3d position_vec_2(measure_sec[0], measure_sec[1], measure_sec[2]);
	                const double position_diff_1 = (position_vec_1 - predicted_position).norm();
	                const double position_diff_2 = (position_vec_2 - predicted_position).norm();

                    tracked_measure = position_diff_1 < position_diff_2 ? measure : measure_sec;
                    min_position_diff = position_diff_1 < position_diff_2 ? position_diff_1 : position_diff_2;
                }
                else
                {
                    tracked_measure = measure;
                    const Eigen::Vector3d predicted_position = getArmorPositionFromState(prediction);
                    const Eigen::Vector3d position_vec_1(measure[0], measure[1], measure[2]);
                    min_position_diff = (position_vec_1 - predicted_position).norm();
                }

                if (min_position_diff < 0.2) // matched
                {
                    vehicle_state = extended_kalman_filter->update(tracked_measure);
                }
                else
                {
                    handleArmorJump(tracked_measure);
                }
            }
            if (vehicle_state(8) < 0.2)
            {
                vehicle_state(8) = 0.2;
                extended_kalman_filter->post_state = vehicle_state;
            }
        }

        return getPredictPoint(vehicle_state, shoot_time, yaw_gimbal);
    }

    void VehicleTracking::getVehicleState(Eigen::Vector4d &measure)
    {
        last_yaw = 0;
        const double x_a = measure[0];
        const double y_a = measure[1];
        const double z_a = measure[2];
        const double yaw = measure[3];

        constexpr double r = 0.2;
        const double x_c = x_a + r * std::sin(yaw);
        const double y_c = y_a + r * std::cos(yaw);
        const double z_c = z_a;
        last_z = z_c; last_r = r;

        vehicle_state << x_c, 0, y_c, 0, z_c, 0, yaw, 0, r;
    }

    Eigen::Vector3d VehicleTracking::getArmorPositionFromState(Eigen::Matrix<double, 9, 1> x)
    {
	    const auto xc = x(0);
	    const auto yc = x(2);
        const auto zc = x(4);
	    const auto yaw = x(6);
	    const auto r = x(8);
        const auto xa = xc - r * std::sin(yaw);
        const auto ya = yc - r * std::cos(yaw);
        return {xa, ya, zc};
    }

    void VehicleTracking::resetTracker()
    {
        is_kalman_init = false;
    }

    void VehicleTracking::rebootKalman(const bool is_get_sec, Eigen::Vector4d armor_1, Eigen::Vector4d armor_2)
    {
        if (is_get_sec)
        {
            if (armor_1[0] * armor_1[0] + armor_1[1] * armor_1[1] >= armor_2[0] * armor_2[0] + armor_2[1] * armor_2[1])
            {
                getVehicleState(armor_1);
            }
            else
            {
                getVehicleState(armor_2);
            }
        }
        else
        {
            getVehicleState(armor_1);
        }
        extended_kalman_filter->post_state = vehicle_state;
        extended_kalman_filter->P_post = Eigen::Matrix<double, 9, 9>::Identity();
    }

    void VehicleTracking::handleArmorJump(Eigen::Vector4d &measure)
    {
	    const double _last_yaw = vehicle_state(6);
	    const double yaw = measure[3];

        if (abs(measure[3] - _last_yaw) > 0.2)
        {
            last_z = vehicle_state(4);
            vehicle_state(4) = measure[2];
            vehicle_state(6) = measure[3];
            std::swap(vehicle_state(8), last_r);

            armor_switch = true;
        }

	    const Eigen::Vector3d current_position(measure[0], measure[1], measure[2]);
	    const Eigen::Vector3d infer_position = getArmorPositionFromState(vehicle_state);

        if ((current_position - infer_position).norm() > 0.2)
        {
	        const double r = vehicle_state(8);
            vehicle_state(0) = measure[0] + r * sin(yaw);
            vehicle_state(2) = measure[1] + r * cos(yaw);
            vehicle_state(1) = 0;
            vehicle_state(3) = 0;
        }

        extended_kalman_filter->post_state = vehicle_state;
    }

    Eigen::Vector3d VehicleTracking::getPredictPoint(const Eigen::Matrix<double, 9, 1>& state, const float &shootTime, const float yawAngle)
    {
        constexpr auto shoot_delay = 0.10f;

        const float shoot_t_ = shootTime + shoot_delay;
        std::vector<Eigen::Vector4d> armor_serial_with_angle;
        armor_serial.clear();
        bool use_1 = true;
        const double yaw = state(6);
        const double r1 = state(8);
    	const double r2 = last_r;
        const double x_c = state(0);
    	const double y_c = state(2);
    	const double z_c = state(4);
    	const double z_2 = last_z;
        const double  v_x = state(1);
    	const double v_y = state(3);
    	const double v_yaw = state(7);
        const double  x_c_pre = x_c + v_x * shoot_t_;
    	const double y_c_pre = y_c + v_y * shoot_t_;
    	const double yaw_pre = yaw + v_yaw * shoot_t_;

        for (int i = 0; i < 4; i++)
        {
            Eigen::Vector3d p;
            double temp_yaw = yaw_pre + i * PI / 2;
            const double r = use_1 ? r1 : r2;

            double x = x_c_pre - r * sin(temp_yaw);
            double y = y_c_pre - r * cos(temp_yaw);
            double z = use_1 ? z_c : z_2;

            p << x, y, z;
            armor_serial.push_back(p);
            armor_serial_with_angle.emplace_back(x, y, z, temp_yaw);
            use_1 = !use_1;
        }

        speed_vector.clear();
        speed_vector.push_back(static_cast<float>(state[1]));
        speed_vector.push_back(static_cast<float>(state[3]));
        speed_vector.push_back(static_cast<float>(state[7]));
        speed_vector.push_back(shootTime);

        std::vector<Eigen::Vector3d> armor_serial_temp = armor_serial;
        const int region = static_cast<int>(std::floor(yawAngle / (2.0f * PI)));
        float yaw_gimbal_norm = static_cast<float>(yawAngle - region * 2 * PI);
        std::ranges::sort(
            armor_serial_with_angle, 
            [&yaw_gimbal_norm](const Eigen::Vector4d &a, const Eigen::Vector4d &b)
            {
	            return std::abs(a[3] + yaw_gimbal_norm) < std::abs(b[3] + yaw_gimbal_norm);
            });
        Eigen::Vector3d armor_to_hit{armor_serial_with_angle[0][0], armor_serial_with_angle[0][1], armor_serial_with_angle[0][2]};
        return armor_to_hit;
    }

    Eigen::Vector4d VehicleTracking::PoseToMeasurement(const Sophus::SE3<double> &pose)
    {
        // 从 pose 中提取 x, y, z, yaw

        const double x = pose.translation().x();
        const double y = pose.translation().y();
        const double z = pose.translation().z();

        // 计算 yaw
        // @磊子哥 : 这里能减少奇异性 （吗？）
        const Eigen::Matrix3d r = pose.so3().matrix().inverse();   
        const Eigen::JacobiSVD svd(r, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix3d r_new = svd.matrixU() * svd.matrixV().transpose();
        Eigen::Vector3d eular_angles = r_new.eulerAngles(2, 1, 0);  //TODO 移除弃用的函数
        const double yaw = eular_angles[0];

        // 留存 yaw 作为 last_yaw，为预测做准备
        last_yaw = yaw;

        return Eigen::Vector4d{x, y, z, yaw};
    }

    void VehicleTracking::setUpdateTime(const double &delta_t)
    {
        // 时间差至少为 5 ms
        const bool is_interval_reasonable = std::fabs(delta_t) < 5e-3;
        update_time = is_interval_reasonable ? 5.0 / 1000.0 : delta_t / 1000.0;
    }

    void VehicleTracking::setTransitionMatrix() const
    {
        extended_kalman_filter->post_state[8] = extended_kalman_filter->post_state[8] < 0.2 ?
            0.2 :
    		extended_kalman_filter->post_state[8] > 0.4 ?
		    0.4 : extended_kalman_filter->post_state[8];
        auto f = [this](const Eigen::VectorXd &x)
        {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * update_time;
            x_new(2) += x(3) * update_time;
            x_new(4) += x(5) * update_time;
            x_new(6) += x(7) * update_time;
            return x_new;
        };

        // f_jet
        auto j_f = [this](const Eigen::VectorXd &)
        {
            Eigen::MatrixXd f(9, 9);
            f << 1, update_time, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, update_time, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 1, update_time, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, update_time, 0,
                0, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 1;
            return f;
        };

        extended_kalman_filter->transition_func = f;
        extended_kalman_filter->transition_func_jet = j_f;
    }

    void VehicleTracking::setObservationMatrix() const
    {
        // h
        auto h = [](const Eigen::VectorXd &x)
        {
            Eigen::VectorXd z(4);
            const auto xc = x(0);
        	const auto yc = x(2);
        	const auto yaw = x(6);
        	const auto r = x(8);
            z(0) = xc - r * sin(yaw); // x
            z(1) = yc - r * cos(yaw); // y
            z(2) = x(4);              // z
            z(3) = x(6);              // yaw
            return z;
        };

        // h_jet
        auto j_h = [](const Eigen::VectorXd &x)
        {
            const auto yaw = x(6);
        	const auto r = x(8);
            //  (xc   v_x   y_c   v_y  z_c   v_z   yaw  v_yaw  r)
            Eigen::MatrixXd _h(4, 9);
            _h << 1, 0, 0, 0, 0, 0, -r * cos(yaw), 0, -sin(yaw),
                0, 0, 1, 0, 0, 0, r * sin(yaw), 0, -cos(yaw),
                0, 0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0;
            return _h;
        };

        extended_kalman_filter->observe_func = h;
        extended_kalman_filter->observe_func_jet = j_h;
    }

    void VehicleTracking::setQandRMatrix() const
    {
        Eigen::Matrix<double, 2, 1> process_noise_vec;
        process_noise_vec << 2 * update_time * update_time, update_time;
        Eigen::Matrix<double, 6, 3> process_noise_matrix_xyz = Eigen::Matrix<double, 6, 3>::Zero();
        for (int i = 0; i < 3; i++)
        {
            process_noise_matrix_xyz.block<2, 1>(2 * i, i) = process_noise_vec;  // NOLINT(bugprone-implicit-widening-of-multiplication-result)
        }
        Eigen::Matrix3d process_noise_xyz = Eigen::Matrix3d::Identity();
        process_noise_xyz.diagonal() << 0.1f, 0.1f, 0.01f;

        const Eigen::Matrix<double, 6, 6> q_xyz = process_noise_matrix_xyz * process_noise_xyz * process_noise_matrix_xyz.transpose();
        process_noise_vec << 0.5 * update_time * update_time, update_time;
        Eigen::Matrix<double, 2, 1> process_noise_matrix_yaw = process_noise_vec;
        const Eigen::Matrix<double, 2, 2> q_yaw = process_noise_matrix_yaw * 2 * process_noise_matrix_yaw.transpose();
        Eigen::Matrix<double, 9, 9> q = Eigen::Matrix<double, 9, 9>::Identity();
        q.block<6, 6>(0, 0) = q_xyz;
        q.block<2, 2>(6, 6) = q_yaw;
        q(8, 8) = 0.1;
        extended_kalman_filter->Q = q;

        Eigen::DiagonalMatrix<double, 4> r;
        r.diagonal() << 0.005, 0.005, 0.005, 0.02;
        extended_kalman_filter->R = r;
    }

}