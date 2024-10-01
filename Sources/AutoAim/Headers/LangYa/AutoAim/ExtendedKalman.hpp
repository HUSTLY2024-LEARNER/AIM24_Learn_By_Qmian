#pragma once

#include <ceres/jet.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>

namespace LangYa:: inline AutoAim
{
    using NonlinearFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
    using JacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;

    template <typename T, int TRows, int TCols>
    struct ExtendedKalman
    {
        Eigen::Matrix<T, TRows, TCols> K; // 卡尔曼增益

        Eigen::Matrix<T, TRows, 1> pri_state;  // 先验估计
        Eigen::Matrix<T, TRows, 1> post_state; // 后验估计

        Eigen::Matrix<T, TCols, 1> pri_state_measure; // 后验估计

        NonlinearFunc transition_func;    // f
        JacobianFunc transition_func_jet; // f_jet
        Eigen::Matrix<T, TRows, TRows> F;         // F

        Eigen::Matrix<T, TRows, TRows> P_post; // P_post
        Eigen::Matrix<T, TRows, TRows> P_pri;  // P_post

        Eigen::Matrix<T, TRows, TRows> Q; // 过程噪声矩阵：Q
        Eigen::Matrix<T, TCols, TCols> R; // 测量噪声矩阵 : R

        NonlinearFunc observe_func;    //  h
        JacobianFunc observe_func_jet; // h_jet
        Eigen::Matrix<T, TCols, TRows> H;      // H

        Eigen::Matrix<T, TRows, TRows> I;

        ExtendedKalman()
        {
            P_post = Eigen::Matrix<T, TRows, TRows>::Identity();
            Q = Eigen::Matrix<T, TRows, TRows>::Identity();
            R = Eigen::Matrix<T, TCols, TCols>::Identity();
            post_state = Eigen::Matrix<T, TRows, 1>::Zero();
            I = Eigen::Matrix<T, TRows, TRows>::Identity();
        }

        Eigen::Matrix<T, TRows, 1> predict() // 根据上一帧的估计值直接利用函数进行计算
        {

            pri_state = transition_func(post_state);

            F = transition_func_jet(post_state);

            P_pri = F * P_post * F.transpose() + Q;

            post_state = pri_state;
            P_post = P_pri;

            return pri_state;
        }

        Eigen::Matrix<T, TRows, 1> update(const Eigen::Matrix<T, TCols, 1> &z) // 根据实际值计算,并更新
        {

            H = observe_func_jet(pri_state);

            K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();

            pri_state_measure = observe_func(pri_state);

            post_state = pri_state + K * (z - pri_state_measure);

            P_post = (I - K * H) * P_pri;

            return post_state;
        }
    };
}
