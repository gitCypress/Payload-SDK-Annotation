#ifndef TRACK_H
#define TRACK_H

// #include <logging.h>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "synt_detector_type.h"
#include "synt_object_detector.h"

namespace stage_2
{
    class KalmanFilter {
    public:
        KalmanFilter() : x(8), P(8, 8), F(8, 8), H(5, 8), Q(8, 8), R(5, 5) {
            x.setZero();
            P.setIdentity();
            // 配置状态转移矩阵 F（假设匀速运动模型，dt=1）
            F << 1, 0, 0, 0, 0, 1, 0, 0,
                0, 1, 0, 0, 0, 0, 1, 0,
                0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 1, 0, 0, 1,
                0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 1;

            H << 1, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 1, 0, 0, 0;
            
            // 调整过程噪声
            Q = Eigen::MatrixXd::Identity(8, 8) * 0.01;
            Q(5,5) = 0.1;  // x速度噪声
            Q(6,6) = 0.1;  // y速度噪声

            // 调整观测噪声
            R.setZero();
            R(0,0) = 200.0; // x
            R(1,1) = 200.0; // y
            R(2,2) = 10.0; // 宽度
            R(3,3) = 10.0; // 高度
            R(4,4) = 15.0; // 角度

            // Q = Eigen::MatrixXd::Identity(8, 8) * 0.01;
            // // R = Eigen::MatrixXd::Identity(5, 5) * 50;
            // R(0,0) = 50.0;  // x坐标的观测噪声
            // R(1,1) = 50.0;  // y坐标的观测噪声
            // R = R * 10;      // 保持其他参数（宽、高、角度）的噪声系数不变
        }

        void predict() {
            x = F * x;
            P = F * P * F.transpose() + Q;
        }

        void update(const Eigen::VectorXd& z) {
            if (z.size() != 5) {
                throw std::invalid_argument("Measurement vector must be of size 5");
            }
            if (x.head<5>().squaredNorm() == 0) { // 如果状态向量初始为零
                x.head<5>() = z.segment(0, 5); // 初始化状态向量的前五个元素
                P = Eigen::MatrixXd::Identity(8, 8) * 100; // 增大初始协方差
            } else {
                // 正常更新逻辑
                Eigen::VectorXd y = z - H * x;
                Eigen::MatrixXd S = H * P * H.transpose() + R;
                Eigen::MatrixXd K = P * H.transpose() * S.inverse();
                x += K * y;
                P = (Eigen::MatrixXd::Identity(8, 8) - K * H) * P;
            }
        }

        cv::RotatedRect getState() const {
            return cv::RotatedRect(cv::Point2f(x(0), x(1)), cv::Size2f(x(2), x(3)), x(4));
        }

    private:
        Eigen::VectorXd x;
        Eigen::MatrixXd P;
        Eigen::MatrixXd F;
        Eigen::MatrixXd H;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
    };

    class Track
    {
    private:
        int next_id;
        void *handle;
        OBJECTLIST *object_list;
        std::unordered_map<int, stage_2::KalmanFilter> trackers;

    public:
        Track(std::string engine_path);

        ~Track();

        OBJECTLIST * operator() (cv::Mat& image);
    };
}

#endif // TRACK_H