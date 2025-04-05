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
        /**
         * @brief KalmanFilter 类的默认构造函数。
         *        初始化状态向量、协方差矩阵以及卡尔曼滤波器的各个参数矩阵 (F, H, Q, R)。
         *        状态向量 x 定义为 [x, y, width, height, angle, vx, vy, v_angle]，共 8 维。
         *        观测向量 z 定义为 [x, y, width, height, angle]，共 5 维。
         */
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

        /**
         * @brief 执行卡尔曼滤波器的预测步骤。
         *        根据状态转移矩阵 F 更新状态向量 x 和协方差矩阵 P。
         */
        void predict() {
            x = F * x;
            P = F * P * F.transpose() + Q;
        }

        /**
         * @brief 执行卡尔曼滤波器的更新步骤。
         * @param z 观测向量 (Eigen::VectorXd, 大小必须为 5)，包含观测到的 [x, y, width, height, angle]。
         * @throws std::invalid_argument 如果观测向量大小不为 5。
         *        如果状态向量首次更新（初始为零），则直接用观测值初始化状态，并增大初始协方差。
         *        否则，执行标准的卡尔曼更新逻辑，计算卡尔曼增益 K，更新状态向量 x 和协方差矩阵 P。
         */
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

        /**
         * @brief 获取当前滤波后的状态，表示为一个旋转矩形。
         * @return 返回 cv::RotatedRect 对象，中心点为 (x(0), x(1))，尺寸为 (x(2), x(3))，角度为 x(4)。
         */
        cv::RotatedRect getState() const {
            return cv::RotatedRect(cv::Point2f(x(0), x(1)), cv::Size2f(x(2), x(3)), x(4));
        }

    private:
        Eigen::VectorXd x; // 状态向量 [x, y, width, height, angle, vx, vy, v_angle]
        Eigen::MatrixXd P; // 状态协方差矩阵
        Eigen::MatrixXd F; // 状态转移矩阵
        Eigen::MatrixXd H; // 观测矩阵
        Eigen::MatrixXd Q; // 过程噪声协方差矩阵
        Eigen::MatrixXd R; // 观测噪声协方差矩阵
    };

    class Track
    {
    private:
        int next_id; // 下一个分配的目标 ID
        void *handle; // 指向某种内部句柄或资源的指针
        OBJECTLIST *object_list; // 指向目标列表的指针（类型定义在 synt_detector_type.h）
        std::unordered_map<int, stage_2::KalmanFilter> trackers; // 存储每个目标 ID 对应的卡尔曼滤波器

    public:
        /**
         * @brief Track 类的构造函数。
         * @param engine_path 模型引擎或配置文件的路径。
         *        (具体初始化逻辑依赖于实现)
         */
        Track(std::string engine_path);

        /**
         * @brief Track 类的析构函数。
         *        (负责释放资源，具体逻辑依赖于实现)
         */
        ~Track();

        /**
         * @brief 重载调用运算符，用于处理单帧图像并执行跟踪。
         * @param image 输入的图像帧 (cv::Mat)。
         * @return 返回更新后的目标列表指针 (OBJECTLIST*)。
         *         (具体处理逻辑，如目标检测、数据关联、卡尔曼滤波更新等，依赖于实现)
         */
        OBJECTLIST * operator() (cv::Mat& image);
    };
}

#endif // TRACK_H