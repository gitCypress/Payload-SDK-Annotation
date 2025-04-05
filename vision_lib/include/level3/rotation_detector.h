#include <deque>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>

namespace stage_2
{
    /**
     * @brief 简单的二维点结构体 (与 cv::Point2f 功能相似)。
     */
    struct Point {
        /** @brief x 坐标 */
        double x;
        /** @brief y 坐标 */
        double y;
        /**
         * @brief 构造函数。
         * @param x_ x 坐标，默认为 0。
         * @param y_ y 坐标，默认为 0。
         */
        Point(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
    };

    class RealTimeRotationDetector 
    {
    public:
        /**
         * @brief RealTimeRotationDetector 类的构造函数。
         * @param max_length 存储轨迹点的最大数量，默认为 50。
         * @param min_cycles 判断为旋转所需的最小周期数（可能指零交叉次数），默认为 1。
         * @param distance_threshold 距离阈值，可能用于判断点是否接近中心或用于去噪，默认为 0.1。
         */
        RealTimeRotationDetector(size_t max_length = 50, int min_cycles = 1, double distance_threshold = 0.1);

        /**
         * @brief 重载调用运算符，处理新的轨迹点并判断是否检测到旋转。
         * @param point 当前帧的轨迹点 (cv::Point2f)。
         * @return 如果检测到满足条件的旋转，返回 true；否则返回 false。
         *         (具体逻辑涉及更新轨迹、计算距离、检测零交叉等)
         */
        bool operator() (const cv::Point2f& point);

        /**
         * @brief RealTimeRotationDetector 类的析构函数。
         */
        ~RealTimeRotationDetector();
    private:
        size_t max_length_;
        int min_cycles_;
        double distance_threshold_;
        std::deque<cv::Point2f> trajectory_;
        std::deque<double> distances_;
        int zero_crossings_;
    };
}
