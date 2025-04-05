#ifndef LEVEL1_CHALLENGE_H
#define LEVEL1_CHALLENGE_H

#include "challenge.h"
#include <opencv2/opencv.hpp>
#include "types.h"
// #include <pyrealsense2/rs.hpp>
// #include <librealsense2/rs.hpp>

#include <vector>
#include <string>
#include <unordered_map>
#include <mutex>

class Level1Challenge : public Challenge 
{
public:
    /**
     * @brief Level1Challenge 类的默认构造函数。
     */
    Level1Challenge();

    /**
     * @brief 检测 Level 1 任务中的障碍物。
     *        重写了基类的 detect_obstacle 方法。
     * @param results 来自目标检测模型的原始检测结果列表。
     * @param width 图像宽度。
     * @param height 图像高度。
     * @return 返回一个向量，其中每个元素是代表一个障碍物轮廓的点集向量。
     */
    std::vector<std::vector<cv::Point>> detect_obstacle(const std::vector<Detection>& results, float width, float height);

    /**
     * @brief 获取当前设置的表面掩码（线程安全）。
     * @return 返回表示表面的 cv::Mat 掩码。
     */
    cv::Mat get_surface_mask();

    /**
     * @brief 设置表面掩码（线程安全）。
     * @param mask 新的表面掩码。
     */
    void set_surface(const cv::Mat& mask);
private:
    cv::Mat surface;  // turntable surface
    mutable std::mutex surface_mutex;  // 用于保护surface
};

#endif // LEVEL1_CHALLENGE_H
