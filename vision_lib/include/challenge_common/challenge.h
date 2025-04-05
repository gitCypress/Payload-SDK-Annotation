#ifndef CHALLENGE_H
#define CHALLENGE_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <random>
#include <librealsense2/rsutil.h>
#include "types.h"

class Challenge {
public:
    /**
     * @brief (似乎是基类方法，具体实现可能在派生类中) 根据某种检测结果（可能是模型输出）检测障碍物。
     * @param results 检测结果的浮点数向量。
     * @return 返回一个向量，其中每个元素是代表一个障碍物轮廓的点集向量。
     */
    std::vector<std::vector<cv::Point>>  detect_obstacle(const std::vector<float>& results);  // 抽象方法
    
    // 计算最大内接矩形
    // std::tuple<std::vector<cv::Point>, float, std::array<float,3>> max_horizontal_rectangle(
    //     const std::vector<cv::Point>& polygon, float min_width, float min_height, 
    //     float dist, const rs2_intrinsics& intrinsics);

    /**
     * @brief 计算给定多边形内的最大水平内接矩形（考虑物理尺寸）。
     * @param polygon 输入的多边形顶点（像素坐标）。
     * @param min_width 矩形的最小物理宽度。
     * @param min_height 矩形的最小物理高度。
     * @param dist 目标区域的平均深度值。
     * @param intrinsics RealSense 相机的内参。
     * @return 返回一个 RealRectangle 结构体，包含矩形的像素顶点、物理尺寸和三维中心点。
     */
    RealRectangle max_horizontal_rectangle(
        const std::vector<cv::Point>& polygon, float min_width, float min_height, 
        float dist, const rs2_intrinsics& intrinsics);

    /**
     * @brief 计算给定掩码区域内的平均深度值（原始实现）。
     * @param frame (似乎未使用) 彩色帧。
     * @param mask 感兴趣区域的二值掩码。
     * @param depth_data RealSense 深度帧数据。
     * @param randnum (似乎未使用) 随机数，用途不明。
     * @return 返回掩码区域内的平均深度值。
     */
    float get_mid_position(cv::Mat& frame, const cv::Mat& mask, const rs2::depth_frame& depth_data, int randnum);

    /**
     * @brief 计算给定掩码区域内的平均深度值（带滤波和坐标转换的版本）。
     * @param frame (似乎未使用) 彩色帧。
     * @param mask 感兴趣区域的二值掩码。
     * @param depth_data RealSense 深度帧数据。
     * @param randum (似乎未使用) 随机数，用途不明。
     * @param intrinsics RealSense 相机的内参。
     * @param filter_kernel 用于深度图滤波的核大小，默认为 5。
     * @return 返回掩码区域内滤波后的平均深度值。
     */
    float get_mid_position_(cv::Mat& frame, const cv::Mat& mask, const rs2::depth_frame& depth_data, int randum, rs2_intrinsics& intrinsics, int filter_kernel = 5);

private:
    // 计算最大矩形面积和矩形信息
    std::tuple<float, cv::Rect> largest_histogram_area_with_constraints(const cv::Mat& histogram, float min_width, float min_height);

    // // 深度坐标转换（与 Python 中的 rs2_deproject_pixel_to_point 相似）
    // rs2_deproject_pixel_to_point(const rs2_intrinsics& intrinsics, const cv::Point& pt, float dist);
};

#endif // CHALLENGE_H
