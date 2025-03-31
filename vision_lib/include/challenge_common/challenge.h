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
    std::vector<std::vector<cv::Point>>  detect_obstacle(const std::vector<float>& results);  // 抽象方法
    
    // 计算最大内接矩形
    // std::tuple<std::vector<cv::Point>, float, std::array<float,3>> max_horizontal_rectangle(
    //     const std::vector<cv::Point>& polygon, float min_width, float min_height, 
    //     float dist, const rs2_intrinsics& intrinsics);

    RealRectangle max_horizontal_rectangle(
        const std::vector<cv::Point>& polygon, float min_width, float min_height, 
        float dist, const rs2_intrinsics& intrinsics);

    // 获取mask内部的平均深度
    float get_mid_position(cv::Mat& frame, const cv::Mat& mask, const rs2::depth_frame& depth_data, int randnum);
    float get_mid_position_(cv::Mat& frame, const cv::Mat& mask, const rs2::depth_frame& depth_data, int randum, rs2_intrinsics& intrinsics, int filter_kernel = 5);

private:
    // 计算最大矩形面积和矩形信息
    std::tuple<float, cv::Rect> largest_histogram_area_with_constraints(const cv::Mat& histogram, float min_width, float min_height);

    // // 深度坐标转换（与 Python 中的 rs2_deproject_pixel_to_point 相似）
    // rs2_deproject_pixel_to_point(const rs2_intrinsics& intrinsics, const cv::Point& pt, float dist);
};

#endif // CHALLENGE_H
