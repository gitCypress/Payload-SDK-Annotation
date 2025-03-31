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

class Level1Challenge : public Challenge 
{
public:
    Level1Challenge();
    std::vector<std::vector<cv::Point>> detect_obstacle(const std::vector<Detection>& results, float width, float height);
    cv::Mat get_surface_mask();
    void set_surface(const cv::Mat& mask);
private:
    cv::Mat surface;  // turntable surface
    mutable std::mutex surface_mutex;  // 用于保护surface
};

#endif // LEVEL1_CHALLENGE_H
