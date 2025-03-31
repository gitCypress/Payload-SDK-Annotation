#ifndef TASK_SEGMENT_H
#define TASK_SEGMENT_H
#include <time.h>
#include <stack>
#include <stdio.h>
#include <vector>
#include <random>
#include <utility>
// #include <logging.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include "task_type.h"
#include "synt_detector_type.h"
#include "synt_object_detector.h"

namespace stage_2
{
    struct RectResult 
    {
        std::array<cv::Point, 2> rect;
        double area;
        cv::Point center;
    };

    class TaskSegment
    {
    public:
        TaskSegment(std::string engine_path);
        ~TaskSegment();

        RotatedGear operator() (cv::Mat& image, rs2::depth_frame frame);
        std::vector<float> find_center (cv::Mat& image, rs2::depth_frame frame);

    private:
        void update(cv::Mat& image, rs2::depth_frame depth_frame);
        void find_farthest_point(const cv::Mat& mask, const cv::Point& reference_point, cv::Point& farthest_point, float& max_distance);
        void rotate_contour(const std::vector<cv::Point>& src, cv::Point center, float angle, std::string dir, std::vector<cv::Point>& dst);
        void calculate_rotation_from_point_with_center(const cv::Point& point, const cv::Point& center, float& angle_radians, std::string& direction);
        int find_mode(const std::vector<int>& data);
        RectResult max_horizontal_rectangle(const std::vector<cv::Point>& polygon, float min_width, float min_height, float dist, rs2_intrinsics intrinsics);
        void touch_circle(cv::Mat& mask, const cv::Point& center, const std::vector<cv::Point>& farther_points, float radius);
        static double calculateDistance(const cv::Point2f& point1, const cv::Point2f& point2) 
        {
            return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
        }
        float get_mid_position(const cv::Mat& mask, const rs2::depth_frame& depth_data, int randnum);

    public:
        double omiga_;
        cv::Point center_;
    private:
        int index = 0;
        void* handle;
        OBJECTLIST *object_list_;
        std::vector<cv::Point> best_rect_;
        RotatedGear rotate_gear_;

        std::vector<int> surface_nums_;
        std::vector<int> teeth_nums_;
        std::mutex rotate_gear_mutex_;
    };
}

#endif // TASK_SEGMENT_H