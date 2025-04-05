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
#include <mutex>
#include <array>

#include "task_type.h"
#include "synt_detector_type.h"
#include "synt_object_detector.h"

namespace stage_2
{
    /**
     * @brief 用于存储矩形检测结果的结构体，包含矩形端点、面积和中心点。
     */
    struct RectResult 
    {
        /** @brief 矩形的两个对角顶点 (cv::Point)。*/
        std::array<cv::Point, 2> rect;
        /** @brief 矩形的面积。*/
        double area;
        /** @brief 矩形的中心点 (cv::Point)。*/
        cv::Point center;
    };

    class TaskSegment
    {
    public:
        /**
         * @brief TaskSegment 类的构造函数。
         * @param engine_path 模型引擎或配置文件的路径。
         *        (具体初始化逻辑依赖于实现)
         */
        TaskSegment(std::string engine_path);

        /**
         * @brief TaskSegment 类的析构函数。
         *        (负责释放资源，具体逻辑依赖于实现)
         */
        ~TaskSegment();

        /**
         * @brief 重载调用运算符，处理图像和深度帧，执行分割任务。
         * @param image 输入的彩色图像帧 (cv::Mat)。
         * @param frame 输入的 RealSense 深度帧 (rs2::depth_frame)。
         * @return 返回处理后的旋转齿轮信息 (RotatedGear)。
         *         (具体处理逻辑，如目标检测、分割、姿态估计等，依赖于实现)
         */
        RotatedGear operator() (cv::Mat& image, rs2::depth_frame frame);

        /**
         * @brief 查找中心点（可能是齿轮中心或其他关键点）。
         * @param image 输入的彩色图像帧 (cv::Mat)。
         * @param frame 输入的 RealSense 深度帧 (rs2::depth_frame)。
         * @return 返回一个包含中心点坐标或其他相关信息的浮点数向量。
         *         (具体逻辑依赖于实现)
         */
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
        /** @brief 可能是计算出的角速度或其他旋转相关的参数。*/
        double omiga_;
        /** @brief 检测或计算出的中心点坐标。*/
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