#ifndef CV_CAMERA_H
#define CV_CAMERA_H

#include <mutex>
#include <memory>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <queue>

#include "task_type.h"
#include "task_segment.h"
#include "task_obbtrack.h"


namespace stage_2
{
    class Camera 
    {
    public:
        /**
         * @brief Camera 类的构造函数。
         * @param engine_path TaskSegment 使用的模型引擎或配置文件路径。
         * @param width 图像宽度，默认为 640。
         * @param height 图像高度，默认为 480。
         * @param fps 帧率，默认为 30。
         * @param camera_exposure 相机曝光设置，-1 表示自动曝光，其他值为手动设置值。
         */
        Camera(std::string engine_path, int width = 640, int height = 480, int fps = 30, int camera_exposure = -1);
        /**
         * @brief 获取对齐后的彩色图像帧和深度帧。
         * @return 返回一个 std::pair，包含 cv::Mat 格式的彩色帧和 rs2::depth_frame 格式的深度帧。
         */
        std::pair<cv::Mat, rs2::depth_frame> get_frame();
        /**
         * @brief 停止相机数据流并释放相关资源。
         */
        void release();
        /**
         * @brief 获取配置的图像宽度。
         * @return 返回图像宽度。
         */
        int get_width() const { return width_; }
        /**
         * @brief 获取配置的图像高度。
         * @return 返回图像高度。
         */
        int get_height() const { return height_; }

        /**
         * @brief 保存当前帧队列中的图像（具体逻辑依赖实现）。
         */
        void save_images();
        /**
         * @brief 进行实时图像处理和显示（可能包含缩放）。
         * @param scale 图像显示的缩放比例。
         *        (具体逻辑依赖实现，可能涉及调用 TaskSegment)
         */
        void realtime_image(float scale);
        /**
         * @brief 执行定位任务（可能基于 TaskSegment 的结果）。
         * @param scale 尺度因子。
         *        (具体逻辑依赖实现)
         */
        void location(float scale);
        /**
         * @brief 查找中心点（调用 TaskSegment 的 find_center）。
         * @return 返回 TaskSegment::find_center 的结果。
         */
        std::vector<float> find_center();

        // RotatedGear get();
        // RotatedGear get()
        /**
         * @brief 线程安全地获取检测到的第一个中心点。
         * @return 返回存储的第一个中心点坐标 (cv::Point)。
         */
        cv::Point get_center()
        {
            std::lock_guard<std::mutex> lock(center_mutex_); // 加锁
            return first_center_; // 返回 first_center_ 的副本
        }

        /**
         * @brief 线程安全地获取最终的处理结果（旋转齿轮信息）。
         * @return 返回最终的 RotatedGear 结果的副本。
         */
        RotatedGear get() 
        {
            std::lock_guard<std::mutex> lock(rotate_gear_mutex_); // 加锁
            return final_result_; // 返回 final_result_ 的副本
        }

        /** @brief 控制实时图像显示循环的标志位。*/
        bool stop_realtime_image_;

    private:
        int width_, height_;
        rs2::pipeline pipeline_;
        rs2::config config_;
        rs2::align align_to_;
        int camera_exposure_;
        std::vector<rs2::sensor> sensors_;
        cv::Mat color_mat_;  // 预分配颜色帧内存
        cv::Point first_center_;
        std::queue<cv::Mat> frame_queue_;

        RotatedGear final_result_;
        std::mutex rotate_gear_mutex_;
        std::mutex center_mutex_;
        stage_2::TaskSegment task_segment_;

        void configure_camera();
        void set_exposure();
    };
}


#endif // CAMERA_H