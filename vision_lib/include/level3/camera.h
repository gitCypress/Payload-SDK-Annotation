#ifndef CV_CAMERA_H
#define CV_CAMERA_H

#include <mutex>
#include <memory>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include "task_type.h"
#include "task_segment.h"
#include "task_obbtrack.h"


namespace stage_2
{
    class Camera 
    {
    public:
        Camera(std::string engine_path, int width = 640, int height = 480, int fps = 30, int camera_exposure = -1);
        std::pair<cv::Mat, rs2::depth_frame> get_frame();
        void release();
        int get_width() const { return width_; }
        int get_height() const { return height_; }

        void save_images();
        void realtime_image(float scale);
        void location(float scale);
        std::vector<float> find_center();

        // RotatedGear get();
        // RotatedGear get()
        cv::Point get_center()
        {
            std::lock_guard<std::mutex> lock(center_mutex_); // 加锁
            return first_center_; // 返回 rotate_gear_ 的副本
        }

        RotatedGear get() 
        {
            std::lock_guard<std::mutex> lock(rotate_gear_mutex_); // 加锁
            return final_result_; // 返回 rotate_gear_ 的副本
        }

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