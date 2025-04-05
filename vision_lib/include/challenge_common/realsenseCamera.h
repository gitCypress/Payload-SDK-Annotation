#ifndef REALSENSECAMERA_H
#define REALSENSECAMERA_H

#include <librealsense2/rs.hpp>
#include <memory>

class RealSenseCamera {
public:
    /**
     * @brief RealSenseCamera 类的构造函数。
     * @param width 图像宽度，默认为 640。
     * @param height 图像高度，默认为 480。
     * @param fps 帧率，默认为 30。
     */
    RealSenseCamera(int width = 640, int height = 480, int fps = 30);

    /**
     * @brief 获取当前的彩色帧和深度帧。
     * @param color_frame 用于接收彩色帧的引用（输出参数）。
     * @param depth_frame 用于接收深度帧的引用（输出参数）。
     */
    void get_frame(rs2::frame& color_frame, rs2::frame& depth_frame);

    /**
     * @brief 停止摄像头并释放资源。
     */
    void release();

    /**
     * @brief 获取摄像头的宽度。
     * @return 返回配置的图像宽度。
     */
    int get_width() const;

    /**
     * @brief 获取摄像头的高度。
     * @return 返回配置的图像高度。
     */
    int get_height() const;

private:
    int width, height;  // 分辨率
    rs2::config config;  // 配置对象
    std::shared_ptr<rs2::pipeline> pipeline;  // 管道对象，管理流
    int camera_exposure = -1;
//    int camera_exposure = 100;
    rs2::sensor sen;
};

#endif // REALSENSECAMERA_H