#ifndef REALSENSECAMERA_H
#define REALSENSECAMERA_H

#include <librealsense2/rs.hpp>
#include <memory>

class RealSenseCamera {
public:
    // 构造函数，设置分辨率和帧率，默认分辨率为640x480，帧率为30
    RealSenseCamera(int width = 640, int height = 480, int fps = 30);

    // 获取当前的彩色帧和深度帧
    void get_frame(rs2::frame& color_frame, rs2::frame& depth_frame);

    // 停止摄像头
    void release();

    int get_width() const;

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