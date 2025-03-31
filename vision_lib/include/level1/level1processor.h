#ifndef LEVEL1_PROCESSOR_H
#define LEVEL1_PROCESSOR_H
#include <librealsense2/rs.hpp>
#include "processor.h"
// #include "predict.h"
#include "infer.h"
#include "level1challenge.h"
#include "realsenseCamera.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <tuple>
#include <atomic>

class Level1Processor : public VideoProcessor {
public:
    Level1Processor(
            const std::string& model_path,
            int num_class,
            const std::vector<std::string>& class_names,  // 新增类别名称参数
            int width = 640,
            int height = 480,
            int fps = 30
    );

    ~Level1Processor() {
         camera.release();
    }

    void process_frame();
    void run();
    // 安全停止检测
    void stop() {
        is_running = false;
    }

    // 线程安全地获取数据
    std::tuple<std::vector<cv::Point>, float, std::array<float, 3>> get_current_collect() const {
        std::lock_guard<std::mutex> lock(data_mutex);
//        std::cout<< "-----函数内----查看current_collect地址"<<&current_collect << std::endl;

        return current_collect;
    }
    bool isRunning() const { return is_running.load(); }
private:
    RealSenseCamera camera;
    Level1Challenge challenger;
    std::atomic<bool> is_running{true};  // 线程安全的运行标志
    mutable  std::mutex data_mutex;                // 保护数据的互斥锁
    std::tuple<std::vector<cv::Point>, float, std::array<float, 3>> current_collect;
};

#endif // LEVEL1_PROCESSOR_H
