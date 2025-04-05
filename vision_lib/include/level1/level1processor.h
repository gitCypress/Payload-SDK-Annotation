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
#include <mutex>
#include <array>

class Level1Processor : public VideoProcessor {
public:
    /**
     * @brief Level1Processor 类的构造函数。
     * @param model_path 用于目标检测的模型的路径。
     * @param num_class 模型预测的类别数量。
     * @param class_names 包含所有类别名称的字符串向量。
     * @param width RealSense 相机图像宽度，默认为 640。
     * @param height RealSense 相机图像高度，默认为 480。
     * @param fps RealSense 相机帧率，默认为 30。
     */
    Level1Processor(
            const std::string& model_path,
            int num_class,
            const std::vector<std::string>& class_names,  // 新增类别名称参数
            int width = 640,
            int height = 480,
            int fps = 30
    );

    /**
     * @brief Level1Processor 类的析构函数。
     *        负责释放 RealSense 相机资源。
     */
    ~Level1Processor() {
         camera.release();
    }

    /**
     * @brief 处理单帧图像（具体实现可能在源文件中）。
     *        重写了基类的 process_frame 方法。
     */
    void process_frame();

    /**
     * @brief 启动处理循环（具体实现可能在源文件中）。
     */
    void run();

    /**
     * @brief 安全地停止处理循环（通过设置原子标志位）。
     */
    void stop() {
        is_running = false;
    }

    /**
     * @brief 线程安全地获取当前处理结果。
     * @return 返回一个元组，包含：
     *         - std::vector<cv::Point>: 可能代表检测到的关键点或轮廓。
     *         - float: 可能代表某个度量值（如置信度、距离）。
     *         - std::array<float, 3>: 可能代表三维坐标或姿态。
     */
    std::tuple<std::vector<cv::Point>, float, std::array<float, 3>> get_current_collect() const {
        std::lock_guard<std::mutex> lock(data_mutex);
//        std::cout<< "-----函数内----查看current_collect地址"<<&current_collect << std::endl;

        return current_collect;
    }

    /**
     * @brief 检查处理循环是否仍在运行。
     * @return 如果正在运行，返回 true；否则返回 false。
     */
    bool isRunning() const { return is_running.load(); }
private:
    RealSenseCamera camera;
    Level1Challenge challenger;
    std::atomic<bool> is_running{true};  // 线程安全的运行标志
    mutable  std::mutex data_mutex;                // 保护数据的互斥锁
    std::tuple<std::vector<cv::Point>, float, std::array<float, 3>> current_collect;
};

#endif // LEVEL1_PROCESSOR_H
