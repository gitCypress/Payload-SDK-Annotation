#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "infer.h"
#include <opencv2/opencv.hpp>

class VideoProcessor {
public:
    /**
     * @brief VideoProcessor 类的构造函数。
     * @param modelPath 模型文件的路径。
     * @param num_class 模型预测的类别数量。
     * @param class_names 包含所有类别名称的字符串向量。
     */
    VideoProcessor(
        const std::string& modelPath,
        int num_class,                          // 新增参数
        const std::vector<std::string>& class_names ); // 新增参数);

    /**
     * @brief VideoProcessor 类的虚析构函数。
     *        负责释放 YoloDetector 对象。
     */
    virtual ~VideoProcessor();

    /**
     * @brief 处理单帧图像的虚函数。
     *        派生类应重写此方法以实现具体的帧处理逻辑。
     * @param frame 输入的图像帧 (cv::Mat)。
     */
    virtual void process_frame(cv::Mat& frame);

// private:
    // YOLO 检测器对象
    YoloDetector* detector;
};

#endif // PROCESSOR_H