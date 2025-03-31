#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "infer.h"
#include <opencv2/opencv.hpp>

class VideoProcessor {
public:
    // 构造函数，初始化 YOLODetector 并加载模型
    VideoProcessor(
        const std::string& modelPath,
        int num_class,                          // 新增参数
        const std::vector<std::string>& class_names ); // 新增参数);

    // 析构函数，释放 YOLODetector
    virtual ~VideoProcessor();

    // 处理每一帧图像
    virtual void process_frame(cv::Mat& frame);

// private:
    // YOLO 检测器对象
    YoloDetector* detector;
};

#endif // PROCESSOR_H