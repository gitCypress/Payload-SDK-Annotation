#ifndef INFER_H
#define INFER_H

#include <opencv2/opencv.hpp>
#include "public.h"
#include "types.h"
#include "config.h"

using namespace nvinfer1;


class YoloDetector
{
public:
    /**
     * @brief YoloDetector 类的构造函数。
     * @param trtFile TensorRT 引擎文件 (.plan) 的路径。
     * @param num_class 模型预测的类别数量。
     * @param class_names 包含所有类别名称的字符串向量。
     *        (构造函数内部会加载引擎、创建执行上下文、分配 CUDA 内存等)
     */
    YoloDetector(const std::string& trtFile, int num_class, const std::vector<std::string>& class_names);

    /**
     * @brief YoloDetector 类的析构函数。
     *        负责释放 TensorRT 对象 (context, engine, runtime) 和 CUDA 内存。
     */
    ~YoloDetector();

    /**
     * @brief 对单张图像执行 YOLO 推理。
     * @param img 输入图像 (cv::Mat)。图像会在此函数内部进行预处理。
     * @return 返回包含所有检测结果的向量 (std::vector<Detection>)。
     *         每个 Detection 结构体包含边界框、置信度、类别 ID 和掩码信息。
     */
    std::vector<Detection> inference(cv::Mat& img);

    /**
     * @brief 在图像上绘制推理结果（边界框和掩码）。
     * @param img 输入/输出图像 (cv::Mat)，结果将绘制在此图像上。
     * @param inferResult 推理函数返回的检测结果向量。
     * @param drawBbox 是否绘制边界框，默认为 true。
     *        (掩码的绘制逻辑可能在此函数或内部调用的 process_mask 中实现)
     */
    void draw_image(cv::Mat& img, std::vector<Detection>& inferResult, bool drawBbox=true);

private:
    void get_engine();
    static void process_mask(
        float* protoDevice, Dims32 protoOutDims, std::vector<Detection>& vDetections, 
        int kInputH, int kInputW, cv::Mat& img, cudaStream_t stream
    );

private:
    int num_class_;  // 动态传入的类别数
    std::vector<std::string> class_names_;  // 动态传入的类别名
    Logger              gLogger;
    std::string         trtFile_;

    ICudaEngine *       engine;
    IRuntime *          runtime;
    IExecutionContext * context;

    cudaStream_t        stream;

    float *             outputData;
    std::vector<void *> vBufferD;
    float *             transposeDevide;
    float *             decodeDevice;

    int                 OUTPUT_CANDIDATES;  // 8400: 80 * 80 + 40 * 40 + 20 * 20
    Dims32              protoOutDims;  // proto shape [1 32 160 160]
};

#endif  // INFER_H
