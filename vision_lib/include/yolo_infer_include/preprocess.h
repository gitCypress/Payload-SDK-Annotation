#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>

/**
 * @brief 在 CUDA 设备上对输入图像执行预处理。
 *        包括调整大小、颜色空间转换 (BGR->RGB)、维度变换 (HWC->CHW) 和归一化。
 * @param srcImg 待处理的源图像 (cv::Mat)。
 * @param dstDevData 指向目标设备内存的指针，用于存储预处理后的数据。
 * @param dstHeight 目标（模型输入）的高度。
 * @param dstWidth 目标（模型输入）的宽度。
 * @param stream CUDA 流。
 */
void preprocess(const cv::Mat& srcImg, float* dstDevData, const int dstHeight, const int dstWidth, cudaStream_t stream);
/*
srcImg:     source image for inference
dstDevData: data after preprocess (resize / bgr to rgb / hwc to chw / normalize)
dstHeight:  CNN input height
dstWidth:   CNN input width
*/

#endif  // PREPROCESS_H
