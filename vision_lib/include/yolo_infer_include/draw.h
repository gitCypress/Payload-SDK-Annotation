#ifndef DRAW_H
#define DRAW_H

#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>

/**
 * @brief 在图像上绘制分割掩码。
 * @param img 输入/输出图像 (cv::Mat)，掩码将绘制在此图像上。
 * @param mask 指向掩码数据的浮点数指针。掩码数据的格式和维度需要根据实现确定。
 */
void draw_mask(cv::Mat& img, float* mask);

#endif  // DRAW_H
