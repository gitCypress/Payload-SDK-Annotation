#ifndef POSTPROCESS_H
#define POSTPROCESS_H

#include <cmath>
#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include "config.h"

/**
 * @brief 在 CUDA 设备上执行矩阵转置。
 * @param src 源矩阵数据指针 (设备内存)。
 * @param dst 目标矩阵数据指针 (设备内存)。
 * @param numBboxes 源矩阵的列数（通常是检测框数量，如 8400）。
 * @param numElements 源矩阵的行数（每个检测框的元素数量，如 116）。
 * @param stream CUDA 流。
 */
void transpose(float* src, float* dst, int numBboxes, int numElements, cudaStream_t stream);
/*
    transpose [116 8400] convert to [8400 116]
src:          Tensor, dim is [116 8400]
dst:          Tensor, dim is [8400 116]
numBboxes:    number of bboxes: default 8400
numElements:  center_x, center_y, width, height, 80 classes, 32 masks
*/

/**
 * @brief 在 CUDA 设备上解码模型的原始输出。
 *        将 [numBboxes, numBoxElement] 的原始输出转换为包含有效检测框信息的扁平化数组。
 *        包括边界框坐标转换 (center_x, center_y, w, h -> x1, y1, x2, y2)，
 *        应用置信度阈值，并组合类别、置信度、边界框和掩码系数。
 * @param src 转置后的模型原始输出数据指针 (设备内存)，形状 [numBboxes, numBoxElement]。
 * @param dst 解码后的输出数据指针 (设备内存)。格式：[count, det1, det2, ...]。
 *            count 是有效检测框数量。detX 是 [x1, y1, x2, y2, conf, classId, keepflag, mask0, ..., mask31]。
 * @param numBboxes 输入的检测框总数 (e.g., 8400)。
 * @param numClasses 模型预测的类别数。
 * @param numMasks 掩码系数的数量 (e.g., 32)。
 * @param confThresh 置信度阈值。
 * @param maxObjects 预设的最大有效检测框数量 (用于输出缓冲区大小)。
 * @param numBoxElement 每个原始检测框的元素数量。
 * @param stream CUDA 流。
 */
void decode(float* src, float* dst, int numBboxes, int numClasses, int numMasks, float confThresh, int maxObjects, int numBoxElement, cudaStream_t stream);
/*
    convert [8400 116] to [39001, ], 39001 = 1 + 1000 * (4bbox + cond + cls + keepflag + 32masks), 1: number of valid bboxes
     1000: max bboxes, valid bboxes may less than 1000, 4bbox: left, top, right, bottom)
*/

/**
 * @brief 在 CUDA 设备上对解码后的检测框执行非极大值抑制 (NMS)。
 *        通过设置 keepflag 来标记要保留或丢弃的框。
 * @param data 指向解码后输出数据的指针 (设备内存)，格式见 decode 函数注释。
 * @param kNmsThresh NMS 的 IoU 阈值。
 * @param maxObjects 最大有效检测框数量 (与 decode 函数中的一致)。
 * @param numBoxElement 每个解码后检测框信息的元素数量 (含掩码)。
 * @param stream CUDA 流。
 */
void nms(float* data, float kNmsThresh, int maxObjects, int numBoxElement, cudaStream_t stream);


/**
 * @brief 在 CUDA 设备上执行矩阵乘法 C = A * B。
 * @param aMatrix 矩阵 A 的数据指针 (设备内存)。
 * @param aRows 矩阵 A 的行数。
 * @param aCols 矩阵 A 的列数。
 * @param bMatrix 矩阵 B 的数据指针 (设备内存)。
 * @param bRows 矩阵 B 的行数 (必须等于 aCols)。
 * @param bCols 矩阵 B 的列数。
 * @param cMatrix 输出矩阵 C 的数据指针 (设备内存)。
 * @param stream CUDA 流。
 * @param sigm 是否对结果矩阵 C 执行 Sigmoid 激活函数，默认为 false。
 */
void matrix_multiply(float* aMatrix, int aRows, int aCols, float* bMatrix, int bRows, int bCols, float* cMatrix, cudaStream_t stream, bool sigm = false);
/*
    matrix multiply, like numpy.matmul() function
aMatrix:          input matrix 1 array on device
aRows:            rows of input matrix 1
aCols:            columns of input matrix 1
bMatrix:          input matrix 2 array on device
bRows:            rows of input matrix 2
bCols:            columns of input matrix 2
cMatrix:          output matrix array on device
sigm:             Whether to do sigmoid() on the result 
*/


/**
 * @brief 在 CUDA 设备上按比例缩放边界框坐标。
 * @param bboxDevice 边界框数据指针 (设备内存)，格式 [x1, y1, x2, y2, x1, y1, ...]。
 * @param length 边界框坐标的总数 (num_boxes * 4)。
 * @param heightRatio 高度方向的缩放比例。
 * @param widthRatio 宽度方向的缩放比例。
 * @param stream CUDA 流。
 */
void downsample_bbox(float* bboxDevice, int length, float heightRatio, float widthRatio, cudaStream_t stream);


/**
 * @brief 在 CUDA 设备上根据边界框裁剪掩码。
 *        将每个掩码在其对应边界框之外的区域置为 0。
 * @param masksDevice 掩码数据指针 (设备内存)，形状 [maskNum, maskHeight * maskWidth]。
 * @param maskNum 掩码的数量。
 * @param maskHeight 每个掩码的高度。
 * @param maskWidth 每个掩码的宽度。
 * @param bboxesDevice 边界框数据指针 (设备内存)，形状 [maskNum, 4]，格式 [x1, y1, x2, y2]。
 * @param stream CUDA 流。
 */
void crop_mask(float* masksDevice, int maskNum, int maskHeight, int maskWidth, float* bboxesDevice, cudaStream_t stream);
/*
    set value 0 where masks out of bboxes
masksDevice:      mask array on device, shape(n, 160 x 160)
maskNum:          n, number of masks
maskHeight:       height of each mask
maskWidth:        width of each mask
bboxesDevice:     bbox array on device, shape(n, 4), 4 : x1, y1, x2, y2
*/


/**
 * @brief (已废弃或用途不明，可能与 crop_mask 重复或用于特定裁剪) 在 CUDA 设备上切割掩码。
 * @param masksDevice 源掩码数据指针 (设备内存)。
 * @param maskNum 掩码数量。
 * @param maskHeight 源掩码高度。
 * @param maskWidth 源掩码宽度。
 * @param cutMasksDevice 目标掩码数据指针 (设备内存)。
 * @param cutMaskTop 裁剪区域的顶部坐标。
 * @param cutMaskLeft 裁剪区域的左侧坐标。
 * @param cutMaskH 裁剪区域的高度。
 * @param cutMaskW 裁剪区域的宽度。
 * @param stream CUDA 流。
 */
void cut_mask(
    float* masksDevice, int maskNum, int maskHeight, int maskWidth,
    float* cutMasksDevice, int cutMaskTop, int cutMaskLeft, int cutMaskH, int cutMaskW, cudaStream_t stream
);


/**
 * @brief 在 CUDA 设备上调整掩码大小（例如，使用双线性插值）。
 * @param masksDevice 源掩码数据指针 (设备内存)。
 * @param maskNum 掩码数量。
 * @param maskHeight 源掩码高度。
 * @param maskWidth 源掩码宽度。
 * @param dstMasksDevice 目标掩码数据指针 (设备内存)。
 * @param dstMaskH 目标掩码高度。
 * @param dstMaskW 目标掩码宽度。
 * @param stream CUDA 流。
 */
void resize(float* masksDevice, int maskNum, int maskHeight, int maskWidth, float* dstMasksDevice, int dstMaskH, int dstMaskW, cudaStream_t stream);


/**
 * @brief (CPU 函数) 将单个边界框从模型输入尺寸缩放到原始图像尺寸。
 *        考虑了图像预处理时可能添加的填充 (letterbox/padding)。
 * @param img 原始图像 (cv::Mat)，用于获取原始尺寸。
 * @param bbox 边界框坐标数组 [x1, y1, x2, y2] (模型输入坐标系)，会被原地修改为原始图像坐标系。
 */
__inline__ void scale_bbox(cv::Mat& img, float bbox[4]){
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);
    float r = std::min(r_w, r_h);
    float pad_h = (kInputH - r * img.rows) / 2;
    float pad_w = (kInputW - r * img.cols) / 2;

    bbox[0] = (bbox[0] - pad_w) / r;
    bbox[1] = (bbox[1] - pad_h) / r;
    bbox[2] = (bbox[2] - pad_w) / r;
    bbox[3] = (bbox[3] - pad_h) / r;
}


#endif  // POSTPROCESS_H
