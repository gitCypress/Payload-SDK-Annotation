#ifndef TYPES_H
#define TYPES_H

#include <string>
#include <vector>
#include <array>
#include <opencv2/opencv.hpp>

/**
 * @brief 用于存储目标检测结果的结构体 (似乎是早期版本或某种特定模型输出格式)。
 */
struct Detection
{
    /** @brief 边界框坐标 [x1, y1, x2, y2]。 */
    float bbox[4];
    /** @brief 检测结果的置信度。 */
    float conf;
    /** @brief 检测到的物体类别 ID。 */
    int classId;
    /** @brief 掩码系数（可能用于生成实例分割掩码）。 */
    float mask[32];
    /** @brief 掩码矩阵（可能是另一种形式的掩码表示）。 */
    std::vector<float> maskMatrix;
};

/**
 * @brief 用于表示带有物理尺寸和3D中心点的矩形。
 */
struct RealRectangle
{
    /** @brief 矩形的四个顶点坐标 (cv::Point)。 */
    std::vector<cv::Point> location;
    /** @brief 矩形的物理面积（单位可能需要根据上下文确定）。 */
    float area;
    /** @brief 矩形在三维空间中的中心点坐标 [x, y, z]。 */
    std::array<float, 3> center;

    /**
     * @brief RealRectangle 的显式构造函数。
     * @param loc 矩形的顶点列表。
     * @param a 矩形的物理面积。
     * @param c 矩形的3D中心点坐标。
     */
    RealRectangle(const std::vector<cv::Point>& loc, float a, const std::array<float, 3>& c)
        : location(loc), area(a), center(c) {}
};

/**
 * @brief 用于存储目标检测结果的结构体（可能是更新的版本或不同模型的输出格式）。
 */
struct Detection3 {
    /** @brief 边界框 (cv::Rect)。 */
    cv::Rect bbox;
    /** @brief 检测结果的置信度。 */
    float confidence;
    /** @brief 检测到的物体类别 ID。 */
    int class_id;
    /** @brief 指向掩码数据的指针（需要调用者管理内存）。 */
    float* maskMatrix = nullptr; // 掩码数据指针（初始化为空）
    /** @brief 掩码的宽度。 */
    int maskWidth = 0;
    /** @brief 掩码的高度。 */
    int maskHeight = 0;
};


#endif  // TYPES_H
