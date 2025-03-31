#ifndef TYPES_H
#define TYPES_H

#include <string>
#include <vector>


struct Detection
{
    // x1, y1, x2, y2
    float bbox[4];
    float conf;
    int classId;
    float mask[32];  // mask coefficient
    std::vector<float> maskMatrix;  
};

struct RealRectangle
{
    std::vector<cv::Point> location;
    float area;
    std::array<float, 3> center;

    // 显式构造函数
    RealRectangle(const std::vector<cv::Point>& loc, float a, std::array<float, 3> c)
        : location(loc), area(a), center(c) {}
};

struct Detection3 {
    cv::Rect bbox;          // 边界框
    float confidence;       // 置信度
    int class_id;          // 类别ID
    float* maskMatrix = nullptr; // 掩码数据指针（初始化为空）
    int maskWidth = 0;     // 掩码宽度
    int maskHeight = 0;    // 掩码高度
};


#endif  // TYPES_H
