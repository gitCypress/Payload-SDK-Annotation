#ifndef __SYNT_DETECTOR_TYPE__
#define __SYNT_DETECTOR_TYPE__
#include <string>
#include <iostream>

#define GPU_RESIZE 0
#define GPU_LETTERBOX 1
#define CPU_RESIZE 2
#define CPU_LETTERBOX 3
using namespace std;

/// @brief 网络信息
struct NETMSG
{
    int width;       // 宽度
    int height;      // 高度
    int channel;     // 通道数
    int batch_size;  // 单次推理图片数目
    int num_classes; // 类别数
    float x;
    float y;
    float z;
    char msg0[1280];
    char msg1[1280];
    char msg2[1280];
    /// @brief 网络信息初始化
    NETMSG()
    {
        width = 0;
        height = 0;
        channel = 0;
        batch_size = 0;
        num_classes = 0;
        x = 0;
        y = 0;
        z = 0;
    }
};

/// @brief 配置信息
struct CONFIGURE
{
    string model_path;   // 模型路径
    int object_num;      // 输出最大目标个数
    int preprocess_mode; // 预处理模式
    int gpu_id;          // gpu id
    int model_id;        // 模型编号
    /// 配置信息初始化
    CONFIGURE()
    {
        object_num = 100;
        preprocess_mode = 0;
    }
};

/// @brief 目标信息结构体
struct SSObject
{
    int label;     // 目标类别
    int track_id;
    float prob;    // 目标得分
    float x1; // 目标x1坐标
    float y1; // 目标y1坐标
    float x2; // 目标x2坐标
    float y2; // 目标y2坐标
    float angle;    // 角度
    std::vector<float> final_mask;
    std::vector<float> high_res_mask;
    /// 目标信息结构体初始化
    SSObject()
    {
        prob = 0;
        x1 = 0;
        y1 = 0;
        x2 = 0;
        y2 = 0;
    }
};

/// @brief 目标列表信息
struct OBJECTLIST
{
    int capacity;  // 最大目标数
    int count;     // 目标数目
    SSObject *obj; // 目标指针
    OBJECTLIST()
    {
        capacity = 100;
        count = 0;
    }
};

/// @brief 检测目标信息
struct SSBoxInfo
{
    // x y w h
    int label;   // 目标类别
    int track_id;
    float mask[32]; // 掩码系数.
    float bbox[4]; // 目标位置坐标信息
    float prob;    // 目标得分
    float angle;   // 角度
    std::vector<float> final_mask;
    std::vector<float> high_res_mask;
};

// struct Detection
// {
//     // x1, y1, x2, y2
//     float bbox[4];
//     float conf;
//     int classId;
//     float mask[32];  // mask coefficient
//     std::vector<float> maskMatrix;  // 2D mask after mask coefficient multiply proto, and scale to original image
// };

#endif
