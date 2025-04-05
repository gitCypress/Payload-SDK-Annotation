#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>

// === General Configuration ===
/** @brief 使用的 GPU 设备 ID。*/
const int kGpuId = 0;
/** @brief 模型预测的总类别数。*/
const int kNumClass = 4;
/** @brief 模型输入的图像高度。*/
const int kInputH = 640;
/** @brief 模型输入的图像宽度。*/
const int kInputW = 640;

// === NMS and Confidence Thresholds ===
/** @brief 非极大值抑制 (NMS) 的 IoU 阈值。*/
const float kNmsThresh = 0.45f;
/** @brief 检测结果的置信度阈值。低于此阈值的检测将被忽略。*/
const float kConfThresh = 0.25f;

// === Output Configuration ===
/** @brief 预估的最大输出边界框数量（用于预分配内存）。*/
const int kMaxNumOutputBbox = 1000;  // assume the box outputs no more than kMaxNumOutputBbox boxes that conf >= kNmsThresh;
/** @brief 每个检测框输出元素的数量（bbox坐标[4] + conf[1] + class[1] + keepflag[1] + mask系数[32]）。*/
const int kNumBoxElement = 7 + 32;  // left, top, right, bottom, confidence, class, keepflag(whether drop when NMS), 32 masks

// === File Paths ===
/** @brief ONNX 模型文件的路径。*/
const std::string onnxFile = "/home/sdses/zh_demo/task1/level-3/model/best.onnx";
// const std::string trtFile = "./yolo11s.plan";
// const std::string testDataDir = "../images";  // 用于推理

// === Precision Modes ===
/** @brief 是否启用 FP16 模式。*/
const bool bFP16Mode = true;
/** @brief 是否启用 INT8 模式。*/
const bool bINT8Mode = false;
/** @brief INT8 校准缓存文件的路径。*/
const std::string cacheFile = "./int8.cache";
/** @brief INT8 量化校准数据集的路径。*/
const std::string calibrationDataPath = "../calibrator";  // 存放用于 int8 量化校准的图像

// === Class Names ===
/** @brief 包含所有类别名称的向量。顺序应与模型训练时一致。*/
const std::vector<std::string> vClassNames {
    "turnable", "circle", "triangle", "rectangle",
};

#endif  // CONFIG_H
