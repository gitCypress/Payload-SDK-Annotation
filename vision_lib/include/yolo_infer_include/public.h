#ifndef PUBLIC_H
#define PUBLIC_H

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string.h>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>
#include <map>

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>

#define CHECK(call) check(call, __LINE__, __FILE__)

/**
 * @brief 检查 CUDA API 调用的返回值。
 * @param e CUDA API 返回的 cudaError_t 错误码。
 * @param iLine 调用发生处的行号 (通常由 __LINE__ 宏提供)。
 * @param szFile 调用发生处的文件名 (通常由 __FILE__ 宏提供)。
 * @return 如果调用成功 (e == cudaSuccess)，返回 true；否则打印错误信息并返回 false。
 */
inline bool check(cudaError_t e, int iLine, const char *szFile)
{
    if (e != cudaSuccess)
    {
        std::cout << "CUDA runtime API error " << cudaGetErrorName(e) << " at line " << iLine << " in file " << szFile << std::endl;
        return false;
    }
    return true;
}

using namespace nvinfer1;


/**
 * @brief TensorRT 日志记录器类。
 *        实现了 nvinfer1::ILogger 接口，用于捕获和打印 TensorRT 的日志信息。
 */
class Logger : public ILogger
{
public:
    Severity reportableSeverity;

    /**
     * @brief Logger 构造函数。
     * @param severity 要报告的最低日志严重级别，默认为 Severity::kINFO。
     *                 低于此级别的日志将被忽略。
     */
    Logger(Severity severity = Severity::kINFO):
        reportableSeverity(severity) {}

    /**
     * @brief 实现 ILogger 接口的日志记录方法。
     * @param severity 当前日志消息的严重级别。
     * @param msg 日志消息内容。
     */
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity > reportableSeverity)
        {
            return;
        }
        switch (severity)
        {
        case Severity::kINTERNAL_ERROR:
            std::cerr << "INTERNAL_ERROR: ";
            break;
        case Severity::kERROR:
            std::cerr << "ERROR: ";
            break;
        case Severity::kWARNING:
            std::cerr << "WARNING: ";
            break;
        case Severity::kINFO:
            std::cerr << "INFO: ";
            break;
        default:
            std::cerr << "VERBOSE: ";
            break;
        }
        std::cerr << msg << std::endl;
    }
};


/**
 * @brief 获取 TensorRT 数据类型对应的字节大小。
 * @param dataType TensorRT 数据类型枚举值 (nvinfer1::DataType)。
 * @return 返回对应数据类型的字节大小 (size_t)。对于未知类型，默认返回 4。
 */
__inline__ size_t dataTypeToSize(nvinfer1::DataType dataType)
{
    switch ((int)dataType)
    {
    case int(nvinfer1::DataType::kFLOAT):
        return 4;
    case int(nvinfer1::DataType::kHALF):
        return 2;
    case int(nvinfer1::DataType::kINT8):
        return 1;
    case int(nvinfer1::DataType::kINT32):
        return 4;
    case int(nvinfer1::DataType::kBOOL):
        return 1;
    default:
        return 4;
    }
}

/**
 * @brief 将 TensorRT 的维度信息转换为字符串表示。
 * @param dim TensorRT 维度对象 (Dims32)。
 * @return 返回维度的字符串表示，格式为 "(d1, d2, ..., dn)"。
 */
__inline__ std::string shapeToString(Dims32 dim)
{
    std::string output("(");
    if (dim.nbDims == 0)
    {
        return output + std::string(")");
    }
    for (int i = 0; i < dim.nbDims - 1; i++)
    {
        output += std::to_string(dim.d[i]) + std::string(", ");
    }
    output += std::to_string(dim.d[dim.nbDims - 1]) + std::string(")");
    return output;
}

/**
 * @brief 将 TensorRT 数据类型枚举值转换为字符串表示。
 * @param dataType TensorRT 数据类型枚举值 (nvinfer1::DataType)。
 * @return 返回数据类型的字符串表示 (e.g., "FP32", "FP16", "INT8")。
 */
__inline__ std::string dataTypeToString(nvinfer1::DataType dataType)
{
    switch (dataType)
    {
    case nvinfer1::DataType::kFLOAT:
        return std::string("FP32 ");
    case nvinfer1::DataType::kHALF:
        return std::string("FP16 ");
    case nvinfer1::DataType::kINT8:
        return std::string("INT8 ");
    case nvinfer1::DataType::kINT32:
        return std::string("INT32");
    case nvinfer1::DataType::kBOOL:
        return std::string("BOOL ");
    default:
        return std::string("Unknown");
    }
}

#endif  // PUBLIC_H
