#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <string>
#include <vector>
#include <NvInfer.h>

using namespace nvinfer1;


class Int8EntropyCalibrator2 : public IInt8EntropyCalibrator2
{
public:
    /**
     * @brief Int8EntropyCalibrator2 类的构造函数。
     * @param batch_size 校准时使用的批量大小。
     * @param input_w 模型输入的宽度。
     * @param input_h 模型输入的高度。
     * @param img_dir 包含校准图像的目录路径。
     * @param calib_table_name 校准缓存文件的名称。
     * @param read_cache 是否尝试读取已存在的校准缓存文件，默认为 true。
     */
    Int8EntropyCalibrator2(int batch_size, int input_w, int input_h, const char* img_dir, const char* calib_table_name, bool read_cache=true);

    /**
     * @brief Int8EntropyCalibrator2 类的虚析构函数。
     *        负责释放分配的设备内存。
     */
    virtual ~Int8EntropyCalibrator2();

    /**
     * @brief 获取校准时使用的批量大小。
     *        实现 IInt8Calibrator 接口。
     * @return 返回配置的批量大小。
     */
    int getBatchSize() const noexcept override;

    /**
     * @brief 获取一个批次的校准数据。
     *        实现 IInt8Calibrator 接口。
     * @param bindings 指向设备缓冲区的指针数组，用于存放输入数据。
     * @param names 输入/输出张量的名称数组。
     * @param nbBindings 绑定的数量。
     * @return 如果成功获取一个批次的数据，则返回 true；如果所有数据都已处理完毕，则返回 false。
     */
    bool getBatch(void* bindings[], const char* names[], int nbBindings) noexcept override;

    /**
     * @brief 读取校准缓存。
     *        实现 IInt8Calibrator 接口。
     * @param length 用于接收缓存大小的引用（输出参数）。
     * @return 返回指向校准缓存数据的指针；如果缓存不可用，则返回 nullptr。
     */
    const void* readCalibrationCache(size_t& length) noexcept override;

    /**
     * @brief 写入校准缓存。
     *        实现 IInt8Calibrator 接口。
     * @param cache 指向要写入的缓存数据的指针。
     * @param length 要写入的缓存数据的大小。
     */
    void writeCalibrationCache(const void* cache, size_t length) noexcept override;

private:
    int batch_size_;
    int input_w_;
    int input_h_;
    int img_idx_;
    std::string img_dir_;
    std::vector<std::string> img_files_;
    size_t input_count_;
    std::string calib_table_name_;
    bool read_cache_;
    float* batch_data;
    void* device_input_;
    std::vector<char> calib_cache_;
};

#endif  // CALIBRATOR_H
