#include <dirent.h>
#include <random>
#include <opencv2/opencv.hpp>


class ChallengeUtils {
    public:
        /**
         * @brief ChallengeUtils 类的构造函数。
         * @param logFile 用于记录日志的文件路径。
         */
        ChallengeUtils(const std::string& logFile);
        /**
         * @brief 根据给定的中心点、宽度、高度和旋转角度计算矩形的顶点。
         * @param x 矩形中心点的 x 坐标。
         * @param y 矩形中心点的 y 坐标。
         * @param w 矩形的宽度。
         * @param h 矩形的高度。
         * @param r 矩形的旋转角度（可能是弧度或角度，需要确认）。
         * @return 返回包含矩形四个顶点的 cv::Point 向量。
         */
        std::vector<cv::Point> calculate_vertices(int x, int y, int w, int h, double r);
        /**
         * @brief 计算两个 OpenCV 点之间的欧氏距离。
         * @param point1 第一个点。
         * @param point2 第二个点。
         * @return 返回两点之间的距离。
         */
        double calculate_distance(const cv::Point& point1, const cv::Point& point2);
        /**
         * @brief 计算给定顶点定义的形状的水平中心线段的两个端点。
         * @param vertices 形状的顶点列表。
         * @return 返回一个包含两个 cv::Point 的 std::pair，代表水平中心线段的端点。
         */
        std::pair<cv::Point, cv::Point> calculate_h_center(const std::vector<cv::Point>& vertices);
        /**
         * @brief 计算由两组点定义的两条线段之间的角度。
         * @param points1 定义第一条线段的点集（通常是两个点）。
         * @param points2 定义第二条线段的点集（通常是两个点）。
         * @return 返回两条线段之间的角度（可能是弧度或角度，需要确认）。
         */
        double angle_between_lines(const std::vector<cv::Point>& points1, const std::vector<cv::Point>& points2);
        /**
         * @brief 从一组数据中移除离群值。
         * @param data 包含 double 类型数据的向量。
         * @param threshold 用于判断离群值的阈值。具体判断方法依赖于实现。
         * @return 返回移除了离群值后的数据向量。
         */
        std::vector<double> remove_outliers(const std::vector<double>& data, double threshold);
    
    private:
        std::string log_file_;
    };
    
/**
 * @brief 读取指定目录下的所有文件名。
 * @param p_dir_name 要读取的目录路径。
 * @param file_names 用于存储读取到的文件名的字符串向量（输出参数）。
 * @return 成功返回 0，打开目录失败返回 -1。
 */
static inline int read_files_in_dir(const char* p_dir_name, std::vector<std::string>& file_names)
{
    DIR *p_dir = opendir(p_dir_name);
    if (p_dir == nullptr) {
        return -1;
    }

    struct dirent* p_file = nullptr;
    while ((p_file = readdir(p_dir)) != nullptr) {
        if (strcmp(p_file->d_name, ".") != 0 &&
            strcmp(p_file->d_name, "..") != 0) {
            //std::string cur_file_name(p_dir_name);
            //cur_file_name += "/";
            //cur_file_name += p_file->d_name;
            std::string cur_file_name(p_file->d_name);
            file_names.push_back(cur_file_name);
        }
    }

    closedir(p_dir);
    return 0;
}


/**
 * @brief 生成指定范围内的随机整数。
 * @param minThres 随机数的最小值（包含）。默认为 0。
 * @param maxThres 随机数的最大值（包含）。默认为 255。
 * @return 返回一个在 [minThres, maxThres] 区间内的随机整数。
 */
static inline int get_random_int(int minThres=0, int maxThres=255){
    // 获取处于某一范围内的一个随机整数
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(minThres, maxThres);

    int random_integer = distrib(gen);

    return random_integer;
}

