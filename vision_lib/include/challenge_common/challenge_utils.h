

#include <dirent.h>
#include <random>
#include <opencv2/opencv.hpp>


class ChallengeUtils {
    public:
        ChallengeUtils(const std::string& logFile);
        std::vector<cv::Point> calculate_vertices(int x, int y, int w, int h, double r);
        double calculate_distance(const cv::Point& point1, const cv::Point& point2);
        std::pair<cv::Point, cv::Point> calculate_h_center(const std::vector<cv::Point>& vertices);
        double angle_between_lines(const std::vector<cv::Point>& points1, const std::vector<cv::Point>& points2);
        std::vector<double> remove_outliers(const std::vector<double>& data, double threshold);
    
    private:
        std::string log_file_;
    };
    
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


static inline int get_random_int(int minThres=0, int maxThres=255){
    // 获取处于某一范围内的一个随机整数
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(minThres, maxThres);

    int random_integer = distrib(gen);

    return random_integer;
}

