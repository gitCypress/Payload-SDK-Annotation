#include <deque>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>

namespace stage_2
{
    struct Point {
        double x;
        double y;
        Point(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
    };

    class RealTimeRotationDetector 
    {
    public:
        RealTimeRotationDetector(size_t max_length = 50, int min_cycles = 1, double distance_threshold = 0.1);
        bool operator() (const cv::Point2f& point);
        ~RealTimeRotationDetector();
    private:
        size_t max_length_;
        int min_cycles_;
        double distance_threshold_;
        std::deque<cv::Point2f> trajectory_;
        std::deque<double> distances_;
        int zero_crossings_;
    };
}
