#ifndef TASK_OBBTRACK_H
#define TASK_OBBTRACK_H

#include <cmath>
#include <vector>
#include <numeric>
// #include <logging.h>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include "track.h"
#include "rotation_detector.h"
#include "synt_detector_type.h"

#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"

namespace stage_2
{

    struct ConvexHullData
    {
        std::vector<cv::Point2f> hull;
        cv::Point2f center;
        float radius;
        double omiga;

        ConvexHullData() : center(cv::Point2f(0, 0)), radius(0.0), omiga(0.0) {}
        // 带参数构造函数
        ConvexHullData(const std::vector<cv::Point2f>& h, const cv::Point2f& c, float r, double o)
        : hull(h), center(c), radius(r), omiga(o) {}
    };

    class KinematicState
    {
    public:
        KinematicState();
        ~KinematicState();

        std::pair<stage_2::ConvexHullData, std::string> operator() (cv::Mat& image, int frame_count, int camera_fps, float scale);

        void update(cv::Mat& image, int frame_count, int camera_fps);

        std::pair<ConvexHullData, std::string> post_processing(float scale);

        std::string analyze_motion_state();

        bool analyze_static(const std::vector<double> moves, double threshold = 2.0);

        bool analyze_translation_speed(const std::vector<double> move_diffs, double threshold = 1.0);

        bool analyze_rotation_rate(const std::vector<double> similarity_rates, double threshold = 0.2);

        double calculate_angle_speed(const std::vector<double> angle_diffs, double threshold = 0.2);

        std::vector<double> remove_outliers(const std:: vector<double>& data, double threshold = 1.0);

        ConvexHullData largestConvexHullArea(const std::vector<std::vector<cv::Point2f>>& sequences, float scale);

        std::string calculate_rotation_direction_multi(const std::vector<cv::Point2f>& points, const cv::Point2f& center);

        static double calculate_mean(const std::vector<double>& data) 
        {
            double sum = std::accumulate(data.begin(), data.end(), 0.0);
            return sum / data.size();
        }

        static double calculate_std_dev(const std::vector<double>& data, double mean) {
            double sum_sq_diff = 0.0;
            for (double x : data) {
                sum_sq_diff += (x - mean) * (x - mean);
            }
            return std::sqrt(sum_sq_diff / data.size());
        }

        static double calculateDistance(const cv::Point2f& point1, const cv::Point2f& point2) 
        {
            return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
        }

        static double angle_between_lines(const std::vector<cv::Point2f>& point_1, const std::vector<cv::Point2f>& point_2);

        std::vector<double> omgia_list_ = {};

    private:
        apriltag_family_t *tf;
        apriltag_detector_t *td;
        apriltag_detection_t *det;

        bool is_circle_;
        std::string current_track_id_; //干什么用
        int frame_interval_ = 5;
        int min_idle_frames_ = 50;
        int max_idle_frames_ = 100;

        double omiga_ = 0.0;
        std::vector<cv::Point2f> best_hull_ = {};

        std::map<int, bool> change_flag_;
        std::map<std::string, int> last_update_;
        std::map<std::string, std::vector<std::vector<cv::Point2f>>> track_point_;

        std::vector<double> moves_ = {};
        std::vector<double> move_diffs_ = {};
        std::vector<double> angle_diffs_ = {};
        std::vector<double> similarity_rates_ = {};
        std::vector<std::vector<cv::Point2f>> circle_coords_ = {};

        // stage_2::Track track_;
        stage_2::RealTimeRotationDetector rotation_detector_;
        OBJECTLIST* object_list_;

        std::map<double, int> omgia_length_ = {{0.3, 2000}, {0.5, 1000}, {0.7, 300}};
    };
}

#endif // TASK_OBBTRACK_H