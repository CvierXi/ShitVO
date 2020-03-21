//
// Created by sunxi on 3/21/20.
//

#ifndef SHITVO_FRONT_TRACKER_H
#define SHITVO_FRONT_TRACKER_H

#include "core/common.h"

class FrontTracker {
public:
    struct FrontConfig {
        int feature_threshold;
        int feature_max_num;
        int pyr_max_level;
        int pyr_win_size;
        bool enable_show_optical_flow;
    };

    explicit FrontTracker(FrontConfig& config);
    void imgCallback(const cv::Mat& img);
    void getTrackPoints(std::vector<cv::Point2f>& track_src_pts, std::vector<cv::Point2f>& track_dst_pts);
    void getTrackPointsNormalized(std::vector<cv::Point2f>& track_src_pts, std::vector<cv::Point2f>& track_dst_pts);
    M3d getNormMatrix();

private:
    void trackFeatures();
    void addFeatures();

    const char* TAG = "FrontTracker";
    FrontConfig config_;
    cv::Ptr<cv::FastFeatureDetector> fast_detector_;
    cv::Size pyr_win_size_;
    std::vector<cv::Mat> prev_pyr_, next_pyr_;
    std::vector<cv::Point2f> prev_pts_, next_pts_;
    std::vector<cv::Point2f> track_src_pts_, track_dst_pts_;
    M3d norm_matrix_;
    bool is_init_;
};

#endif //SHITVO_FRONT_TRACKER_H
