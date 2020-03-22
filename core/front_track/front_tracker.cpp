//
// Created by sunxi on 3/21/20.
//

#include <numeric>

#include "front_tracker.h"
#include "core/util/logging.h"

#define FRONT_FEATURE_TH_MIN 10
#define FRONT_FEATURE_TH_MAX 30
#define FRONT_PYR_LEVEL_MIN 0
#define FRONT_PYR_LEVEL_MAX 3
#define FRONT_PYR_WIN_SIZE_DEFAULT 21
#define FRONT_KLT_BACK_TH 0.25f

FrontTracker::FrontTracker(FrontConfig &config) : config_(config) {
    if (config_.feature_threshold < FRONT_FEATURE_TH_MIN || config_.feature_threshold > FRONT_FEATURE_TH_MAX) {
        LOGE(TAG, "Input parameter - 'feature_threshold' must in [%d, %d]; now use default value: %d", FRONT_FEATURE_TH_MIN, FRONT_FEATURE_TH_MAX, FRONT_FEATURE_TH_MIN);
        config_.feature_threshold = FRONT_FEATURE_TH_MIN;
    }
    fast_detector_ = cv::FastFeatureDetector::create(config_.feature_threshold);
    if (config_.pyr_max_level < FRONT_PYR_LEVEL_MIN || config_.pyr_max_level > FRONT_PYR_LEVEL_MAX) {
        LOGE(TAG, "Input parameter - 'pyr_max_level' must in [%d, %d]; now use default value: %d", FRONT_PYR_LEVEL_MIN, FRONT_PYR_LEVEL_MAX, FRONT_PYR_LEVEL_MAX);
        config_.pyr_max_level = FRONT_PYR_LEVEL_MAX;
    }
    if (config_.pyr_win_size % 2 == 0) {
        LOGE(TAG, "Input parameter - 'pyr_win_size' must be odd; now use default value: %d", FRONT_PYR_WIN_SIZE_DEFAULT);
        config_.pyr_win_size = FRONT_PYR_WIN_SIZE_DEFAULT;
    }
    pyr_win_size_ = cv::Size(config_.pyr_win_size, config_.pyr_win_size);
    norm_matrix_.setIdentity();
    is_init_ = false;
}

void FrontTracker::imgCallback(const cv::Mat &img) {
    prev_pyr_ = next_pyr_;
    prev_pts_ = next_pts_;
    next_pyr_.clear();
    cv::buildOpticalFlowPyramid(img, next_pyr_, pyr_win_size_, config_.pyr_max_level);
    if (!prev_pyr_.empty()) {
        trackFeatures();
//        LOGD(TAG, "trackFeatures result: %d -> %d", prev_pts_.size(), track_src_pts_.size());
    }
    addFeatures();
    if (!is_init_) {
        norm_matrix_ << 1.0 / img.cols, 0, 0,
                        0, 1.0 / img.cols, 0,
                        0, 0, 1;
        is_init_ = true;
    }
}

void FrontTracker::getTrackPoints(vector<cv::Point2f>& track_src_pts, vector<cv::Point2f>& track_dst_pts) {
//    track_src_pts.assign(track_src_pts_.begin(), track_src_pts_.end());
//    track_dst_pts.assign(track_dst_pts_.begin(), track_dst_pts_.end());
    track_src_pts = track_src_pts_;
    track_dst_pts = track_dst_pts_;
}

/*
 * To ensure the mathmatic stability for calculating about p(p_x, p_y, 1), because p_x can be 600 times than p_z!
 * There are kinds of normlization methods, such as
 *   - scale to [0,1]
 *   - scale to [-1,1]
 *   - using unit bearing vector
 *   - using centroid of points
 *   - ...
 */
void FrontTracker::getTrackPointsNormalized(vector<cv::Point2f>& track_src_pts, vector<cv::Point2f>& track_dst_pts) {
    int num_pts = track_src_pts_.size();
    if (num_pts == 0) {
        return;
    }
    int img_w = prev_pyr_[0].cols;
    int img_h = prev_pyr_[0].rows;
    for (int i = 0; i < num_pts; i++) {
        cv::Point2f track_src_pt(track_src_pts_[i].x / img_w, track_src_pts_[i].y / img_w);
        cv::Point2f track_dst_pt(track_dst_pts_[i].x / img_w, track_dst_pts_[i].y / img_w);
        track_src_pts.push_back(track_src_pt);
        track_dst_pts.push_back(track_dst_pt);
    }
}


void FrontTracker::trackFeatures() {
    int n = prev_pts_.size();
    track_src_pts_.clear();
    track_dst_pts_.clear();
    vector<cv::Point2f> klt_next_pts, klt_back_pts;
    vector<uchar> klt_next_status, klt_back_status;
    cv::calcOpticalFlowPyrLK(prev_pyr_, next_pyr_, prev_pts_, klt_next_pts, klt_next_status, cv::noArray(), pyr_win_size_, config_.pyr_max_level);
    cv::calcOpticalFlowPyrLK(next_pyr_, prev_pyr_, klt_next_pts, klt_back_pts, klt_back_status, cv::noArray(), pyr_win_size_, config_.pyr_max_level);
    for (int i = 0; i < n; i++) {
        if (klt_next_status[i] && klt_back_status[i]) {
            double back_error = (klt_back_pts[i] - prev_pts_[i]).dot((klt_back_pts[i] - prev_pts_[i]));
            if (back_error < FRONT_KLT_BACK_TH) {
                track_src_pts_.push_back(prev_pts_[i]);
                track_dst_pts_.push_back(klt_next_pts[i]);
            }
        }
    }
}

/// todo: use continuous feature_track
void FrontTracker::addFeatures() {
    next_pts_.clear();
//    next_pts_.assign(track_dst_pts_.begin(), track_dst_pts_.end());
    int num_pts = next_pts_.size();
    vector<cv::KeyPoint> kps;
    fast_detector_->detect(next_pyr_[0], kps);

    vector<int> kp_responses;
    for (const auto& kp : kps) {
        kp_responses.push_back(kp.response);
    }
    vector<int> idx(kp_responses.size());
    iota(std::begin(idx), std::end(idx), 0);
    cv::sortIdx(kp_responses, idx, CV_SORT_DESCENDING);

    int num_new_kps = kps.size();
    int num_new_features = min(config_.feature_max_num - num_pts, num_new_kps);
//    LOGD(TAG, "addFeatures request: %d", num_new_features);
    for (int i = 0; i < num_new_features; i++) {
        next_pts_.emplace_back(kps[idx[i]].pt.x, kps[idx[i]].pt.y);
    }
//    LOGD(TAG, "addFeatures result: %d -> %d", num_pts, next_pts_.size());
}

M3d FrontTracker::getNormMatrix() {
    return norm_matrix_;
}
