//
// Created by sunxi on 3/20/20.
//

#include "homo_vo.h"
#include "core/config_parse/config_parser.h"
#include "core/util/util.h"
#include "core/util/logging.h"

Pose::Pose() {
    p = V3d(0, 0, 0);
    q = Q4d(1, 0, 0 , 0);
    state = STATE_UNKNOWN;
}

Pose::Pose(V3d _p, Q4d _q) : p(_p), q(_q) {
    state = STATE_UNKNOWN;
}

HomoVo::HomoVo(const string& config_file_path) {
    if (!ConfigParser::setParameterFile(config_file_path)) {
        LOGE(TAG, "Invalid config_file_path");
        return;
    }

    auto fx = ConfigParser::get<float>("camera.fx");
    auto fy = ConfigParser::get<float>("camera.fy");
    auto cx = ConfigParser::get<float>("camera.cx");
    auto cy = ConfigParser::get<float>("camera.cy");
    camera_ = make_shared<Camera>(fx, fy, cx, cy);

    FrontTracker::FrontConfig config{};
    config.feature_threshold = ConfigParser::get<int>("feature_threshold");
    config.feature_max_num = ConfigParser::get<int>("feature_max_num");
    config.pyr_max_level = ConfigParser::get<int>("pyr_max_level");
    config.pyr_win_size = ConfigParser::get<int>("pyr_win_size");
    config.enable_show_optical_flow = ConfigParser::get<int>("enable_show_optical_flow");
    front_tracker_ = make_shared<FrontTracker>(config);

    time_imu_ = -1;
    time_img_ = -1;
    R_b_c_ << 0, -1, 0, -1, 0, 0, 0, 0, -1;

    reset();
}

void HomoVo::reset() {
    H_c_c0_.setIdentity();
    t_w_c0_ = V3d(0, 0, 1);
    is_init_ = false;
}

void HomoVo::imuCallback(double timestamp, const M3d& R_w_b) {
//    LOGD(TAG, "imuCallback");
    time_imu_ = timestamp;
    R_w_b_ = R_w_b;
}

void HomoVo::imgCallback(double timestamp, const cv::Mat &img) {
//    LOGD(TAG, "imgCallback");
    if (time_imu_ < 0 || timestamp - time_imu_ > 0.1) {
        LOGE(TAG, "Invalid img timestamp.");
        return;
    }
//    LOGI(TAG, "process img ...");
    front_tracker_->imgCallback(img);
    computeCameraPose();
}

void HomoVo::computeCameraPose() {
    vector<cv::Point2f> track_src_pts, track_dst_pts;
    front_tracker_->getTrackPointsNormalized(track_src_pts, track_dst_pts);
//    front_tracker_->getTrackPoints(track_src_pts, track_dst_pts);
    bool track_suc = false;
    if (!is_init_) {
        R_w_c0_ = R_w_b_ * R_b_c_;
        M3d R_c0_w = R_w_c0_.transpose();
        V3d t_c0_w = -R_c0_w * t_w_c0_;
        util::constructHomographyFromPose(H_c0_w_, R_c0_w, t_c0_w);
        pose_.state = STATE_INIT;
        LOGI(TAG, "Init!");
        is_init_ = true;
    } else if (!track_src_pts.empty()) {
        /// p2 = H * p1
        /// But notice that, camera moving right, pts in image moving left.
        cv::Mat H_cv = cv::findHomography(track_src_pts, track_dst_pts, cv::RANSAC, 0.1);
        M3d H;
        cv::cv2eigen(H_cv, H);

        H_c_c0_ = H * H_c_c0_;
        pose_.state = STATE_TRACKING;
    } else {
        /// todo: use delta_R
        pose_.state = STATE_LOST;
    }

    H_c_w_ = H_c_c0_ * H_c0_w_;

    M3d R_c_w;
    V3d t_c_w;
    util::recoverPoseFromHomography(H_c_w_, R_c_w, t_c_w);

    M3d R_w_c = R_c_w.transpose();
    V3d t_w_c = - R_w_c * t_c_w;

    pose_.p = t_w_c;
    pose_.q = Q4d(R_w_c);

//    M3d iR_c_c0 = R_w_c.transpose() * R_w_c0_;
//    util::constructHomographyFromPose(H_c_c0_, iR_c_c0, t_c_c0);
}

shared_ptr<Camera> HomoVo::getCamera() const {
    return camera_;
}

Pose HomoVo::getCameraPose() const {
    return pose_;
}

M3d HomoVo::getHomography() const {
    M3d N = front_tracker_->getNormMatrix();
    return N.inverse() * H_c_w_ * N;
}
