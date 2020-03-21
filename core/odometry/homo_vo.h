//
// Created by sunxi on 3/20/20.
//

#ifndef SHITVO_HOMO_VO_H
#define SHITVO_HOMO_VO_H

#include "core/common.h"
#include "core/camera/camera.h"
#include "core/front_track/front_tracker.h"

enum TrackingState {
    STATE_UNKNOWN,
    STATE_INIT,
    STATE_TRACKING,
    STATE_LOST
};

struct Pose {
    V3d p;
    Q4d q;
    TrackingState state;

    Pose();
    Pose(V3d _p, Q4d _q);
};

class HomoVo {
public:
    explicit HomoVo(const std::string& config_file_path);

    void reset();

    void imuCallback(double timestamp, const M3d& R_w_b);
    void imgCallback(double timestamp, const cv::Mat& img);

    shared_ptr<Camera> getCamera() const;
    Pose getCameraPose() const;
    M3d getHomography() const;

private:
    void computeCameraPose();

    const char* TAG = "HomoVo";
    bool is_init_;
    shared_ptr<Camera> camera_;
    shared_ptr<FrontTracker> front_tracker_;
    double time_imu_, time_img_;
    M3d R_w_b_, R_b_c_, R_w_c0_, H_c_w_, H_c0_w_, H_c_c0_;
    V3d t_w_c0_;
    Pose pose_;
};

#endif //SHITVO_HOMO_VO_H
