//
// Created by sunxi on 3/20/20.
//

#ifndef SHITVO_CAMERA_H
#define SHITVO_CAMERA_H

#include "core/common.h"

class Camera {
public:
    explicit Camera(float fx, float fy, float cx, float cy);
    explicit Camera(M3d& K);

    V3d projectToCamera(const V3d& P_c);
    cv::Point2f projectToCameraPixel(const V3d& P_c);

    void setK(M3d& K);
    M3d K();
    float fx();
    float fy();
    float cx();
    float cy();

private:
    M3d K_;
};

#endif //SHITVO_CAMERA_H
