//
// Created by sunxi on 3/20/20.
//

#include "camera.h"

Camera::Camera(float fx, float fy, float cx, float cy) {
    K_ << fx,  0, cx,
           0, fy, cy,
           0,  0,  1;
}

V3d Camera::projectToCamera(const V3d &P_c) {
    V3d p_c = K_ * P_c;
    p_c /= p_c(2);
    return p_c;
}

cv::Point2f Camera::projectToCameraPixel(const V3d &P_c) {
    V3d p_c = projectToCamera(P_c);
    cv::Point2f pt(p_c(0), p_c(1));
    return pt;
}

Camera::Camera(M3d& K) : K_(K) {
}

void Camera::setK(M3d &K) {
    K_ = K;
}

M3d Camera::K() {
    return K_;
}

float Camera::fx() {
    return static_cast<float>(K_(0, 0));
}

float Camera::fy() {
    return static_cast<float>(K_(1, 1));
}

float Camera::cx() {
    return static_cast<float>(K_(0, 2));
}

float Camera::cy() {
    return static_cast<float>(K_(1, 2));
}
