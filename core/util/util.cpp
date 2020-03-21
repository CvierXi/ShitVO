//
// Created by sunxi on 3/20/20.
//

#include "util.h"

namespace util {

void recoverPoseFromHomography(const M3d& H, M3d& R, V3d& t) {
    V3d r1 = H.col(0);
    const float scale1 = r1.norm();
    r1 = r1 / scale1;

    V3d r2 = H.col(1);
    r2 -= r2.dot(r1) * r1;
    const float scale2 = r2.norm();
    r2 = r2 / scale2;

    V3d r3 = r1.cross(r2);

    M3d rotation;
    rotation << r1, r2, r3;

    Eigen::JacobiSVD<M3d> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R.noalias() = svd.matrixU() * svd.matrixV().transpose();
    t = H.col(2) / (scale1 + scale2) * 2.f;
}

void constructHomographyFromPose(M3d& H, const M3d& R, const V3d& t) {
    H = R;
    H.col(2) = t;
}

}