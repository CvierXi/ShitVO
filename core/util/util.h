//
// Created by sunxi on 3/20/20.
//

#ifndef SHITVO_UTIL_H
#define SHITVO_UTIL_H

#include "core/common.h"

namespace util {

void recoverPoseFromHomography(const M3d& H, M3d& R, V3d& t);

void constructHomographyFromPose(M3d& H, const M3d& R, const V3d& t);

#endif //SHITVO_UTIL_H

}