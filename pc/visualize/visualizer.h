//
// Created by sunxi on 3/21/20.
//

#ifndef SHITVO_VISUALIZER_H
#define SHITVO_VISUALIZER_H

#ifdef HAVE_VIZ

#include "core/common.h"

using namespace cv;

class Visualizer {
public:
    Visualizer();

    void addCamera(const M3d& K);
    void updateCameraPose(const M3d& R_rc, const V3d& t_rc);
    void holdOn();

private:
    Point3d getPointInWorld(const V3d& p);

    viz::Viz3d myWindow_;
    viz::WCameraPosition cpw_; // Coordinate axes
    viz::WCameraPosition cpw_frustum_; // Camera frustum
    bool have_camera_;
    vector<Point3d> points_;
};

#endif

#endif //SHITVO_VISUALIZER_H
