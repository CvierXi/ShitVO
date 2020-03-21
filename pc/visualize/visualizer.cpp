//
// Created by sunxi on 3/21/20.
//

#include "visualizer.h"

Visualizer::Visualizer() {
    /// Create a window
    myWindow_ = viz::Viz3d("World Frame");

    /// Add coordinate axes
    myWindow_.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    /// Start event loop.
//    myWindow_.spinOnce();

    /// Add a plane
    Point3d center(0, 0, 0);
    Vec3d norm(0, 0, 1);
    Vec3d new_yaxis(0, 1, 0);
    viz::WPlane plane(center, norm, new_yaxis);
    myWindow_.showWidget("Horizontal Plane", plane);

    have_camera_ = false;
}

void Visualizer::addCamera(const M3d& K) {
    cpw_ = viz::WCameraPosition(0.5);
    cv::Matx33d cam_K;
    cv::eigen2cv(K, cam_K);
    cpw_frustum_ = viz::WCameraPosition(cam_K, 1.0, viz::Color::silver());
    have_camera_ = true;
}

void Visualizer::updateCameraPose(const M3d& R_w_c, const V3d& t_w_c) {
    if (!have_camera_) {
        return;
    }

    V3d cam_z_unit_w = R_w_c * V3d(0, 0, 1) + t_w_c;
    V3d cam_y_unit_w = R_w_c * V3d(0, 1, 0);

    cv::Point3d cam_pos = getPointInWorld(t_w_c);
    cv::Vec3d cam_focal_point = getPointInWorld(cam_z_unit_w);
    cv::Vec3d cam_y_dir = getPointInWorld(cam_y_unit_w);

    points_.push_back(cam_pos);
    viz::WPolyLine trajectory(points_, viz::Color::green());
    myWindow_.showWidget("trajectory", trajectory);

    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

    myWindow_.showWidget("CPW", cpw_, cam_pose);
    myWindow_.showWidget("CPW_FRUSTUM", cpw_frustum_, cam_pose);
    myWindow_.spinOnce();
}

void Visualizer::holdOn() {
    myWindow_.spin();
}


Point3d Visualizer::getPointInWorld(const V3d& p) {
    return {p(0), p(1), p(2)};
}
