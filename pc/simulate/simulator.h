//
// Created by sunxi on 3/21/20.
//

#ifndef SHITVO_SIMULATOR_H
#define SHITVO_SIMULATOR_H

#include "core/common.h"
#include "core/odometry/homo_vo.h"
#include "pc/dataset_parse/dataset_parser.h"
#ifdef HAVE_VIZ
#include "pc/visualize/visualizer.h"
#endif

class Simulator {
public:
    explicit Simulator(const string& config_file_path, const string& dataset_path);

    void runOdometry();

private:
    void drawAR();

    void cvtColorGray2Color(cv::Mat& img);
    void cvtColorColor2Gray(cv::Mat& img);

    const char* TAG = "Simulator";
    shared_ptr<HomoVo> vo_;
    shared_ptr<DatasetParser> parser_;
    cv::Mat img_;
    bool imshow_pause_;
    bool have_anchor_;
    V3d t_w_m_;
    float scale_;
    cv::VideoWriter writer_;
#ifdef HAVE_VIZ
    shared_ptr<Visualizer> visualizer_;
#endif
};

#endif //SHITVO_SIMULATOR_H
