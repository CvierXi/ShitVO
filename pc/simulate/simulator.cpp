//
// Created by sunxi on 3/21/20.
//

#include "simulator.h"
#include "core/util/logging.h"

#define AR_SCALE 50

Simulator::Simulator(const string &config_file_path, const string &dataset_path) {
    vo_ = make_shared<HomoVo>(config_file_path);
    parser_ = make_shared<DatasetParser>(dataset_path);
    scale_ = AR_SCALE;
    have_anchor_ = false;
    imshow_pause_ = true;
    writer_ = cv::VideoWriter("./out.avi", CV_FOURCC('X','V','I','D'), 30, cv::Size(640, 480), true);
#ifdef HAVE_VIZ
    visualizer_ = make_shared<Visualizer>();
    visualizer_->addCamera(vo_->getCamera()->K());
#endif
}

void Simulator::runOdometry() {
    if (!parser_->parseData()) {
        LOGE(TAG, "Failed to parse dataset.");
        return;
    }

    vector<shared_ptr<BaseData>> datas = parser_->getDatas();

    for (auto& it : datas) {
        if (it->type_ == DATA_IMU) {
            /// IMU
            auto data = dynamic_pointer_cast<ImuData>(it);
            vo_->imuCallback(data->timestamp_, data->R_w_b_);
        } else if (it->type_ == DATA_IMG) {
            /// IMG
            auto data = dynamic_pointer_cast<ImgData>(it);
            string img_path = parser_->getDatasetPath() + "/cam/" + data->img_name_;
            cv::Mat img = cv::imread(img_path);
            if (img.empty()) {
                continue;
            }
            img_ = img.clone();
            cvtColorColor2Gray(img);
            vo_->imgCallback(data->timestamp_, img);
            drawAR();
#ifdef HAVE_VIZ
            Pose pose = vo_->getCameraPose();
            if (pose.state == STATE_TRACKING) {
                visualizer_->updateCameraPose(pose.q.toRotationMatrix(), pose.p);
            }
#endif
        } else {
            LOGE(TAG, "Unknown data type.");
        }
    }
    LOGI(TAG, "Odometry finished!");
#ifdef HAVE_VIZ
    visualizer_->holdOn();
#endif
}

cv::Point2f getHomoPixel(const V3d& P_w, const M3d& H_c_w) {
    V3d P_c = H_c_w * P_w;
    P_c /= P_c(2);
    return cv::Point2f(P_c(0), P_c(1));
}

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        auto* data = (int*)userdata;
        data[0] = x;
        data[1] = y;
    }
}

void Simulator::drawAR() {
    cv::Mat ar_img = img_.clone();
    M3d H_c_w = vo_->getHomography();

    if (have_anchor_) {
        vector<Point> vertexes;
        vertexes.push_back(getHomoPixel(t_w_m_ + V3d(-scale_, -scale_, 0), H_c_w));
        vertexes.push_back(getHomoPixel(t_w_m_ + V3d(+scale_, -scale_, 0), H_c_w));
        vertexes.push_back(getHomoPixel(t_w_m_ + V3d(+scale_, +scale_, 0), H_c_w));
        vertexes.push_back(getHomoPixel(t_w_m_ + V3d(-scale_, +scale_, 0), H_c_w));

        fillConvexPoly(ar_img, vertexes, Scalar(0, 255, 0));
    } else {
        LOGD(TAG, "No any anchor. You can place an anchor by clicking the window.");
    }

    cv::Point2f org(0, ar_img.rows - 15);
    cv::Scalar color  = imshow_pause_ ? Scalar(0, 0, 255) : Scalar(255, 0, 255);
    cv::putText(ar_img, "Press 'p' to pause or play.", org, 0, 0.8, color, 2);
    string win_name = "ar";
    cv::imshow(win_name, ar_img);
    writer_ << ar_img;

    int pt_click[2] = {-1, -1};
    setMouseCallback(win_name, onMouse, pt_click);
    int ch = imshow_pause_ ? cv::waitKey() : cv::waitKey(30);
    if (ch == 'p' || ch == 'P') {
        imshow_pause_ = !imshow_pause_;
    }

    if (pt_click[0] > 0) {
        LOGD(TAG, "Place anchor at -> (%d, %d).", pt_click[0], pt_click[1]);
        t_w_m_ = H_c_w.inverse() * V3d(pt_click[0], pt_click[1], 1);
        have_anchor_ = true;
        /// todo: exist bugs, fix them
//        vo_->reset();
    }
}

void Simulator::cvtColorGray2Color(cv::Mat& img) {
    if (img.channels() == 1) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }
}

void Simulator::cvtColorColor2Gray(cv::Mat& img) {
    if (img.channels() > 1) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
}
