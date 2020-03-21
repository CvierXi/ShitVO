//
// Created by sunxi on 3/20/20.
//

#include <sstream>

#include "dataset_parser.h"
#include "core/util/logging.h"
#include "core/util/util.h"

BaseData::BaseData(double timestamp, DataType type) : timestamp_(timestamp), type_(type), is_valid_(false) {
}

ImuData::ImuData(double timestamp, M3d& R_w_b) : BaseData(timestamp, DATA_IMU), R_w_b_(R_w_b) {
    is_valid_ = true;
}

ImgData::ImgData(double timestamp, const string& img_name) : BaseData(timestamp, DATA_IMG), img_name_(img_name) {
    is_valid_ = true;
}

DatasetParser::DatasetParser(const string& dataset_path) : dataset_path_(dataset_path) {
}

bool DatasetParser::parseData() {
    return parseImuData() && parseImgData() && sortData();
}

/*
 * Imu data is arranged as:
 * timestamp(unit: ms), acc[3], gyr[3], att[9]
 */
bool parseImuDataInLine(const string& data, double& timestamp, M3d& R) {
    istringstream out(data);
    string str;
    int index = -1;
    double r[9];
    while (getline(out, str, ',')) {
        index++;
        if (index == 0) {
            timestamp = atol(str.c_str()) / 1e9;
        } else if (index > 6 && index < 16) {
            r[index - 7] = atof(str.c_str());
        }
    }
    M3d Rt(r);
    R = Rt.transpose();
    return (index >= 15);
}

bool DatasetParser::parseImuData() {
    string imu_data_path = dataset_path_ + "/imu_data.txt";
    ifstream in(imu_data_path);
    if (!in.is_open()) {
        LOGE(TAG, "Invalid imu_data_path");
        return false;
    }
    double timestamp;
    M3d R_w_b;
    string data;
    while (getline(in, data)) {
        if (!parseImuDataInLine(data, timestamp, R_w_b)) {
            continue;
        }
        datas_.emplace_back(make_shared<ImuData>(ImuData(timestamp, R_w_b)));
    }
    return true;
}

bool isValidImgName(const string& img_name) {
    int index = img_name.find('.');
    string format = img_name.substr(index+1, img_name.size()-1);
    return (format == "jpg") || (format == "png");
}

double imgName2Timestamp(std::string img_name) {
    double timestamp = -1;
    img_name.resize(img_name.find('.'));
    timestamp = atof(img_name.c_str()) / 1e9;
    return timestamp;
}

bool DatasetParser::parseImgData() {
    string img_list_path = dataset_path_ + "/img_list.txt";
    ifstream in(img_list_path);
    if (!in.is_open()) {
        LOGE(TAG, "Invalid img_list_path");
        return false;
    }
    string img_name;
    double timestamp;
    while (getline(in, img_name)) {
        if (!isValidImgName(img_name)) {
            continue;
        }
        timestamp = imgName2Timestamp(img_name);
        datas_.emplace_back(make_shared<ImgData>(timestamp, img_name));
    }
    return true;
}

bool sortFunc(const shared_ptr<BaseData> &data1, const shared_ptr<BaseData> &data2)
{
    return data1->timestamp_ < data2->timestamp_;     // ascending order
}

bool DatasetParser::sortData() {
    sort(datas_.begin(), datas_.end(), sortFunc);
    return true;
}
