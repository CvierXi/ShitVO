//
// Created by sunxi on 3/20/20.
//

#ifndef SHITVO_DATASET_PARSER_H
#define SHITVO_DATASET_PARSER_H

#include "core/common.h"

enum DataType {
    DATA_UNKNOWN,
    DATA_IMU,
    DATA_IMG
};

class BaseData {
public:
    double timestamp_ = -1;
    DataType type_ = DATA_UNKNOWN;
    bool is_valid_ = false;

protected:
    explicit BaseData(double timestamp, DataType type);
    virtual ~BaseData() = default;
};

class ImuData : public BaseData {
public:
    explicit ImuData(double timestamp, M3d& R_w_b);
    M3d R_w_b_;
};

class ImgData : public BaseData {
public:
    explicit ImgData(double timestamp, const string& img_name);
    string img_name_;
};


class DatasetParser {
public:
    explicit DatasetParser(const string& dataset_path);
    bool parseData();
    vector<shared_ptr<BaseData>> getDatas() {
        return datas_;
    }
    string getDatasetPath() {
        return dataset_path_;
    }

private:
    bool parseImgData();
    bool parseImuData();
    bool sortData();

    const char* TAG = "DatasetParser";
    const string dataset_path_;
    vector<shared_ptr<BaseData>> datas_;
};

#endif //SHITVO_DATASET_PARSER_H
