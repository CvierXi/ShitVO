//
// Created by sunxi on 3/20/20.
//

#include "config_parser.h"

bool ConfigParser::setParameterFile(const std::string& file_path) {
    if (config_parser_ == nullptr) {
        config_parser_ = shared_ptr<ConfigParser>(new ConfigParser);
    }
    config_parser_->file_ = cv::FileStorage(file_path.c_str(), cv::FileStorage::READ);
    if (!config_parser_->file_.isOpened()) {
        config_parser_->file_.release();
        return false;
    }
    return true;
}

ConfigParser::~ConfigParser() {
    if (file_.isOpened()) {
        file_.release();
    }
}

shared_ptr<ConfigParser> ConfigParser::config_parser_ = nullptr;
