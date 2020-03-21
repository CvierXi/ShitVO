//
// Created by sunxi on 3/20/20.
//

#ifndef SHITVO_CONFIG_PARSER_H
#define SHITVO_CONFIG_PARSER_H

#include "core/common.h"

class ConfigParser {
public:
    ~ConfigParser();

    static bool setParameterFile(const std::string& file_path);

    template <typename T>
    static T get(const std::string& key) {
        return T(ConfigParser::config_parser_->file_[key]);
    }

private:
    static std::shared_ptr<ConfigParser> config_parser_;
    cv::FileStorage file_;

    ConfigParser() {
    }
};

#endif //SHITVO_CONFIG_PARSER_H
