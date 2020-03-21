//
// Created by sunxi on 3/20/20.
//

#ifndef SHITVO_LOGGING_H
#define SHITVO_LOGGING_H

#include <cstdarg>

#define ENABLE_DEBUG_LOG
#define ENABLE_INFO_LOG
#define ENABLE_ERROR_LOG

#ifndef LOG_TAG
#define LOG_TAG "myslam_logging"
#endif

#ifdef SLAM_USING_ANDROID
#include <android/log.h>  // NOLINT
#endif

constexpr int SLAM_DEBUG = 0, SLAM_INFO = 1, SLAM_ERROR = 2;

#ifdef SLAM_USING_ANDROID
const int ANDROID_LOG_MAPPING[] = {
  ANDROID_LOG_DEBUG,
  ANDROID_LOG_INFO,
  ANDROID_LOG_ERROR,
};
#else
const char* const LOG_MAPPING[] = {
        "D",
        "I",
        "E",
};
const int COLOR_MAPPING[] = {
        0,  // default
//        33, // yellow
        32, // green
        31, //red
};
#endif
inline void PrintFunc(const char* user_tag, int severity ...) {
    if (severity == SLAM_DEBUG || severity == SLAM_INFO || severity == SLAM_ERROR) {
        va_list args;
        va_start(args, severity);
        const char* fmt = va_arg(args, const char*);
#ifdef SLAM_USING_ANDROID
        ((void) __android_log_vprint(common::ANDROID_LOG_MAPPING[severity], LOG_TAG, fmt, args));
#else
//        printf("%s(%s)-[%s]: ", LOG_TAG, LOG_MAPPING[severity], user_tag);
        printf("\033[%dm%s(%s)-[%s]: ", COLOR_MAPPING[severity], LOG_TAG, LOG_MAPPING[severity], user_tag);
        vprintf(fmt, args);
        fflush(stdout);
#endif
        va_end(args);
    }
    printf("\033[0m\n");
}
#define SLAM_LOG_FUNC(user_tag, severity, ...) PrintFunc(user_tag, severity, __VA_ARGS__)

inline void PrintLoggerState() {
    bool logger_on = false;
#ifdef ENABLE_DEBUG_LOG
    logger_on = true;
    printf("Debug logging is enable\n");
#endif
#ifdef ENABLE_INFO_LOG
    logger_on = true;
    printf("Info logging is enable\n");
#endif
#ifdef ENABLE_ERROR_LOG
    logger_on = true;
    printf("Error logging is enable\n");
#endif
    if (!logger_on) {
        printf("No logging is enable\n");
    }
}

#ifdef ENABLE_DEBUG_LOG
#define LOGD(user_tag, ...) SLAM_LOG_FUNC(user_tag, SLAM_DEBUG, __VA_ARGS__)
#else
#define LOGD(...)
#endif
#ifdef ENABLE_INFO_LOG
#define LOGI(user_tag, ...) SLAM_LOG_FUNC(user_tag, SLAM_INFO, __VA_ARGS__)
#else
#define LOGI(...)
#endif
#ifdef ENABLE_ERROR_LOG
#define LOGE(user_tag, ...) SLAM_LOG_FUNC(user_tag, SLAM_ERROR, __VA_ARGS__)
#else
#define LOGE(...)
#endif

#define LOG_ONCE(severity, user_tag, ...)             \
  {                                         \
    static bool _log_local_print = true;    \
    if (_log_local_print) {                 \
      _log_local_print = false;             \
      SLAM_LOG_FUNC(user_tag, severity, __VA_ARGS__); \
    }                                       \
  }

#ifdef ENABLE_DEBUG_LOG
#define LOGD_ONCE(...) LOG_ONCE(nail_slam::common::SLAM_DEBUG, __VA_ARGS__)
#else
#define LOGD_ONCE(...)
#endif
#ifdef ENABLE_INFO_LOG
#define LOGI_ONCE(...) LOG_ONCE(nail_slam::common::SLAM_INFO, __VA_ARGS__)
#else
#define LOGI_ONCE(...)
#endif
#ifdef ENABLE_ERROR_LOG
#define LOGE_ONCE(...) LOG_ONCE(nail_slam::common::SLAM_ERROR, __VA_ARGS__)
#else
#define LOGE_ONCE(...)
#endif

#endif //SHITVO_LOGGING_H
