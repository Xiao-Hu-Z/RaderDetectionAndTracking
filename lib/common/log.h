#pragma once

#include <cstdarg>
#include <iostream>


#include "glog/logging.h"
#include "glog/raw_logging.h"


#define ADEBUG_MODULE(module) VLOG(4) << "[DEBUG] "
#define PrintDbg ADEBUG_MODULE()
#define PrintInfo ALOG_MODULE(INFO)
#define PrintWarn ALOG_MODULE(WARN)
#define PrintErr ALOG_MODULE(ERROR)

#ifndef ALOG_MODULE_STREAM
#define ALOG_MODULE_STREAM(log_severity) ALOG_MODULE_STREAM_##log_severity
#endif

#ifndef ALOG_MODULE
#define ALOG_MODULE(log_severity) ALOG_MODULE_STREAM(log_severity)
#endif

#define ALOG_MODULE_STREAM_INFO                                                \
    google::LogMessage(__FILE__, __LINE__, google::INFO).stream()

#define ALOG_MODULE_STREAM_WARN                                                \
    google::LogMessage(__FILE__, __LINE__, google::WARNING).stream()

#define ALOG_MODULE_STREAM_ERROR                                               \
    google::LogMessage(__FILE__, __LINE__, google::ERROR).stream()

#define LPPrintCHECK(cond) CHECK(cond)


