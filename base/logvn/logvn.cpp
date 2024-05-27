#include "logvn.h"
#include <cstdlib>
#include <iostream>
#include <log4cplus/configurator.h>
#include <log4cplus/helpers/loglog.h>
#include <log4cplus/helpers/stringhelper.h>
#include <log4cplus/loggingmacros.h>
#include <log4cplus/logger.h>
#include <string>
#include <log4cplus/tstring.h>
#include <stdlib.h>

#ifndef win32
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#endif

using namespace std;
using namespace log4cplus;
using namespace log4cplus::helpers;
namespace VNSim{


Logvn::Logvn()
{

}

Logvn::~Logvn()
{
    log4cplus::Logger _logger = log4cplus::Logger::getRoot();
    LOG4CPLUS_INFO(_logger, "Logger Stop.");
}

#ifdef win32
#define DO_LOGGER(logLevel, filename, fileline, func, pFormat, bufSize)\
log4cplus::Logger _logger = log4cplus::Logger::getRoot();\
    \
    if(_logger.isEnabledFor(logLevel))\
{                \
        va_list args;            \
        va_start(args, pFormat);        \
        char buf[bufSize] = {0};        \
        _vsnprintf(buf, sizeof(buf), pFormat, args);    \
        va_end(args);           \
        size_t len = strlen(buf) + 1;\
        size_t converted = 0;\
        wchar_t *WStr;\
        WStr=(wchar_t*)malloc(len*sizeof(wchar_t));\
        mbstowcs_s(&converted, WStr, len, buf, _TRUNCATE);\
        _logger.log(logLevel, WStr, filename, fileline,func); \
}
#else
//#include <stdio.h>
//#include <stdarg.h>
//#include <string.h>

#define DO_LOGGER(logLevel, filename, fileline, func, pFormat, bufSize)\
log4cplus::Logger _logger = log4cplus::Logger::getRoot();\
    \
    if(_logger.isEnabledFor(logLevel))\
{                \
        va_list args;            \
        va_start(args, pFormat);        \
        char buf[bufSize] = {0};        \
        vsnprintf(buf, sizeof(buf), pFormat, args);    \
        va_end(args);           \
        _logger.log(logLevel, buf, filename, fileline,func); \
}
#endif

void Logvn::Debug(const char* filename, const int fileline, const char *func, const char* pFormat,... )
{
#ifdef LOG_COLOR
    printf(GREEN);
#endif
    DO_LOGGER(log4cplus::DEBUG_LOG_LEVEL, filename, fileline, func, pFormat, 256);
}

void Logvn::Error( const char* filename, const int fileline,  const char * func,const char* pFormat,...   )
{
#ifdef LOG_COLOR
    printf(RED);
#endif
    DO_LOGGER(log4cplus::ERROR_LOG_LEVEL, filename, fileline, func, pFormat, 256);
}

void Logvn::Fatal( const char* filename, const int fileline,  const char * func,const char* pFormat,...   )
{
    DO_LOGGER(log4cplus::FATAL_LOG_LEVEL, filename, fileline, func, pFormat, 256);
}

void Logvn::Info( const char* filename, const int fileline,  const char * func,const char* pFormat, ...  )
{
#ifdef LOG_COLOR
    printf(GRAY);
#endif
    DO_LOGGER(log4cplus::INFO_LOG_LEVEL, filename, fileline, func, pFormat, 512);
}

void Logvn::Warn( const char* filename, const int fileline,  const char * func,const char* pFormat, ...  )
{
#ifdef LOG_COLOR
    printf(YELLOW);
#endif
    DO_LOGGER(log4cplus::WARN_LOG_LEVEL, filename, fileline, func, pFormat, 256);
}

void Logvn::Trace( const char* filename, const int fileline,  const char * func,const char* pFormat,...   )
{
    DO_LOGGER(log4cplus::TRACE_LOG_LEVEL, filename, fileline, func, pFormat, 1024);
}

void Logvn::initLog( const char* logCfgFileName )
{
    if (logCfgFileName==NULL) return;

    tstring path=LOG4CPLUS_TEXT("");
#ifdef win32
    size_t len = strlen(logCfgFileName) + 1;
    size_t converted = 0;
    wchar_t *WStr;
    WStr=(wchar_t*)malloc(len*sizeof(wchar_t));
    mbstowcs_s(&converted, WStr, len, logCfgFileName, _TRUNCATE);
    path.append(WStr);
#else
    path.append(logCfgFileName);
#endif
    log4cplus::helpers::LogLog::getLogLog()->setInternalDebugging(false);
    log4cplus::PropertyConfigurator::doConfigure(path);
    log4cplus::Logger _logger = log4cplus::Logger::getRoot();
    LOG4CPLUS_INFO(_logger, "Logger Start.");
}


}
