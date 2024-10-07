#ifndef LOGVN_H
#define LOGVN_H

#if defined(_MSC_VER) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__) || defined(WIN32) \
|| defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define Q_DECL_EXPORT __declspec(dllexport)
#define Q_DECL_IMPORT __declspec(dllimport)
#else
#define Q_DECL_EXPORT __attribute__((visibility("default")))
#define Q_DECL_IMPORT __attribute__((visibility("default")))
#endif

#if defined(ACTIONSERVICE_LIBRARY)
#define ACTIONSERVICE_EXPORT Q_DECL_EXPORT
#else
#define ACTIONSERVICE_EXPORT Q_DECL_IMPORT
#endif
namespace VNSim{

#define LOG_DEBUG(...)    g_Logger.Debug(__FILE__, __LINE__,__FUNCTION__, __VA_ARGS__);
#define LOG_ERROR(...)    g_Logger.Error(__FILE__, __LINE__,__FUNCTION__, __VA_ARGS__);
#define LOG_FATAL(...)    g_Logger.Fatal(__FILE__, __LINE__,__FUNCTION__, __VA_ARGS__);
#define LOG_INFO(...)    g_Logger.Info(__FILE__, __LINE__,__FUNCTION__, __VA_ARGS__);
#define LOG_WARN(...)    g_Logger.Warn(__FILE__, __LINE__,__FUNCTION__, __VA_ARGS__);
#define LOG_TRACE(...)    g_Logger.Trace(__FILE__, __LINE__,__FUNCTION__, __VA_ARGS__);

class Logvn
{
public:
    Logvn();
    virtual ~Logvn();

    /// 启动日志系统
    /// @param[in] properties_filename 日志系统配置文件文件名
    void initLog(const char* logCfgFileName,const char* logFileName = nullptr);

public:
    void Debug(const char* filename, const int fileline, const char * func,const char* pFormat,... );

    void Error(const char* filename, const int fileline, const char * func,const char* pFormat,... );

    void Fatal(const char* filename, const int fileline, const char * func,const char* pFormat,... );

    void Info(const char* filename, const int fileline, const char * func,const char* pFormat,... );

    void Warn(const char* filename, const int fileline, const char * func,const char* pFormat,... );

    void Trace(const char* filename, const int fileline, const char * func,const char* pFormat,... );

public:
    static inline Logvn* getSingletonPtr()
    {
        return &getSingleton();
    }
    static inline Logvn& getSingleton()
    {
        static Logvn _instance;
        return _instance;
    }
};

#define g_Logger Logvn::getSingleton()
#define g_pLogger Logvn::getSingletonPtr()

//////////////////////////////////////////////////////////////////////////
// 断言日志
//////////////////////////////////////////////////////////////////////////
#define ASSERT_LOG(expr)\
if ( (expr) ) {;} else g_Logger.Error(__FILE__, __LINE__, __FUNCTION__,#expr);

}
#endif // LOGVN_H
