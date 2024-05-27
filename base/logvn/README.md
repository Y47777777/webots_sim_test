### 说明
 在程序读取 *.ini 文件后 可正常使用

 example:
 ```c++
 int main()
{
    g_Logger.initLog("../paralle_server_test.ini");


    // 使用
    LOG_DEBUG("debug:%s","11111");
    LOG_ERROR("input:%sd",input.c_str());
    
    // 种类
    LOG_DEBUG(...)   
    LOG_ERROR(...)
    LOG_FATAL(...)
    LOG_INFO(...) 
    LOG_WARN(...)   
}
 ```

 配置文件说明：
 ```
 
log4cplus.rootLogger=DEBUG, tofile  //根日志记录器的配置，DEBUG是日志级别，tofile是附加器的名称。

log4cplus.appender.console=log4cplus::ConsoleAppender   // 控制台附加器的定义，它将日志消息输出到控制台。

log4cplus.appender.console.layout=log4cplus::PatternLayout  // 控制台附加器的布局定义，PatternLayout表示使用特定的模式格式化日志消息。
log4cplus.appender.console.layout.ConversionPattern=%D{%H:%M:%S(%q)} [%p] - %m%n // 控制台附加器的转换模式，定义了日志消息的格式。

log4cplus.appender.console.filters.1=log4cplus::spi::LogLevelRangeFilter    // 控制台附加器的过滤器定义，LogLevelRangeFilter表示只接受特定级别范围内的日志消息。
log4cplus.appender.console.filters.1.LogLevelMin=DEBUG      // 过滤器的最小日志级别
log4cplus.appender.console.filters.1.LogLevelMax=FATAL      // 过滤器的最大日志级别
log4cplus.appender.console.filters.1.AcceptOnMatch=true

log4cplus.appender.tofile=log4cplus::RollingFileAppender    // RollingFileAppender表示将日志消息输出到文件，并且当文件达到特定大小时会进行滚动。
log4cplus.appender.tofile.File=test_server.log              // 文件名定义。
log4cplus.appender.tofile.MaxFileSize=1MB                   // 文件大小，当到1MD时滚动
log4cplus.appender.tofile.MaxBackupIndex=10                 // 表示最多保留10个备份文件
log4cplus.appender.tofile.layout=log4cplus::PatternLayout   // 布局定义。
log4cplus.appender.tofile.layout.ConversionPattern=%D{%Y-%m-%d %H:%M:%S(%q)} [%p] %l - %m%n // 转换模式，定义了日志消息的格式。
log4cplus.appender.tofile.filters.2=log4cplus::spi::LogLevelRangeFilter // 过滤器定义
log4cplus.appender.tofile.filters.2.LogLevelMin=INFO    // 过滤器的最小日志级别
log4cplus.appender.tofile.filters.2.LogLevelMax=FATAL   // 过滤器的最大日志级别
log4cplus.appender.tofile.filters.2.AcceptOnMatch=true

 ```