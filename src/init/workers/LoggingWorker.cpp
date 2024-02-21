/**
  * Copyright (c) 2024 M. Kajdak. All rights reserved.
  */
#include <platform_controller/init/workers/LoggingWorker.hpp>

namespace platform_controller::init::workers
{

LoggingWorker::LoggingWorker(IContext& context)
    : m_logger(context.createLogger("LoggingWorker - MDC"))
    , m_logging_proxy(context.getLogsProxy())
{
    RCLCPP_INFO(m_logger, "Initialized logs collector for MDC");
}

void LoggingWorker::work()
{
    auto log_bytes = m_logging_proxy.read();
    std::string log_str;
    std::transform(log_bytes.begin(), log_bytes.end(), std::back_inserter(log_str), [](const std::byte& b){
        return static_cast<char>(b);
    });
    RCLCPP_INFO(m_logger, log_str.c_str());
}

} // namespace platform_controller::init::workers
