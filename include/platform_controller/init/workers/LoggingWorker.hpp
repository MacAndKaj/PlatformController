/**
  * Copyright (c) 2024 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_WORKERS_LOGGINGWORKER_HPP_
#define PLATFORM_CONTROLLER_INIT_WORKERS_LOGGINGWORKER_HPP_

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>

namespace platform_controller::init::workers
{

class LoggingWorker
{
public:
    explicit LoggingWorker(IContext& context);
    void work();

private:
    rclcpp::Logger m_logger;
    transport::ITransportProxy& m_logging_proxy;
};

} // namespace platform_controller::init::workers

#endif // PLATFORM_CONTROLLER_INIT_WORKERS_LOGGINGWORKER_HPP_
