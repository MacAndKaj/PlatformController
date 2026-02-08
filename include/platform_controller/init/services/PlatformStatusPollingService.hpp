/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_SERVICES_PLATFORMSTATUSPOLLINGSERVICE_HPP_
#define PLATFORM_CONTROLLER_INIT_SERVICES_PLATFORMSTATUSPOLLINGSERVICE_HPP_

#include <platform_controller/init/services/IService.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/init/ITimersManager.hpp>
#include <platform_controller/syscom/ISysCom.hpp>


#include <rclcpp/rclcpp.hpp>

namespace platform_controller::init::services
{
struct PlatformStatusPollingServiceImpl;

class PlatformStatusPollingService : public IService
{
public:
    explicit PlatformStatusPollingService(IContext& context);
    virtual ~PlatformStatusPollingService();

protected:
    void handle(std::shared_ptr<mi_services::PlatformStatusPolling::Request> req,
                std::shared_ptr<mi_services::PlatformStatusPolling::Response> resp);
    void handleResponse(const syscom::Response& msg);
    void pollStatus();

private:
    rclcpp::Logger m_logger;
    ITimersManager& m_timers_manager;
    std::shared_ptr<rclcpp::ServiceBase> m_service;
    syscom::ISysCom& m_syscom;
    roscom::IRosCom& m_roscom;
    std::shared_ptr<PlatformStatusPollingServiceImpl> m_impl;
};

} // namespace platform_controller::init::services

#endif // PLATFORM_CONTROLLER_INIT_SERVICES_PLATFORMSTATUSPOLLINGSERVICE_HPP_
