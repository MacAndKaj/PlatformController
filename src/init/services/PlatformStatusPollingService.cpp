/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#include <platform_controller/init/services/PlatformStatusPollingService.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/roscom/PublisherBuilder.hpp>

#include <cmath>

namespace platform_controller::init::services
{
static constexpr int MDC_CONTROLLER_BIT = 0;
static constexpr int MDC_FEEDBACK_BIT = 1;
static constexpr int MDC_LOG_BIT = 2;
static constexpr int MDC_SYSCOM_BIT = 3;
static constexpr int MDC_MONITORING_BIT = 4;


std::vector<int> handleStatusBits(const mi_services::PlatformStatusPolling::Request& req, uint8_t& status_bits)
{
    std::vector<int> status_bits_vec;
    if (req.mdc_controller_status_polling)
    {
        status_bits_vec.push_back(MDC_CONTROLLER_BIT);
        status_bits |= (1 << MDC_CONTROLLER_BIT);
    }
    if (req.mdc_feedback_status_polling)
    {
        status_bits_vec.push_back(MDC_FEEDBACK_BIT);
        status_bits |= (1 << MDC_FEEDBACK_BIT);
    }
    if (req.mdc_monitoring_status_polling)
    {
        status_bits_vec.push_back(MDC_MONITORING_BIT);
        status_bits |= (1 << MDC_MONITORING_BIT);
    }
    // if (req->mdc_log_status_polling) polling_req.statusSet |= (1 << MDC_LOG_BIT);
    // if (req->mdc_syscom_status_polling) polling_req.statusSet |= (1 << MDC_SYSCOM_BIT);

    return status_bits_vec;
}

PlatformStatusPollingService::PlatformStatusPollingService(IContext& context)
    : m_logger(context.createLogger("PlatformStatusPollingService"))
    , m_timers_manager(context.getTimersManager())
    , m_service(context.getRosCom().createServiceForPollPlatformStatus([this](auto req, auto resp){handle(req, resp);}))
    , m_syscom(context.getSysCom())
    , m_roscom(context.getRosCom())
    , m_status_pub_timer_id (-1)
{
    RCLCPP_INFO(m_logger, "PlatformStatusPollingService waiting for messages");
}

PlatformStatusPollingService::~PlatformStatusPollingService()
{
    RCLCPP_INFO(m_logger, "PlatformStatusPollingService destructed");
}

void PlatformStatusPollingService::handle(std::shared_ptr<mi_services::PlatformStatusPolling::Request> req,
                                          std::shared_ptr<mi_services::PlatformStatusPolling::Response> resp)
{
    using StatusType = mi_services::PlatformStatusPolling::Response::_response_status_type;
    RCLCPP_INFO(m_logger, "Received PlatformStatusPolling request");
    resp->response_status = StatusType();
    roscom::PublisherBuilder<motoros_interfaces::msg::PlatformStatus> builder{
        "platform_controller/platform_status"
    };

    m_roscom_sender = m_roscom.createSender(builder);
    m_status_pub_timer_id = m_timers_manager.startOneShotTimer(std::chrono::milliseconds(req->period), [this]{handleTimer();});
    m_syscom.subscribeForStatus([this](auto status) { handleStatus(status); });

    RCLCPP_INFO(m_logger, "PlatformStatusPollingService started correctly with period: %d ms", req->period);
    resp->response_status.status = StatusType::GENERIC_STATUS_OK;
}

void PlatformStatusPollingService::handleStatus(const PlatformStatus status)
{
    m_current_status = std::move(status);
}

void PlatformStatusPollingService::handleTimer() const
{
    motoros_interfaces::msg::PlatformStatus status{};
    status.l_speed = static_cast<double>(m_current_status.l_speed_i) + (static_cast<double>(m_current_status.l_speed_f) / 100.0);
    status.r_speed = static_cast<double>(m_current_status.r_speed_i) + (static_cast<double>(m_current_status.r_speed_f) / 100.0);

    const roscom::PubMsg pub_msg = status;
    status.mdc_controller_status.status = (m_current_status.system_state & 1 << MDC_CONTROLLER_BIT)
        ? motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_OK
        : motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_UNKNOWN_ERROR;
    status.mdc_feedback_status.status = (m_current_status.system_state & 1 << MDC_FEEDBACK_BIT)
        ? motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_OK
        : motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_UNKNOWN_ERROR;
    status.mdc_monitoring_status.status = (m_current_status.system_state & 1 << MDC_MONITORING_BIT)
        ? motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_OK
        : motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_UNKNOWN_ERROR;

    m_roscom_sender->send(pub_msg);
    m_timers_manager.restartTimer(m_status_pub_timer_id);
}

} // namespace platform_controller::init::services
