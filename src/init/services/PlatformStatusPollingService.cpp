/**
  * Copyright (c) 2025 MacAndKaj. All rights reserved.
  */
#include <platform_controller/init/services/PlatformStatusPollingService.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/roscom/IRosComSender.hpp>
#include <platform_controller/roscom/PublisherBuilder.hpp>

#include <cmath>

namespace platform_controller::init::services
{
static constexpr int MDC_CONTROLLER_BIT = 0;
static constexpr int MDC_FEEDBACK_BIT = 1;
static constexpr int MDC_LOG_BIT = 2;
static constexpr int MDC_SYSCOM_BIT = 3;
static constexpr int MDC_MONITORING_BIT = 4;

struct PlatformStatusPollingServiceImpl
{
    syscom::Request request;
    int m_syscom_subscription_id;
    int m_timer_id;
    std::vector<int> m_status_bits;
    std::shared_ptr<roscom::IRosComSender> m_roscom_sender;
};

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
    , m_service(context.getRosCom().createServiceForPollPlatformStatus(
        std::bind(&PlatformStatusPollingService::handle, this, std::placeholders::_1, std::placeholders::_2)))
    , m_syscom(context.getSysCom())
    , m_roscom(context.getRosCom())
{
    RCLCPP_INFO(m_logger, "PlatformStatusPollingService waiting for messages");
}

PlatformStatusPollingService::~PlatformStatusPollingService()
{
    // Stop timers if impl is initialized
    if (m_impl)
    {
        RCLCPP_INFO(m_logger, "Stopping PlatformStatusPollingService");
        m_timers_manager.removeTimer(m_impl->m_timer_id);
        m_impl.reset();
    }
}

void PlatformStatusPollingService::handle(std::shared_ptr<mi_services::PlatformStatusPolling::Request> req,
    std::shared_ptr<mi_services::PlatformStatusPolling::Response> resp)
{
    using StatusType = mi_services::PlatformStatusPolling::Response::_response_status_type;
    RCLCPP_INFO(m_logger, "Received PlatformStatusPolling request");
    resp->response_status = StatusType();

    if (m_impl)
    {
        RCLCPP_ERROR(m_logger, "Service is already initialized");
        resp->response_status.status = StatusType::GENERIC_STATUS_REJECT;
        return;
    }

    syscom::Request request{};
    auto& polling_req = request.msg.platform_poll_status_req;
    request.msg_id = PLATFORM_POLL_STATUS_REQ_ID;


    auto syscom_request_sender = [this] () { pollStatus(); };
    auto syscom_response_handler = [this] (const auto& msg) { handleResponse(msg); };

    roscom::PublisherBuilder<motoros_interfaces::msg::PlatformStatus> builder{
        "platform_controller/platform_status"
    };

    polling_req.statusSet = 0;
    m_impl = std::make_shared<PlatformStatusPollingServiceImpl>(request,
        m_syscom.subscribe(PLATFORM_POLL_STATUS_RESP_ID, syscom_response_handler),
        m_timers_manager.startOneShotTimer(std::chrono::milliseconds(req->period), syscom_request_sender),
        handleStatusBits(*req, polling_req.statusSet),
        m_roscom.createSender(builder)
    );

    RCLCPP_INFO(m_logger, "PlatformStatusPollingService started correctly with period: %d ms", req->period);
    resp->response_status.status = StatusType::GENERIC_STATUS_OK;
}

void PlatformStatusPollingService::pollStatus()
{
    if (m_impl)
    {
        RCLCPP_INFO(m_logger, "Polling platform status");
        m_syscom.send(m_impl->request);
    }
}

void PlatformStatusPollingService::handleResponse(const syscom::Response& msg)
{
    if (msg.msg_id != PLATFORM_POLL_STATUS_RESP_ID)
    {
        RCLCPP_ERROR(m_logger, "Received unexpected message ID: %d", msg.msg_id);
        return;
    }

    const auto& resp = msg.msg.poll_status_resp;
    motoros_interfaces::msg::PlatformStatus status{};
    status.l_speed = static_cast<double>(resp.lSpeedI) + (static_cast<double>(resp.lSpeedF) / 100.0);
    status.r_speed = static_cast<double>(resp.rSpeedI) + (static_cast<double>(resp.rSpeedF) / 100.0);

    const roscom::PubMsg pub_msg = status;
    status.mdc_controller_status.status = (resp.moduleStatus & 1 << MDC_CONTROLLER_BIT)
        ? motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_OK
        : motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_UNKNOWN_ERROR;
    status.mdc_feedback_status.status = (resp.moduleStatus & 1 << MDC_FEEDBACK_BIT)
        ? motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_OK
        : motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_UNKNOWN_ERROR;
    status.mdc_monitoring_status.status = (resp.moduleStatus & 1 << MDC_MONITORING_BIT)
        ? motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_OK
        : motoros_interfaces::msg::GenericStatus::GENERIC_STATUS_UNKNOWN_ERROR;
    RCLCPP_INFO(m_logger, "Sending PlatformStatus message");
    m_impl->m_roscom_sender->send(pub_msg);
    m_timers_manager.restartTimer(m_impl->m_timer_id);
}

} // namespace platform_controller::init::services
