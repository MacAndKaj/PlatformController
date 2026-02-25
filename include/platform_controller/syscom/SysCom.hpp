/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_SYSCOM_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_SYSCOM_HPP_

#include <platform_controller/syscom/ISysCom.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>
#include <platform_controller/syscom/CommandQueue.hpp>
#include <platform_controller/syscom/defs/Frame.hpp>
#include <platform_controller/syscom/ConnectionStatus.hpp>


namespace platform_controller::syscom
{

class SysCom : public ISysCom
{
public:
    explicit SysCom(init::IContext& context);
    virtual ~SysCom() = default;
    void work() override;
    void send(const Command& msg) override;
    int subscribeForStatus(const Callback& callback) override;
    void setDebug(bool enabled) override;

private:
    Frame create_next_frame(const std::vector<std::uint8_t>& payload, std::uint8_t id) const;
    Frame create_next_heartbeat_frame() const;
    void handle_received_frame(const Frame& frame);

    bool m_debug_mode;
    rclcpp::Logger m_logger;
    std::uint64_t m_subscriptions_counter;

    std::unordered_map<std::uint64_t, Callback> m_subscriptions;
    transport::ITransportProxy& m_proxy;
    CommandQueue m_command_queue;
    ConnectionStatus m_connection_status;
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_SYSCOM_HPP_
