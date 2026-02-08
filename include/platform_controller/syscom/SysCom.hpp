/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_SYSCOM_SYSCOM_HPP_
#define PLATFORM_CONTROLLER_SYSCOM_SYSCOM_HPP_

#include <platform_controller/syscom/ISysCom.hpp>

#include <platform_controller/init/IContext.hpp>
#include <platform_controller/transport/ITransportProxy.hpp>

#include <mutex>
#include <unordered_map>
#include <utility>

namespace platform_controller::syscom
{

class SysCom : public ISysCom
{
public:
    explicit SysCom(init::IContext& context);
    virtual ~SysCom() = default;
    void work() override;
    bool send(const Request& msg) override;    
    int subscribe(MessageId msgId, const Callback& callback);
private:
    void dispatch(const std::vector<std::uint8_t>& bytes);

    rclcpp::Logger m_logger;
    std::uint64_t m_subscriptions_counter;

    using MsgSubscription = std::pair<MessageId, Callback>;
    std::unordered_map<std::uint64_t, MsgSubscription> m_subscriptions;
    transport::ITransportProxy& m_proxy;
};

} // namespace platform_controller::syscom

#endif // PLATFORM_CONTROLLER_SYSCOM_SYSCOM_HPP_
