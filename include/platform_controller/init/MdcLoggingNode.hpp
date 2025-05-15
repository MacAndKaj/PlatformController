/**
  * Copyright (c) 2024 M. Kajdak. All rights reserved.
  */
#ifndef PLATFORM_CONTROLLER_INIT_MDCLOGGINGNODE_HPP_
#define PLATFORM_CONTROLLER_INIT_MDCLOGGINGNODE_HPP_

#include <platform_controller/init/IContext.hpp>

// #include <platform_controller/init/workers/LoggingWorker.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <deque>
#include <vector>

namespace platform_controller::init
{

class MdcLoggingNode : public rclcpp::Node
{
public:
    explicit MdcLoggingNode(const std::string& node_name);
    virtual ~MdcLoggingNode() = default;
    void setContext(std::shared_ptr<IContext> context);
    void setup();

protected:
    void work();

private:
    rclcpp::Logger m_node_logger;
    std::shared_ptr<IContext> m_context;
    rclcpp::TimerBase::SharedPtr m_node_timer;
    // std::unique_ptr<workers::LoggingWorker> m_worker;
    std::deque<char> m_log_queue;
};

} // namespace platform_controller::init

#endif // PLATFORM_CONTROLLER_INIT_MDCLOGGINGNODE_HPP_
