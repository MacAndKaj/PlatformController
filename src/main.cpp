/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <platform_controller/init/Context.hpp>
#include <platform_controller/init/PlatformControllerNode.hpp>
#include <platform_controller/init/MdcLoggingNode.hpp>
#include <platform_controller/init/RosCom.hpp>
#include <platform_controller/init/UserCommunicationNode.hpp>
#include <platform_controller/syscom/SysCom.hpp>

#include <rclcpp/rclcpp.hpp>

#include <thread>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto platform_controller_node = std::make_shared<platform_controller::init::PlatformControllerNode>(
        "PlatformController"
    );
    {
        auto pc_context = std::make_shared<platform_controller::init::Context>(*platform_controller_node);
        platform_controller_node->setContext(pc_context);
        auto roscom = std::make_unique<platform_controller::init::RosCom>(*platform_controller_node);
        pc_context->setRosCom(std::move(roscom));
        auto syscom = std::make_unique<platform_controller::syscom::SysCom>(*pc_context);
        pc_context->setSysCom(std::move(syscom));
    }

    auto mdc_logging_node = std::make_shared<platform_controller::init::MdcLoggingNode>("MdcLoggingNode");
    {
        auto log_context = std::make_shared<platform_controller::init::Context>(*mdc_logging_node);
        mdc_logging_node->setContext(log_context);
    }

    auto user_comm_node = std::make_shared<platform_controller::init::UserCommunicationNode>("UserCommunicationNode");
    {
        auto user_comm_context = std::make_shared<platform_controller::init::Context>(*user_comm_node);
        user_comm_node->setContext(user_comm_context);
    }

    std::thread platform_controller_node_thread([&](){
        platform_controller_node->setup();
        rclcpp::spin(platform_controller_node);
    });

    std::thread mdc_loggind_thread([&](){
        mdc_logging_node->setup();
        rclcpp::spin(mdc_logging_node);
    });
    
    if (platform_controller_node_thread.joinable()) platform_controller_node_thread.join();
    if (mdc_loggind_thread.joinable()) mdc_loggind_thread.join();

    rclcpp::shutdown();
    return 0;
}
