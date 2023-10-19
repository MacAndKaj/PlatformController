/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <platform_controller/init/Context.hpp>
#include <platform_controller/init/PlatformControllerNode.hpp>
#include <platform_controller/init/RosCom.hpp>
#include <platform_controller/syscom/SysCom.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<platform_controller::init::PlatformControllerNode>(
        "PlatformController"
    );


    std::shared_ptr<platform_controller::init::IContext> context = 
        std::make_shared<platform_controller::init::Context>(*node);

    node->setContext(context);

    auto roscom = std::make_unique<platform_controller::init::RosCom>(*node);
    context->setRosCom(std::move(roscom));
    
    auto syscom = std::make_unique<platform_controller::syscom::SysCom>(*context);
    context->setSysCom(std::move(syscom));

    node->setup();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
