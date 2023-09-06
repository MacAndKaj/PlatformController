/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <platform_controller/init/PlatformControllerNode.hpp>
#include <platform_controller/init/Context.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<platform_controller::init::IContext> context = 
        std::make_shared<platform_controller::init::Context>();
    rclcpp::spin(std::make_shared<platform_controller::init::PlatformControllerNode>(
        "PlatformController", 
        std::move(context))
    );

    rclcpp::shutdown();
    return 0;
}
