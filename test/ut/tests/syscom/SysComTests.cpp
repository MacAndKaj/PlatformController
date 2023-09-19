/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <platform_controller/syscom/SysCom.hpp>

#include <platform_controller_mocks/init/ContextMock.hpp>
#include <platform_controller_mocks/transport/TransportProxyMock.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace platform_controller::syscom
{

using ::testing::_;
using ::testing::Return;
using ::testing::ReturnRef;
using ::testing::Test;

struct SysComTests : public Test
{
    void SetUp() override
    {
        m_context_mock = std::make_unique<init::ContextMock>();
        m_transport_proxy_mock = std::make_unique<transport::TransportProxyMock>();
    }

    void TearDown() override
    {
        m_context_mock.reset();
        m_transport_proxy_mock.reset();
    }

    std::unique_ptr<SysCom> createSut()
    {
        EXPECT_CALL(*m_context_mock, createLogger(_))
            .Times(1)
            .WillOnce(Return(rclcpp::get_logger("TEST")));
        EXPECT_CALL(*m_context_mock, getTransportProxy())
            .Times(1)
            .WillOnce(ReturnRef(*m_transport_proxy_mock));
        return std::make_unique<SysCom>(*m_context_mock);
    }

    std::unique_ptr<init::ContextMock> m_context_mock;
    std::unique_ptr<transport::TransportProxyMock> m_transport_proxy_mock;
};

TEST_F(SysComTests, FailDuringSend)
{
    auto sut = createSut();

    PlatformSetMotorSpeedReq req{
        .lSpeedI=100,
        .lSpeedF=0,
        .rSpeedI=-100,
        .rSpeedF=10
    };

    EXPECT_CALL(*m_transport_proxy_mock, send(_))
        .Times(1)
        .WillOnce(Return(false));
    
    EXPECT_FALSE(sut->send(req));
}

} // namespace platform_controller::init::controllers
