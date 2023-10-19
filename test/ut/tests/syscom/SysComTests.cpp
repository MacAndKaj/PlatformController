/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <cstdint>
#include <platform_controller/syscom/SysCom.hpp>

#include <platform_controller_mocks/init/ContextMock.hpp>
#include <platform_controller_mocks/transport/TransportProxyMock.hpp>

#include <platform_controller/syscom/defs/Frame.hpp>
#include <platform_controller/syscom/defs/Messages.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace platform_controller::syscom
{

using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::ReturnRef;
using ::testing::SaveArg;
using ::testing::Test;
using ::testing::ValuesIn;

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

    std::vector<std::uint8_t> bytes_for_message(std::uint8_t id, const std::vector<std::uint8_t>& data)
    {
        std::vector<std::uint8_t> v(FRAME_SIZE, 0);
        (*v.begin()) = HEADER_BYTE;
        (*(v.begin()+1)) = id;
        std::copy(data.begin(), data.end(), v.begin()+2);
        return v;
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

struct PlatformSetMotorSpeedReqPair
{
    PlatformSetMotorSpeedReq req;
    std::vector<std::uint8_t> serialized_req;
};

class SysComTestsPlatformSetMotorSpeedReq :    
    public SysComTests,
    public ::testing::WithParamInterface<PlatformSetMotorSpeedReqPair>
{
public:
    static PlatformSetMotorSpeedReqPair generate(const PlatformSetMotorSpeedReq& msg)
    {
        return {
            .req=msg,
            .serialized_req={
                (std::uint8_t)msg.lSpeedI,
                (std::uint8_t)msg.lSpeedF,
                (std::uint8_t)msg.rSpeedI,
                (std::uint8_t)msg.rSpeedF,
            }
        };
    }
};

TEST_P(SysComTestsPlatformSetMotorSpeedReq, BasicScenario)
{
    auto sut = createSut();

    auto req = GetParam().req;
    auto serialized_req = GetParam().serialized_req;

    std::vector<std::uint8_t> received_bytes;
    EXPECT_CALL(*m_transport_proxy_mock, send(_))
        .Times(1)
        .WillOnce(DoAll(
            SaveArg<0>(&received_bytes),
            Return(true)));
    EXPECT_TRUE(sut->send(req));
    
    auto expected_bytes = bytes_for_message(PLATFORM_SET_MOTOR_SPEED_REQ_ID, serialized_req);
    EXPECT_EQ(received_bytes.size(), expected_bytes.size());
    for (unsigned long i = 0; i < received_bytes.size(); ++i)
    {
        EXPECT_EQ(received_bytes[i], expected_bytes[i]) << "Not matching index: " << i;
    }
}

static const std::vector<PlatformSetMotorSpeedReqPair> PARAMS = {
    SysComTestsPlatformSetMotorSpeedReq::generate(PlatformSetMotorSpeedReq{.lSpeedI=0, .lSpeedF=0, .rSpeedI=0, .rSpeedF=0}),
    SysComTestsPlatformSetMotorSpeedReq::generate(PlatformSetMotorSpeedReq{.lSpeedI=1, .lSpeedF=1, .rSpeedI=-1, .rSpeedF=1}),
    SysComTestsPlatformSetMotorSpeedReq::generate(PlatformSetMotorSpeedReq{.lSpeedI=100, .lSpeedF=0, .rSpeedI=-100, .rSpeedF=55}),
    SysComTestsPlatformSetMotorSpeedReq::generate(PlatformSetMotorSpeedReq{.lSpeedI=-127, .lSpeedF=0, .rSpeedI=127, .rSpeedF=255}),
};

INSTANTIATE_TEST_SUITE_P(SetPlatformSpeedHandlerTestsParametrizedInstance,
    SysComTestsPlatformSetMotorSpeedReq,
    ValuesIn(PARAMS.begin(), PARAMS.end())
);

} // namespace platform_controller::init::controllers
