/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#include <platform_controller/init/controllers/SetPlatformSpeedHandler.hpp>

#include <platform_controller_mocks/init/ContextMock.hpp>
#include <platform_controller_mocks/roscom/RosComMock.hpp>
#include <platform_controller_mocks/syscom/SysComMock.hpp>

#include <motoros_interfaces/msg/set_platform_speed.hpp>

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <optional>
#include <sstream>

bool operator==(const PlatformSetMotorSpeedReq& a1, const PlatformSetMotorSpeedReq& a2)
{
    return  std::tie(a1.lSpeedF, a1.lSpeedI, a1.rSpeedF, a1.rSpeedI) == 
            std::tie(a2.lSpeedF, a2.lSpeedI, a2.rSpeedF, a2.rSpeedI);
}

namespace platform_controller::init::controllers
{

using ::testing::_;
using ::testing::AllOf;
using ::testing::DoAll;
using ::testing::Field;
using ::testing::Return;
using ::testing::ReturnRef;
using ::testing::SaveArg;
using ::testing::Test;
using ::testing::ValuesIn;

struct SetPlatformSpeedHandlerTests : public Test
{
    void SetUp() override
    {
        m_context_mock = std::make_unique<ContextMock>();
        m_ros_com_mock = std::make_unique<roscom::RosComMock>();
        m_sys_com_mock = std::make_unique<syscom::SysComMock>();
        m_callback.reset();
    }

    void TearDown() override
    {
        m_context_mock.reset();
        m_ros_com_mock.reset();
        m_sys_com_mock.reset();
    }

    std::unique_ptr<SetPlatformSpeedHandler> createSut()
    {
        EXPECT_CALL(*m_context_mock, createLogger(_))
            .Times(1)
            .WillOnce(Return(rclcpp::get_logger("TEST")));
        EXPECT_CALL(*m_context_mock, getRosCom())
            .Times(1)
            .WillOnce(ReturnRef(*m_ros_com_mock));
        EXPECT_CALL(*m_context_mock, getSysCom())
            .Times(1)
            .WillOnce(ReturnRef(*m_sys_com_mock));
        EXPECT_CALL(*m_ros_com_mock, subscribeForSetPlatformSpeed(_))
            .Times(1)
            .WillOnce(DoAll(
                SaveArg<0>(&m_callback),
                Return(nullptr)
            ));
        return std::make_unique<SetPlatformSpeedHandler>(*m_context_mock);
    }

    void launchHandler(double r_speed, double l_speed)
    {
        motoros_interfaces::msg::SetPlatformSpeed msg;
        msg.set__l_speed(l_speed);
        msg.set__r_speed(r_speed);
        (*m_callback)(msg);
    }

    std::unique_ptr<ContextMock> m_context_mock;
    std::unique_ptr<roscom::RosComMock> m_ros_com_mock;
    std::unique_ptr<syscom::SysComMock> m_sys_com_mock;
    std::optional<std::function<void(const motoros_interfaces::msg::SetPlatformSpeed&)>> m_callback;
};

TEST_F(SetPlatformSpeedHandlerTests, SysComFailDuringSend)
{
    auto sut = createSut();

    EXPECT_CALL(*m_sys_com_mock, send(_))
        .Times(1)
        .WillOnce(Return(false));
    EXPECT_NO_THROW(launchHandler(.0, .0));
}

struct ParametrizedTestSet
{
    double r_speed;
    double l_speed;
    PlatformSetMotorSpeedReq req;
};

class SetPlatformSpeedHandlerTestsParametrized :    
    public SetPlatformSpeedHandlerTests,
    public ::testing::WithParamInterface<ParametrizedTestSet>
{
};

std::string printReq(const PlatformSetMotorSpeedReq& r)
{
    std::stringstream str;
    str << "lSpeedI: " << static_cast<int>(r.lSpeedI);
    str << " lSpeedF: " << static_cast<int>(r.lSpeedF);
    str << " rSpeedI: " << static_cast<int>(r.rSpeedI);
    str << " rSpeedF: " << static_cast<int>(r.rSpeedF);
    return str.str();
}

TEST_P(SetPlatformSpeedHandlerTestsParametrized, BasicScenario)
{
    auto sut = createSut();

    syscom::Request received_val;
    EXPECT_CALL(*m_sys_com_mock, send(_))
        .Times(1)
        .WillOnce(DoAll(
            SaveArg<0>(&received_val),
            Return(true)));
    EXPECT_NO_THROW(launchHandler(GetParam().r_speed, GetParam().l_speed));
    EXPECT_EQ(received_val.msg_id, PLATFORM_SET_MOTOR_SPEED_REQ_ID);
    EXPECT_EQ(received_val.msg.set_motor_speed_req, GetParam().req)
        << "Received " << printReq(received_val.msg.set_motor_speed_req) 
        << std::endl
        << "Expected " << printReq(GetParam().req);
}

static const std::vector<ParametrizedTestSet> PARAMS = {
    {.r_speed=.0, .l_speed=.0, .req={0, 0, 0, 0}},
    {.r_speed=1.01, .l_speed=1.01, .req={1, 1, 1, 1}},
    {.r_speed=-11.05, .l_speed=100.075, .req={100, 7, -11, 5}},
    {.r_speed=99.99, .l_speed=-99.099999, .req={-99, 10, 99, 99}},
};

INSTANTIATE_TEST_SUITE_P(SetPlatformSpeedHandlerTestsParametrizedInstance,
    SetPlatformSpeedHandlerTestsParametrized,
    ValuesIn(PARAMS.begin(), PARAMS.end())
);

} // namespace platform_controller::init::controllers
