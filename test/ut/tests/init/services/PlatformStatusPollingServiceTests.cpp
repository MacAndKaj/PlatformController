/**
  * Copyright (c) 2023 MacAndKaj. All rights reserved.
  */
#include <platform_controller/init/services/PlatformStatusPollingService.hpp>

#include <platform_controller_mocks/init/ContextMock.hpp>
#include <platform_controller_mocks/roscom/RosComMock.hpp>
#include <platform_controller_mocks/roscom/RosComSenderMock.hpp>
#include <platform_controller_mocks/init/TimersManagerMock.hpp>
#include <platform_controller_mocks/syscom/SysComMock.hpp>

#include <gtest/gtest.h>

using testing::DoAll;
using testing::Return;
using testing::ReturnRef;
using testing::SaveArg;
using testing::Test;
using testing::_;

using CallbackType = std::function<void(
    std::shared_ptr<mi_services::PlatformStatusPolling::Request>,
    std::shared_ptr<mi_services::PlatformStatusPolling::Response>)>;
using SyscomResponseCallbackType = std::function<void(const platform_controller::syscom::Response&)>;
using TimerCallbackType = std::function<void()>;
using ResponseStatusType = mi_services::PlatformStatusPolling::Response::_response_status_type;

namespace platform_controller::init::services
{

static constexpr int TEST_SUBSCRIPTION_ID = 1;
static constexpr int TEST_TIMER_ID = 2;
static constexpr std::uint16_t TEST_TIMER_PERIOD = 10;

struct PlatformStatusPollingServiceTests : public Test
{
    void SetUp() override
    {
        m_context_mock = std::make_unique<ContextMock>();
        m_sys_com_mock = std::make_unique<syscom::SysComMock>();
        m_ros_com_mock = std::make_unique<roscom::RosComMock>();
        m_ros_com_sender_mock = std::make_shared<roscom::RosComSenderMock>();
        m_timers_manager_mock = std::make_unique<TimersManagerMock>();
    }

    std::unique_ptr<PlatformStatusPollingService> createSut()
    {
        EXPECT_CALL(*m_context_mock, createLogger(_))
            .Times(1)
            .WillOnce(Return(rclcpp::get_logger("TEST")));
        EXPECT_CALL(*m_context_mock, getSysCom())
            .Times(1)
            .WillOnce(ReturnRef(*m_sys_com_mock));
        EXPECT_CALL(*m_context_mock, getRosCom())
            .Times(2)
            .WillRepeatedly(ReturnRef(*m_ros_com_mock))
            .RetiresOnSaturation();
        EXPECT_CALL(*m_context_mock, getTimersManager())
            .Times(1)
            .WillOnce(ReturnRef(*m_timers_manager_mock));
        EXPECT_CALL(*m_ros_com_mock, createServiceForPollPlatformStatus(_))
            .Times(1)
            .WillOnce(DoAll(
                SaveArg<0>(&m_callback),
                Return(nullptr)
            ));
        return std::make_unique<PlatformStatusPollingService>(*m_context_mock);
    }

    void invokeCallback(
        std::shared_ptr<mi_services::PlatformStatusPolling::Request> req,
        std::shared_ptr<mi_services::PlatformStatusPolling::Response> resp,
        ResponseStatusType::_status_type expected_status = ResponseStatusType::GENERIC_STATUS_OK)
    {
        EXPECT_TRUE(m_callback);
        m_callback(req, resp);
        EXPECT_EQ(resp->response_status.status, expected_status);
    }

    void invokeTimerCallback()
    {
        EXPECT_TRUE(m_timer_callback);
        m_timer_callback();
    }

    void invokeSyscomResponseCallback()
    {
        syscom::Response msg{};
        msg.msg_id = PLATFORM_POLL_STATUS_RESP_ID;
        msg.msg.poll_status_resp.moduleStatus = 0;
        EXPECT_TRUE(m_syscom_response_callback);
        m_syscom_response_callback(msg);
    }

    void expectSyscomSubscribe()
    {
        EXPECT_CALL(*m_sys_com_mock, subscribe(PLATFORM_POLL_STATUS_RESP_ID, _))
            .Times(1)
            .WillOnce(DoAll(
                SaveArg<1>(&m_syscom_response_callback),
                Return(TEST_TIMER_ID)
            ))
            .RetiresOnSaturation();
    }

    void expectSyscomSend()
    {
        EXPECT_CALL(*m_sys_com_mock, send(_))
            .Times(1)
            .WillOnce(Return(true))
            .RetiresOnSaturation();
    }

    void expectRoscomSend()
    {
        EXPECT_CALL(*m_ros_com_sender_mock, send(_));
    }

    void expectCreateSender()
    {
        EXPECT_CALL(*m_ros_com_mock, createSender(_))
            .WillOnce(Return(m_ros_com_sender_mock))
            .RetiresOnSaturation();
    }

    void expectTimerStart(uint16_t period)
    {
        const auto expected_period_ms = std::chrono::milliseconds(period);
        EXPECT_CALL(*m_timers_manager_mock, startCyclicTimer(expected_period_ms, _))
            .Times(1)
            .WillOnce(DoAll(
                SaveArg<1>(&m_timer_callback),
                Return(TEST_TIMER_ID)
            ))
            .RetiresOnSaturation();
    }

    std::unique_ptr<ContextMock> m_context_mock;
    std::unique_ptr<roscom::RosComMock> m_ros_com_mock;
    std::shared_ptr<roscom::RosComSenderMock> m_ros_com_sender_mock;
    std::unique_ptr<syscom::SysComMock> m_sys_com_mock;
    std::unique_ptr<TimersManagerMock> m_timers_manager_mock;
    CallbackType m_callback;
    SyscomResponseCallbackType m_syscom_response_callback;
    TimerCallbackType m_timer_callback;
};

TEST_F(PlatformStatusPollingServiceTests, BasicScenario)
{
    std::unique_ptr<PlatformStatusPollingService> sut = createSut();
    auto req = std::make_shared<mi_services::PlatformStatusPolling::Request>();
    req->period = TEST_TIMER_PERIOD;
    auto resp = std::make_shared<mi_services::PlatformStatusPolling::Response>();
    
    expectSyscomSubscribe();
    expectTimerStart(TEST_TIMER_PERIOD);
    expectCreateSender();
    invokeCallback(req, resp);

    expectSyscomSend();
    invokeTimerCallback();

    expectRoscomSend();
    invokeSyscomResponseCallback();
}
 
} // namespace platform_controller::init::services
