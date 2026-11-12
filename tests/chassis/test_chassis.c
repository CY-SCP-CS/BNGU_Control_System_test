/* 执行真实底盘源文件和真实 PID/数学/滤波库，CAN 与中断操作由主机桩替代。 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../../Self_core/App/sentry/chassis/app_sentry_chassis.c"

static unsigned assertions;
static app_chassis_comm_rx_t test_rx;
#define NEAR(actual, expected, tolerance) do { \
    float a_ = (actual), e_ = (expected); ++assertions; \
    if (!isfinite(a_) || fabsf(a_ - e_) > (tolerance)) { \
        fprintf(stderr, "line %d: %s = %.9g; expected %.9g\n", \
                __LINE__, #actual, (double)a_, (double)e_); exit(1); \
    } \
} while (0)

uint8_t app_chassis_comm_read_rx(app_chassis_comm_rx_t *rx)
{ *rx = test_rx; return 1U; }
uint8_t app_chassis_comm_power_tx(int16_t power) { (void)power; return 0U; }
uint8_t app_chassis_comm_omega_tx(float omega) { (void)omega; return 0U; }

static void reset_state(void)
{
    memset(&app_sentry_chassis_debug, 0, sizeof(app_sentry_chassis_debug));
    memset(&test_rx, 0, sizeof(test_rx));
    pid_init_all();
}

static void test_speed_ramp(void)
{
    reset_state();
#if SENTRY_CHASSIS_RAMP_ENABLE
    s_motion_state.cmd_speed.vx_speed = SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2;
    for (int n = 0; n < 1000; ++n) {
        chassis_ramp_update(0.001f);
        NEAR(s_motion_state.tar_speed.vx_speed, SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 * 0.001f * (float)(n + 1), 0.03f);
    }
    /* 到达目标不继续累加；普通回中制动平滑，换向经过零。 */
    chassis_ramp_update(0.001f);
    NEAR(s_motion_state.tar_speed.vx_speed, SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2, 0.01f);
    s_motion_state.cmd_speed.vx_speed = 0.0f;
    for (int n = 0; n < 1000; ++n) chassis_ramp_update(0.001f);
    NEAR(s_motion_state.tar_speed.vx_speed, 0.0f, 0.01f);
    s_motion_state.tar_speed.vx_speed = SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2;
    s_motion_state.cmd_speed.vx_speed = -SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2;
    for (int n = 0; n < 1000; ++n) chassis_ramp_update(0.001f);
    NEAR(s_motion_state.tar_speed.vx_speed, 0.0f, 0.01f);
    for (int n = 0; n < 1000; ++n) chassis_ramp_update(0.001f);
    NEAR(s_motion_state.tar_speed.vx_speed, -SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2, 0.01f);

    reset_state();
    s_motion_state.cmd_speed.vx_speed = SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2;
    s_motion_state.cmd_speed.vy_speed = SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2;
    s_motion_state.cmd_speed.vw_speed = 1.5f;
    for (int n = 0; n < 1000; ++n) chassis_ramp_update(0.001f);
    NEAR(s_motion_state.tar_speed.vx_speed, SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 * 0.70710678f, 0.1f);
    NEAR(s_motion_state.tar_speed.vy_speed, SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 * 0.70710678f, 0.1f);
    NEAR(s_motion_state.tar_speed.vw_speed, 1.5f, 0.0001f);

    /* 任意连续变向：每步平移向量长度和角速度增量均满足独立上限。 */
    reset_state();
    for (int n = 0; n < 5000; ++n) {
        app_sentry_chassis_speed_t previous = s_motion_state.tar_speed;
        s_motion_state.cmd_speed.vx_speed = 3000.0f * sinf(n * 0.17f);
        s_motion_state.cmd_speed.vy_speed = 3000.0f * cosf(n * 0.11f);
        s_motion_state.cmd_speed.vw_speed = 10.0f * sinf(n * 0.13f);
        chassis_ramp_update(0.001f);
        float dx = s_motion_state.tar_speed.vx_speed - previous.vx_speed;
        float dy = s_motion_state.tar_speed.vy_speed - previous.vy_speed;
        float step = sqrtf(dx * dx + dy * dy);
        NEAR(fminf(step, SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 * 0.001f), step, 0.001f);
        float dw = fabsf(s_motion_state.tar_speed.vw_speed - previous.vw_speed);
        NEAR(fminf(dw, SENTRY_CHASSIS_ANGULAR_ACCEL_RAD_S2 * 0.001f), dw, 0.00001f);
    }
#else
    s_motion_state.cmd_speed = (app_sentry_chassis_speed_t){ SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2, -500.0f, 1.5f };
    chassis_ramp_update(0.001f);
    NEAR(s_motion_state.tar_speed.vx_speed, SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2, 0.0f);
    NEAR(s_motion_state.tar_speed.vy_speed, -500.0f, 0.0f);
    NEAR(s_motion_state.tar_speed.vw_speed, 1.5f, 0.0f);
#endif
}

static void test_ramp_control_integration(void)
{
    reset_state();
    test_rx.speed.vx = 1000;
    app_sentry_chassis_ctrl();
    NEAR(s_motion_state.cmd_speed.vx_speed, 1000.0f, 0.0f);
#if SENTRY_CHASSIS_RAMP_ENABLE
    NEAR(s_motion_state.tar_speed.vx_speed, fminf(1000.0f, SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 * 0.001f), 0.0001f);
#else
    NEAR(s_motion_state.tar_speed.vx_speed, 1000.0f, 0.0f);
#endif
    /* 急停必须立刻清输出、清规划历史；解除后从零重新起步。 */
    test_rx.speed.vx = SENTRY_CHASSIS_ESTOP;
    app_sentry_chassis_ctrl();
    NEAR(s_motion_state.cmd_speed.vx_speed, 0.0f, 0.0f);
    NEAR(s_motion_state.tar_speed.vx_speed, 0.0f, 0.0f);
    NEAR(s_motion_state.tar_speed.vy_speed, 0.0f, 0.0f);
    NEAR(s_motion_state.tar_speed.vw_speed, 0.0f, 0.0f);
    for (int i = 0; i < 4; ++i) NEAR(s_motor_state.current[i], 0.0f, 0.0f);
    test_rx.speed.vx = 1000;
    app_sentry_chassis_ctrl();
#if SENTRY_CHASSIS_RAMP_ENABLE
    NEAR(s_motion_state.tar_speed.vx_speed, fminf(1000.0f, SENTRY_CHASSIS_LINEAR_ACCEL_MM_S2 * 0.001f), 0.0001f);
#endif
    app_sentry_chassis_init();
    NEAR(s_motion_state.tar_speed.vx_speed, 0.0f, 0.0f);
}

int main(void)
{
    test_speed_ramp();
    test_ramp_control_integration();
    printf("PASS: %u assertions (ramp=%d)\n", assertions, SENTRY_CHASSIS_RAMP_ENABLE);
    return 0;
}