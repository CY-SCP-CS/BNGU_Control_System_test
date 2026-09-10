/**
 * @file    drv_imu.h
 * @brief   BMI088 六轴 IMU 驱动 (纯逻辑层)
 * @note    通过函数指针解耦 SPI 访问, 不依赖 BSP / HAL
 *          支持三种欧拉角模式, 可选择万向锁锁定的轴
 */
#ifndef DRV_IMU_H
#define DRV_IMU_H

#include "lib_typedef.h"
//少一个封装的更彻底的函数，还可以增加其他陀螺仪算法，考虑增加二阶龙格库塔法做积分
// ─── 数据类型 ─────────────────────────────────────

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} drv_imu_raw_t;

typedef struct {
    float x;
    float y;
    float z;
} drv_imu_real_t;

typedef struct {
    float q0;   /* 实部 */
    float q1;   /* 虚部 i */
    float q2;   /* 虚部 j */
    float q3;   /* 虚部 k */
} drv_imu_quat_t;

typedef struct {
    float pitch;    /* 俯仰角 (度) */
    float roll;     /* 横滚角 (度) */
    float yaw;      /* 偏航角 (度) */
} drv_imu_euler_t;

// ─── 欧拉角模式 ──────────────────────────────────

/**
 * @enum   drv_imu_euler_mode_t
 * @brief  欧拉角转换模式, 决定万向锁锁哪个轴
 * @note   XYZ → 锁 pitch (默认)
 *         ZYX → 锁 roll
 *         YXZ → 锁 yaw
 */
typedef enum {
    DRV_IMU_EULER_XYZ = 0,  /* 锁 pitch */
    DRV_IMU_EULER_ZYX = 1,  /* 锁 roll  */
    DRV_IMU_EULER_YXZ = 2,  /* 锁 yaw   */
} drv_imu_euler_mode_t;

// ─── 总线函数指针类型 ─────────────────────────────

typedef void (*drv_imu_spi_xfer_t)(void *ctx, const uint8_t *tx,
                                   uint8_t *rx, uint16_t len);
typedef void (*drv_imu_gpio_write_t)(void *ctx, uint8_t state);
typedef void (*drv_imu_delay_t)(uint32_t ms);

typedef struct {
    void               *spi_ctx;
    drv_imu_spi_xfer_t  spi_xfer;

    void               *acc_cs_ctx;
    drv_imu_gpio_write_t acc_cs;

    void               *gyro_cs_ctx;
    drv_imu_gpio_write_t gyro_cs;

    drv_imu_delay_t      delay_ms;
} drv_imu_bus_t;

// ─── IMU 句柄 ─────────────────────────────────────

typedef struct {
    drv_imu_bus_t       bus;
    drv_imu_euler_mode_t euler_mode;

    drv_imu_raw_t   acc_raw;
    drv_imu_raw_t   gyro_raw;

    drv_imu_real_t  acc;
    drv_imu_real_t  gyro;

    drv_imu_real_t  gyro_offset;
    drv_imu_real_t  acc_offset;

    float           acc_lsb_to_g;
    float           gyro_lsb_to_dps;

    drv_imu_quat_t  quat;
    drv_imu_euler_t euler;
    drv_imu_euler_t euler_offset;   /* 零位偏置, 由 calibrate_pose 记录 */

    float           temperature;

    float           integral_fb[3];  /* Mahony 积分项 */
} drv_imu_t;

// ─── 接口声明 ─────────────────────────────────────

void drv_imu_init(drv_imu_t *imu, const drv_imu_bus_t *bus);
void drv_imu_start(drv_imu_t *imu);
void drv_imu_set_euler_mode(drv_imu_t *imu, drv_imu_euler_mode_t mode);

void drv_imu_read_acc_raw(drv_imu_t *imu);
void drv_imu_read_gyro_raw(drv_imu_t *imu);
void drv_imu_data_convert(drv_imu_t *imu);
void drv_imu_read_temp(drv_imu_t *imu);

void drv_imu_mahony_update(drv_imu_t *imu, float dt);
void drv_imu_quat_to_euler(drv_imu_t *imu);

void drv_imu_calibrate_gyro(drv_imu_t *imu, uint16_t sample_count);
void drv_imu_initial_alignment(drv_imu_t *imu);
void drv_imu_calibrate_pose(drv_imu_t *imu);
void drv_imu_restart(drv_imu_t *imu);

// ─── Port: BSP 适配 ─────────────────────────────

/**
 * @brief  初始化 BMI088 (BSP 适配版)
 * @param  imu  IMU 句柄
 * @note   内部填充 drv_imu_bus_t, 挂接 SPI1 / PA4(ACC_CS) / PB0(GYRO_CS)
 */
void drv_imu_port_init(drv_imu_t *imu);

/** @brief 启动一次 ACC→GYRO SPI DMA 采样链，返回 1 表示已启动。 */
uint8_t drv_imu_port_async_start(void);

/** @brief 将最近一次完整 DMA 快照复制到 IMU 实例，返回 1 表示有新快照。 */
uint8_t drv_imu_port_snapshot_update(drv_imu_t *imu);

/** @brief 判断异步采样是否在指定时间内产生过完整快照。 */
uint8_t drv_imu_port_is_online(uint32_t timeout_ms);

/** @brief 获取 DMA 忙导致的采样跳过次数。 */
uint32_t drv_imu_port_get_busy_count(void);

/** @brief 获取 DMA 启动、传输或超时错误次数。 */
uint32_t drv_imu_port_get_error_count(void);

/**
 * @brief  启动加热 PWM (TIM10_CH1)
 * @note   调用 HAL_TIM_Base_Start_IT / HAL_TIM_PWM_Start
 */
void drv_imu_port_heater_start(void);

/**
 * @brief  设置加热 PWM 比较值
 * @param  val  比较值
 */
void drv_imu_port_heater_set(uint16_t val);

/**
 * @brief  延时 (毫秒)
 * @param  ms  毫秒数
 */
void drv_imu_port_delay_ms(uint32_t ms);

#endif
