/**
 * @file    drv_imu.c
 * @brief   BMI088 六轴 IMU 驱动实现 (纯逻辑层)
 */
#include "drv_imu.h"
#include <math.h>
#include <string.h>

// ─── 常量 ─────────────────────────────────────────

#define DRV_IMU_RAD_TO_DEG              57.29578f
#define DRV_IMU_DEG_TO_RAD              0.01745329f
#define DRV_IMU_GYRO_2000_LSB_TO_DPS    (2000.0f / 32768.0f)
#define DRV_IMU_ACC_6G_LSB_TO_G         (6.0f / 32768.0f)

#define DRV_IMU_MAHONY_KP               0.2f
#define DRV_IMU_MAHONY_KI               0.05f

#define BMI088_ACC_CONF_NORMAL_1600_HZ   0xACU
#define BMI088_GYRO_BW_116_ODR_1000_HZ  0x02U

#define BMI088_ACC_CS_HIGH(imu, ctx) \
    do { if ((imu)->bus.acc_cs) (imu)->bus.acc_cs(ctx, 1); } while (0)
#define BMI088_ACC_CS_LOW(imu, ctx)  \
    do { if ((imu)->bus.acc_cs) (imu)->bus.acc_cs(ctx, 0); } while (0)
#define BMI088_GYRO_CS_HIGH(imu, ctx) \
    do { if ((imu)->bus.gyro_cs) (imu)->bus.gyro_cs(ctx, 1); } while (0)
#define BMI088_GYRO_CS_LOW(imu, ctx)  \
    do { if ((imu)->bus.gyro_cs) (imu)->bus.gyro_cs(ctx, 0); } while (0)

// ─── 私有函数: SPI 读写 ───────────────────────────

static uint8_t acc_read_reg(drv_imu_t *imu, uint8_t reg)
{
    uint8_t tx[3] = {reg | 0x80, 0, 0};
    uint8_t rx[3] = {0};

    BMI088_ACC_CS_LOW(imu, imu->bus.acc_cs_ctx);
    imu->bus.spi_xfer(imu->bus.spi_ctx, tx, rx, 3);
    BMI088_ACC_CS_HIGH(imu, imu->bus.acc_cs_ctx);

    return rx[2];   /* ACC 需要先发地址, 再收 2 字节, 第 2 字节为有效数据 */
}

static void acc_write_reg(drv_imu_t *imu, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg & 0x7F, data};
    uint8_t rx[2] = {0};

    BMI088_ACC_CS_LOW(imu, imu->bus.acc_cs_ctx);
    imu->bus.spi_xfer(imu->bus.spi_ctx, tx, rx, 2);
    BMI088_ACC_CS_HIGH(imu, imu->bus.acc_cs_ctx);
}

static void gyro_write_reg(drv_imu_t *imu, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg & 0x7F, data};
    uint8_t rx[2] = {0};

    BMI088_GYRO_CS_LOW(imu, imu->bus.gyro_cs_ctx);
    imu->bus.spi_xfer(imu->bus.spi_ctx, tx, rx, 2);
    BMI088_GYRO_CS_HIGH(imu, imu->bus.gyro_cs_ctx);
}

// ─── 私有函数: 延时 ───────────────────────────────

static void imu_delay(drv_imu_t *imu, uint32_t ms)
{
    if (imu->bus.delay_ms)
        imu->bus.delay_ms(ms);
}

// ─── 私有函数: 欧拉角转换 (三种模式) ─────────────

/**
 * @brief  XYZ 模式 — 锁 pitch (默认, 原始代码行为)
 * @note   singularity: |2*(q0*q1 + q2*q3)| >= 0.9995 → roll = ±90°
 *         lock: pitch = 0
 */
static void euler_xyz(drv_imu_t *imu)
{
    float q0 = imu->quat.q0, q1 = imu->quat.q1;
    float q2 = imu->quat.q2, q3 = imu->quat.q3;

    float sinr = 2.0f * (q0 * q1 + q2 * q3);

    if (fabsf(sinr) < 0.9995f) {
        imu->euler.roll  = asinf(sinr) * DRV_IMU_RAD_TO_DEG;
        imu->euler.pitch = atan2f(2.0f * (q0 * q2 - q1 * q3),
                                  1.0f - 2.0f * (q1 * q1 + q2 * q2)) * DRV_IMU_RAD_TO_DEG;
        imu->euler.yaw   = atan2f(2.0f * (q0 * q3 - q1 * q2),
                                  1.0f - 2.0f * (q1 * q1 + q3 * q3)) * DRV_IMU_RAD_TO_DEG;
    } else {
        imu->euler.roll  = copysignf(90.0f, sinr);
        imu->euler.pitch = 0.0f;    /* ← LOCKED */
        imu->euler.yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3),
                                  q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * DRV_IMU_RAD_TO_DEG;
    }
}

/**
 * @brief  ZYX 模式 — 锁 roll
 * @note   singularity: cos(pitch) ≈ 0 → pitch = ±90°
 *         lock: roll = 0
 *         formulas based on ZYX intrinsic (R = Rx * Ry * Rz):
 *           yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2² + q3²))
 *           pitch = -asin(2*(q1*q3 - q0*q2))
 *           roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1² + q2²))
 */
static void euler_zyx(drv_imu_t *imu)
{
    float q0 = imu->quat.q0, q1 = imu->quat.q1;
    float q2 = imu->quat.q2, q3 = imu->quat.q3;

    float sinp = 2.0f * (q1 * q3 - q0 * q2);

    if (fabsf(sinp) < 0.9995f) {
        imu->euler.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * DRV_IMU_RAD_TO_DEG;
        imu->euler.pitch = -asinf(sinp) * DRV_IMU_RAD_TO_DEG;
        imu->euler.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                  1.0f - 2.0f * (q1 * q1 + q2 * q2)) * DRV_IMU_RAD_TO_DEG;
    } else {
        imu->euler.pitch = copysignf(90.0f, -sinp);
        imu->euler.roll  = 0.0f;    /* ← LOCKED */
        imu->euler.yaw   = atan2f(2.0f * (q1 * q2 - q0 * q3),
                                  2.0f * (q0 * q1 + q2 * q3)) * DRV_IMU_RAD_TO_DEG;
    }
}

/**
 * @brief  YXZ 模式 — 锁 yaw
 * @note   singularity: |2*(q0*q1 + q2*q3)| >= 0.9995 → roll = ±90°
 *         lock: yaw = 0
 *         在 roll=±90°, yaw=0 时, pitch 由 asin(2*(q1*q3 - q0*q2)) 计算
 */
static void euler_yxz(drv_imu_t *imu)
{
    float q0 = imu->quat.q0, q1 = imu->quat.q1;
    float q2 = imu->quat.q2, q3 = imu->quat.q3;

    float sinr = 2.0f * (q0 * q1 + q2 * q3);

    if (fabsf(sinr) < 0.9995f) {
        /* 正常情况同 XYZ */
        imu->euler.roll  = asinf(sinr) * DRV_IMU_RAD_TO_DEG;
        imu->euler.pitch = atan2f(2.0f * (q0 * q2 - q1 * q3),
                                  1.0f - 2.0f * (q1 * q1 + q2 * q2)) * DRV_IMU_RAD_TO_DEG;
        imu->euler.yaw   = atan2f(2.0f * (q0 * q3 - q1 * q2),
                                  1.0f - 2.0f * (q1 * q1 + q3 * q3)) * DRV_IMU_RAD_TO_DEG;
    } else {
        imu->euler.roll = copysignf(90.0f, sinr);
        imu->euler.yaw  = 0.0f;     /* ← LOCKED */
        /* 在 roll=±90°, yaw=0 时计算 pitch:
         * R = Rz(0) * Ry(pitch) * Rx(±90°) = Ry(pitch) * Rx(±90°)
         * R[2][0] = 2*(q1*q3 - q0*q2) = sin(pitch)
         * R[1][0] = 2*(q0*q3 + q1*q2) = 0 (yaw=0 时恒为 0)
         * → pitch = asin(2*(q1*q3 - q0*q2))
         */
        float sp = 2.0f * (q1 * q3 - q0 * q2);
        if (sp > 1.0f) sp = 1.0f;
        if (sp < -1.0f) sp = -1.0f;
        imu->euler.pitch = asinf(sp) * DRV_IMU_RAD_TO_DEG;
    }
}

// ─── 接口实现 ─────────────────────────────────────

void drv_imu_init(drv_imu_t *imu, const drv_imu_bus_t *bus)
{
    memset(imu, 0, sizeof(*imu));

    if (bus) imu->bus = *bus;

    imu->euler_mode    = DRV_IMU_EULER_XYZ;
    imu->acc_lsb_to_g  = DRV_IMU_ACC_6G_LSB_TO_G;
    imu->gyro_lsb_to_dps = DRV_IMU_GYRO_2000_LSB_TO_DPS;
    imu->quat.q0       = 1.0f;
}

void drv_imu_set_euler_mode(drv_imu_t *imu, drv_imu_euler_mode_t mode)
{
    if (mode > DRV_IMU_EULER_YXZ) return;
    imu->euler_mode = mode;
}

void drv_imu_start(drv_imu_t *imu)
{
    uint8_t dummy = 0;
    uint8_t tx[1] = {0};

    /* ── ACC 软复位前的 SPI 唤醒 ── */
    BMI088_ACC_CS_HIGH(imu, imu->bus.acc_cs_ctx);
    imu_delay(imu, 1);
    BMI088_ACC_CS_LOW(imu, imu->bus.acc_cs_ctx);
    imu->bus.spi_xfer(imu->bus.spi_ctx, tx, &dummy, 1);
    BMI088_ACC_CS_HIGH(imu, imu->bus.acc_cs_ctx);
    imu_delay(imu, 50);

    /* ACC 软复位 */
    acc_write_reg(imu, 0x7E, 0xB6);
    imu_delay(imu, 150);

    /* ACC SPI 唤醒 (第二次) */
    BMI088_ACC_CS_LOW(imu, imu->bus.acc_cs_ctx);
    imu->bus.spi_xfer(imu->bus.spi_ctx, tx, &dummy, 1);
    BMI088_ACC_CS_HIGH(imu, imu->bus.acc_cs_ctx);
    imu_delay(imu, 10);

    /* ACC 配置: 正常带宽, ODR=1600Hz */
    acc_write_reg(imu, 0x7D, 0x04);     /* PWR_CTRL: 正常模式 */
    imu_delay(imu, 50);
    acc_write_reg(imu, 0x41, 0x01);     /* RANGE: ±6g */
    acc_write_reg(imu, 0x40, BMI088_ACC_CONF_NORMAL_1600_HZ);
    acc_write_reg(imu, 0x53, 0x0A);     /* INT1_IO_CTRL */
    acc_write_reg(imu, 0x58, 0x04);     /* INT_MAP: data ready on INT1 */

    /* ── GYRO 配置 ── */
    gyro_write_reg(imu, 0x14, 0xB6);    /* 软复位 */
    imu_delay(imu, 80);
    gyro_write_reg(imu, 0x0F, 0x00);    /* RANGE: ±2000°/s */
    gyro_write_reg(imu, 0x10, BMI088_GYRO_BW_116_ODR_1000_HZ);
    gyro_write_reg(imu, 0x15, 0x80);    /* INT_CTRL: data ready on INT3 */
    gyro_write_reg(imu, 0x16, 0x0C);
    gyro_write_reg(imu, 0x18, 0x01);    /* INT3_INT4_IO_CONF */
}

void drv_imu_read_acc_raw(drv_imu_t *imu)
{
    uint8_t buf[8];
    uint8_t tx_buf[8] = {0x12 | 0x80, 0, 0, 0, 0, 0, 0, 0};

    BMI088_ACC_CS_LOW(imu, imu->bus.acc_cs_ctx);
    imu->bus.spi_xfer(imu->bus.spi_ctx, tx_buf, buf, 8);
    BMI088_ACC_CS_HIGH(imu, imu->bus.acc_cs_ctx);

    imu->acc_raw.x = (int16_t)((buf[3] << 8) | buf[2]);
    imu->acc_raw.y = (int16_t)((buf[5] << 8) | buf[4]);
    imu->acc_raw.z = (int16_t)((buf[7] << 8) | buf[6]);
}

void drv_imu_read_gyro_raw(drv_imu_t *imu)
{
    uint8_t tx_buf[7] = {0x02 | 0x80, 0, 0, 0, 0, 0, 0};
    uint8_t rx_buf[7] = {0};

    BMI088_GYRO_CS_LOW(imu, imu->bus.gyro_cs_ctx);
    imu->bus.spi_xfer(imu->bus.spi_ctx, tx_buf, rx_buf, 7);
    BMI088_GYRO_CS_HIGH(imu, imu->bus.gyro_cs_ctx);

    imu->gyro_raw.x = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
    imu->gyro_raw.y = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
    imu->gyro_raw.z = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);
}

void drv_imu_data_convert(drv_imu_t *imu)
{
    imu->acc.x = (float)imu->acc_raw.x * imu->acc_lsb_to_g;
    imu->acc.y = (float)imu->acc_raw.y * imu->acc_lsb_to_g;
    imu->acc.z = (float)imu->acc_raw.z * imu->acc_lsb_to_g;

    imu->gyro.x = (float)imu->gyro_raw.x * imu->gyro_lsb_to_dps;
    imu->gyro.y = (float)imu->gyro_raw.y * imu->gyro_lsb_to_dps;
    imu->gyro.z = (float)imu->gyro_raw.z * imu->gyro_lsb_to_dps;
}

void drv_imu_read_temp(drv_imu_t *imu)
{
    uint8_t buf[2];
    buf[0] = acc_read_reg(imu, 0x22);
    buf[1] = acc_read_reg(imu, 0x23);

    uint16_t temp_raw = (uint16_t)((buf[0] << 3) | (buf[1] >> 5));
    imu->temperature = (float)temp_raw * 0.125f + 23.0f;
}

void drv_imu_mahony_update(drv_imu_t *imu, float dt)
{
    float q0 = imu->quat.q0, q1 = imu->quat.q1;
    float q2 = imu->quat.q2, q3 = imu->quat.q3;

    float gx = imu->gyro.x - imu->gyro_offset.x;
    float gy = imu->gyro.y - imu->gyro_offset.y;
    float gz = imu->gyro.z - imu->gyro_offset.z;
    float ax = imu->acc.x - imu->acc_offset.x;
    float ay = imu->acc.y - imu->acc_offset.y;
    float az = imu->acc.z - imu->acc_offset.z;

    /* 归一化加速度计 */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    /* 估计重力方向 (由四元数) */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 误差 = 测量 × 估计 的叉积 */
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    /* PI 补偿 */
    imu->integral_fb[0] += ex * DRV_IMU_MAHONY_KI * dt;
    imu->integral_fb[1] += ey * DRV_IMU_MAHONY_KI * dt;
    imu->integral_fb[2] += ez * DRV_IMU_MAHONY_KI * dt;

    gx += DRV_IMU_MAHONY_KP * ex + imu->integral_fb[0];
    gy += DRV_IMU_MAHONY_KP * ey + imu->integral_fb[1];
    gz += DRV_IMU_MAHONY_KP * ez + imu->integral_fb[2];

    /* 四元数积分 (一阶龙格-库塔) */
    float d2 = 0.5f * dt;
    imu->quat.q0 += (-q1 * gx - q2 * gy - q3 * gz) * d2;
    imu->quat.q1 += ( q0 * gx + q2 * gz - q3 * gy) * d2;
    imu->quat.q2 += ( q0 * gy - q1 * gz + q3 * gx) * d2;
    imu->quat.q3 += ( q0 * gz + q1 * gy - q2 * gx) * d2;

    /* 归一化 */
    norm = sqrtf(imu->quat.q0 * imu->quat.q0 +
                 imu->quat.q1 * imu->quat.q1 +
                 imu->quat.q2 * imu->quat.q2 +
                 imu->quat.q3 * imu->quat.q3);
    if (norm < 1e-6f) return;
    imu->quat.q0 /= norm;
    imu->quat.q1 /= norm;
    imu->quat.q2 /= norm;
    imu->quat.q3 /= norm;
}

void drv_imu_quat_to_euler(drv_imu_t *imu)
{
    switch (imu->euler_mode) {
        case DRV_IMU_EULER_ZYX:
            euler_zyx(imu);
            break;
        case DRV_IMU_EULER_YXZ:
            euler_yxz(imu);
            break;
        default:
            euler_xyz(imu);
            break;
    }
}

void drv_imu_calibrate_gyro(drv_imu_t *imu, uint16_t sample_count)
{
    float sum[3] = {0};

    for (uint16_t i = 0; i < sample_count; i++) {
        drv_imu_read_gyro_raw(imu);
        drv_imu_data_convert(imu);

        sum[0] += imu->gyro.x;
        sum[1] += imu->gyro.y;
        sum[2] += imu->gyro.z;

        imu_delay(imu, 2);
    }

    imu->gyro_offset.x = sum[0] / sample_count;
    imu->gyro_offset.y = sum[1] / sample_count;
    imu->gyro_offset.z = sum[2] / sample_count;

    imu->acc_offset.x = 0.0f;
    imu->acc_offset.y = 0.0f;
    imu->acc_offset.z = 0.0f;
}

void drv_imu_initial_alignment(drv_imu_t *imu)
{
    float ax = imu->acc.x;
    float ay = imu->acc.y;
    float az = imu->acc.z;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.1f) return;
    ax /= norm; ay /= norm; az /= norm;

    float initial_roll  = atan2f(ay, az);
    float initial_pitch = -asinf(ax);
    float initial_yaw   = 0.0f;

    float cr = cosf(initial_roll * 0.5f);
    float sr = sinf(initial_roll * 0.5f);
    float cp = cosf(initial_pitch * 0.5f);
    float sp = sinf(initial_pitch * 0.5f);
    float cy = cosf(initial_yaw * 0.5f);
    float sy = sinf(initial_yaw * 0.5f);

    imu->quat.q0 = cr * cp * cy + sr * sp * sy;
    imu->quat.q1 = sr * cp * cy - cr * sp * sy;
    imu->quat.q2 = cr * sp * cy + sr * cp * sy;
    imu->quat.q3 = cr * cp * sy - sr * sp * cy;

    imu->integral_fb[0] = 0.0f;
    imu->integral_fb[1] = 0.0f;
    imu->integral_fb[2] = 0.0f;
}

void drv_imu_calibrate_pose(drv_imu_t *imu)
{
    /* 记录当前欧拉角作为零位偏置 (调用前需先调用 quat_to_euler) */
    imu->euler_offset.pitch = imu->euler.pitch;
    imu->euler_offset.roll  = imu->euler.roll;
    imu->euler_offset.yaw   = imu->euler.yaw;
}

void drv_imu_restart(drv_imu_t *imu)
{
    acc_write_reg(imu, 0x7E, 0xB6);    /* ACC 软复位 */
    gyro_write_reg(imu, 0x14, 0xB6);   /* GYRO 软复位 */

    imu_delay(imu, 100);

    /* 恢复初始对准 */
    imu->quat.q0 = 1.0f;
    imu->quat.q1 = 0.0f;
    imu->quat.q2 = 0.0f;
    imu->quat.q3 = 0.0f;
    imu->integral_fb[0] = 0.0f;
    imu->integral_fb[1] = 0.0f;
    imu->integral_fb[2] = 0.0f;

    drv_imu_start(imu);
}
