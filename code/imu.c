// #include "zf_common_headfile.h"
// #include "imu.h"
// #include "stddef.h"
// #include "math.h"

// // ��������医�����鐚�紮炊��茹ｇ����後��鐚�
// float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// // 腱����莚�綏�鐚����篋�Mahony膊�羈�鐚�
// float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
// // 膊�羈�������
// #define Kp 5.0f  // 罸�箴�紜����
// #define Ki 0.1f // 腱����紜����
// #define dt 0.002f

// // ��≦��������
// float gyroOffsetZ = 0.0f;
// float gyroOffsetX = 0.0f;
// float gyroOffsetY = 0.0f;
// float accelOffsetX = 0.0f;
// float accelOffsetY = 0.0f;
// float accelOffsetZ = 0.0f;
// uint8_t gyroCalibrated = 0;
// uint8_t accelCalibrated = 0;

// void MahonyAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz)
// {
//     float norm;
//     float vx, vy, vz; // 篌域�∞����������劫��
//     float ex, ey, ez; // 莚�綏�蕁�
//     float qa, qb, qc; // 筝贋�九�����

//     // 綵�筝����������綺�莅≧�井��鐚�羔���ら�����紊у�鏄怨��鐚�
//     norm = sqrt(ax * ax + ay * ay + az * az);
//     if (norm == 0)
//         return; // ��水����ら�狗��莚�
//     ax /= norm;
//     ay /= norm;
//     az /= norm;

//     // ��号��綵������������遺式莅♂�������劫��
//     vx = 2 * (q1 * q3 - q0 * q2);
//     vy = 2 * (q0 * q1 + q2 * q3);
//     vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

//     // 莅∞��������綺�莅≧�������寂��篌域�≦�主��莚�綏�
//     ex = ay * vz - az * vy;
//     ey = az * vx - ax * vz;
//     ez = ax * vy - ay * vx;

//     // 腱����莚�綏�蕁刻�����篋���炊��篆�罩ｏ��
//     exInt += Ki * ex * dt;
//     eyInt += Ki * ey * dt;
//     ezInt += Ki * ez * dt;

//     // ���莚�綏�茵ュ�翠����坂鯖��井��
//     gx += Kp * ex + exInt;
//     gy += Kp * ey + eyInt;
//     gz += Kp * ez + ezInt;

//     // ��贋�医�������逸��茹ｅ小�����合��鐚�
//     qa = q0;
//     qb = q1;
//     qc = q2;
//     q0 += (-qb * gx - qc * gy - q3 * gz) * dt / 2;
//     q1 += (qa * gx + qc * gz - q3 * gy) * dt / 2;
//     q2 += (qa * gy - qb * gz + q3 * gx) * dt / 2;
//     q3 += (qa * gz + qb * gy - qc * gx) * dt / 2;

//     // 綵�筝������������逸��篆�������篏���������亥�号�э��
//     norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//     if (norm == 0)
//         return;
//     q0 /= norm;
//     q1 /= norm;
//     q2 /= norm;
//     q3 /= norm;
// }

// /**
//  * @brief ��������域習罨ф��茹�鐚�綣у墾鐚�
//  * @param roll 罔�羯�茹�鐚�膸�X莉器��
//  * @param pitch 篆�篁域��鐚�膸�Y莉器��
//  * @param yaw ������茹�鐚�膸�Z莉器��
//  */
// void QuaternionToEuler(float *yaw)
// {
//     *yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1);
// }

// /**
//  * @brief �����九�拷����坂鯖筝�������綺�莅≦����倶����≦��鐚����綛九�醇�逸��
//  * @param samples ��≦�������傑�≧�逸��綮肴��1000罨￥��
//  * ��≦����＞散鐚�罔≦�����罩≫��Z莉贋��筝�鐚����篋�������綺�莅￥��
//  */
// void imu660_Calibrate(uint16_t samples)
// {
//     int32_t sumGx = 0, sumGy = 0, sumGz = 0;
//     int32_t sumAx = 0, sumAy = 0, sumAz = 0;
//     uint16_t i;

//     // ���臀���霡脂��
//     gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0.0f;
//     accelOffsetX = accelOffsetY = accelOffsetZ = 0.0f;

//     // ��������傑��鐚����罩∝�倶��筝���������坂鯖筝�������綺�莅≧�井��鐚�
//     for (i = 0; i < samples; i++)
//     {
//         imu660ra_get_acc();
//         imu660ra_get_gyro();

//         sumAx += imu660ra_acc_x;
//         sumAy += imu660ra_acc_y;
//         sumAz += imu660ra_acc_z;

//         sumGx += imu660ra_gyro_x;
//         sumGy += imu660ra_gyro_y;
//         sumGz += imu660ra_gyro_z;
//         ips200_show_int(0, 30, i, 5); // ��丞ず��≦����傑�����
//         system_delay_ms(2); // ���篏������潔�����
//     }

//     // 莅∞�������坂鯖綛喝����霡脂��鐚����莉�筝� 属/s鐚����莉�筝� rad/s鐚�
//     float avgGx = (float)sumGx / samples;
//     float avgGy = (float)sumGy / samples;
//     float avgGz = (float)sumGz / samples;
//     gyroOffsetX = imu660ra_gyro_transition(avgGx) * M_PI / 180.0f;
//     gyroOffsetY = imu660ra_gyro_transition(avgGy) * M_PI / 180.0f;
//     gyroOffsetZ = imu660ra_gyro_transition(avgGz) * M_PI / 180.0f;
//     // 莅∞��������綺�莅≦抗�����種��莉���≫減���������篏�鐚�g鐚�
//     float avgAx = imu660ra_acc_transition((float)sumAx / samples);
//     float avgAy = imu660ra_acc_transition((float)sumAy / samples);
//     float avgAz = imu660ra_acc_transition((float)sumAz / samples);
//     // �����括�倶��鐚�羂翫抗���罩∽��X=0g, Y=0g, Z=1g鐚�������鐚�
//     accelOffsetX = avgAx;        // X莉顔�倶��茵ュ��
//     accelOffsetY = avgAy;        // Y莉顔�倶��茵ュ��
//     accelOffsetZ = avgAz - 1.0f; // Z莉顔�倶��茵ュ�随������ｻ������鐚�

//     gyroCalibrated = 1;
//     accelCalibrated = 1; // ���莅井�≦��絎����

// }

/*
 * imu.c
 *
 *  Created on: 2026年3月7日
 *      Author: 25307
 *
 *  基于 Mahony 6轴姿态解算的优化封装
 */

#include "imu.h"
#include <math.h>
#include <stdint.h>

// ==========================================================
// 全局/静态变量定义
// ==========================================================
// float gyro_offset_x = 0.9465, gyro_offset_y = -3.885, gyro_offset_z = 0.3562;
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float ax, ay, az, gx, gy, gz;
float a1, a2, a3;
// 内部四元数变量
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// 内部积分误差变量
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
// 采样周期的一半 (确保 SAMPLE_FREQ 在 imu.h 中已定义)
static float halfT = 0.5f / SAMPLE_FREQ;

// 对外输出的角度全局变量 (外部直接读取数值)
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

// ==========================================================
// 1. 快速平方根倒数算法 (Quake III 经典实现)
// ==========================================================
static float invSqrt(float yaw_x)
{
    float halfx = 0.5f * yaw_x;
    float yaw_y = yaw_x;
    int32_t i = *(int32_t *)&yaw_y;
    i = 0x5f3759df - (i >> 1);
    yaw_y = *(float *)&i;
    yaw_y = yaw_y * (1.5f - (halfx * yaw_y * yaw_y));
    return yaw_y;
}

// ==========================================================
// 2. 陀螺仪零偏校准 (静态内部函数)
// ==========================================================
void IMU_gyro_offset_Init(void)
{
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    const int sample_count = 2000;

    for (int i = 0; i < sample_count; i++)
    {
        imu660ra_get_gyro();
        gyro_sum_x += (float)imu660ra_gyro_x;
        gyro_sum_y += (float)imu660ra_gyro_y;
        gyro_sum_z += (float)imu660ra_gyro_z;
        system_delay_ms(2);
    }

    gyro_offset_x = gyro_sum_x / (float)sample_count;
    gyro_offset_y = gyro_sum_y / (float)sample_count;
    gyro_offset_z = gyro_sum_z / (float)sample_count;
    a1 = gyro_offset_x;
    a2 = gyro_offset_y;
    a3 = gyro_offset_z;
}

// ==========================================================
// 3. 用加速度计状态初始化四元数 (避免倾斜导致Yaw突变)
// ==========================================================
static void IMU_Quaternion_Init_From_Accel(float init_ax, float init_ay, float init_az)
{
    // 归一化加速度计
    float norm = invSqrt(init_ax * init_ax + init_ay * init_ay + init_az * init_az);
    init_ax *= norm;
    init_ay *= norm;
    init_az *= norm;

    // 计算初始 Roll 和 Pitch
    float initial_roll = atan2f(init_ay, init_az);

    // 防溢出处理
    if (init_ax > 1.0f)
        init_ax = 1.0f;
    if (init_ax < -1.0f)
        init_ax = -1.0f;
    float initial_pitch = asinf(-init_ax);

    // 计算半角
    float half_roll = initial_roll * 0.5f;
    float half_pitch = initial_pitch * 0.5f;

    float sin_r = sinf(half_roll);
    float cos_r = cosf(half_roll);
    float sin_p = sinf(half_pitch);
    float cos_p = cosf(half_pitch);

    // 构建初始四元数 (假定初始 Yaw = 0)
    q0 = cos_r * cos_p;
    q1 = sin_r * cos_p;
    q2 = cos_r * sin_p;
    q3 = -sin_r * sin_p;

    // 重置历史积分
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

// ==========================================================
// 4. 对外暴露的唯一初始化接口 (在 main() 中只调用一次)
// ==========================================================
void Mahony_Init(void)
{
    // 1. 重置变量
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;

    // 2. 采集陀螺仪零偏 (设备需静止约4秒)
    IMU_gyro_offset_Init();

    // 3. 读取一帧加速度计数据
    imu660ra_get_acc();
    float init_ax = imu660ra_acc_transition(imu660ra_acc_x);
    float init_ay = imu660ra_acc_transition(imu660ra_acc_y);
    float init_az = imu660ra_acc_transition(imu660ra_acc_z);

    // 4. 根据当前倾斜状态动态初始化四元数
    IMU_Quaternion_Init_From_Accel(init_ax, init_ay, init_az);
}

// ==========================================================
// 5. 核心更新函数 (内部调用)
// ==========================================================
static void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1. 陀螺仪死区过滤 (使用 fabsf 绝对值，滤除静止时的微小噪声)
    if (fabsf(gx) < GYRO_DEADZONE)
        gx = 0.0f;
    if (fabsf(gy) < GYRO_DEADZONE)
        gy = 0.0f;
    if (fabsf(gz) < GYRO_DEADZONE)
        gz = 0.0f;

    // 2. 单位转换: 度/秒 -> 弧度/秒
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // 3. 防止加速度计超限
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        integralFBx += ex * Ki_DEFAULT * (2.0f * halfT);
        integralFBy += ey * Ki_DEFAULT * (2.0f * halfT);
        integralFBz += ez * Ki_DEFAULT * (2.0f * halfT);

        gx += Kp_DEFAULT * ex + integralFBx;
        gy += Kp_DEFAULT * ey + integralFBy;
        gz += Kp_DEFAULT * ez + integralFBz;
    }

    // 4. 四元数微分方程更新
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * halfT;
    q1 += (qa * gx + qc * gz - q3 * gy) * halfT;
    q2 += (qa * gy - qb * gz + q3 * gx) * halfT;
    q3 += (qa * gz + qb * gy - qc * gx) * halfT;

    // 5. 归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// ==========================================================
// 6. 计算最终的欧拉角 (低通滤波防止抖动，Yaw 不做滤波保证响应速度)
// ==========================================================
static void Mahony_ComputeAngles(void)
{
    float raw_Roll = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29578f;

    // 限制 asin 输入范围，防止超出范围导致 NaN 异常
    float sin_pitch = -2.0f * (q1 * q3 - q0 * q2);
    if (sin_pitch > 1.0f)
        sin_pitch = 1.0f;
    if (sin_pitch < -1.0f)
        sin_pitch = -1.0f;
    float raw_Pitch = asinf(sin_pitch) * 57.29578f;

    float raw_Yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29578f;

    // 低通滤波平滑处理 (roll/pitch 用滤波，yaw 直接缩放以保持响应速度)
    roll = (ANGLE_LPF_ALPHA * raw_Roll) + ((1.0f - ANGLE_LPF_ALPHA) * roll);
    pitch = (ANGLE_LPF_ALPHA * raw_Pitch) + ((1.0f - ANGLE_LPF_ALPHA) * pitch);
    yaw = raw_Yaw / 2;
}

// ==========================================================
// 7. 周期调用接口，读取传感器数据并解算姿态
// ==========================================================
void imu660ra_get(void)
{
    imu660ra_get_gyro();
    imu660ra_get_acc();

    ax = imu660ra_acc_transition(imu660ra_acc_x);
    ay = imu660ra_acc_transition(imu660ra_acc_y);
    az = imu660ra_acc_transition(imu660ra_acc_z);
    gx = imu660ra_gyro_transition(imu660ra_gyro_x - gyro_offset_x);
    gy = imu660ra_gyro_transition(imu660ra_gyro_y - gyro_offset_y);
    gz = imu660ra_gyro_transition(imu660ra_gyro_z - gyro_offset_z);

    // 运行算法解算姿态
    Mahony_Update(gx, gy, gz, ax, ay, az);
    Mahony_ComputeAngles();
}