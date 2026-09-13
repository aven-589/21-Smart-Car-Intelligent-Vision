// #ifndef _imu_H
// #define _imu_H
// #include "zf_common_headfile.h"
// #define M_PI 3.1415926
// #define ACCEL_SCALE 2048.0f  // 加速度计量程±16g时的缩放因子
// #define GYRO_SCALE (16.4f * 180.0f / M_PI)  // 陀螺仪量程±2000°/s时的缩放因子

// // EKF相关定义
// #define STATE_DIM 7 // 状态维度：四元数(q0,q1,q2,q3) + 陀螺仪偏差(bx,by,bz)
// #define MEAS_DIM 3  // 测量维度：加速度计(x,y,z)

// void MahonyAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz) ;
// void QuaternionToEuler(float *yaw);
// void imu660_Calibrate(uint16_t samples);

// extern float exInt, eyInt, ezInt;
// extern float gyroOffsetZ;
// extern float gyroOffsetX;
// extern float gyroOffsetY;
// extern float accelOffsetX;
// extern float accelOffsetY;
// extern float accelOffsetZ;
// extern uint8_t gyroCalibrated;
// extern uint8_t accelCalibrated;

// #endif

/*- * yaw.h
 *
 *  Created on: 2026��3��7��
 *      Author: 25307
 */

#ifndef _imu_H
#define _imu_H


#include "zf_common_headfile.h"
#include <math.h>


extern float a1,a2,a3;
// ==========================================================
// �㷨���Ĳ������� (�����ʵ�����΢��)
// ==========================================================
#define SAMPLE_FREQ     1000.0f      // ���ʵ�ʲ���Ƶ��(Hz)������Ͷ�ʱ��Ƶ��һ�£�
#define Kp_DEFAULT      0.5f        // �������� (��Сһ��ɼ��پ�ֹʱ�Ķ���������0.2~0.5)
#define Ki_DEFAULT      0.001f      // �������� (�����������ھ�̬���)

// ==========================================================
// �����Ż����� (ר��Ư�ƺ�ë��)
// ==========================================================
#define GYRO_DEADZONE   0.6f       // ����������(��/��)��ר�� Yaw ������ʱ�䲻���ۼ�Ư��
#define ANGLE_LPF_ALPHA 0.02f       // �Ƕȵ�ͨ�˲�ϵ��(0~1)��ר�� Pitch/Roll С�����2��3λ������ (ԽСԽƽ��)

// ==========================================================
// �ⲿ��ȡ�����սǶȱ��� (��λ����)
// ==========================================================
extern float roll;
extern float pitch;
extern float yaw;

// ==========================================================
// ���ⲿ���õĺ�������
// ==========================================================
void Mahony_Init(void);
static void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az);
static void Mahony_ComputeAngles(void);
void IMU_gyro_offset_Init(void);
void imu660ra_get(void);
#endif /* CODE_YAW_H_ */
