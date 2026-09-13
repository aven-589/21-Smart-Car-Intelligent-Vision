#ifndef _move_H_
#define _move_H_
#include "zf_common_headfile.h"

extern int flag_a;
void motor_init(void);
void motor_a(int pwm);
void motor_b(int pwm);
void motor_c(int pwm);
void motor_d(int pwm);
#define MOTOR1_PWM1                 (PWM2_MODULE1_CHB_C9)
#define MOTOR1_PWM2                 (PWM2_MODULE1_CHA_C8)

#define MOTOR2_PWM1                 (PWM2_MODULE0_CHB_C7)
#define MOTOR2_PWM2                 (PWM2_MODULE0_CHA_C6)

#define MOTOR3_PWM1                 (PWM2_MODULE3_CHA_D2)
#define MOTOR3_PWM2                 (PWM2_MODULE3_CHB_D3)

#define MOTOR4_PWM1                 (PWM2_MODULE2_CHB_C11)
#define MOTOR4_PWM2                 (PWM2_MODULE2_CHA_C10)

#endif