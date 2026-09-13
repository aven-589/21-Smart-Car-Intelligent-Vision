#ifndef _pid_H
#define _pid_H
#include "zf_common_headfile.h"
#define go 0
#define back 180
#define left 90
#define right -90
#define SPEED_MAX 4000
extern float car_kp,car_ki,car_kd;
extern float a_pid;
extern float b_pid;
extern float c_pid;
extern float d_pid;
void motor_a_pid(int speed_a);
void motor_b_pid(int speed_b);
void motor_c_pid(int speed_c);
void motor_d_pid(int speed_d);
void move(int pwm,int way);
void key_change_pid(void);
extern uint8_t start_game_flag;
void menu_setting(void);
#endif