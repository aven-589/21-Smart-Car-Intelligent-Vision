#include "pid.h"
#include <math.h>
#include "zf_common_headfile.h"

int OUT_MAX = 0;
// float car_kp = 9.8;
float car_kp = 9.8;
// float car_ki = 4.5;
float car_ki = 4.5;
// float car_kd = 30;
float car_kd = 28;
void key_change_pid()
{
    static int which_k = 0;
    key_scanner();
    if (KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        which_k++;
        if (which_k > 3)
        {
            which_k = 0;
        }
    }
    if (KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        if (which_k == 0)
        {
            car_kp += 0.5;
        }
        else if (which_k == 1)
        {
            car_ki += 0.5;
        }
        else if (which_k == 2)
        {
            car_kd += 0.5;
        }
        else if (which_k == 3)
        {
            car_speed += 150;
        }
    }
    if (KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        if (which_k == 0)
        {
            car_kp -= 0.5;
        }
        else if (which_k == 1)
        {
            car_ki -= 0.5;
        }
        else if (which_k == 2)
        {
            car_kd -= 0.5;
        }
        else if (which_k == 3)
        {
            car_speed -= 30;
        }
    }
    if (which_k == 0)
    {
        ips200_show_string(150, 0, "kp");
    }
    else if (which_k == 1)
    {
        ips200_show_string(150, 0, "ki");
    }
    else if (which_k == 2)
    {
        ips200_show_string(150, 0, "kd");
    }
    else if (which_k == 3)
    {
        ips200_show_string(150, 0, "sp");
    }
    key_clear_all_state();
    ips200_show_string(0, 0, "car_kp:");
    ips200_show_float(80, 0, car_kp, 2, 1);
    ips200_show_string(0, 30, "car_ki:");
    ips200_show_float(80, 30, car_ki, 2, 1);
    ips200_show_string(0, 60, "car_kd:");
    ips200_show_float(80, 60, car_kd, 2, 1);
    ips200_show_string(0, 90, "car_speed:");
    ips200_show_int(80, 90, car_speed, 3);
    ips200_show_float(0, 120, a_pid, 4, 1);
    ips200_show_float(0, 150, b_pid, 4, 1);
    ips200_show_float(0, 180, c_pid, 4, 1);
    ips200_show_float(0, 210, d_pid, 4, 1);
}

// 全局变量声明（需要添加到头文件或文件顶部）
uint8_t start_game_flag = 0; // 开始发车标志位
void menu_setting()
{
    static uint8_t which_menu = 4;
    static uint8_t initial_level = 0;
    key_scanner();

    // KEY_1: 光标向上移动
    if (KEY_SHORT_PRESS == key_get_state(KEY_1))
    {
        if (which_menu > 0)
        {
            which_menu--;
        }
    }

    // KEY_2: 光标向下移动
    if (KEY_SHORT_PRESS == key_get_state(KEY_2))
    {
        if (which_menu < 5)
        {
            which_menu++;
        }
    }

    // KEY_3: 减小当前选项的值 / 确认开始发车
    if (KEY_SHORT_PRESS == key_get_state(KEY_3))
    {
        if (which_menu == 0)
        {
            if (initial_level > 0)
            {
                initial_level--;
            }
        }
        else if (which_menu == 1)
        {
            vision_high_fast_speed -= 20;
        }
        else if (which_menu == 2)
        {
            vision_fast_speed -= 20;
        }
        else if (which_menu == 3)
        {
            vision_stop_distance -= 0.1f;
        }
        else if (which_menu == 4)
        {
            start_game_flag = 1;
        }
        else if(which_menu==5)
        {
            error_distance-=0.02f;
        }
    }

    // KEY_4: 增大当前选项的值 / 确认开始发车
    if (KEY_SHORT_PRESS == key_get_state(KEY_4))
    {
        if (which_menu == 0)
        {
            if (initial_level < 2)
            {
                initial_level++;
            }
        }
        else if (which_menu == 1)
        {
            vision_high_fast_speed += 20;
        }
        else if (which_menu == 2)
        {
            vision_fast_speed += 20;
        }
        else if (which_menu == 3)
        {
            vision_stop_distance += 0.1f;
        }
        else if (which_menu == 4)
        {
            start_game_flag = 1;
        }
        else if(which_menu==5)
        {
            error_distance+=0.02f;
        }
    }

    // 根据设置更新level值
    switch (initial_level)
    {
    case 0:
        level = HAVE_NOTHING;
        break;
    case 1:
        level = HAVE_MODEL;
        break;
    case 2:
        level = HAVE_MODEL_AND_BOMBS;
        break;
    default:
        break;
    }

    // 绘制光标
    for (uint8_t i = 0; i < 6; i++)
    {
        if (i == which_menu)
        {
            ips200_show_string(0, i * 30, ">"); // 选中项显示光标
        }
        else
        {
            ips200_show_string(0, i * 30, " "); // 未选中项不显示光标
        }
    }

    // 显示选项名称和当前值
    ips200_show_string(20, 0, "Level:");
    ips200_show_int(80, 0, initial_level + 1, 1);

    ips200_show_string(20, 30, "FastSpeed:");
    ips200_show_int(120, 30, vision_fast_speed, 3);

    ips200_show_string(20, 60, "MidSpeed:");
    ips200_show_int(110, 60, vision_medium_speed, 3);

    ips200_show_string(20, 90, "StopDist:");
    ips200_show_float(100, 90, vision_stop_distance, 1, 1);

    ips200_show_string(20, 120, "Start:");
    ips200_show_int(80, 120, start_game_flag, 1);

    ips200_show_string(20, 150, "e_dis:");
    ips200_show_float(80, 150, error_distance, 1,2);
    key_clear_all_state();
}
extern float angle_out;
float error_a0 = 0, error_a1 = 0, error_a2 = 0;
float a_pid = 0;

void motor_a_pid(int speed_a)
{

    error_a2 = error_a1;
    error_a1 = error_a0;
    error_a0 = speed_a - encoder_data_quaddec_L1;
    // a_pid += car_kp * error_a0 + car_kd * (error_a0 - error_a1); // 经验增量式pid
    a_pid += car_kp * (error_a0 - error_a1) + car_ki * error_a0 + car_kd * (error_a0 - 2 * error_a1 + error_a2); // 真实增量式pid
    if (a_pid > OUT_MAX)
        a_pid = OUT_MAX;
    if (a_pid < -OUT_MAX)
        a_pid = -OUT_MAX;
    motor_a(a_pid);
}

float error_b0 = 0, error_b1 = 0, error_b2 = 0;

float b_pid = 0;

void motor_b_pid(int speed_b)
{

    error_b2 = error_b1;
    error_b1 = error_b0;
    error_b0 = speed_b - encoder_data_quaddec_R1;
    // b_pid += car_kp * error_b0 + car_kd * (error_b0 - error_b1); // 经验增量式pid
    b_pid += car_kp * (error_b0 - error_b1) + car_ki * error_b0 + car_kd * (error_b0 - 2 * error_b1 + error_b2); // 真实增量式pid
    if (b_pid > OUT_MAX)
        b_pid = OUT_MAX;
    if (b_pid < -OUT_MAX)
        b_pid = -OUT_MAX;
    motor_b(b_pid);
}
float c_pid = 0;

float error_c0 = 0, error_c1 = 0, error_c2 = 0;
void motor_c_pid(int speed_c)
{

    error_c2 = error_c1;
    error_c1 = error_c0;
    error_c0 = speed_c - encoder_data_quaddec_L2;
    // c_pid += car_kp * error_c0 + car_kd * (error_c0 - error_c1); // 经验增量式pid
    c_pid += car_kp * (error_c0 - error_c1) + car_ki * error_c0 + car_kd * (error_c0 - 2 * error_c1 + error_c2); // 真实增量式pid
    if (c_pid > OUT_MAX)
        c_pid = OUT_MAX;
    if (c_pid < -OUT_MAX)
        c_pid = -OUT_MAX;
    motor_c(c_pid);
}

float d_pid = 0;
float error_d0 = 0, error_d1 = 0, error_d2 = 0;

void motor_d_pid(int speed_d)
{

    error_d2 = error_d1;
    error_d1 = error_d0;
    error_d0 = speed_d - encoder_data_quaddec_R2;
    // d_pid += car_kp * error_d0 + car_kd * (error_d0 - error_d1); // 经验增量式pid
    d_pid += car_kp * (error_d0 - error_d1) + car_ki * error_d0 + car_kd * (error_d0 - 2 * error_d1 + error_d2); // 真实增量式pid
    if (d_pid > OUT_MAX)
        d_pid = OUT_MAX;
    if (d_pid < -OUT_MAX)
        d_pid = -OUT_MAX;
    motor_d(d_pid);
}

void move(int pwm, int way) // way: go left right back 四个参数
{
    if (pwm > SPEED_MAX)
        pwm = SPEED_MAX;
    if (pwm < -SPEED_MAX)
        pwm = -SPEED_MAX;
    if ((encoder_data_quaddec_L1 + encoder_data_quaddec_R1 + encoder_data_quaddec_L2 + encoder_data_quaddec_R2) / 4 < 120)
    {
        OUT_MAX = 2000;
    }
    else
    {
        OUT_MAX = 3000;
    }
    // vision_error_dx=0;
    // vision_error_dy=0;
    if(vision_error_dy<0.1)
    {
        vision_error_dy=0;
    }

    if(vision_error_dx<0.1)
    {
        vision_error_dx=0;
    }

    // if (way == go)
    // {
    //     motor_a_pid(pwm - angle_out );
    //     motor_b_pid(pwm + angle_out );
    //     motor_c_pid(pwm - angle_out );
    //     motor_d_pid(pwm + angle_out );
    // }
    // else if (way == left)
    // {
    //     motor_a_pid(-pwm - angle_out);
    //     motor_b_pid(pwm + angle_out) ;
    //     motor_c_pid(pwm - angle_out );
    //     motor_d_pid(-pwm + angle_out);
    // }
    // else if (way == right)
    // {
    //     motor_a_pid(pwm - angle_out );
    //     motor_b_pid(-pwm + angle_out) ;
    //     motor_c_pid(-pwm - angle_out );
    //     motor_d_pid(pwm + angle_out );
    // }
    // else if (way == back)
    // {
    //     motor_a_pid(-pwm - angle_out );
    //     motor_b_pid(-pwm + angle_out);
    //     motor_c_pid(-pwm - angle_out );
    //     motor_d_pid(-pwm + angle_out );
    // }
    // else
    // {
    //     motor_a_pid(0 - angle_out);
    //     motor_b_pid(0 + angle_out);
    //     motor_c_pid(0 - angle_out);
    //     motor_d_pid(0 + angle_out);
    //}
    if (way == go)
    {
        motor_a_pid(pwm - angle_out + vision_calibrate_kp * vision_error_dy);
        motor_b_pid(pwm + angle_out - vision_calibrate_kp * vision_error_dy);
        motor_c_pid(pwm - angle_out - vision_calibrate_kp * vision_error_dy);
        motor_d_pid(pwm + angle_out + vision_calibrate_kp * vision_error_dy);
    }
    else if (way == left)
    {
        motor_a_pid(-pwm - angle_out + vision_calibrate_kp * vision_error_dx);
        motor_b_pid(pwm + angle_out + vision_calibrate_kp * vision_error_dx);
        motor_c_pid(pwm - angle_out + vision_calibrate_kp * vision_error_dx);
        motor_d_pid(-pwm + angle_out + vision_calibrate_kp * vision_error_dx);
    }
    else if (way == right)
    {
        motor_a_pid(pwm - angle_out + vision_calibrate_kp * vision_error_dx);
        motor_b_pid(-pwm + angle_out + vision_calibrate_kp * vision_error_dx);
        motor_c_pid(-pwm - angle_out + vision_calibrate_kp * vision_error_dx);
        motor_d_pid(pwm + angle_out + vision_calibrate_kp * vision_error_dx);
    }
    else if (way == back)
    {
        motor_a_pid(-pwm - angle_out + vision_calibrate_kp * vision_error_dy);
        motor_b_pid(-pwm + angle_out - vision_calibrate_kp * vision_error_dy);
        motor_c_pid(-pwm - angle_out - vision_calibrate_kp * vision_error_dy);
        motor_d_pid(-pwm + angle_out + vision_calibrate_kp * vision_error_dy);
    }
}
