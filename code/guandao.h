#ifndef __GUANDAO_H
#define __GUANDAO_H
#include "zf_common_headfile.h"

// #define encoder_x_max  2500//机械实验室数据
// #define encoder_y_max  3650

#define encoder_x_max  2400
#define encoder_y_max  3200

#define TEST_NUM -1  //测试或者给某个参数初始化用，目的是给一个与初始不相关的状态
#define PPR 1024   //每圈的脉冲数
#define ZHIJING 63 //63mm,轮子直径
#define ENCODER_STOP_MARGIN 5  //到目标点前5mm认为到达
#define ENCODER_SLOWDOWN_DISTANCE 300  //距离目标点300mm内开始减速
#define ENCODER_SLOW_SPEED_DISTANCE 300  //起步300mm内低速
#define ENCODER_FAST_SPEED 100  //快速移动速度
#define ENCODER_SLOW_SPEED 60   //减速区速度
#define ENCODER_VISION_CALIBRATE_SPEED 20  //视觉校准速度

extern int encoder_count_go;//前进的编码器值
extern int encoder_count_back;//后退的编码器值
extern int encoder_count_left;//左移的编码器值
extern int encoder_count_right;//右移的编码器值
extern float encoder_distance_go;//前进的距离
extern float encoder_distance_back;//后退的距离
extern float encoder_distance_left;//左移的距离
extern float encoder_distance_right;//右移的距离
extern float encoder_move_go_start;//前进的初始距离
extern float encoder_move_back_start;//后退的初始距离
extern float encoder_move_left_start;//左移的初始距离
extern float encoder_move_right_start;//右移的初始距离

void get_encoder_value();//获取编码器值
int encoder_update();//更新编码器值
int encoder_move_distance(float distance,int way);//distance(mm): 移动距离,way: go left right back 四个参数

#endif
