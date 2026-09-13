#include "zf_common_headfile.h"
#include "guandao.h"
#include "isr.h"
#include "math.h"

int encoder_count_go = 0;	 // 前进的累计编码器值
int encoder_count_back = 0;	 // 后退的累计编码器值
int encoder_count_left = 0;	 // 左移的累计编码器值
int encoder_count_right = 0; // 右移的累计编码器值

int encoder_go = 0;	   // 前进的编码器值
int encoder_back = 0;  // 后退的编码器值
int encoder_left = 0;  // 左移的编码器值
int encoder_right = 0; // 右移的编码器值

float encoder_distance_go = 0;	  // 前进的距离
float encoder_distance_back = 0;  // 后退的距离
float encoder_distance_left = 0;  // 左移的距离
float encoder_distance_right = 0; // 右移的距离

int last_way = TEST_NUM;		 // 上一次的移动方向，初始为与任何方向都不相关的状态
int way_flag = 0;				 // 当前移动方向
int encoder_move_go_flag = 0;	 // 前进时是否已经记录起点距离
int encoder_move_back_flag = 0;	 // 后退时是否已经记录起点距离
int encoder_move_left_flag = 0;	 // 左移时是否已经记录起点距离
int encoder_move_right_flag = 0; // 右移时是否已经记录起点距离

float encoder_move_go_start = 0;	// 前进起点距离
float encoder_move_back_start = 0;	// 后退起点距离
float encoder_move_right_start = 0; // 右移起点距离
float encoder_move_left_start = 0;	// 左移起点距离
// 放在中断零中，中断10ms，该函数返回这10ms内的编码器总值
void get_encoder_value() // way: go left right back 四个参数,返回值为编码器值
{
	encoder_go = 0;
	encoder_back = 0;
	encoder_left = 0;
	encoder_right = 0;
	if (encoder_data_quaddec_L1 > 0 && encoder_data_quaddec_L2 > 0 && encoder_data_quaddec_R1 > 0 && encoder_data_quaddec_R2 > 0) // 前进的编码器值
	{
		way_flag = go;
		encoder_go = (encoder_data_quaddec_L1 + encoder_data_quaddec_L2 + encoder_data_quaddec_R1 + encoder_data_quaddec_R2) / 4;
	}
	else if (encoder_data_quaddec_L1 < 0 && encoder_data_quaddec_L2 < 0 && encoder_data_quaddec_R1 < 0 && encoder_data_quaddec_R2 < 0) // 后退的编码器值
	{
		way_flag = back;
		encoder_back = -(encoder_data_quaddec_L1 + encoder_data_quaddec_L2 + encoder_data_quaddec_R1 + encoder_data_quaddec_R2) / 4;
	}
	else if (encoder_data_quaddec_L1 < 0 && encoder_data_quaddec_L2 > 0 && encoder_data_quaddec_R1 > 0 && encoder_data_quaddec_R2 < 0) // 左移的编码器值
	{
		way_flag = left;
		encoder_left = -(encoder_data_quaddec_L1 - encoder_data_quaddec_L2 - encoder_data_quaddec_R1 + encoder_data_quaddec_R2) / 4;
	}
	else if (encoder_data_quaddec_L1 > 0 && encoder_data_quaddec_L2 < 0 && encoder_data_quaddec_R1 < 0 && encoder_data_quaddec_R2 > 0) // 右移的编码器值
	{
		way_flag = right;
		encoder_right = -(-encoder_data_quaddec_L1 + encoder_data_quaddec_L2 + encoder_data_quaddec_R1 - encoder_data_quaddec_R2) / 4;
	}
	else
	{
		return;
	}
}

int encoder_update() // 更新编码器值
{
	if (way_flag == go)
	{
		encoder_count_go += encoder_go;										 // 更新前进的编码器值
		encoder_distance_go = encoder_count_go * (PI * ZHIJING / (PPR * 2)); // 前进的距离
	}

	else if (way_flag == back)
	{
		encoder_count_back += encoder_back;										 // 更新后退的编码器值
		encoder_distance_back = encoder_count_back * (PI * ZHIJING / (PPR * 2)); // 后退的距离
	}
	else if (way_flag == left)
	{
		encoder_count_left += encoder_left;										 // 更新左移的编码器值
		encoder_distance_left = encoder_count_left * (PI * ZHIJING / (PPR * 2)); // 左移的距离
	}
	else if (way_flag == right)
	{
		encoder_count_right += encoder_right;									   // 更新右移的编码器值
		encoder_distance_right = encoder_count_right * (PI * ZHIJING / (PPR * 2)); // 右移的距离
	}
	return 0;
}

int encoder_move_distance(float distance, int way) // distance(格数): 移动距离,way: go left right back 四个参数
{
	// if(distance>5)
	if (way == go)
	{
		if (encoder_move_go_flag == 0)
		{
			encoder_move_go_start = encoder_distance_go; // 记录本次前进开始时的距离
			encoder_move_go_flag = 1;
		}
		if (fabs(encoder_distance_go - encoder_move_go_start) >= distance / 10 * encoder_x_max - ENCODER_STOP_MARGIN) // 当前前进距离减去起点距离，判断是否到达目标距离
		{
			car_speed = 0, car_way = 0;
			encoder_move_go_flag = 0; // 停车后清零，下次再次进入时重新记录起点
			last_way = way;
			return 1;
		}
		else if (distance < 2) // 距离太短就低速
		{
			double dist = fabs(encoder_distance_go - encoder_move_go_start);
			car_speed = ENCODER_VISION_CALIBRATE_SPEED, car_way = way;
			return 0;
		}
		else if (fabs(encoder_distance_go - encoder_move_go_start) <= ENCODER_SLOW_SPEED_DISTANCE) // 开始时线性加速
		{
			double dist = fabs(encoder_distance_go - encoder_move_go_start);
			car_speed = ENCODER_SLOW_SPEED + dist / ENCODER_SLOW_SPEED_DISTANCE * (ENCODER_FAST_SPEED - ENCODER_SLOW_SPEED), car_way = way;
		}
		else if (fabs(encoder_distance_go - encoder_move_go_start) >= distance / 10 * encoder_x_max - ENCODER_SLOWDOWN_DISTANCE) // 快到了，减速
		{
			double total_pulse = distance / 10 * encoder_x_max - ENCODER_SLOWDOWN_DISTANCE;
			double run_dist = fabs(encoder_distance_go - encoder_move_go_start);
			double remain_dist = run_dist - total_pulse;
			car_speed = ENCODER_FAST_SPEED - remain_dist / ENCODER_SLOWDOWN_DISTANCE * (ENCODER_FAST_SPEED - ENCODER_SLOW_SPEED);
			car_way = way;
		}
		else
		{
			car_speed = ENCODER_FAST_SPEED, car_way = way; // 其他情况就正常跑
		}
	}
	else if (way == back)
	{
		if (encoder_move_back_flag == 0)
		{
			encoder_move_back_start = encoder_distance_back; // 记录本次后退开始时的距离
			encoder_move_back_flag = 1;
		}
		if (fabs(encoder_distance_back - encoder_move_back_start) >= distance / 10 * encoder_x_max - ENCODER_STOP_MARGIN) // 当前后退距离减去起点距离，判断是否到达目标距离
		{
			car_speed = 0, car_way = way;
			encoder_move_back_flag = 0; // 停车后清零，下次再次进入时重新记录起点
			last_way = way;
			return 1;
		}
		else if (distance < 2) // 距离太短就低速
		{
			double dist = fabs(encoder_distance_back - encoder_move_back_start);
			car_speed = ENCODER_VISION_CALIBRATE_SPEED, car_way = way;
			return 0;
		}
		else if (fabs(encoder_distance_back - encoder_move_back_start) <= ENCODER_SLOW_SPEED_DISTANCE) // 开始时线性加速
		{
			double dist = fabs(encoder_distance_back - encoder_move_back_start);
			car_speed = ENCODER_SLOW_SPEED + dist / ENCODER_SLOW_SPEED_DISTANCE * (ENCODER_FAST_SPEED - ENCODER_SLOW_SPEED), car_way = way;
		}
		else if (fabs(encoder_distance_back - encoder_move_back_start) >= distance / 10 * encoder_x_max - ENCODER_SLOWDOWN_DISTANCE) // 快到了，减速
		{
			double total_pulse = distance / 10 * encoder_x_max - ENCODER_SLOWDOWN_DISTANCE;
			double run_dist = fabs(encoder_distance_back - encoder_move_back_start);
			double remain_dist = run_dist - total_pulse;
			car_speed = ENCODER_FAST_SPEED - remain_dist / ENCODER_SLOWDOWN_DISTANCE * (ENCODER_FAST_SPEED - ENCODER_SLOW_SPEED);
			car_way = way;
		}
		else
		{
			car_speed = ENCODER_FAST_SPEED, car_way = way;
		}
	}
	else if (way == left)
	{
		if (encoder_move_left_flag == 0)
		{
			encoder_move_left_start = encoder_distance_left; // 记录本次左移开始时的距离
			encoder_move_left_flag = 1;
		}
		if (fabs(encoder_distance_left - encoder_move_left_start) >= distance / 14 * encoder_y_max - ENCODER_STOP_MARGIN) // 当前左移距离减去起点距离，判断是否到达目标距离
		{
			car_speed = 0, car_way = way;
			encoder_move_left_flag = 0; // 停车后清零，下次再次进入时重新记录起点
			last_way = way;
			return 1;
		}
		else if (distance < 2) // 距离太短就低速
		{
			double dist = fabs(encoder_distance_left - encoder_move_left_start);
			car_speed = ENCODER_VISION_CALIBRATE_SPEED, car_way = way;
			return 0;
		}
		else if (fabs(encoder_distance_left - encoder_move_left_start) <= ENCODER_SLOW_SPEED_DISTANCE) // 开始时线性加速
		{
			double dist = fabs(encoder_distance_left - encoder_move_left_start);
			car_speed = ENCODER_SLOW_SPEED + dist / ENCODER_SLOW_SPEED_DISTANCE * (ENCODER_FAST_SPEED - ENCODER_SLOW_SPEED), car_way = way;
		}
		else if (fabs(encoder_distance_left - encoder_move_left_start) >= distance / 14 * encoder_y_max - ENCODER_SLOWDOWN_DISTANCE) // 快到了，减速
		{
			double total_pulse = distance / 14 * encoder_y_max - ENCODER_SLOWDOWN_DISTANCE;
			double run_dist = fabs(encoder_distance_left - encoder_move_left_start);
			double remain_dist = run_dist - total_pulse;
			car_speed = ENCODER_FAST_SPEED - remain_dist / ENCODER_SLOWDOWN_DISTANCE * (ENCODER_FAST_SPEED - ENCODER_SLOW_SPEED);
			car_way = way;
		}
		else
		{
			car_speed = ENCODER_FAST_SPEED, car_way = way;
		}
	}
	else if (way == right)
	{
		if (encoder_move_right_flag == 0)
		{
			encoder_move_right_start = encoder_distance_right; // 记录本次右移开始时的距离
			encoder_move_right_flag = 1;
		}
		if (fabs(encoder_distance_right - encoder_move_right_start) >= distance / 14 * encoder_y_max - ENCODER_STOP_MARGIN) // 当前右移距离减去起点距离，判断是否到达目标距离
		{
			car_speed = 0, car_way = way;
			encoder_move_right_flag = 0; // 停车后清零，下次再次进入时重新记录起点
			last_way = way;
			return 1;
		}
		// else if (distance < 2) // 距离太短就低速
		// {
		// 	double dist = fabs(encoder_distance_right - encoder_move_right_start);
		// 	car_speed = ENCODER_VISION_CALIBRATE_SPEED, car_way = way;
		// 	return 0;
		// }
		else if (fabs(encoder_distance_right - encoder_move_right_start) <= ENCODER_SLOW_SPEED_DISTANCE) // 开始时线性加速
		{
			double dist = fabs(encoder_distance_right - encoder_move_right_start);
			car_speed = ENCODER_SLOW_SPEED + dist / ENCODER_SLOW_SPEED_DISTANCE * (ENCODER_FAST_SPEED - ENCODER_SLOW_SPEED), car_way = way;
		}
		else if (fabs(encoder_distance_right - encoder_move_right_start) >= distance / 14 * encoder_y_max - ENCODER_SLOWDOWN_DISTANCE) // 快到了，减速
		{
			double total_pulse = distance / 14 * encoder_y_max - ENCODER_SLOWDOWN_DISTANCE;
			double run_dist = fabs(encoder_distance_right - encoder_move_right_start);
			double remain_dist = run_dist - total_pulse;
			car_speed = ENCODER_FAST_SPEED - remain_dist / ENCODER_SLOWDOWN_DISTANCE * (ENCODER_FAST_SPEED - ENCODER_SLOW_SPEED);
			car_way = way;
		}
		else
		{
			car_speed = ENCODER_FAST_SPEED, car_way = way;
		}
	}
	return 0;
}
