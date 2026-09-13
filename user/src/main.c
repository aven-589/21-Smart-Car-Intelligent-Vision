#include "zf_common_headfile.h"
#include <math.h>
#define SWITCH1 (C15)

extern int esp_uart1_rx_flag;
extern OpenMV_Union_t mv_packet;
extern OpenMV_Map_t map_packet;
extern OpenMV_model_Union_t model_packet; // 模型数据包
extern float esp_pid_kp;
extern float esp_pid_ki;
extern float esp_pid_kd;
extern int esp_move;
extern int map_rx_flag;
extern int rx_flag; // 数据标志位
int16 encoder_data_quaddec_L1 = 0;
int16 encoder_data_quaddec_R1 = 0;
int16 encoder_data_quaddec_L2 = 0;
int16 encoder_data_quaddec_R2 = 0;
extern float man_x, man_y;
extern int path_ok;
extern int manhattan_get_path_error;
extern int wait_ok_flag;
extern int model_destination_num;
int a = 0;
int b = 0;
int c = 0;
uint8_t wait_for_uart = 0;
extern float d;
extern int stuck_x;
extern int stuck_y;
extern int unlock_bomb_x;
extern int unlock_bomb_y;
extern int bomb_finish_flag;
extern uint8_t only_one_destination_flag;
extern uint8_t have_used_destination_num;
extern uint8_t destination_used[BOX_DESTINATION_NUM_MAX];
extern uint8_t choose_vision_or_encoder_to_run_path;
extern int current_level;
extern uint8_t error_flag;
uint8_t menu_flag = 0;
extern float last_destination_x, last_destination_y;
extern uint8_t error_count;
extern uint8_t get_better_path_flag;
extern int bba, bbb, bbc, bbd, bbe, bbf;
extern uint8_t temp_byte;
int main(void)
{
	clock_init(SYSTEM_CLOCK_600M); // 不可删除
	debug_init();				   // 调试端口初始化
	system_delay_ms(300);		   // 等待主板其他外设上电完成
								   // 此处编写用户代码 例如外设初始化代码等
	ips200_init(IPS200_TYPE_PARALLEL8);
	ips200_full(RGB565_BLACK);
	motor_init();										   // 电机初始化
	Encoder_Init();										   // 编码器初始化
	uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17); // 初始化编码器模块与引脚 正交解码编码器模式
	uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13); // 初始化编码器模块与引脚 正交解码编码器模式
	uart_rx_interrupt(UART_1, ZF_ENABLE);				   // 开启 UART_1 的接收中断
	uart_rx_interrupt(UART_4, ZF_ENABLE);				   // 开启 UART_4 的接收中断
	interrupt_set_priority(LPUART1_IRQn, 1);			   // 串口中断1的优先级为0，0是最大
	interrupt_set_priority(LPUART4_IRQn, 0);			   // 串口中断4的优先级为1
	imu660ra_init();									   // 陀螺仪初始化
	//Mahony_Init();
	ips200_clear(); // 清屏
	key_init(5);	// 按键初始化
	pit_ms_init(PIT_CH0, 10);
	pit_ms_init(PIT_CH1, 1);
	pit_ms_init(PIT_CH2, 2);
	interrupt_global_enable(0); // 打开全局中断
    level = HAVE_NOTHING;
	//level = HAVE_MODEL;
	//level = HAVE_MODEL_AND_BOMBS;
	//  此处编写用户代码 例如外设初始化代码等
	while (1)
	{
		if(esp_uart1_rx_flag == 1)
		{
			ips200_show_int(0, 0, temp_byte, 5);
		}
		

		// if (error_flag)
		// {
		// 	if (path[point_idx + 1][x] < 1 && path[point_idx + 1][y] < 1)
		// 	{
		// 		wait_ms(2000);
		// 		if (wait_ok_flag)
		// 		{
		// 			error_count=0;
		// 			wait_ok_flag = 0;
		// 			error_flag = 0;
		// 			last_destination_x = 0;
		// 			last_destination_y = 0;
		// 			can_run_flag = 0;
		// 			path_flag = 0; // 重置路径标志
		// 			push_flag = 1;
		// 			no_need_to_recognize_destination_model = 0; // 重置识别目的地模型标志
		// 			keep_destination_pos_flag = 0;				// 重置保持目的地位置标志
		// 			go_to_get_map_flag = 0;						// 往前一格从而载入地图标志重置
		// 			point_idx = 0;
		// 			map_rx_flag = 0;
		// 			have_used_destination_num = 0;
		// 			only_one_destination_flag = 0;
		// 			have_recognized_nearest_box = 0;
		// 			have_found_destination = 0;
		// 			target_destination[x]=0;
		// 			target_destination[y]=0;
		// 			target_box[x]=0;
		// 			target_box[y]=0;
		// 			memset(destination_idx, -1, sizeof(destination_idx));
		// 			for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
		// 			{
		// 				destination_model[i] = -1; // 重置目的地模型
		// 				destination_used[i] = 0;   // 重置目的地使用标志
		// 			}
		// 			memset(path, 0, sizeof(path));
		// 			level = current_level++;
		// 		}
		// 	}
		// 	else
		// 	{
		// 		car_run_path();
		// 	}
		// }
		menu_flag = 1;
		// key_change_pid();
		//  if (start_game_flag == 0)
		//  {
		//  	menu_setting();
		//  }
		//  else if (menu_flag == 0 && start_game_flag == 1)
		//  {
		//  	ips200_clear();
		//  	menu_flag = 1;
		//  }
		// if (menu_flag == 1)
		// {
		// 	ips200_show_int(0, 0, destination_model[0], 2);
		// 	ips200_show_int(0, 20, destination_model[1], 2);
		// 	ips200_show_int(0, 40, destination_model[2], 2);
		// 	ips200_show_int(0, 60, destination_model[3], 2);
		// 	ips200_show_int(0, 80, destination_model[4], 2);
		// 	ips200_show_int(20, 0, b, 2);
		// 	ips200_show_int(20, 20, box_model, 2);

		// 	// ips200_show_int(30, 60, box_num, 2);
		// 	// ips200_show_int(30, 80, destination_num, 2);
		// 	// ips200_show_int(30, 100, last_box_num, 2);
		// 	// ips200_show_int(30, 120, last_destination_num, 2);
		// 	// ips200_show_int(30, 140, bomb_num, 2);
		// 	// ips200_show_int(30, 160, last_bomb_num, 2);
		// 	// ips200_show_int(0, 100, b, 2);
		// 	// ips200_show_int(0, 120, initial_destination_num, 2);
		// 	// ips200_show_int(0, 140, have_used_destination_num, 2);
		// 	ips200_show_float(0, 280, yaw, 2, 2);
		// }
		// // printf("%d,%d,%d,%d,%d\n",car_speed,encoder_data_quaddec_L1,encoder_data_quaddec_R1,encoder_data_quaddec_L2,encoder_data_quaddec_R2);
		// //  printf("%d,%d,%d\n",imu660ra_gyro_x,imu660ra_gyro_y,imu660ra_gyro_z);
		// //go_to_get_map_flag = 1;
		// if (go_to_get_map_flag == 0 && menu_flag == 1)
		// {
		// 	LPUART_WriteByte(LPUART4, 0x99); // 关闭视觉识别
		// 	if (encoder_move_distance(1.5, right) == 1)
		// 	{
		// 		go_to_get_map_flag = 1;
		// 	}
		// }

		// if (map_rx_flag == 0 && go_to_get_map_flag == 1)
		// {
		// 	LPUART_WriteByte(LPUART4, 0x66); // 打开视觉识别
			
		// }
		// //keep_destination_pos_flag = 1;
		// if (menu_flag == 1 && map_rx_flag == 1 && go_to_get_map_flag == 1 && keep_destination_pos_flag == 1)
		// // if (map_rx_flag == 1 && go_to_get_map_flag == 1 && keep_destination_pos_flag == 1)
		// {
		// 	init_game_map();
		// 	deal_current_level(); // 处理当前关卡
		// 	// show_car_to_box_path();
		// 	// a_test();
		// 	// show_box_path();
		// 	// show_bomb_map();
		// 	// show_push_box_path();
		// 	// run_bomb_path_plus();
		// 	// show_map_number();
		// 	// test_func();
		// 	// ips200_show_int(30, 0, initial_destination_pos[0][x], 2);
		// 	// ips200_show_int(60, 0, initial_destination_pos[0][y], 2);
		// 	// ips200_show_int(30, 20, initial_destination_pos[1][x], 2);
		// 	// ips200_show_int(60, 20, initial_destination_pos[1][y], 2);
		// 	// ips200_show_int(30, 40, initial_destination_pos[2][x], 2);
		// 	// ips200_show_int(60, 40, initial_destination_pos[2][y], 2);
		// 	// ips200_show_int(40, 0, mv_packet.data.box[0][x], 2);
		// 	// ips200_show_int(70, 0, mv_packet.data.box[0][y], 2);
		// 	// ips200_show_int(40, 20, mv_packet.data.box[1][x], 2);
		// 	// ips200_show_int(70, 20, mv_packet.data.box[1][y], 2);
		// 	// ips200_show_int(40, 40, mv_packet.data.box[2][x], 2);
		// 	// ips200_show_int(70, 40, mv_packet.data.box[2][y], 2);

		// 	// ips200_show_int(0, 300, bba, 3);
		// 	// ips200_show_int(40, 300, bbb, 3);
		// 	// ips200_show_int(80, 300, bbc, 3);
		// 	// ips200_show_int(120, 300, bbd,  3);
		// 	// ips200_show_int(160, 300, bbe, 3);
		// 	// ips200_show_int(200, 300, bbf, 3);

		// 	// ips200_show_int(0, 80, initial_box_pos[3][x], 2);
		// 	// ips200_show_int(20, 80, initial_box_pos[3][y], 2);
		// 	// ips200_show_int(0, 100, initial_box_pos[4][x], 2);
		// 	// ips200_show_int(20, 100, initial_box_pos[4][y], 2);
		// 	// ips200_show_int(0, 30, together_destination_and_can_push_flag, 2);
		// 	// ips200_show_int(0, 60, model_destination_num, 2);
		// 	// ips200_show_int(0, 90, level, 2);
		// 	// ips200_show_string(30, 140, "bmodel:");
		// 	// ips200_show_int(90, 140, box_model, 2);
		// 	// ips200_show_int(0, 80, initial_destination_num, 2);
		// 	// ips200_show_int(0, 100, have_used_destination_num, 2);
		// 	// ips200_show_int(0, 120, a, 2);
		// 	// ips200_show_int(30, 150, no_need_to_recognize_destination_model, 2);
		// 	// ips200_show_int(60, 150, have_recognized_nearest_box, 2);
		// 	// ips200_show_float(0, 190, model_four_point[model_idx][x], 2, 1);
		// 	// ips200_show_float(50, 190, model_four_point[model_idx][y], 2, 1);
		// 	// ips200_show_float(0, 230, bfs_end_x, 2, 1);
		// 	// ips200_show_float(50, 230, bfs_end_y, 2, 1);
		// 	ips200_show_float(0, 250, target_model[x], 2, 1);
		// 	ips200_show_float(50, 250, target_model[y], 2, 1);
		// 	ips200_show_float(0, 270, target_box[x], 2, 1);
		// 	ips200_show_float(50, 270, target_box[y], 2, 1);
		// 	// ips200_show_float(0, 290, target_destination[x], 2, 1);
		// 	// ips200_show_float(50, 290, target_destination[y], 2, 1);
		// 	ips200_show_int(0, 150, destination_used[0], 1);
		// 	ips200_show_int(0, 170, destination_used[1], 1);
		// 	ips200_show_int(0, 190, destination_used[2], 1);
		// 	ips200_show_int(0, 210, destination_used[3], 1);
		// 	ips200_show_int(0, 230, destination_used[4], 1);
		// 	// show_car_to_box_path();
		// 	// show_push_box_path();
		// 	// show_box_path();
		// 	// 打印十个拐点看看有没有问题
		// 	ips200_show_float(130, 0, path[0][x], 2, 1);
		// 	ips200_show_float(160, 0, path[0][y], 2, 1);
		// 	ips200_show_float(130, 20, path[1][x], 2, 1);
		// 	ips200_show_float(160, 20, path[1][y], 2, 1);
		// 	ips200_show_float(130, 40, path[2][x], 2, 1);
		// 	ips200_show_float(160, 40, path[2][y], 2, 1);
		// 	ips200_show_float(130, 60, path[3][x], 2, 1);
		// 	ips200_show_float(160, 60, path[3][y], 2, 1);
		// 	ips200_show_float(130, 80, path[4][x], 2, 1);
		// 	ips200_show_float(160, 80, path[4][y], 2, 1);
		// 	ips200_show_float(130, 100, path[5][x], 2, 1);
		// 	ips200_show_float(160, 100, path[5][y], 2, 1);
		// 	ips200_show_float(130, 120, path[6][x], 2, 1);
		// 	ips200_show_float(160, 120, path[6][y], 2, 1);
		// 	ips200_show_float(130, 140, path[7][x], 2, 1);
		// 	ips200_show_float(160, 140, path[7][y], 2, 1);
		// 	ips200_show_float(130, 160, path[8][x], 2, 1);
		// 	ips200_show_float(160, 160, path[8][y], 2, 1);
		// 	ips200_show_float(130, 180, path[9][x], 2, 1);
		// 	ips200_show_float(160, 180, path[9][y], 2, 1);
		// 	ips200_show_float(130, 200, path[10][x], 2, 1);
		// 	ips200_show_float(160, 200, path[10][y], 2, 1);
		// 	ips200_show_float(130, 220, path[11][x], 2, 1);
		// 	ips200_show_float(160, 220, path[11][y], 2, 1);
		// 	ips200_show_float(130, 240, path[12][x], 2, 1);
		// 	ips200_show_float(160, 240, path[12][y], 2, 1);
		// 	ips200_show_float(130, 260, path[13][x], 2, 1);
		// 	ips200_show_float(160, 260, path[13][y], 2, 1);
		// 	ips200_show_float(130, 280, path[14][x], 2, 1);
		// 	ips200_show_float(160, 280, path[14][y], 2, 1);
		// 	ips200_show_float(130, 300, path[15][x], 2, 1);
		// 	ips200_show_float(160, 300, path[15][y], 2, 1);
		// 	// ips200_show_float(160, 0, path[16][x], 2, 1);
		// 	// ips200_show_float(190, 0, path[16][y], 2, 1);
		// 	// ips200_show_float(160, 20, path[17][x], 2, 1);
		// 	// ips200_show_float(190, 20, path[17][y], 2, 1);
		// 	// ips200_show_float(160, 40, path[18][x], 2, 1);
		// 	// ips200_show_float(190, 40, path[18][y], 2, 1);
		// 	// ips200_show_int(70, 260, stuck_x, 2);
		// 	// ips200_show_int(100, 260, stuck_y, 2);
		// 	// ips200_show_int(0, 180, unlock_bomb_x, 1);
		// 	// ips200_show_int(30, 180, unlock_bomb_y, 1);
		// 	// ips200_show_int(120, 220, destination_num, 2);
		// 	// ips200_show_int(150, 220, box_num, 2);
		// 	// ips200_show_float(0, 240, mv_packet.data.car_x, 2, 1);
		// 	// ips200_show_float(30, 240, mv_packet.data.car_y, 2, 1);
		// 	// key_change_pid();
		// 	// printf("%d,%d,%d,%d,%d\n",car_speed,encoder_data_quaddec_L1,encoder_data_quaddec_R1,encoder_data_quaddec_L2,encoder_data_quaddec_R2);
		// }
	}
}