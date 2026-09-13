#ifndef _isr_h
#define _isr_h
#include "path.h"

#define DITONG_YAW 0.6

#pragma pack(1)
typedef struct
{
    uint8_t head;
    uint8_t move;
    uint8_t tail;
} Esp_Uart1_Data_t;

typedef struct
{
    uint8_t head;
    float car_x;
    float car_y;
    uint8_t tail;
} Car_To_Esp_Uart1_Data_t;
#pragma pack()

typedef union
{
    Esp_Uart1_Data_t data;
    uint8_t bytes[sizeof(Esp_Uart1_Data_t)];
} Esp_Uart1_Union_t;

typedef union
{
    Car_To_Esp_Uart1_Data_t data;
    uint8_t bytes[sizeof(Car_To_Esp_Uart1_Data_t)];
} Car_To_Esp_Uart1_Union_t;

// 数据结构体
#pragma pack(1)
typedef struct
{
    uint8_t head;              // 对应 'B',帧头,0x5A
    uint8_t model_data;        //
    uint8_t tail;              // 对应 'B',帧尾0xA5
}OpenMV_model_data_t;
#pragma pack()

typedef union
{
    OpenMV_model_data_t data;
    uint8_t bytes[sizeof(OpenMV_model_data_t)];
} OpenMV_model_Union_t;

extern float esp_pid_kp;
extern float esp_pid_ki;
extern float esp_pid_kd;
extern float last_yaw;
extern float this_yaw;
extern float angle_out;//角度环输出
extern float ax;
extern float ay;
extern float yaw;
extern volatile uint32_t wait_time;
extern int yaw_ok_flag;
extern int map_rx_flag;
extern volatile int box_model;// 存储箱子上的卡通人物对应数字
extern volatile int destination_model[BOX_DESTINATION_NUM_MAX];// 存储目的地上的数字，最多3个目的地
extern uint8_t model_cnt;// 模型数据计数器
extern uint16_t mv_cnt;             // 收到的数据个数
extern uint16_t map_cnt;            // 收到地图数据的个数
extern uint8_t keep_destination_pos_flag; // 存储是否保存过目的地坐标
extern volatile float initial_destination_pos[BOX_DESTINATION_NUM_MAX][2]; // 存储初始目的地坐标的数组
extern volatile float initial_box_pos[BOX_DESTINATION_NUM_MAX][2]; // 存储初始箱子坐标的数组

extern float ax, ay, az, gx, gy, gz; // 陀螺仪各轴四元数转换后的角度

void esp_uart1_send_car_position(void);//发送车的目的地坐标，最多5个箱子
void esp_uart1_send_car_position(void);//发送车位置


#endif


