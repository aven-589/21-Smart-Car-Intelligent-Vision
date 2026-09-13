#ifndef _path_H
#define _path_H
#include "zf_common_headfile.h"

#define map_road 0                // 可行走的路
#define map_wall 2                // 墙
#define map_destination 4         // 目的地
#define map_bomb 6                // 炸弹
#define map_box 3                 // 箱子
#define accept_distance_error 0.3 // 接受的误差范围
#define hengshu 1
#define shuheng 2

#define HAVE_NOTHING 1
#define HAVE_MODEL   2
#define HAVE_MODEL_AND_BOMBS 3

#define BOMBS_NUM_MAX 5
#define BOX_DESTINATION_NUM_MAX 5
#define x 0
#define y 1

#define MODEL_ERROR_CAR_TO_DESTINATION 1.15
#define MODEL_ACCEPT_ERROR 0.15

// #define vision_stop_distance 0.4
#define vision_slow_distance 2//两格以内减速
#define vision_medium_distance 4//4格之间中速
#define vision_fast_distance 6//大于6格就高速

// #define vision_high_fast_speed 150
// #define vision_fast_speed 120
// #define vision_medium_speed 60
// #define vision_slow_speed 35
#define vision_slow_speed 15
#define vision_stop_speed 0

#define ENCODER 0
#define VISION  1

extern int manhattan_get_path_error;
extern float vision_calibrate_kp;
extern float vision_error_dx;
extern float vision_error_dy;
extern float vision_stop_distance;
extern int vision_high_fast_speed;
extern int vision_fast_speed;
extern int vision_medium_speed;
extern int last_bomb_num;
extern uint8_t level;
extern uint8_t path_flag;  
extern volatile int car_speed,car_way;
extern volatile int point_idx;          // 当前路径点的索引
extern uint8_t destination_num;//目的地数量
extern uint8_t box_num;//箱子数量
extern uint8_t bomb_num;//炸弹数量
extern uint8_t initial_box_num;//初始箱子数量
extern uint8_t initial_destination_num;//初始目的地数量
extern uint8_t initial_bomb_num;//初始炸弹数量
extern int last_box_num;                          // 上一次的箱子数量
extern int last_destination_num;                          // 上一次的目的地数量
extern float target_box[2];//x,y   找到最近的箱子的坐标
extern float target_destination[2];//x,y   找到最近的目的地的坐标
extern float manhattan_turn[2];//x,y   曼哈顿路径拐点坐标
extern int push_dir;                //小车要把箱子推到曼哈顿点的方向
extern float box_manhattan_back_x;         // 曼哈顿拐点后面坐标
extern float box_manhattan_back_y;         // 曼哈顿拐点后面坐标
extern float manhattan_destination_back_x ; // 曼哈顿拐点后面坐标
extern float manhattan_destination_back_y ; // 曼哈顿拐点后面坐标
extern float destination_back_x;           // 目的地后面的坐标
extern float destination_back_y;          // 目的地后面的坐标
extern float path[50][2];                    // 计算出的推箱子路径，暂定最多10个拐点
extern float car_x,car_y;                    // 存储小车当前的坐标
extern int push_flag;                        // 推箱子标志，为1就说明算完了，正在推

extern float push_box_to_destination_dx, push_box_to_destination_dy;
extern int wait_sure_flag; // 等待确认标志位，确保小车已经停止了才进入视觉校准状态

extern int wait_flag;                        // 等待标志位
extern int wait_ok_flag;                     // 等待完成标志位

extern uint8_t go_to_get_map_flag; // 往前一格从而载入地图标志位
extern uint8_t vision_calibrate_ok_flag; // 视觉校准标志位，为1说明视觉校准完成，可以进行下一步了  
extern uint8_t return_to_which_side;//返回到哪个方向
extern uint8_t can_run_flag; // 可以运行标志位，为1说明小车可以运行
extern int current_model; // 当前的模型

// 数据结构体
#pragma pack(1)
typedef struct
{
    uint8_t head;              // 对应 'B',帧头,0x5A
    float car_x;             // 对应 'f'，列
    float car_y;                // 对应 'f'，行
    float bomb[BOMBS_NUM_MAX][2];          // 对应 '10f'，使用示例 第一个炸弹的坐标，bomb[0][x]，bomb[0][y] 
    float box[BOX_DESTINATION_NUM_MAX][2];           // 对应 '10f'，使用示例 第一个箱子的坐标，box[0][x]，box[0][y]
    float destination[BOX_DESTINATION_NUM_MAX][2]; // 对应 '10f'，使用示例 第一个目的地的坐标，destination[0][x]，destination[0][y]
    uint8_t tail;              // 对应 'B',帧尾0xA5
} OpenMV_Data_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint8_t head;        // 对应 'B',帧头,0x6A
    uint16_t map[16][12]; // 对应'192B*2'，行，列
    uint8_t tail;        // 对应 'B',帧尾0xA6
} Map_Data_t;
#pragma pack()

// 联合体 (方便接收)
typedef union
{
    OpenMV_Data_t data;
    uint8_t bytes[sizeof(OpenMV_Data_t)];
} OpenMV_Union_t;

typedef union
{
    Map_Data_t map_data;
    uint8_t map_bytes[sizeof(Map_Data_t)];
} OpenMV_Map_t;

typedef enum
{
    PATH_INIT,              // 初始化
    PATH_TO_BOX_BACK,       // 计算移动到箱子后面的路径
    PATH_TO_MANHATTAN,      // 计算把箱子推到曼哈顿拐点的路径
    PATH_TO_MANHATTAN_BACK, // 计算小车到曼哈顿拐点后面的路径
    PATH_TO_DESTINATION,    // 计算小车把箱子推到目的地的路径
    PATH_FINISH,            // 路径算完了，检测小车是否已经推完该箱子，然后回到初始化状态
    PATH_READY_NEXT_LEVEL   // 回到发车区，然后进入下一关

} get_path_t;               // 计算路径的枚举状态

typedef enum
{
    MOVE_JUDGE,//判断这个拐点存在不
    MOVE_TO_POINT,//存在，就过去
    MOVE_CALIBRATE,//到达了，根据视觉校准
    MOVE_FINISH//到达目标点
} move_state_t;

typedef enum
{
    BFS_INIT,//初始化
    BFS_GET_PATH,//得到bfs路径
    BFS_TRANSFER_PATH,//将bfs路径传到path数组
    BFS_FINISH,//完成bfs路径
    BFS_READY_NEXT_LEVEL//准备下一关
} get_bfs_path_t;

typedef enum
{
    CALIBRATE_ALIGN_X,// 先对齐x坐标
    CALIBRATE_ALIGN_Y,// 再对齐y坐标
    CALIBRATE_FINISH,// 对齐完成后，到达目标点
    // CALIBRATE_MODEL_RECOGNIZATION// 补偿小车走进了目的地里面，导致识别不准或者直接识别不到我真他妈服了
    
} vision_calibrate_t;//视觉校正坐标误差

typedef enum
{
    MODEL_CAR_LIGHT_OR_RIGHT,//小车在目的地的左侧或右侧
    MODEL_CAR_ABOVE_OR_BELOW,//小车在目的地的上方或下方
    MODEL_CAR_FINISH//小车到达目的地，完成任务

} model_solve_car_destination_error_t;//模型状态

extern get_path_t path_state;
extern get_bfs_path_t bfs_path_state;
void get_map_message();//得到地图信息
void get_target_box();//得到目标箱子的坐标
void get_target_destination();//得到目标目的地的坐标
void find_nearest_start_address();//找到最近的起始地址，准备进入下一关
int deal_current_level();//处理当前关卡
int move_path();//根据path数组移动到目标点
int vision_calibrate(float dx,float dy);//根据视觉校准坐标误差
int manhattan_path(float x1, float y1, float x2, float y2);//得到曼哈顿路径
int the_same_x_or_y(float x1, float y1, float x2, float y2);//判断两个点是否在同一行或同一列,并且中间没有阻挡
int wait_ms(int time);//延时时间，ms
int get_bfs_path();//得到bfs路径
int get_manhattan_path();//得到曼哈顿路径
int car_run_path();//小车走path数组的每一个点
int run_bomb_path();//运行炸弹路径
int run_bomb_path_plus();//运行炸弹路径，但是要避免重复找
int vision_run_path(int way);
void turn_final_path();
void clear_same_path();
#endif
