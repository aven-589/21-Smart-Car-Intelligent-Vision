#ifndef _MODEL_H
#define _MODEL_H
#include "zf_common_headfile.h"

#define big_head_son             6//大头儿子
#define calabash_brothers        8//葫芦娃
#define donald_duck              4//唐老鸭
#define gg_bond                  7//猪猪侠
#define grey_wolf                9//灰太狼
#define mickey_mouse             0//米老鼠
#define nezha                    5//哪吒
#define pikachu                  1//皮卡丘
#define pleasant_sheep           3//喜羊羊
#define spongebob_squarepants    2//海绵宝宝

#define LEFT 0
#define UP 1
#define RIGHT 2
#define DOWN 3
typedef enum
{
   MODEL_FIND_TARGET,            // 寻找目标点
   MODEL_TO_TARGET,              // 移动到目标点 
   MODEL_VISION_CALIBRATION,  // 视觉校准，也可能需要转向
   MODEL_GO_BACK_OR_GO,              // 后退或前进一点，识别准一点
   MODEL_JUDGE,               // 开始识别，看是啥人物或者数字
   MODEL_FINISH               // 结束，开始识别下一个目标点
}Model_t;                     // 识别模型状态机

typedef enum
{
   SMART_CHOOOSE_DESTINATION,// 先确认好首和尾两个目的地，
   SMART_FIND_TARGET_BOX_AND_DESTINATION,        // 寻找目标箱子和目的地,首尾目的地哪个近选哪个
   SMART_RUN_PATH,           // 把箱子推推过去
   SMART_TEST_ONE_BY_ONE,    // 一个一个测试
   SMART_RUN_TEST_PATH,           // 把箱子推过去
   SMART_FINISH               // 结束，开始识别下一个目标点
}SmartSolution_t;             // 智能解决方案,如果三个目的地在一起并且要识别模型，那就把箱子推过去一个一个试，省时间还准，这波我给夯

extern float error_distance;
extern int test_cnt_for_model;
extern int test_cnt_for_model_go_flag;
extern int test_cnt_for_model_back_flag;
extern uint8_t together_destination_and_can_push_flag;                            // 箱子是否在一起
extern uint8_t model_back_flag;
extern int last_bfs_end[2];
extern uint8_t model_total_num;
extern uint8_t have_found_destination; // 已经找到的目的地数量
extern volatile float model_four_point[4][2];      // 存储目标点上下左右四个点的坐标，分别是上下左右
extern volatile float target_model[2];       // 目标模型的目的地坐标
extern uint8_t have_recognized_nearest_box;               // 是否已经识别了最近的箱子
extern int no_need_to_recognize_destination_model;               // 是否需要识别目的地模型
extern int target_idx; // 最近的箱子索引，假设最开始是第一个箱子
extern uint8_t  model_idx; // 目标模型索引，假设最开始是第一个模型
extern int tempt_path_len;               // 临时路径长度
extern Model_t model_state; // 识别模型状态机初始状态为寻找目标点
extern volatile uint8_t uart1_flag;         // 为1时就开始接收摄像头返回的数据
extern volatile uint8_t uart1_rx_flag;      // 为1时，说明已经接收到了摄像头返回的数据

extern uint8_t together_destination_and_can_push_flag; // 箱子是否在一起
extern float smart_target_destination[2][2];         // 俩目标点被选中的那个
extern float the_other_destination[2];            // 俩目标点中的另一个
extern uint8_t destination_same_x_flag, destination_same_y_flag;
extern uint8_t single_destination_flag;

extern uint8_t have_found_destination;
extern int destination_idx[BOX_DESTINATION_NUM_MAX];
int model_recognization();             // 识别模型主函数状态机，根据状态机的不同状态执行不同的功能
void model_get_target(void);
int judge_all_destination_is_same_x_or_y(); // 判断所有目的地是否在同一列或同一行
int smart_solution_to_combined_destination(); // 智能解决方案，合并目的地

#endif
