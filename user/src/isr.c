#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"
#include <math.h>
#include <stdint.h>
/////////////////////////////////////////////////////
// yaw角相关
// float ax, ay, az, gx, gy, gz; // 陀螺仪各轴四元数转化
// float yaw_deg;                // 初始yaw角（弧度）
// float yaw;                    // 初始yaw角（角度）
// float last_yaw;               // 上一次yaw角（角度）
// float this_yaw;               // 当前yaw角（角度）
// int yaw_ok_flag = 0;          // yaw角求斜率定时去零漂标志位
/////////////////////////////////////////////////////
// 角度环相关
float angle_out; // 角度环输出
/////////////////////////////////////////////////////
// EKF相关
volatile uint32_t wait_time = 0; // 等待坐标稳定的时间
// EKF 采样缓冲（ISR 写入，主循环读取）
volatile uint8_t ekf_sample_ready = 0;
volatile float ekf_ax = 0.0f, ekf_ay = 0.0f, ekf_az = 0.0f;
volatile float ekf_gx = 0.0f, ekf_gy = 0.0f, ekf_gz = 0.0f;
/////////////////////////////////////////////////////
// 串口4通信相关
int rx_flag;         // 数据标志位
int map_rx_flag = 0; // 地图接收标志位
int send_ok_flag = 0;
extern OpenMV_Union_t mv_packet; // 数据包
extern OpenMV_Map_t map_packet;  // 地图数据包
uint16_t mv_cnt = 0;             // 收到的数据个数
uint16_t map_cnt = 0;            // 收到地图数据的个数
uint8_t keep_destination_pos_flag = 0;
volatile float initial_destination_pos[BOX_DESTINATION_NUM_MAX][2] = {0.0f}; // 存储初始目的地坐标的数组
volatile float initial_box_pos[BOX_DESTINATION_NUM_MAX][2] = {0.0f};         // 存储初始箱子坐标的数组
/////////////////////////////////////////////////////
// 串口1通信相关
/*
#define big_head_son             6//大头儿子    0x06
#define calabash_brothers        8//葫芦娃      0x08
#define donald_duck              4//唐老鸭      0x04
#define gg_bond                  7//猪猪侠      0x07
#define grey_wolf                9//灰太狼      0x09
#define mickey_mouse             0//米老鼠      0xFF
#define nezha                    5//哪吒        0x05
#define pikachu                  1//皮卡丘      0x01
#define pleasant_sheep           3//喜羊羊      0x03
#define spongebob_squarepants    2//海绵宝宝    0x02
*/
volatile int box_model = -1;                                                    // 存储箱子上的卡通人物对应数字
volatile int destination_model[BOX_DESTINATION_NUM_MAX] = {-1, -1, -1, -1, -1}; // 存储目的地上的数字，最多3个目的地
OpenMV_model_Union_t model_packet;                                              // 模型数据包
uint8_t model_cnt = 0;                                                          // 收到模型数据的个数
/////////////////////////////////////////////////////
// esp8266串口通信相关
volatile int esp_uart1_rx_flag = 0;
float esp_pid_kp = 0.0f;
float esp_pid_ki = 0.0f;
float esp_pid_kd = 0.0f;
int esp_move = 0;
uint16_t esp_uart1_cnt = 0;
Esp_Uart1_Union_t esp_uart1_packet;
Car_To_Esp_Uart1_Union_t car_to_esp_uart1_packet;
float temp_dx, temp_dy;
int timeout_cnt = 0;
extern int final_path_len;

uint8_t error_flag = 0;
uint8_t error_count = 0;
static void esp_uart1_apply_move_command(void) // 应用移动命令
{
    if (esp_move == 1)
    {
        if (encoder_move_distance(1, left) == 1)
        {
            esp_move = 0;
        }
    }
    else if (esp_move == 2)
    {
        if (encoder_move_distance(1, right) == 1)
        {
            esp_move = 0;
        }
    }
    else if (esp_move == 3)
    {
        if (encoder_move_distance(1, back) == 1)
        {
            esp_move = 0;
        }
    }
    else if (esp_move == 4)
    {
        if (encoder_move_distance(1, go) == 1)
        {
            esp_move = 0;
        }
    }
    else
    {
        esp_move = 0;
    }
}
extern int wait_ok_flag;
extern int a;
void esp_uart1_send_car_position(void) // 发送车位置
{
    uint16_t i = 0;

    car_to_esp_uart1_packet.data.head = 0xB5;
    car_to_esp_uart1_packet.data.car_x = mv_packet.data.car_x;
    car_to_esp_uart1_packet.data.car_y = mv_packet.data.car_y;
    car_to_esp_uart1_packet.data.tail = 0x5B;

    for (i = 0; i < sizeof(Car_To_Esp_Uart1_Data_t); i++)
    {
        while (!(kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(LPUART1)))
            ;
        LPUART_WriteByte(LPUART1, car_to_esp_uart1_packet.bytes[i]);
    }
}
/////////////////////////////////////////////////////////////////
void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();
    __DSB();
}
extern uint8_t menu_flag;
int current_level = 0;
uint8_t have_remember_level_flag = 0;
void PIT_IRQHandler(void)
{
    if (pit_flag_get(PIT_CH0))
    {
        Get_Encoder();
        angle_out = angle_pid(target_angle);
        get_encoder_value();
        encoder_update();
        // esp_uart1_apply_move_command();
        move(car_speed, car_way);
        pit_flag_clear(PIT_CH0);
    }

    if (pit_flag_get(PIT_CH1))
    {

        // Mahony算法得到yaw角

        // imu660ra_get_acc();
        // imu660ra_get_gyro();
        // ax = imu660ra_acc_transition(imu660ra_acc_x) - accelOffsetX;
        // ay = imu660ra_acc_transition(imu660ra_acc_y) - accelOffsetY;
        // az = imu660ra_acc_transition(imu660ra_acc_z) - accelOffsetZ;
        // gx = imu660ra_gyro_transition(imu660ra_gyro_x) * M_PI / 180.0f - gyroOffsetX;
        // gy = imu660ra_gyro_transition(imu660ra_gyro_y) * M_PI / 180.0f - gyroOffsetY;
        // gz = imu660ra_gyro_transition(imu660ra_gyro_z) * M_PI / 180.0f - gyroOffsetZ;
        // MahonyAHRSupdate(ax, ay, az, gx, gy, gz);
        // QuaternionToEuler(&yaw_deg);
        // this_yaw = yaw_deg * 180.0f / M_PI;
        // //yaw=DITONG_YAW*this_yaw+(1-DITONG_YAW)*last_yaw;
        // if(fabs(this_yaw-last_yaw)>0.001)
        // {
        //     yaw+=this_yaw-last_yaw;
        // }
        // last_yaw=this_yaw;
        imu660ra_get();
        pit_flag_clear(PIT_CH1);
    }

    if (pit_flag_get(PIT_CH2))
    {
        if (wait_flag)
        {
            wait_time++;
        }
        if (car_speed < 20&&menu_flag)
        {
            timeout_cnt++;
            if (timeout_cnt >= 2000)
            {
                timeout_cnt = 0;
                error_count++;
                if (have_remember_level_flag)
                {
                    current_level = level;
                    have_remember_level_flag = 1;
                }
                if (error_count > 1)
                {
                    can_run_flag = 1;
                    error_flag = 1;
                    start_x = mv_packet.data.car_x;
                    start_y = mv_packet.data.car_y;
                    end_x = 5;
                    end_y = 1;
                    aPoint tmp_path_tmp[APATH_NODE];
                    int tmp_path_tmp_len = 0;
                    init_game_map();
                    astar_integrate(start_x, start_y, end_x, end_y, tmp_path_tmp, &tmp_path_tmp_len);

                    for (int i = 0; i < tmp_path_tmp_len && point_idx < 30; i++)
                    {
                        path[point_idx][x] = (float)tmp_path_tmp[i].ax + 0.5f;
                        path[point_idx][y] = (float)tmp_path_tmp[i].ay + 0.5f;
                        point_idx++;
                    }
                    final_path_len = point_idx;
                    turn_final_path();
                    turn_final_path();
                    point_idx = 0;
                }
                else
                {
                    path_flag = 1;
                    manhattan_get_path_error = 0;
                    target_box[x] = 0;
                    target_box[y] = 0;
                    target_destination[x] = 0;
                    target_destination[y] = 0;
                    level = HAVE_NOTHING;
                    point_idx = 0;
                    memset(path, 0, sizeof(path));
                    bfs_path_state = BFS_INIT; // 回到初始化状态
                    path_state = PATH_INIT;    // 进入初始化状态
                }
            }
        }
        else
        {
            timeout_cnt = 0;
        }
        if (uart1_flag)
        {
            test_cnt_for_model++;
            if (test_cnt_for_model >= 1000)
            {
                if (fabs(target_model[x] - mv_packet.data.car_x) < 0.3 && (target_model[y] - mv_packet.data.car_y > 1.4) || (target_model[y] - mv_packet.data.car_y < -1.4))
                {
                    test_cnt_for_model_go_flag = 1;
                }
                else if (fabs(target_model[x] - mv_packet.data.car_x) < 0.3 && (target_model[y] - mv_packet.data.car_y < 1.2) || (target_model[y] - mv_packet.data.car_y > -1.2))
                {
                    test_cnt_for_model_back_flag = 1;
                }
                else if (fabs(target_model[y] - mv_packet.data.car_y) < 0.3 && (target_model[x] - mv_packet.data.car_x > 1.4) || (target_model[x] - mv_packet.data.car_x < -1.4))
                {
                    test_cnt_for_model_go_flag = 1;
                }
                else if (fabs(target_model[y] - mv_packet.data.car_y) < 0.3 && (target_model[x] - mv_packet.data.car_x < 1.2) || (target_model[x] - mv_packet.data.car_x > -1.2))
                {
                    test_cnt_for_model_back_flag = 1;
                }
                // 其他情况暂不考虑
            }
        }
        pit_flag_clear(PIT_CH2);
    }

    if (pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}
uint8_t temp_byte;
void LPUART1_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
#if DEBUG_UART_USE_INTERRUPT
        debug_interrupr_handler();
#endif
        if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
        {
            uint8_t byte = LPUART_ReadByte(LPUART1);
            //temp_byte = byte;
            //UART1 独立协议：0xB5 + 3个float + 2个int8 + 0x5B
            if (esp_uart1_cnt == 0)
            {
                if (byte == 0x01)
                {
                    esp_uart1_packet.bytes[esp_uart1_cnt++] = byte;
                    esp_uart1_rx_flag = 0;
                }
            }
            else
            {
                esp_uart1_packet.bytes[esp_uart1_cnt++] = byte;
                if (esp_uart1_cnt >= sizeof(Esp_Uart1_Data_t))
                {
                    if (esp_uart1_packet.data.head == 0x01 && esp_uart1_packet.data.tail == 0x02)
                    {
                        temp_byte = esp_uart1_packet.data.move;
                        esp_uart1_rx_flag = 1;
                        //esp_uart1_send_car_position();
                    }
                    esp_uart1_cnt = 0;
                }
            }
            // if (uart1_flag)
            // {

            //     if (model_cnt == 0)
            //     {
            //         if (byte == 0x5A || byte == 0x6A) // 识别到模型数据包头
            //         {
            //             model_packet.bytes[model_cnt++] = byte;
            //         }
            //     }
            //     else
            //     {
            //         model_packet.bytes[model_cnt++] = byte;
            //         if (model_cnt >= sizeof(OpenMV_model_data_t)) // 长度应为 6
            //         {
            //             if (model_packet.data.head == 0x5A && model_packet.data.tail == 0xA5)
            //             {
            //                 // 成功接收模型数据，存储箱子上的卡通人物
            //                 box_model = model_packet.data.model_data; // 存储模型数据
            //                 model_cnt = 0;                            // 复位计数器
            //                 uart1_rx_flag = 1;                        // 模型数据接收成功
            //             }
            //             else if (model_packet.data.head == 0x6A && model_packet.data.tail == 0xA6)
            //             {
            //                 // 成功接收模型数据，存储箱子上的卡通人物
            //                 destination_model[target_idx] = model_packet.data.model_data; // 存储模型数据
            //                 model_cnt = 0;                                                // 复位计数器
            //                 uart1_rx_flag = 1;                                            // 模型数据接收成功
            //             }
            //             else
            //             {
            //                 model_cnt = 0; // 数据错误，复位计数器
            //             }
            //         }
            //     }
            // }
            LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag); // ?????????
        }
    }
}

void LPUART2_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // ?????ж?
    }

    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag); // ?????????
}

void LPUART3_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // ?????ж?
    }

    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag); // ?????????
}

void LPUART4_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        uint8_t byte = LPUART_ReadByte(LPUART4);

        // 状态机判断：寻找包头
        if (mv_cnt == 0 && map_cnt == 0)
        {
            if (byte == 0xA5) // 识别到坐标包头 (OpenMV发送的第一个字节)
            {
                mv_packet.bytes[mv_cnt++] = byte;
                rx_flag = 0;
            }
            else if (byte == 0xA6) // 识别到地图包头
            {
                map_packet.map_bytes[map_cnt++] = byte;
                map_rx_flag = 0;
            }
        }
        // 路径1：坐标包接收
        else if (mv_cnt > 0)
        {
            mv_packet.bytes[mv_cnt++] = byte;
            if (mv_cnt >= sizeof(OpenMV_Data_t)) // 长度应为 130
            {
                // 校验：头是 0xA5，尾是 0x5A
                if (mv_packet.data.head == 0xA5 && mv_packet.data.tail == 0x5A)
                {
                    rx_flag = 1; // 数据接收成功
                    get_map_message();
                    if (box_num > 0 && destination_num < 6 && box_num < 6)
                    {
                        if (keep_destination_pos_flag == 0) // 如果还没有保存过目的地坐标
                        {
                            initial_box_num = box_num;
                            initial_destination_num = destination_num;
                            initial_bomb_num = bomb_num;

                            for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
                            {
                                for (int j = 0; j < 2; j++)
                                {
                                    initial_destination_pos[i][j] = mv_packet.data.destination[i][j];
                                    initial_box_pos[i][j] = mv_packet.data.box[i][j];
                                }
                            }
                            keep_destination_pos_flag = 1; // 设置标志，表示已经保存过目的地坐标了
                        }
                    }
                    if (bomb_num > 0)
                    {
                        level = HAVE_MODEL_AND_BOMBS;
                    }
                }
                mv_cnt = 0; // 复位计数器
            }
        }
        // 路径2：地图包接收
        else if (map_cnt > 0)
        {
            map_packet.map_bytes[map_cnt++] = byte;
            if (map_cnt >= sizeof(Map_Data_t)) // 长度应为 386
            {
                // 校验：头是 0xA6，尾是 0x6A
                if (map_packet.map_data.head == 0xA6 && map_packet.map_data.tail == 0x6A)
                {
                    map_rx_flag = 1; // 地图更新成功
                }
                map_cnt = 0; // 复位计数器
            }
        }
    }
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag); // ?????????
}

void LPUART5_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // ?????ж?
        camera_uart_handler();
    }

    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag); // ?????????
}

void LPUART6_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // ?????ж?
    }

    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag); // ?????????
}

void LPUART8_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // ?????ж?
        wireless_module_uart_handler();
    }

    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag); // ?????????
}

void GPIO1_Combined_0_15_IRQHandler(void)
{
    if (exti_flag_get(B0))
    {
        exti_flag_clear(B0); // ????ж???λ
    }
}

void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if (exti_flag_get(B16))
    {
        exti_flag_clear(B16); // ????ж???λ
    }
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();

    if (exti_flag_get(C0))
    {
        exti_flag_clear(C0); // ????ж???λ
    }
}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT ?????ж? ????ж???????? *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT ?????ж? ????ж???????? *-----------------

    if (exti_flag_get(C16))
    {
        exti_flag_clear(C16); // ????ж???λ
    }
}

void GPIO3_Combined_0_15_IRQHandler(void)
{

    if (exti_flag_get(D4))
    {
        exti_flag_clear(D4); // ????ж???λ
    }
}

/*
?ж?????????????????????????ж????
Sample usage:??????????????????ж?
void PIT_IRQHandler(void)
{
    //?????????λ
    __DSB();
}
??y????ж????????λ
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/
