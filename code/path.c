// #include <math.h>
// #include "zf_common_headfile.h"
// /*
//       小车初始车头是朝向右边
//               left
//                |
//                |
//      back<----car---->go
//                |
//                |
//              right
// */
// // int vision_high_fast_speed = 150;
// // int vision_fast_speed = 120;
// // int vision_medium_speed = 60;
// int vision_high_fast_speed = 150;
// int vision_fast_speed = 120;
// int vision_medium_speed = 60;
// extern int a, b, c;
// uint8_t return_to_which_side = LEFT;
// int bomb_finish_flag = 0;
// extern uint8_t wait_for_uart;
// OpenMV_Union_t mv_packet;
// OpenMV_Map_t map_packet;
// uint8_t destination_num = 0;              // 目的地数量
// uint8_t box_num = 0;                      // 箱子数量
// int last_box_num = 0;                     // 上一次的箱子数量
// uint8_t bomb_num = 0;                     // 炸弹数量
// float target_box[2];                      // x,y   找到最近的箱子的坐标
// float target_destination[2];              // x,y   找到最近的目的地的坐标
// float manhattan_turn[2] = {-1.0f, -1.0f}; // x,y   曼哈顿路径拐点坐标
// int wait_flag = 0;                        // 等待标志位
// int wait_ok_flag = 0;                     // 等待完成标志位
// uint8_t initial_box_num = 0;
// uint8_t initial_destination_num = 0;
// uint8_t initial_bomb_num = 0;

// void clear_same_path()
// {
//     float temp_path_point[50][2];
//     int temp_point_idx = 0;
//     for (int i = 0; i < 50; i++)
//     {
//         if (fabs(path[i + 1][x] - path[i][x]) < 0.2 && fabs(path[i + 1][y] - path[i][y]) < 0.2)
//         {
//             continue;
//         }
//         else
//         {
//             temp_path_point[temp_point_idx][x] = path[i][x];
//             temp_path_point[temp_point_idx][y] = path[i][y];
//             temp_point_idx++;
//         }
//     }
//     memset(path, 0, sizeof(path));
//     for (int i = 0; i < temp_point_idx; i++)
//     {
//         path[i][x] = temp_path_point[i][x];
//         path[i][y] = temp_path_point[i][y];
//     }
// }
// int wait_ms(int time) // 延时时间，ms
// {
//     if (wait_flag == 0 && time > 0) // 等待时间大于0并且等待标志位为0时
//     {
//         pit_disable(PIT_CH2); // 先关闭定时器，防止误触发
//         wait_time = 0;        // 重置等待时间
//         pit_enable(PIT_CH2);  // 开启定时器
//         wait_flag = 1;        // 设置等待标志位为1
//     }
//     if (wait_flag) // 等待标志位为1时
//     {
//         if (wait_time < time) // 等待时间小于指定时间时
//         {
//             return 0; // 返回0
//         }
//         else
//         {
//             wait_time = 0; // 重置等待时间
//             wait_flag = 0; // 设置等待标志位为0
//             wait_ok_flag = 1;
//         }
//     }
//     return 0;
// }

// uint8_t choose_vision_or_encoder_to_run_path = VISION;
// // 视觉定位
// // float vision_stop_distance = 0.5;
// float vision_stop_distance = 0.4;
// float vision_calibrate_kp = 60;
// float vision_error_dx = 0;
// float vision_error_dy = 0;
// int vision_run_path(int way) // 传入方向就行了
// {
//     vision_error_dx = path[point_idx + 1][x] - mv_packet.data.car_x;
//     vision_error_dy = path[point_idx + 1][y] - mv_packet.data.car_y;
//     b = 11;
//     if (way == go)
//     {
//         if (level == HAVE_NOTHING)
//         {
//             vision_high_fast_speed = 160;
//             vision_fast_speed = 120;
//             vision_medium_speed = 70;
//         }
//         else
//         {
//             vision_high_fast_speed = 160;
//             vision_fast_speed = 120;
//             vision_medium_speed = 60;
//         }

//         if (mv_packet.data.car_x >= path[point_idx + 1][x]) // 当小车超过目的地时，强制停止
//         {
//             car_speed = vision_stop_speed, car_way = way;
//             vision_error_dx = 0;
//             vision_error_dy = 0;
//             return 1;
//         }
//         if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_fast_distance) // 当距离大于等于6时，以最高速度行驶
//         {
//             car_speed = vision_high_fast_speed, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_medium_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_fast_distance) // 当距离在4到6之间，以中等速度行驶
//         {
//             float speed_go0 = (6 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_high_fast_speed - vision_fast_speed);
//             car_speed = vision_high_fast_speed - speed_go0, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_slow_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_medium_distance) // 当距离在2到4之间，以中等速度行驶
//         {
//             float speed_go1 = (4 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_fast_speed - vision_medium_speed);
//             car_speed = vision_fast_speed - speed_go1, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_slow_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) > vision_stop_distance) // 当距离在0.4到2之间，以慢速度行驶
//         {
//             float speed_go2 = (2 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_medium_speed - vision_slow_speed);
//             car_speed = vision_medium_speed - speed_go2, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_stop_distance) // 当距离小于0.4时，停止
//         {
//             car_speed = vision_stop_speed, car_way = way;
//             vision_error_dx = 0;
//             vision_error_dy = 0;
//             return 1;
//         }
//     }
//     else if (way == back)
//     {
//         if (level == HAVE_NOTHING)
//         {
//             vision_high_fast_speed = 160;
//             vision_fast_speed = 120;
//             vision_medium_speed = 70;
//         }
//         else
//         {
//             vision_high_fast_speed = 160;
//             vision_fast_speed = 120;
//             vision_medium_speed = 60;
//         }
//         if (mv_packet.data.car_x <= path[point_idx + 1][x])
//         {
//             car_speed = vision_stop_speed, car_way = way;
//             vision_error_dx = 0;
//             vision_error_dy = 0;
//             return 1;
//         }
//         if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_fast_distance)
//         {
//             car_speed = vision_high_fast_speed, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_medium_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_fast_distance)
//         {
//             float speed_back0 = (6 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_high_fast_speed - vision_fast_speed);
//             car_speed = vision_high_fast_speed - speed_back0, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_slow_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_medium_distance)
//         {
//             float speed_back1 = (4 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_fast_speed - vision_medium_speed);
//             car_speed = vision_fast_speed - speed_back1, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_slow_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) > vision_stop_distance) // 当距离在0.4到2之间，以慢速度行驶
//         {
//             float speed_back2 = (2 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_medium_speed - vision_slow_speed);
//             car_speed = vision_medium_speed - speed_back2, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_stop_distance)
//         {
//             car_speed = vision_stop_speed, car_way = way;
//             vision_error_dx = 0;
//             vision_error_dy = 0;
//             return 1;
//         }
//     }
//     else if (way == left)
//     {
//         vision_high_fast_speed = 120;
//         vision_fast_speed = 100;
//         vision_medium_speed = 60;
//         if (mv_packet.data.car_y <= path[point_idx + 1][y])
//         {
//             car_speed = vision_stop_speed, car_way = way;
//             vision_error_dx = 0;
//             vision_error_dy = 0;
//             return 1;
//         }
//         if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_fast_distance)
//         {
//             car_speed = vision_high_fast_speed, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_medium_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_fast_distance)
//         {
//             float speed_left0 = (6 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_high_fast_speed - vision_fast_speed);
//             car_speed = vision_high_fast_speed - speed_left0, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_slow_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_medium_distance)
//         {
//             float speed_left1 = (4 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_fast_speed - vision_medium_speed);
//             car_speed = vision_fast_speed - speed_left1, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_slow_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) > vision_stop_distance)
//         {
//             float speed_left2 = (2 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_medium_speed - vision_slow_speed);
//             car_speed = vision_medium_speed - speed_left2, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_stop_distance)
//         {
//             car_speed = vision_stop_speed, car_way = way;
//             vision_error_dx = 0;
//             vision_error_dy = 0;
//             return 1;
//         }
//     }
//     else if (way == right)
//     {
//         vision_high_fast_speed = 120;
//         vision_fast_speed = 100;
//         vision_medium_speed = 60;
//         if (mv_packet.data.car_y >= path[point_idx + 1][y])
//         {
//             car_speed = vision_stop_speed, car_way = way;
//             vision_error_dx = 0;
//             vision_error_dy = 0;
//             return 1;
//         }
//         if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_fast_distance)
//         {
//             car_speed = vision_high_fast_speed, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_medium_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_fast_distance)
//         {
//             float speed_right0 = (6 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_high_fast_speed - vision_fast_speed);
//             car_speed = vision_high_fast_speed - speed_right0, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_slow_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_medium_distance)
//         {
//             float speed_right1 = (4 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_fast_speed - vision_medium_speed);
//             car_speed = vision_fast_speed - speed_right1, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_slow_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) > vision_stop_distance)
//         {
//             float speed_right2 = (2 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_medium_speed - vision_slow_speed);
//             car_speed = vision_medium_speed - speed_right2, car_way = way;
//         }
//         else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_stop_distance)
//         {
//             car_speed = vision_stop_speed, car_way = way;
//             vision_error_dx = 0;
//             vision_error_dy = 0;
//             return 1;
//         }
//     }
//     return 0;
// }

// void get_map_message() // 根据接收到的坐标判断箱子、炸弹、目的地的数量
// {
//     box_num = 0;
//     destination_num = 0;
//     bomb_num = 0;

//     // 根据接收到的坐标判断箱子数量（最多3个，x和y都大于1）
//     for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
//     {
//         if (mv_packet.data.box[i][x] > 1 && mv_packet.data.box[i][y] > 1)
//         {
//             box_num++;
//         }
//     }

//     // 根据接收到的坐标判断炸弹数量（最多3个，x和y都大于1）
//     for (int i = 0; i < BOMBS_NUM_MAX; i++)
//     {
//         if (mv_packet.data.bomb[i][x] > 1 && mv_packet.data.bomb[i][y] > 1)
//         {
//             bomb_num++;
//         }
//     }

//     // 根据接收到的坐标判断目的地数量（最多3个，x和y都大于1）
//     for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
//     {
//         if (mv_packet.data.destination[i][x] > 1 && mv_packet.data.destination[i][y] > 1)
//         {
//             destination_num++;
//         }
//     }
// }

// int target_box_idx = 0;                                            // 最近的箱子索引，假设最开始是第一个
// int have_used_box[BOX_DESTINATION_NUM_MAX] = {-1, -1, -1, -1, -1}; // 已使用的箱子坐标，初始化为0.0f
// int have_used_box_num = 0;                                         // 已使用的箱子数量
// float level_one_car_x = 0.0f;
// float level_one_car_y = 0.0f;
// uint8_t level_one_path_idx = 0;              // 第一关的路径索引
// uint8_t level_one_have_found_box_num = 0;    // 第一关已经找到的箱子数量，用于判断是否需要继续找箱子
// uint8_t level_one_finish_find_path_flag = 0; // 第一关路径有没有找完的标志位
// uint8_t box_have_used_flag = 0;              // 箱子是否被使用过标志位
// float d;
// void get_target_box(void) // 构建best序列存储箱子，best先后由astar_distance()决定
// {
//     if (box_num == 0)
//         return;

//     int best_dist = 999; // 初始化一个很大的数，当作最小值，用来比较距离
//     level_one_car_x = mv_packet.data.car_x;
//     level_one_car_y = mv_packet.data.car_y;

//     for (int i = 0; i < box_num; i++)
//     {
//         float box_x = mv_packet.data.box[i][x];
//         float box_y = mv_packet.data.box[i][y];
//         if (box_x == 0 && box_y == 0)
//             continue;

//         d = astar_distance(level_one_car_x, level_one_car_y, box_x, box_y);
//         if (d < best_dist)
//         {
//             best_dist = d;      // 更新最近距离
//             target_box_idx = i; // 更新最近箱子索引
//         }
//     }

//     if (best_dist == 999)
//         return; // 没找到任何可达箱子

//     target_box[x] = mv_packet.data.box[target_box_idx][x];
//     target_box[y] = mv_packet.data.box[target_box_idx][y];
// }

// int have_used_destination[BOX_DESTINATION_NUM_MAX] = {-1, -1, -1, -1, -1}; // 已使用的目的地坐标，初始化为0.0f
// uint8_t have_used_destination_num = 0;                                     // 已使用的目的地数量
// int best_idx = 0;                                                          // 最近的目的地索引，假设最开始是第一个
// uint8_t destination_have_used_flag = 0;
// void get_target_destination(void) // 构建best序列存储目的地，best先后由astar_distance()决定
// {
//     if (destination_num == 0)
//     {
//         return;
//     }
//     int best_dist = 999; // 初始化一个很大的数，当作最小值，用来比较距离
//     float box_x = target_box[x];
//     float box_y = target_box[y];
//     for (int i = 0; i < destination_num; i++)
//     {
//         float dst_x = mv_packet.data.destination[i][x]; // 列
//         float dst_y = mv_packet.data.destination[i][y]; // 行
//         if (dst_x == 0 && dst_y == 0)
//             continue;
//         float d = astar_distance(box_x, box_y, dst_x, dst_y);
//         if (d < best_dist)
//         {
//             best_dist = d;
//             best_idx = i;
//         }
//     }
//     if (best_dist == 999)
//         return; // 没找到任何可达目的地
//     target_destination[x] = mv_packet.data.destination[best_idx][x];
//     target_destination[y] = mv_packet.data.destination[best_idx][y];
// }

// int choose_box_with_destination()
// {
//     return 0;
// }
// int the_same_x_or_y(float x1, float y1, float x2, float y2) // 判断两个点是否在同一行或同一列,并且中间没有阻挡
// {
//     int sx = (int)x1;
//     int sy = (int)y1;
//     int tx = (int)x2;
//     int ty = (int)y2;
//     if (fabsf(x1 - x2) <= 0.3 && fabsf(y1 - y2) >= 0.8) // 同一列
//     {
//         for (int j = (int)(y1 < y2 ? y1 : y2); j <= (int)(y2 > y1 ? y2 : y1); j++)
//         {
//             if (j == sy) // 判断是否为起点
//             {
//                 continue;
//             }
//             if (map_packet.map_data.map[j][(int)(x2)] != map_road && map_packet.map_data.map[j][(int)(x2)] != map_destination)
//             {
//                 return 0; // 路径被阻挡，返回失败
//             }
//         }
//         return 1; // 说明在同一列
//     }
//     else if (fabsf(x1 - x2) >= 0.8 && fabsf(y1 - y2) <= 0.3) // 同一行
//     {
//         for (int i = (int)(x1 < x2 ? x1 : x2); i <= (int)(x2 > x1 ? x2 : x1); i++)
//         {
//             if (i == sx) // 判断是否为起点
//             {
//                 continue;
//             }
//             if (map_packet.map_data.map[(int)(y1)][i] != map_road && map_packet.map_data.map[(int)(y1)][i] != map_destination)
//             {
//                 return 0; // 路径被阻挡，返回失败
//             }
//         }
//         return 1; // 说明在同一行
//     }
//     return 0; // 说明不在同一行或同一列
// }

// volatile int car_speed = 0, car_way = 0; // 小车速度和方向
// int move_path()
// {
//     float dx = path[point_idx + 1][x] - path[point_idx][x]; // x方向距离
//     float dy = path[point_idx + 1][y] - path[point_idx][y]; // y方向距离
//     if (fabsf(dx) > fabsf(dy) && fabs(dy) < 0.7)
//     {
//         // a = 1;
//         if (dx > 0)
//         {
//             if (choose_vision_or_encoder_to_run_path == ENCODER)
//             {
//                 if (encoder_move_distance(dx, go) == 1)
//                 {
//                     return 1;
//                 }
//             }
//             else if (choose_vision_or_encoder_to_run_path == VISION)
//             {
//                 // b = 4;
//                 if (vision_run_path(go) == 1)
//                 {
//                     return 1;
//                 }
//             }
//         }
//         else
//         {
//             if (choose_vision_or_encoder_to_run_path == ENCODER)
//             {
//                 if (encoder_move_distance(-dx, back) == 1)
//                 {
//                     return 1;
//                 }
//             }
//             else if (choose_vision_or_encoder_to_run_path == VISION)
//             {
//                 if (vision_run_path(back) == 1)
//                 {
//                     return 1;
//                 }
//             }
//         }
//     }
//     // else if (fabsf(dx) < 0.5 && fabsf(dy) > 0.3) // x方向距离小于0.5，y方向距离大于0.3
//     else if (fabsf(dx) < fabsf(dy) && fabs(dx) < 0.7)
//     {
//         a = 2;
//         if (dy > 0)
//         {
//             if (choose_vision_or_encoder_to_run_path == ENCODER)
//             {
//                 if (encoder_move_distance(dy, right) == 1)
//                 {
//                     return 1;
//                 }
//             }
//             else if (choose_vision_or_encoder_to_run_path == VISION)
//             {
//                 // a = 12;
//                 if (vision_run_path(right) == 1)
//                 {
//                     return 1;
//                 }
//             }
//         }
//         else
//         {
//             if (choose_vision_or_encoder_to_run_path == ENCODER)
//             {
//                 if (encoder_move_distance(-dy, left) == 1)
//                 {
//                     return 1;
//                 }
//             }
//             else if (choose_vision_or_encoder_to_run_path == VISION)
//             {
//                 if (vision_run_path(left) == 1)
//                 {
//                     return 1;
//                 }
//             }
//         }
//     }
//     else
//     {
//         car_speed = 0;
//     }
//     return 0;
// }

// vision_calibrate_t vision_calibrate_state = CALIBRATE_ALIGN_X;
// uint8_t vision_calibrate_ok_flag = 1; // 视觉校准标志位
// int vision_calibrate(float dx, float dy)
// {
//     a = 10;
//     // 根据视觉反馈进行校准，调整小车位置
//     switch (vision_calibrate_state)
//     {
//     case CALIBRATE_ALIGN_X:
//         if (dx > accept_distance_error)
//         {
//             if (encoder_move_distance(dx, go) == 1)
//             {
//                 vision_calibrate_state = CALIBRATE_ALIGN_Y;
//                 return 0;
//             }
//         }
//         else if (dx < -accept_distance_error)
//         {
//             if (encoder_move_distance(-dx, back) == 1)
//             {
//                 vision_calibrate_state = CALIBRATE_ALIGN_Y;
//                 return 0;
//             }
//         }
//         else
//         {
//             vision_calibrate_state = CALIBRATE_ALIGN_Y;
//             return 0;
//         }
//         return 0;
//     case CALIBRATE_ALIGN_Y:
//         if (dy > accept_distance_error)
//         {
//             if (encoder_move_distance(dy, right) == 1)
//             {
//                 vision_calibrate_state = CALIBRATE_FINISH;
//                 return 0;
//             }
//         }
//         else if (dy < -accept_distance_error)
//         {
//             if (encoder_move_distance(-dy, left) == 1)
//             {
//                 vision_calibrate_state = CALIBRATE_FINISH;
//                 return 0;
//             }
//         }
//         else
//         {
//             vision_calibrate_state = CALIBRATE_FINISH;
//             return 0;
//         }
//         return 0;

//     case CALIBRATE_FINISH:

//         if (fabsf(encoder_data_quaddec_L1) < 10 && fabsf(encoder_data_quaddec_R1) < 10) // 可看成停止
//         {
//             vision_calibrate_state = CALIBRATE_ALIGN_X;
//             vision_calibrate_ok_flag = 1; // 标记校准完成
//             return 1;
//         }
//         else
//         {
//             return 0;
//         }
//         return 0;
//     }
// }

// // 极简曼哈顿路径实现
// // 规则：先计算两条路径（先横再纵；先纵再横），逐格检查是否可走（只能走 map_road 或 map_destination），
// // 只要其中一条完全无遮挡就直接返回该路径（若两条都可走，优先返回先横再纵的路径）。
// // 返回值：曼哈顿拐点，失败返回0。
// int manhattan_path(float x1, float y1, float x2, float y2)
// {
//     int sx = (int)x1;
//     int sy = (int)y1;
//     int tx = (int)x2;
//     int ty = (int)y2;

//     if (sx < 1 || sx >= 11 || tx < 1 || tx >= 11 || sy < 1 || sy >= 15 || ty < 1 || ty >= 15)
//     {
//         return 0;
//     } // 坐标基本合法性判断
//     if (the_same_x_or_y(x1, y1, x2, y2)) // 小车与箱子后面的坐标在同一行或者列
//     {
//         manhattan_turn[x] = x2, manhattan_turn[y] = y2; // 把目标点直接作为曼哈顿路径拐点返回
//         return 3;
//     }
//     /////////////////////////////////////////////////////////////////////////////////////////////////////
//     int heng_shu = 1; // 先横后纵路径可行标志
//     int shu_heng = 1; // 先纵后横路径可行标志
//     if (heng_shu == 1)
//     {
//         for (int i = (int)(x1 < x2 ? x1 : x2); i <= (int)(x2 > x1 ? x2 : x1); i++)
//         {
//             if ((int)(y1) == sy && i == sx) // 判断是否为起点
//             {
//                 continue;
//             }
//             if (map_packet.map_data.map[(int)(y1)][i] != map_road && map_packet.map_data.map[(int)(y1)][i] != map_destination)
//             {
//                 heng_shu = 0;
//                 break; // 路径被阻挡，返回失败
//             }
//         }

//         for (int j = (int)(y1 < y2 ? y1 : y2); j <= (int)(y2 > y1 ? y2 : y1); j++)
//         {
//             if (j == ty && (int)(x2) == tx) // 判断是否为终点
//             {
//                 continue;
//             }
//             if (map_packet.map_data.map[j][(int)(x2)] != map_road && map_packet.map_data.map[j][(int)(x2)] != map_destination)
//             {
//                 heng_shu = 0;
//                 break; // 路径被阻挡，返回失败
//             }
//         }
//     }
//     //////////////////////////////////////////////////////
//     if (shu_heng == 1)
//     {

//         for (int j = (int)(y1 < y2 ? y1 : y2); j <= (int)(y2 > y1 ? y2 : y1); j++)
//         {
//             if (j == sy && (int)(x1) == sx) // 判断是否为起点
//             {
//                 continue;
//             }
//             if (map_packet.map_data.map[j][(int)(x1)] != map_road && map_packet.map_data.map[j][(int)(x1)] != map_destination)
//             {
//                 shu_heng = 0;
//                 break; // 路径被阻挡，返回失败
//             }
//         }
//         for (int i = (int)(x1 < x2 ? x1 : x2); i <= (int)(x2 > x1 ? x2 : x1); i++)
//         {
//             if ((int)(y2) == ty && i == tx) // 判断是否为终点
//             {
//                 continue;
//             }
//             if (map_packet.map_data.map[(int)(y2)][i] != map_road && map_packet.map_data.map[(int)(y2)][i] != map_destination)
//             {
//                 shu_heng = 0;
//                 break; // 路径被阻挡，返回失败
//             }
//         }
//     }

//     if (heng_shu == 1)
//     {
//         // 先横后纵的拐点：(x2, y1)
//         manhattan_turn[x] = x2;
//         manhattan_turn[y] = y1;
//         return hengshu;
//     }
//     else if (shu_heng == 1)
//     {
//         // 先纵后横的拐点：(x1, y2)
//         manhattan_turn[x] = x1;
//         manhattan_turn[y] = y2;
//         return shuheng;
//     }
//     return 0; // 都不可行
// }

// uint8_t vision_calibrate_judge_flag = 0; // 视觉校准判断标志位
// int vision_calibrate_judge()
// {
//     if ((path[target_idx + 1][x] < 1 && path[target_idx + 1][y] < 1) || (path[target_idx + 2][x] < 1 && path[target_idx + 2][y] < 1)) // 说明到了最后一个点，需要校准
//     {                                                                                                                                 // 最后俩点需要校准
//         return 1;
//     }
//     if ((fabs(path[target_idx + 1][x] - path[target_idx][x]) + fabs(path[target_idx + 1][y] - path[target_idx][y])) >= 5 || (fabs(path[target_idx + 1][x] - path[target_idx][x]) + fabs(path[target_idx + 1][y] - path[target_idx][y])) < 2)
//     { // 长距离或者短距离，都需要校准
//         return 1;
//     }

//     int dx = path[target_idx + 1][x] - path[target_idx][x];
//     int dy = path[target_idx + 1][y] - path[target_idx][y];
//     if (dx > 1 && dy < 0.5) // 往右
//     {
//         if (map_packet.map_data.map[(int)path[target_idx][y] + 1][(int)path[target_idx][x] + 1] != map_road || map_packet.map_data.map[(int)path[target_idx][y] - 1][(int)path[target_idx][x] + 1] != map_road)
//         {
//             return 1;
//         }
//     }
//     else if (dx < -1 && dy < 0.5) // 往左
//     {
//         if (map_packet.map_data.map[(int)path[target_idx][y] + 1][(int)path[target_idx][x] - 1] != map_road || map_packet.map_data.map[(int)path[target_idx][y] - 1][(int)path[target_idx][x] - 1] != map_road)
//         {
//             return 1;
//         }
//     }
//     else if (dx < 0.5 && dy > 1) // 往下
//     {
//         if (map_packet.map_data.map[(int)path[target_idx][y] + 1][(int)path[target_idx][x] - 1] != map_road || map_packet.map_data.map[(int)path[target_idx][y] + 1][(int)path[target_idx][x] + 1] != map_road)
//         {
//             return 1;
//         }
//     }
//     else if (dx < 0.5 && dy < -1) // 往上
//     {
//         if (map_packet.map_data.map[(int)path[target_idx][y] - 1][(int)path[target_idx][x] - 1] != map_road || map_packet.map_data.map[(int)path[target_idx][y] - 1][(int)path[target_idx][x] + 1] != map_road)
//         {
//             return 1;
//         }
//     }

//     return 0; // 都不满足，返回0，不需要校准
// }

// move_state_t move_state = MOVE_JUDGE;
// float push_box_to_destination_dx, push_box_to_destination_dy;
// int wait_sure_flag = 0; // 等待确认标志位，确保小车已经停止了才进入视觉校准状态
// int car_run_path()      // 小车走path数组的每一个点
// {
//     if (can_run_flag == 0)
//     {
//         car_speed = 0, car_way = 0;
//         return 0;
//     }
//     switch (move_state)
//     {
//     case MOVE_JUDGE:
//         b = 1;
//         if (path[point_idx][x] == 0 && path[point_idx][y] == 0) // 说明这个拐点可能不存在
//         {
//             if (point_idx == 0) // 第一个拐点是小车坐标，咋可能是0
//             {
//                 point_idx = 0; // 要么没拐点，要么推完了，重置索引
//                 return 0;
//             }
//             else if (point_idx > 0) // 后面有的拐点要是坐标是0，就直接跳过
//             {
//                 car_speed = 0, car_way = 0;
//             }
//         }
//         else // 说明这个拐点存在，直接进入推箱子状态
//         {
//             move_state = MOVE_TO_POINT;
//         }
//         return 0;

//     case MOVE_TO_POINT:
//         b = 2;
//         if (path[point_idx + 1][x] == path[point_idx][x] && path[point_idx + 1][y] == path[point_idx][y])
//         { // 该点与下一个拐点一样，直接跳过
//             move_state = MOVE_FINISH;
//             return 0;
//         }
//         if (wait_sure_flag == 0 && vision_calibrate_ok_flag == 1) // 等待小车到达了拐点，确保小车已经停止了才进入视觉校准状态
//         {

//             if (move_path() == 1)
//             {
//                 wait_sure_flag = 1; // 小车已经到达了拐点
//             }
//         }

//         if (wait_sure_flag)
//         {
//             wait_sure_flag = 0; // 小车已经到达了拐点
//             move_state = MOVE_FINISH;
//         }

//         return 0;

//     case MOVE_CALIBRATE:

//         push_box_to_destination_dx = path[point_idx + 1][x] - car_x; // 拐点与车的x误差
//         push_box_to_destination_dy = path[point_idx + 1][y] - car_y; // 拐点与车的y误差
//         vision_calibrate(push_box_to_destination_dx, push_box_to_destination_dy);
//         if (vision_calibrate_ok_flag == 1) // 标记校准完成
//         {
//             move_state = MOVE_FINISH;
//         }
//         return 0;
//     case MOVE_FINISH:
//         b = 3;

//         if (fabsf(encoder_data_quaddec_L1) < 15 && fabsf(encoder_data_quaddec_R1) < 15) // 可看成停止
//         {
//             // if((mv_packet.data.car_x - path[point_idx + 1][x]) < accept_distance_error && (mv_packet.data.car_y - path[point_idx + 1][y]) < accept_distance_error)
//             // {
//             move_state = MOVE_JUDGE;
//             point_idx++; // 下一个拐点
//             return 1;    // 说明到达目标点了
//             // }
//             // else
//             // {
//             //     car_x = mv_packet.data.car_x;
//             //     car_y = mv_packet.data.car_y;
//             //     move_state = MOVE_CALIBRATE;
//             //     car_speed = 0, car_way = 0;
//             //     return 0;
//             // }
//         }
//     }
//     return 0;
// }

// uint8_t level = HAVE_NOTHING;                 // 当前关卡,1:只有墙 2：有模型 3：有炸弹
// uint8_t path_flag = 0;                        // 可以进行计算推箱子路线标志位
// uint8_t go_to_get_map_flag = 0;               // 往前一格从而载入地图
// uint8_t try_if_same_destination_x_y_flag = 0; // 尝试看看目的地是否同一行或者同一列
// int push_flag = 0;                            // 推箱子标志，为1就说明算完了，正在推
// int manhattan_get_path_error = 0;             // 曼哈顿路径错误标志
// float box_manhattan_back_x = 0;               // 曼哈顿拐点后面坐标
// float box_manhattan_back_y = 0;               // 曼哈顿拐点后面坐标
// float manhattan_destination_back_x = 0;       // 曼哈顿拐点后面坐标
// float manhattan_destination_back_y = 0;       // 曼哈顿拐点后面坐标
// float destination_back_x = 0;                 // 目的地后面的坐标
// float destination_back_y = 0;                 // 目的地后面的坐标
// float path[50][2];                            // 计算出的推箱子路径，暂定最多50个拐点
// int final_path_len = 0;                       // 最终路径长度
// volatile int point_idx = 0;                   // 路径当前点的索引
// float car_x, car_y;                           // 存放小车当前位置
// get_path_t path_state = PATH_INIT;            // 设置初始状态为初始化状态
// int push_dir = 0;                             // 小车要把箱子推到目标点的方向
// float man_x, man_y;                           // 存放曼哈顿路径拐点坐标
// uint8_t found_match = 0;                      // 是否找到匹配的目的地
// int nearest_index = -1;                       // 最近的目的地索引
// extern int b;
// int last_destination_num = 0;

// void turn_final_path()
// {
//     if (final_path_len < 2)
//         return;
//     float tmp_path[50][2];
//     int tmp_len = 0;
//     float last_dx = path[1][x] - path[0][x];
//     float last_dy = path[1][y] - path[0][y];
//     tmp_path[tmp_len][x] = path[0][x];
//     tmp_path[tmp_len][y] = path[0][y];
//     tmp_len++;

//     for (int i = 1; i < final_path_len - 1; i++)
//     {
//         if (fabs(path[i][x] - path[i - 1][x]) < 0.3f &&
//             fabs(path[i][y] - path[i - 1][y]) < 0.3f)
//         {
//             continue;
//         }

//         float dx = path[i + 1][x] - path[i][x];
//         float dy = path[i + 1][y] - path[i][y];

//         // 归一到方向符号（正→1, 负→-1, 零→0），比较符号而非数值大小
//         int sx = (fabs(dx) > 0.3f) ? (dx > 0 ? 1 : -1) : 0;
//         int sy = (fabs(dy) > 0.3f) ? (dy > 0 ? 1 : -1) : 0;
//         int lx = (fabs(last_dx) > 0.3f) ? (last_dx > 0 ? 1 : -1) : 0;
//         int ly = (fabs(last_dy) > 0.3f) ? (last_dy > 0 ? 1 : -1) : 0;

//         if (sx != lx || sy != ly)
//         {
//             tmp_path[tmp_len][x] = path[i][x];
//             tmp_path[tmp_len][y] = path[i][y];
//             tmp_len++;
//             last_dx = dx;
//             last_dy = dy;
//         }
//     }

//     if (tmp_len == 0 ||
//         fabs(tmp_path[tmp_len - 1][x] - path[final_path_len - 1][x]) > 0.2f ||
//         fabs(tmp_path[tmp_len - 1][y] - path[final_path_len - 1][y]) > 0.2f)
//     {
//         tmp_path[tmp_len][x] = path[final_path_len - 1][x];
//         tmp_path[tmp_len][y] = path[final_path_len - 1][y];
//         tmp_len++;
//     }

//     for (int i = 0; i < tmp_len; i++)
//     {
//         path[i][x] = tmp_path[i][x];
//         path[i][y] = tmp_path[i][y];
//     }

//     for (int i = tmp_len; i < final_path_len; i++)
//     {
//         path[i][x] = 0.0f;
//         path[i][y] = 0.0f;
//     }

//     final_path_len = tmp_len;
// }

// int deal_current_level() // 处理当前关卡
// {
//     if (destination_num == 0 && box_num == 0) // 不能推，因为啥都没有
//     {
//         path_flag = 0;
//     }

//     if (level == HAVE_NOTHING) // 只有墙
//     {
//         path_flag = 1;
//         get_map_message();
//         get_manhattan_path(); //   先看能不能直接找到曼哈顿路径
//         get_bfs_path();       //   曼哈顿路径要是不行就上bfs，这俩函数完全不冲突
//     }
//     else if (level == HAVE_MODEL) // 有模型
//     {
//         if (have_recognized_nearest_box == 0)
//         {
//             path_flag = 0;
//             model_recognization(); // 识别模型
//         }

//         else if (no_need_to_recognize_destination_model == 1 && have_recognized_nearest_box == 1) // 识别目的地模型和最近箱子模型已经完成
//         {
//             path_flag = 1;
//             get_manhattan_path(); //   先看能不能直接找到曼哈顿路径
//             get_bfs_path();       //   曼哈顿路径要是不行就上bfs，这俩函数完全不冲突
//         }

//         else
//         {
//             path_flag = 0;
//             return 0;
//         }
//     }

//     else if (level == HAVE_MODEL_AND_BOMBS) // 有模型和炸弹
//     {
//         run_bomb_path_plus();
//         if (bomb_finish_flag == 1)
//         {
//             level = HAVE_MODEL;
//         }
//         path_flag = 1;
//     }

//     return 0;
// }

// void find_nearest_start_address() // 找到最近的起始地址，准备进入下一关
// {
//     point_idx = 0;
//     path[0][x] = mv_packet.data.car_x;
//     path[0][y] = mv_packet.data.car_y;
//     path[1][x] = 6;
//     path[1][y] = mv_packet.data.car_y;
//     path[2][x] = 6;
//     path[2][y] = 1.5;
// }

// uint8_t destination_used[BOX_DESTINATION_NUM_MAX] = {false, false, false, false, false};
// uint8_t temp_num = 0;
// uint8_t only_one_destination_flag = 0;
// uint8_t can_run_flag = 0; // 是否可以运行路径
// float last_destination_x = 0;
// float last_destination_y = 0;
// int current_model;
// int encounter_bug_flag; // 这个东西吧，反正遇到bug了就会启用这个开始计时
// int encounter_bug_cnt;  // 这个东西吧，反正遇到bug了就会启用这个开始计时
// uint8_t get_better_path_flag = 0;
// int get_manhattan_path() // 计算路径的函数，参数是当前状态
// {

//     if (path_flag == 0) // 如果是第二关，且识别目的地模型未完成
//     {
//         return 0; // 说明模型未识别完成，不能计算路径
//     }
//     float dx, dy; // 用来判断移动方向，本质是x或者y的误差
//     switch (path_state)
//     {
//         // a++;
//     case PATH_INIT: // “初始化”，得到目标箱子以及目标目的地

//         if (!manhattan_get_path_error && map_rx_flag == 1 && mv_packet.data.car_x != 0 && mv_packet.data.car_y != 0) // 收到地图才开始初始化
//         {
//             //init_game_map();
//             get_better_path_flag = 0;
//             b = 10;
//             manhattan_get_path_error = 0; // 曼哈顿路径错误标志重置
//             point_idx = 0;                // 索引重置
//             push_flag = 0;
//             if (level == HAVE_NOTHING)
//             {
//                 a = 2;
//                 get_target_box();
//                 get_target_destination();
//             } // 推箱子标志重置

//             else if (no_need_to_recognize_destination_model == 1 && have_recognized_nearest_box == 1) // 如果需要识模型
//             {

//                 if (only_one_destination_flag == 1) // 说明只有一个目的地，直接赋值
//                 {
//                     path_state = PATH_TO_BOX_BACK;
//                 }
//                 else // 说明有多个目的地，需要找到最近的目的地并且符合对应关系
//                 {
//                     float min_distance = 999.9f;
//                     current_model = box_model;
//                     int nearest_index = -1; // 建议初始化
//                     int found_match = 0;
//                     for (int i = 0; i < initial_destination_num; i++)
//                     {
//                         if (destination_used[i])
//                             continue; // 跳过已被分配的目的地

//                         if (destination_model[i] == current_model)
//                         {
//                             float distance = astar_distance(target_box[x], target_box[y], initial_destination_pos[i][x], initial_destination_pos[i][y]);
//                             if (distance < min_distance)
//                             {
//                                 min_distance = distance;
//                                 nearest_index = i;
//                             }
//                             found_match = 1;
//                         }
//                     }

//                     if (found_match == 1 && nearest_index != -1) // 找到了对应关系的目的地并且是最近的那个
//                     {
//                         b = 11;
//                         target_destination[x] = initial_destination_pos[nearest_index][x];
//                         target_destination[y] = initial_destination_pos[nearest_index][y];
//                         destination_used[nearest_index] = true; // 新增：标记已使用
//                         have_used_destination_num++;            // 新增：已使用已使用目的地数量
//                         nearest_index = -1;
//                     }
//                     else // 如果没有找到匹配的目的地，可能是这个目的地就是没有被识别的那个
//                     {

//                         for (int i = 0; i < initial_destination_num; i++)
//                         {
//                             if (destination_model[i] != -1 || destination_used[i])
//                             {
//                                 continue; // 跳过已经识别过的目的地和用过的目的地
//                             }
//                             else // 找到未被识别的目的地，然后标记这个目的地
//                             {
//                                 target_destination[x] = initial_destination_pos[i][x];
//                                 target_destination[y] = initial_destination_pos[i][y];
//                                 destination_used[i] = true;  // 标记已使用
//                                 have_used_destination_num++; // 已使用已使用目的地数量
//                                 break;                       // 找到第一个未使用的目的地后退出循环
//                             }
//                         }

//                         // 新增：已使用已使用目的地数量
//                     }

//                     if (fabs(target_destination[x] - last_destination_x) < 0.2 && fabs(target_destination[y] - last_destination_y) < 0.2)
//                     {
//                         if (destination_num == 1)
//                         {
//                             target_destination[x] = mv_packet.data.destination[0][x];
//                             target_destination[y] = mv_packet.data.destination[0][y];
//                             have_used_destination_num++;
//                         }
//                         else
//                         {
//                             for (int i = 0; i < initial_destination_num; i++)
//                             {
//                                 if (destination_used[i])
//                                 {
//                                     continue;
//                                 }
//                                 else
//                                 {
//                                     target_destination[x] = initial_destination_pos[i][x];
//                                     target_destination[y] = initial_destination_pos[i][y];
//                                     destination_used[i] = true;
//                                     have_used_destination_num++;
//                                     break;
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }
//             else
//             {
//                 break;
//             }

//             car_x = mv_packet.data.car_x; // 获取小车初始坐标
//             car_y = mv_packet.data.car_y; // 获取小车初始坐标
//             memset(path, 0, sizeof(path));
//             path[0][x] = car_x;     // 路径的第一个点是小车当前位置
//             path[0][y] = car_y;     // 路径的第一个点是小车当前位置
//             last_box_num = box_num; // 更新上一次箱子数量
//             last_destination_num = destination_num;
//             if (target_box[x] != 0 && target_box[y] != 0 && target_destination[x] != 0 && target_destination[y] != 0)
//             {
//                 path_state = PATH_TO_BOX_BACK; // 进入到“移动到箱子后面”状态
//                 return 0;
//             }
//             // manhattan_get_path_error=1;
//         }
//         else
//         {
//             return 0;
//         }
//         /////////////////////////////////////////////////////////////////////////////////////////////////////
//     case PATH_TO_BOX_BACK: // 计算移动到箱子后面的路径

//         if (manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 1 || manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 2 || manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 3)
//         // 大条件，曼哈顿拐点需要存在，如果返回值是3，也就是把目标点作为曼哈顿拐点返回
//         {
//             a = 3;
//             dx = manhattan_turn[x] - target_box[x]; // 曼哈顿拐点与箱子x坐标的差值，用来判断小车移动方向
//             dy = manhattan_turn[y] - target_box[y]; // 曼哈顿拐点与箱子y坐标的差值，用来判断小车移动方向
//             if (fabs(dx) < 0.5 && fabs(dy) > 0.8)   // 说明箱子和第一个曼哈顿拐点x坐标几乎一样(同一列)，要到上面/下面
//             {
//                 if (dy > 0)
//                 {
//                     push_dir = right; // 说明等会要往下推箱子
//                     path[2][x] = target_box[x];
//                     path[2][y] = target_box[y] - 1; // 到箱子上面一格
//                 }
//                 else
//                 {
//                     push_dir = left; // 说明等会要往上推箱子
//                     path[2][x] = target_box[x];
//                     path[2][y] = target_box[y] + 1; // 到箱子下面一格
//                 }
//             }
//             else if (fabs(dy) < 0.3 && fabs(dx) > 0.5) // 说明箱子和第一个曼哈顿拐点y坐标几乎一样(同一行)，要到左面/右面
//             {
//                 if (dx > 0)
//                 {
//                     push_dir = go;                  // 说明等会要往前推箱子
//                     path[2][x] = target_box[x] - 1; // 到箱子左边
//                     path[2][y] = target_box[y];
//                 }
//                 else
//                 {
//                     push_dir = back;                // 说明等会要往后推箱子
//                     path[2][x] = target_box[x] + 1; // 到箱子右边
//                     path[2][y] = target_box[y];
//                 }
//             }
//             else
//             {
//                 return 0; // 大条件都不满足，直接退出
//             }

//             if ((manhattan_path(car_x, car_y, path[2][x], path[2][y]) == 1 || manhattan_path(car_x, car_y, path[2][x], path[2][y]) == 2 || manhattan_path(car_x, car_y, path[2][x], path[2][y]) == 3) && map_packet.map_data.map[(int)path[2][y]][(int)path[2][x]] == 0)
//             {
//                 path[1][x] = manhattan_turn[x]; // 上面的函数计算出曼哈顿拐点坐标
//                 path[1][y] = manhattan_turn[y]; // 第一个拐点坐标，小车先到这个点
//             }
//             else
//             {
//                 path_state = PATH_INIT;       // 进入初始化状态
//                 manhattan_get_path_error = 1; // 曼哈顿路径错误标志
//                 return 0;
//             }

//             path_state = PATH_TO_MANHATTAN; // 进入“把箱子推到曼哈顿拐点状态”
//         }
//         else
//         {
//             manhattan_get_path_error = 1; // 曼哈顿路径错误标志
//             return 0;
//         }
//         /////////////////////////////////////////////////////////////////////////////////////////////////////
//     case PATH_TO_MANHATTAN: // 计算把箱子推到曼哈顿拐点的路径

//         if (manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 1 || manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 2 || manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 3)
//         // 大条件，曼哈顿拐点存在，如果返回值是3，也就是把目标点作为曼哈顿拐点返回
//         {
//             man_x = manhattan_turn[x];
//             man_y = manhattan_turn[y];
//             // 曼哈顿拐点更新，计算从箱子到目的地的曼哈顿拐点
//             if (push_dir == right) // 说明等会要往右推箱子
//             {
//                 box_manhattan_back_x = manhattan_turn[x]; // 小车把箱子推到曼哈顿拐点后，小车理论的坐标
//                 box_manhattan_back_y = manhattan_turn[y] - 1;
//             }
//             else if (push_dir == left) // 说明等会要往左推箱子
//             {
//                 box_manhattan_back_x = manhattan_turn[x]; // 小车把箱子推到曼哈顿拐点后，小车理论的坐标
//                 box_manhattan_back_y = manhattan_turn[y] + 1;
//             }
//             else if (push_dir == go) // 说明等会要往前推箱子
//             {
//                 box_manhattan_back_x = manhattan_turn[x] - 1; // 小车把箱子推到曼哈顿拐点后，小车理论的坐标
//                 box_manhattan_back_y = manhattan_turn[y];
//             }
//             else if (push_dir == back) // 说明等会要往后推箱子
//             {
//                 box_manhattan_back_x = manhattan_turn[x] + 1; // 小车把箱子推到曼哈顿拐点后，小车理论的坐标
//                 box_manhattan_back_y = manhattan_turn[y];
//             }
//             path[3][x] = box_manhattan_back_x;
//             path[3][y] = box_manhattan_back_y; // 把箱子推到曼哈顿点后，小车的位置
//             if (manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 3)
//             {
//                 path_state = PATH_FINISH; // 直接进入目的地状态
//                 return 0;
//             }
//             path_state = PATH_TO_MANHATTAN_BACK; // 进入“计算小车到曼哈顿拐点后面的路径”
//         }
//         else
//         {
//             return 0;
//         }
//         ////////////////////////////////////////////////////////////////////////////////////////////////////
//     case PATH_TO_MANHATTAN_BACK: // 计算小车到曼哈顿拐点后面的路径

//         dx = target_destination[x] - man_x;
//         dy = target_destination[y] - man_y;   // 目的地与曼哈顿拐点的误差
//         if (fabs(dx) < 0.5 && fabs(dy) > 0.8) // 说明目的地和曼哈顿拐点的x坐标几乎一样(同一列)，要到上面/下面
//         {
//             if (dy > 0)
//             {
//                 push_dir = right; // 说明等会要往下推箱子
//                 manhattan_destination_back_x = man_x;
//                 manhattan_destination_back_y = man_y - 1; // 到箱子上面的坐标
//                 path[4][x] = box_manhattan_back_x;        // 小车此时要移动到箱子后面，需要先经过这个拐点
//                 path[4][y] = box_manhattan_back_y - 1;
//             }
//             else
//             {
//                 push_dir = left; // 说明等会要往上推箱子
//                 manhattan_destination_back_x = manhattan_turn[x];
//                 manhattan_destination_back_y = manhattan_turn[y] + 1; // 到箱子下面的坐标
//                 path[4][x] = box_manhattan_back_x;                    // 小车此时要移动到箱子后面，需要先经过这个拐点
//                 path[4][y] = box_manhattan_back_y + 1;
//             }
//         }
//         else if (fabs(dy) < 0.3 && fabs(dx) > 0.8) // 说明目的地和曼哈顿拐点的y坐标几乎一样(同一行)，要到左面/右面
//         {
//             if (dx > 0)
//             {
//                 push_dir = go; // 说明等会要往前推箱子
//                 manhattan_destination_back_x = manhattan_turn[x] - 1;
//                 manhattan_destination_back_y = manhattan_turn[y]; // 到箱子左面的坐标
//                 path[4][x] = box_manhattan_back_x - 1;            // 小车此时要移动到箱子后面，需要先经过这个拐点
//                 path[4][y] = box_manhattan_back_y;
//             }
//             else
//             {
//                 push_dir = back; // 说明等会要往后推箱子
//                 manhattan_destination_back_x = manhattan_turn[x] + 1;
//                 manhattan_destination_back_y = manhattan_turn[y]; // 到箱子右面的坐标
//                 path[4][x] = box_manhattan_back_x + 1;            // 小车此时要移动到箱子后面，需要先经过这个拐点
//                 path[4][y] = box_manhattan_back_y;
//             }
//         }
//         if (map_packet.map_data.map[(int)(path[4][y])][(int)(path[4][x])] != map_road && map_packet.map_data.map[(int)(path[4][y])][(int)(path[4][x])] != map_destination)
//         {
//             manhattan_get_path_error = 1; // 曼哈顿路径错误标志
//             path_state = PATH_INIT;       // 进入初始化状态
//             return 0;
//         }
//         path[5][x] = manhattan_destination_back_x;
//         path[5][y] = manhattan_destination_back_y; // 算出小车要到达的箱子后面的坐标
//         if (map_packet.map_data.map[(int)(path[5][y])][(int)(path[5][x])] != map_road && map_packet.map_data.map[(int)(path[5][y])][(int)(path[5][x])] != map_destination && map_packet.map_data.map[(int)(path[4][y])][(int)(path[4][x])] != map_road && map_packet.map_data.map[(int)(path[4][y])][(int)(path[4][x])] != map_destination)
//         {
//             manhattan_get_path_error = 1; // 曼哈顿路径错误标志
//             path_state = PATH_INIT;       // 进入初始化状态
//             return 0;
//         }
//         path_state = PATH_TO_DESTINATION; // 进入“计算小车把箱子推到目的地的路径”
//         return 0;
//         /////////////////////////////////////////////////////////////////////////////////////////////////////
//     case PATH_TO_DESTINATION: // 计算小车把箱子推到目的地的路径

//         if (push_dir == right) // 说明等会要往右推箱子
//         {
//             destination_back_x = target_destination[x]; // 小车把箱子推到目的地后，小车的理论坐标
//             destination_back_y = target_destination[y] - 1;
//         }
//         else if (push_dir == left) // 说明等会要往左推箱子
//         {
//             destination_back_x = target_destination[x]; // 小车把箱子推到目的地后，小车的理论坐标
//             destination_back_y = target_destination[y] + 1;
//         }
//         else if (push_dir == go) // 说明等会要往前推箱子
//         {
//             destination_back_x = target_destination[x] - 1; // 小车把箱子推到目的地后，小车的理论坐标
//             destination_back_y = target_destination[y];
//         }
//         else if (push_dir == back) // 说明等会要往后推箱子
//         {
//             destination_back_x = target_destination[x] + 1; // 小车把箱子推到目的地后，小车的理论坐标
//             destination_back_y = target_destination[y];
//         }
//         path[6][x] = destination_back_x;
//         path[6][y] = destination_back_y; // 计算出小车把箱子推到目的地后，小车的理论坐标
//         if (map_packet.map_data.map[(int)(path[6][y])][(int)(path[6][x])] != map_road && map_packet.map_data.map[(int)(path[6][y])][(int)(path[6][x])] != map_destination)
//         {
//             manhattan_get_path_error = 1; // 曼哈顿路径错误标志
//             path_state = PATH_INIT;       // 进入初始化状态
//             return 0;
//         }
//         int temp_len = 0;
//         for (int i = 0; i < 20; i++)
//         {
//             if (path[i][x] > 1 && path[i][y] > 1)
//             {
//                 temp_len++;
//             }
//         }

//         final_path_len = temp_len;
//         turn_final_path();
//         turn_final_path();
//         path_state = PATH_FINISH; // 进入“路径算完了，检测小车是否已经推完该箱子，然后回到初始化状态”
//         return 0;
//         /////////////////////////////////////////////////////////////////////////////////////////////////////
//     case PATH_FINISH:
//         if (get_better_path_flag == 0)
//         {
//             int temp_lens = 0;
//             for (int i = 0; i < 20; i++)
//             {
//                 if (path[i][x] > 1 && path[i][y] > 1)
//                 {
//                     temp_lens++;
//                 }
//             }
//             final_path_len = temp_lens;
//             turn_final_path();
//             //clear_same_path();
//             get_better_path_flag = 1;
//         }
//         can_run_flag = 1;
//         if ((int)path[point_idx + 1][x] == 0 && (int)path[point_idx + 1][y] == 0 && point_idx != 0 && vision_calibrate_ok_flag == 1 && destination_num < last_destination_num && box_num < last_box_num) // 说明箱子已经被推走了，且视觉校准完成了，可以进行下一个箱子的路径计算了
//         {
//             can_run_flag = 0;
//             b = 4;
//             car_speed = 0;
//             if (fabsf(encoder_data_quaddec_L1) < 10 && fabsf(encoder_data_quaddec_R1) < 10)
//             {
//                 memset(path, 0, sizeof(path)); // 给数组清0
//                 // a++;
//                 // wait_ms(100);
//                 wait_ok_flag = 1;
//                 if (wait_ok_flag)
//                 {
//                     last_destination_x = target_destination[x];
//                     last_destination_y = target_destination[y];
//                     if ((initial_destination_num - have_used_destination_num) == 1) // 说明只有一个目的地，直接赋值
//                     {
//                         only_one_destination_flag = 1;
//                         for (int i = 0; i < initial_destination_num; i++)
//                         {
//                             if (destination_used[i])
//                             {
//                                 continue; // 跳过已被分配的目的地
//                             }
//                             else
//                             {
//                                 target_destination[x] = initial_destination_pos[i][x];
//                                 target_destination[y] = initial_destination_pos[i][y];
//                                 if (fabs(target_destination[x] - mv_packet.data.destination[0][x]) > 0.8 || fabs(target_destination[y] - mv_packet.data.destination[0][y]) > 0.8)
//                                 {
//                                     target_destination[x] = mv_packet.data.destination[0][x];
//                                     target_destination[y] = mv_packet.data.destination[0][y];
//                                 }
//                                 destination_used[i] = true; // 标记为已使用
//                                 have_used_destination_num++;
//                                 break; // 找到第一个未使用的目的地后退出循环
//                             }
//                         }
//                     }
//                     last_destination_num = destination_num;
//                     last_box_num = box_num;
//                     box_model = -1; // 重置当前箱子模型
//                     wait_ok_flag = 0;
//                     if (box_num == 0 || last_destination_num == 0 || last_box_num == 0) // 所有箱子都被推到目的地
//                     {
//                         have_used_destination_num = 0;
//                         c++;
//                         point_idx = 0;                      // 索引重置
//                         memset(path, 0, sizeof(path));      // 给数组清0
//                         find_nearest_start_address();       // 找到最近的起始地址，准备进入下一关
//                         path_state = PATH_READY_NEXT_LEVEL; // 回到发车区，然后进入下一关
//                         return 0;
//                     }
//                     else
//                     {
//                         path_state = PATH_INIT;          // 回到初始化状态
//                         model_state = MODEL_FIND_TARGET; // 重置识别模型状态机
//                         have_recognized_nearest_box = 0; // 重置最近箱子识别标志
//                         path_flag = 0;                   // 每次推完一个就把这个置0，等下一次选好了箱子和目的地，就会置1
//                         push_flag = 1;
//                         point_idx = 0;
//                         return 1;
//                     }
//                 }
//             }
//             else
//             {
//                 break;
//             }
//         }
//         else
//         {
//             car_run_path(); // 小车走path数组的每一个点
//         }
//         break;
//     case PATH_READY_NEXT_LEVEL:
//         a = 3;
//         can_run_flag = 1;
//         if ((int)path[point_idx + 1][x] == 0 && (int)path[point_idx + 1][y] == 0 && point_idx != 0 && vision_calibrate_ok_flag == 1) // 说明回到发车区了，且视觉校准完成了，可以进入下一关了
//         {
//             last_destination_x = 0;
//             last_destination_y = 0;
//             can_run_flag = 0;
//             level++;       // 关卡增加1
//             path_flag = 0; // 重置路径标志
//             push_flag = 1;
//             no_need_to_recognize_destination_model = 0; // 重置识别目的地模型标志
//             path_state = PATH_INIT;                     // 回到初始化状态，准备下一关
//             keep_destination_pos_flag = 0;              // 重置保持目的地位置标志
//             go_to_get_map_flag = 0;                     // 往前一格从而载入地图标志重置
//             point_idx = 0;
//             map_rx_flag = 0;
//             have_used_destination_num = 0;
//             only_one_destination_flag = 0;
//             have_recognized_nearest_box = 0;
//             have_found_destination = 0;
//             memset(destination_idx, -1, sizeof(destination_idx));
//             for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
//             {
//                 destination_model[i] = -1; // 重置目的地模型
//                 destination_used[i] = 0;   // 重置目的地使用标志
//             }
//             memset(path, 0, sizeof(path));
//             return 1;
//         }
//         else
//         {
//             car_run_path(); // 小车走path数组的每一个点
//         }
//     }
//     return 0;
// }

// // Point bfs_car_to_box_path[PATH_NODE];//小车到箱子旁边的数组
// // Point push_path[PATH_NODE];//推箱子的数组
// extern aPoint abox_path[APATH_NODE]; // 箱子路径数组，包含箱子路径的起点和终点
// get_bfs_path_t bfs_path_state = BFS_INIT;
// int bfs_flag = 0; // bfs路径标志
// int get_bfs_path()
// {
//     if (path_flag == 0)
//     {
//         return 0;
//     }
//     if (!manhattan_get_path_error) // 曼哈顿路径错误标志
//     {
//         return 0;
//     }
//     switch (bfs_path_state)
//     {
//     case BFS_INIT:
//         point_idx = 0;
//         push_flag = 0;                                                                  // 初始化路径索引
//         if (map_rx_flag == 1 && mv_packet.data.car_x != 0 && mv_packet.data.car_y != 0) // 曼哈顿路径条件不满足，无法计算，改用bfs
//         {
//             bfs_path_state = BFS_GET_PATH;
//         }
//         return 0;
//     case BFS_GET_PATH:
//         //init_game_map();
//         memset(path, 0, sizeof(path));
//         if (amake_box_path() && bfs_flag == 0) // 得到箱子路径
//         {
//             if (amake_car_to_box_path(abox_path[0].ax, abox_path[0].ay, abox_path[1].ax, abox_path[1].ay)) // 得到小车到箱子的路径
//             {

//                 bfs_flag = 1;
//             }
//         }
//         if (bfs_flag == 1)
//         {
//             amake_push_box_path();
//             get_astar_turn_path(apush_path, &apush_path_len);
//             bfs_flag = 0;
//             bfs_path_state = BFS_TRANSFER_PATH;
//         }
//         return 0;
//     case BFS_TRANSFER_PATH:
//         for (int i = 0; i < acar_to_box_path_len && point_idx < 30; i++)
//         {
//             path[point_idx][x] = (float)acar_to_box_path[i].ax + 0.5f;
//             path[point_idx][y] = (float)acar_to_box_path[i].ay + 0.5f;
//             point_idx++;
//         }
//         for (int k = 0; k < 30 && point_idx < 30; k++)
//         {
//             if (apush_path[k].ax != 0 && apush_path[k].ay != 0)
//             {
//                 apush_path_len++;
//             }
//         }
//         // 再复制推箱子到目的地的路径
//         for (int j = 0; j < apush_path_len && point_idx < 50; j++)
//         {
//             path[point_idx][x] = (float)(apush_path[j].ax + 0.5f);
//             path[point_idx][y] = (float)(apush_path[j].ay + 0.5f);
//             point_idx++;
//         }
//         final_path_len = point_idx;
//         turn_final_path();
//         turn_final_path();
//         point_idx = 0;
//         bfs_path_state = BFS_FINISH;
//         return 0;
//     case BFS_FINISH:
//         // a++;
//         can_run_flag = 1;
//         if ((int)path[point_idx + 1][x] == 0 && (int)path[point_idx + 1][y] == 0 && point_idx != 0 && box_num < last_box_num && destination_num < last_destination_num) // 说明箱子已经被推走了，且视觉校准完成了，可以进行下一个箱子的路径计算了
//         {
//             b = 5;
//             can_run_flag = 0;
//             car_speed = 0;
//             if (fabsf(encoder_data_quaddec_L1) < 10 && fabsf(encoder_data_quaddec_R1) < 10)
//             {

//                 // wait_ms(100);
//                 wait_ok_flag = 1;
//                 if (wait_ok_flag)
//                 {
//                     last_destination_x = target_destination[x];
//                     last_destination_y = target_destination[y];
//                     if ((initial_destination_num - have_used_destination_num) == 1) // 说明只有一个目的地，直接赋值
//                     {
//                         only_one_destination_flag = 1;
//                         for (int i = 0; i < initial_destination_num; i++)
//                         {
//                             if (destination_used[i])
//                             {
//                                 continue; // 跳过已被分配的目的地
//                             }
//                             else
//                             {
//                                 target_destination[x] = initial_destination_pos[i][x];
//                                 target_destination[y] = initial_destination_pos[i][y];
//                                 if (fabs(target_destination[x] - mv_packet.data.destination[0][x]) > 0.8 || fabs(target_destination[y] - mv_packet.data.destination[0][y]) > 0.8)
//                                 {
//                                     target_destination[x] = mv_packet.data.destination[0][x];
//                                     target_destination[y] = mv_packet.data.destination[0][y];
//                                 }
//                                 destination_used[i] = true; // 标记为已使用
//                                 have_used_destination_num++;
//                                 break; // 找到第一个未使用的目的地后退出循环
//                             }
//                         }
//                     }
//                     last_destination_num = destination_num;
//                     last_box_num = box_num;
//                     box_model = -1; // 重置当前箱子模型
//                     wait_ok_flag = 0;
//                     memset(acar_to_box_path, 0, sizeof(acar_to_box_path));
//                     memset(apush_path, 0, sizeof(apush_path));
//                     memset(path, 0, sizeof(path));
//                     if (box_num == 0  || last_destination_num == 0 || last_box_num == 0)
//                     {
//                         have_used_destination_num = 0;
//                         c++;
//                         point_idx = 0; // 索引重置
//                         memset(path, 0, sizeof(path));
//                         find_nearest_start_address();          // 找到最近的起始地址，准备进入下一关
//                         bfs_path_state = BFS_READY_NEXT_LEVEL; // 回到发车区，然后进入下一关
//                         return 0;
//                     }
//                     else
//                     {
//                         point_idx = 0;
//                         bfs_path_state = BFS_INIT;       // 回到初始化状态
//                         model_state = MODEL_FIND_TARGET; // 重置识别模型状态机
//                         path_state = PATH_INIT;          // 进入初始化状态
//                         manhattan_get_path_error = 0;
//                         have_recognized_nearest_box = 0; // 重置最近箱子识别标志
//                         path_flag = 0;
//                         push_flag = 1;
//                         memset(path, 0, sizeof(path));
//                         return 1;
//                     }
//                 }
//             }
//         }
//         else
//         {
//             car_run_path(); // 小车走path数组的每一个点
//         }
//         break;

//     case BFS_READY_NEXT_LEVEL:
//         b=19;
//         can_run_flag = 1;
//         if ((int)path[point_idx + 1][x] == 0 && (int)path[point_idx + 1][y] == 0 && point_idx != 0 && vision_calibrate_ok_flag == 1) // 说明回到发车区了，且视觉校准完成了，可以进入下一关了
//         {
//             last_destination_x = 0;
//             last_destination_y = 0;
//             can_run_flag = 0;
//             manhattan_get_path_error = 0;
//             level++;                                    // 关卡增加1
//             path_flag = 0;                              // 重置路径标志
//             no_need_to_recognize_destination_model = 0; // 重置识别目的地模型标志
//             bfs_path_state = BFS_INIT;                  // 回到初始化状态，准备下一关
//             path_state = PATH_INIT;
//             keep_destination_pos_flag = 0; // 重置保持目的地位置标志
//             go_to_get_map_flag = 0;        // 往前一格从而载入地图标志重置
//             point_idx = 0;
//             map_rx_flag = 0;
//             have_used_destination_num = 0;
//             only_one_destination_flag = 0;
//             have_recognized_nearest_box = 0;
//             have_found_destination = 0;
//             memset(destination_idx, -1, sizeof(destination_idx));
//             for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
//             {
//                 destination_model[i] = -1; // 重置目的地模型
//                 destination_used[i] = 0;   // 重置目的地使用标志
//             }
//             memset(acar_to_box_path, 0, sizeof(acar_to_box_path));
//             memset(apush_path, 0, sizeof(apush_path));
//             memset(path, 0, sizeof(path));
//             return 1;
//         }
//         else
//         {
//             car_run_path(); // 小车走path数组的每一个点
//         }
//     }
//     return 0;
// }

// typedef enum
// {
//     BOMB_PATH_INIT, // 初始化
//     BOMB_GET_STUCK_PATH,
//     BOMB_GET_FREE_PATH, // 调用 make_free_bomb_path 规划炸弹+箱子路径
//     BOMB_TRANSFER_PATH, // 将炸弹路径数据传到 path 数组
//     BOMB_FINISH,        // 执行路径，检测是否推完了
// } bomb_run_state_t;

// bomb_run_state_t bomb_run_state = BOMB_PATH_INIT;
// uint8_t bomb_path_flag = 0;

// extern aPoint car_to_bomb_path[APATH_NODE];
// extern int car_to_bomb_path_len;
// extern aPoint push_bomb_path[APATH_NODE];
// extern int push_bomb_len;
// extern aPoint bomb_path[APATH_NODE];
// extern int bomb_path_len;

// extern int16_t pair_list_len;

// extern int8_t nearlist_bomb_x;
// extern int8_t nearlist_bomb_y;
// int last_bomb_num = 0;
// uint8_t state_a = 0;
// int check_stuck_ret = -1;
// uint8_t lock_flag = 0;
// uint8_t free_flag = 0;
// uint8_t ra = 0;
// uint8_t rb = 0;
// uint8_t lock = 0;
// extern int stuck_x, stuck_y;

// extern int8_t unlock_bomb_x;
// extern int8_t unlock_bomb_y;
// extern int8_t ul_debug, ul_wx, ul_wy, ul_bx, ul_by;

// uint8_t nearlist_flag_a = 0;
// uint8_t nearlist_box_index_flag = 0;

// uint8_t game_a = 0;
// static uint8_t game_init_flag = 0;
// int run_bomb_path_plus(void)
// {
//     static uint8_t debug_path_type = 0;
//     static uint8_t debug_bomb_x = 0;
//     static uint8_t debug_bomb_y = 0;
//     static uint8_t debug_explode_x = 0;
//     static uint8_t debug_explode_y = 0;

//     static int8_t executed_bomb_x = -1;
//     static int8_t executed_bomb_y = -1;

//     static uint8_t local_map_ready = 0;

//     switch (bomb_run_state)
//     {

//     case BOMB_PATH_INIT:
//     {
//         if (local_map_ready == 0)
//         {

//             if (map_rx_flag != 1)
//                 return 0;

//             if (mv_packet.data.car_x == 0 || mv_packet.data.car_y == 0)
//                 return 0;

//             static int init_bomb_num = 0;

//             if (init_bomb_num == 0)
//             {
//                 last_bomb_num = bomb_num;
//                 init_bomb_num = 1;
//             }

//             if (game_init_flag == 0)
//             {

//                 //init_game_map();
//                 game_a++;
//                 memcpy(bomb_map, game_map, sizeof(bomb_map));
//                 if (game_a > 3)
//                 {
//                     game_init_flag = 1;
//                 }
//             }
//             state_a = 1;
//             point_idx = 0;
//             bomb_path_flag = 0;
//             // 有了新视觉地图，允许后续走本地连炸模式
//             if (game_init_flag == 1)
//             {
//                 local_map_ready = 1;

//             }
//             return 0;
//         }

//         check_stuck_ret = check_bomb_lock(&stuck_x, &stuck_y);

//         ips200_show_int(0, 0, check_stuck_ret, 1);
//         if (check_stuck_ret == 1 && lock == 0)
//         {
//             int out_wall_x = -1, out_wall_y = -1;
//             ra++;
//             find_unlock_wall(stuck_x, stuck_y, &out_wall_x, &out_wall_y);

//             bomb_pair();
//             if (ra >= 3)
//             {
//                 lock = 1;
//             }
//         }

//         else if (check_stuck_ret != 1 && lock == 0)
//         {
//             rb++;
//             bomb_pair();
//             if (rb >= 3)
//             {
//                 lock = 1;
//             }
//         }
//     }

//         if (nearlist_box_index_flag == 0)
//         {
//             // find_unlock_wall → make_bomb_path → sokoban_search 会污染 game_map，
//             // 必须在此处重新同步，否则 nearlist_bomb_get 会找到错误的炸弹
//             memcpy(game_map, bomb_map, sizeof(game_map));
//             nearlist_bomb_get();
//             nearlist_box_index_flag = 1;

//             // ips200_show_int(0,180,nearlist_bomb_x,2);
//             // ips200_show_int(30,180,nearlist_bomb_x,2);
//         }

//         if (mv_packet.data.car_x != 0 &&
//             mv_packet.data.car_y != 0)
//         {
//             if (nearlist_bomb_x == unlock_bomb_x &&
//                 nearlist_bomb_y == unlock_bomb_y)
//             {
//                 // 解锁炸弹

//                 bomb_run_state = BOMB_GET_STUCK_PATH;
//             }

//             else if (nearlist_bomb_x == stuck_x &&
//                      nearlist_bomb_y == stuck_y)
//             {

//                 // 死锁炸弹
//                 if (check_stuck_ret == 1)
//                 {
//                     bomb_run_state = BOMB_GET_STUCK_PATH;
//                 }

//                 else
//                 {
//                     bomb_run_state = BOMB_GET_FREE_PATH;
//                 }
//             }

//             else
//             {
//                 // 普通炸弹
//                 bomb_run_state = BOMB_GET_FREE_PATH;
//             }
//         }

//         else
//         {
//             return 0;
//         }

//         return 0;

//     case BOMB_GET_STUCK_PATH:
//     {
//         state_a = 2;

//         if (lock_flag == 0 && make_unlock_path())
//         {
//             lock_flag = 1;
//             bomb_path_flag = 1;

//             // 保存调试信息：STUCK路径 炸弹坐标 + 炸点坐标
//             debug_path_type = 1;
//             debug_bomb_x = nearlist_bomb_x;
//             debug_bomb_y = nearlist_bomb_y;

//             executed_bomb_x = nearlist_bomb_x;
//             executed_bomb_y = nearlist_bomb_y;
//             if (bomb_path_len > 0)
//             {
//                 debug_explode_x = bomb_path[bomb_path_len - 1].ax;
//                 debug_explode_y = bomb_path[bomb_path_len - 1].ay;
//             }

//             // 路径规划完成后立即显示，保证是最新值
//             // ips200_show_int(0, 300, debug_path_type, 1);
//             // ips200_show_int(20, 300, debug_bomb_x, 2);
//             // ips200_show_int(45, 300, debug_bomb_y, 2);
//             // ips200_show_int(70, 300, debug_explode_x, 2);
//             // ips200_show_int(95, 300, debug_explode_y, 2);

//             bomb_run_state = BOMB_TRANSFER_PATH;
//         }

//         else
//         {
//             bomb_run_state = BOMB_PATH_INIT;
//         }

//         return 0;
//     }

//     case BOMB_GET_FREE_PATH:
//         state_a = 3;
//         int ok = make_free_bomb_path();

//         // ips200_show_string(0, 260, "FREE");
//         // ips200_show_int(0,280,ok,3);
//         // ips200_show_int(60, 280, bomb_path_len, 3);
//         // ips200_show_float(0,230,nearlist_bomb_x,3,2);
//         // ips200_show_float(30,230,nearlist_bomb_y,3,2);

//         if (free_flag == 0 && ok)
//         {
//             free_flag = 1;
//             bomb_path_flag = 2;

//             // 保存调试信息：FREE路径 炸弹坐标 + 炸点坐标
//             debug_path_type = 2;
//             debug_bomb_x = nearlist_bomb_x;
//             debug_bomb_y = nearlist_bomb_y;

//             executed_bomb_x = nearlist_bomb_x;
//             executed_bomb_y = nearlist_bomb_y;
//             if (bomb_path_len > 0)
//             {
//                 debug_explode_x = bomb_path[bomb_path_len - 1].ax;
//                 debug_explode_y = bomb_path[bomb_path_len - 1].ay;
//             }

//             // 路径规划完成后立即显示，保证是最新值
//             // ips200_show_int(0, 300, debug_path_type, 1);
//             // ips200_show_int(20, 300, debug_bomb_x, 2);
//             // ips200_show_int(45, 300, debug_bomb_y, 2);
//             // ips200_show_int(70, 300, debug_explode_x, 2);
//             // ips200_show_int(95, 300, debug_explode_y, 2);

//             bomb_run_state = BOMB_TRANSFER_PATH;
//         }

//         else
//         {
//             bomb_run_state = BOMB_PATH_INIT;
//         }

//         return 0;

//     case BOMB_TRANSFER_PATH:
//         state_a = 4;
//         {
//             memset(path, 0, sizeof(path));

//             point_idx = 0;
//             // 1. 复制 car_to_bomb_path（小车 → 炸弹后面）
//             for (int i = 0; i < car_to_bomb_path_len && point_idx < 30; i++)
//             {
//                 path[point_idx][x] = (float)(car_to_bomb_path[i].ax + 0.5f);
//                 path[point_idx][y] = (float)(car_to_bomb_path[i].ay + 0.5f);
//                 point_idx++;
//             }

//             // 2. 复制 push_bomb_path（推炸弹到墙）
//             for (int i = 0; i < push_bomb_len && point_idx < 50; i++)
//             {
//                 if (push_bomb_path[i].ax == 0 && push_bomb_path[i].ay == 0)
//                     continue;
//                 path[point_idx][x] = (float)(push_bomb_path[i].ax + 0.5f);
//                 path[point_idx][y] = (float)(push_bomb_path[i].ay + 0.5f);
//                 point_idx++;
//             }

//             final_path_len = point_idx; // 最终路径长度
//             turn_final_path();
//             turn_final_path();
//             point_idx = 0;
//             bomb_run_state = BOMB_FINISH;
//             return 0;
//         }

//     case BOMB_FINISH:
//     {
//         state_a = 5;
//         car_run_path();
//         can_run_flag = 1;
//         // ips200_show_int(0, 300, debug_path_type, 1);
//         // ips200_show_int(20, 300, debug_bomb_x, 2);
//         // ips200_show_int(45, 300, debug_bomb_y, 2);
//         // ips200_show_int(70, 300, debug_explode_x, 2);
//         // ips200_show_int(95, 300, debug_explode_y, 2);
//         if (bomb_num < last_bomb_num)
//         {
//             wait_ms(800);
//             if (wait_ok_flag)
//             {
//                 wait_ok_flag = 0;
//                 if (bomb_num == 0)
//                 {
//                     bomb_finish_flag = 1;
//                     return 0;
//                 }
//                 static int commit_backup[3][3];

//                 car_speed = 0;
//                 can_run_flag = 0;
//                 // 1. 把本轮执行的炸弹从“规划地图”中删掉
//                 if (executed_bomb_x >= 0 && executed_bomb_x < COLS &&
//                     executed_bomb_y >= 0 && executed_bomb_y < ROWS)
//                 {
//                     bomb_map[executed_bomb_y][executed_bomb_x] = 0;
//                 }

//                 // 2. 在炸点对规划地图施加真实爆炸效果（九宫格炸墙）
//                 bomb_apply(debug_explode_x,
//                            debug_explode_y,
//                            commit_backup,
//                            bomb_map);

//                 last_bomb_num = bomb_num;

//                 bomb_path_len = 0;
//                 car_to_bomb_path_len = 0;
//                 push_bomb_len = 0;

//                 memset(path, 0, sizeof(path));
//                 memset(bomb_path, 0, sizeof(bomb_path));
//                 memset(car_to_bomb_path, 0, sizeof(car_to_bomb_path));
//                 memset(push_bomb_path, 0, sizeof(push_bomb_path));

//                 lock_flag = 0;
//                 free_flag = 0;
//                 nearlist_box_index_flag = 0;

//                 executed_bomb_x = -1;
//                 executed_bomb_y = -1;

//                 local_map_ready = 1;
//                 bomb_run_state = BOMB_PATH_INIT;
//             }
//         }
//     }
//     }
//     return 0;
// }
#include <math.h>
#include "zf_common_headfile.h"
/*
      小车初始车头是朝向右边
              left
               |
               |
     back<----car---->go
               |
               |
             right
*/
// int vision_high_fast_speed = 150;
// int vision_fast_speed = 120;
// int vision_medium_speed = 60;
int vision_high_fast_speed = 150;
int vision_fast_speed = 120;
int vision_medium_speed = 60;
extern int a, b, c;
uint8_t return_to_which_side = LEFT;
int bomb_finish_flag = 0;
extern uint8_t wait_for_uart;
OpenMV_Union_t mv_packet;
OpenMV_Map_t map_packet;
uint8_t destination_num = 0;              // 目的地数量
uint8_t box_num = 0;                      // 箱子数量
int last_box_num = 0;                     // 上一次的箱子数量
uint8_t bomb_num = 0;                     // 炸弹数量
float target_box[2];                      // x,y   找到最近的箱子的坐标
float target_destination[2];              // x,y   找到最近的目的地的坐标
float manhattan_turn[2] = {-1.0f, -1.0f}; // x,y   曼哈顿路径拐点坐标
int wait_flag = 0;                        // 等待标志位
int wait_ok_flag = 0;                     // 等待完成标志位
uint8_t initial_box_num = 0;
uint8_t initial_destination_num = 0;
uint8_t initial_bomb_num = 0;

void clear_same_path()
{
    float temp_path_point[50][2];
    int temp_point_idx = 0;
    for (int i = 0; i < 50; i++)
    {
        if (fabs(path[i + 1][x] - path[i][x]) < 0.2 && fabs(path[i + 1][y] - path[i][y]) < 0.2)
        {
            continue;
        }
        else
        {
            temp_path_point[temp_point_idx][x] = path[i][x];
            temp_path_point[temp_point_idx][y] = path[i][y];
            temp_point_idx++;
        }
    }
    memset(path, 0, sizeof(path));
    for (int i = 0; i < temp_point_idx; i++)
    {
        path[i][x] = temp_path_point[i][x];
        path[i][y] = temp_path_point[i][y];
    }
}
int wait_ms(int time) // 延时时间，ms
{
    if (wait_flag == 0 && time > 0) // 等待时间大于0并且等待标志位为0时
    {
        pit_disable(PIT_CH2); // 先关闭定时器，防止误触发
        wait_time = 0;        // 重置等待时间
        pit_enable(PIT_CH2);  // 开启定时器
        wait_flag = 1;        // 设置等待标志位为1
    }
    if (wait_flag) // 等待标志位为1时
    {
        if (wait_time < time) // 等待时间小于指定时间时
        {
            return 0; // 返回0
        }
        else
        {
            wait_time = 0; // 重置等待时间
            wait_flag = 0; // 设置等待标志位为0
            wait_ok_flag = 1;
        }
    }
    return 0;
}

uint8_t choose_vision_or_encoder_to_run_path = VISION;
// 视觉定位
// float vision_stop_distance = 0.5;
float vision_stop_distance = 0.4;
float vision_calibrate_kp = 60;
float vision_error_dx = 0;
float vision_error_dy = 0;
int vision_run_path(int way) // 传入方向就行了
{
    vision_error_dx = path[point_idx + 1][x] - mv_packet.data.car_x;
    vision_error_dy = path[point_idx + 1][y] - mv_packet.data.car_y;
    b = 11;
    if (way == go)
    {
        if (level == HAVE_NOTHING)
        {
            vision_high_fast_speed = 130;
            vision_fast_speed = 100;
            vision_medium_speed = 70;
        }
        else
        {
            vision_high_fast_speed = 130;
            vision_fast_speed = 100;
            vision_medium_speed = 70;
        }

        if (mv_packet.data.car_x >= path[point_idx + 1][x]) // 当小车超过目的地时，强制停止
        {
            car_speed = vision_stop_speed, car_way = way;
            vision_error_dx = 0;
            vision_error_dy = 0;
            return 1;
        }
        if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_fast_distance) // 当距离大于等于6时，以最高速度行驶
        {
            car_speed = vision_high_fast_speed, car_way = way;
        }
        else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_medium_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_fast_distance) // 当距离在4到6之间，以中等速度行驶
        {
            float speed_go0 = (6 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_high_fast_speed - vision_fast_speed);
            car_speed = vision_high_fast_speed - speed_go0, car_way = way;
        }
        else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_slow_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_medium_distance) // 当距离在2到4之间，以中等速度行驶
        {
            float speed_go1 = (4 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_fast_speed - vision_medium_speed);
            car_speed = vision_fast_speed - speed_go1, car_way = way;
        }
        else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_slow_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) > vision_stop_distance) // 当距离在0.4到2之间，以慢速度行驶
        {
            float speed_go2 = (2 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_medium_speed - vision_slow_speed);
            car_speed = vision_medium_speed - speed_go2, car_way = way;
        }
        else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_stop_distance) // 当距离小于0.4时，停止
        {
            car_speed = vision_stop_speed, car_way = way;
            vision_error_dx = 0;
            vision_error_dy = 0;
            return 1;
        }
    }
    else if (way == back)
    {
        if (level == HAVE_NOTHING)
        {
            vision_high_fast_speed = 130;
            vision_fast_speed = 100;
            vision_medium_speed = 70;
        }
        else
        {
            vision_high_fast_speed = 130;
            vision_fast_speed = 100;
            vision_medium_speed = 70;
        }
        if (mv_packet.data.car_x <= path[point_idx + 1][x])
        {
            car_speed = vision_stop_speed, car_way = way;
            vision_error_dx = 0;
            vision_error_dy = 0;
            return 1;
        }
        if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_fast_distance)
        {
            car_speed = vision_high_fast_speed, car_way = way;
        }
        else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_medium_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_fast_distance)
        {
            float speed_back0 = (6 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_high_fast_speed - vision_fast_speed);
            car_speed = vision_high_fast_speed - speed_back0, car_way = way;
        }
        else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) >= vision_slow_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_medium_distance)
        {
            float speed_back1 = (4 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_fast_speed - vision_medium_speed);
            car_speed = vision_fast_speed - speed_back1, car_way = way;
        }
        else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_slow_distance && fabs(mv_packet.data.car_x - path[point_idx + 1][x]) > vision_stop_distance) // 当距离在0.4到2之间，以慢速度行驶
        {
            float speed_back2 = (2 - fabs(mv_packet.data.car_x - path[point_idx + 1][x])) / vision_slow_distance * (vision_medium_speed - vision_slow_speed);
            car_speed = vision_medium_speed - speed_back2, car_way = way;
        }
        else if (fabs(mv_packet.data.car_x - path[point_idx + 1][x]) < vision_stop_distance)
        {
            car_speed = vision_stop_speed, car_way = way;
            vision_error_dx = 0;
            vision_error_dy = 0;
            return 1;
        }
    }
    else if (way == left)
    {
        vision_high_fast_speed = 90;
        vision_fast_speed = 80;
        vision_medium_speed = 60;
        if (mv_packet.data.car_y <= path[point_idx + 1][y])
        {
            car_speed = vision_stop_speed, car_way = way;
            vision_error_dx = 0;
            vision_error_dy = 0;
            return 1;
        }
        if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_fast_distance)
        {
            car_speed = vision_high_fast_speed, car_way = way;
        }
        else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_medium_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_fast_distance)
        {
            float speed_left0 = (6 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_high_fast_speed - vision_fast_speed);
            car_speed = vision_high_fast_speed - speed_left0, car_way = way;
        }
        else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_slow_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_medium_distance)
        {
            float speed_left1 = (4 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_fast_speed - vision_medium_speed);
            car_speed = vision_fast_speed - speed_left1, car_way = way;
        }
        else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_slow_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) > vision_stop_distance)
        {
            float speed_left2 = (2 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_medium_speed - vision_slow_speed);
            car_speed = vision_medium_speed - speed_left2, car_way = way;
        }
        else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_stop_distance)
        {
            car_speed = vision_stop_speed, car_way = way;
            vision_error_dx = 0;
            vision_error_dy = 0;
            return 1;
        }
    }
    else if (way == right)
    {
        vision_high_fast_speed = 90;
        vision_fast_speed = 80;
        vision_medium_speed = 60;
        if (mv_packet.data.car_y >= path[point_idx + 1][y])
        {
            car_speed = vision_stop_speed, car_way = way;
            vision_error_dx = 0;
            vision_error_dy = 0;
            return 1;
        }
        if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_fast_distance)
        {
            car_speed = vision_high_fast_speed, car_way = way;
        }
        else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_medium_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_fast_distance)
        {
            float speed_right0 = (6 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_high_fast_speed - vision_fast_speed);
            car_speed = vision_high_fast_speed - speed_right0, car_way = way;
        }
        else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) >= vision_slow_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_medium_distance)
        {
            float speed_right1 = (4 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_fast_speed - vision_medium_speed);
            car_speed = vision_fast_speed - speed_right1, car_way = way;
        }
        else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_slow_distance && fabs(mv_packet.data.car_y - path[point_idx + 1][y]) > vision_stop_distance)
        {
            float speed_right2 = (2 - fabs(mv_packet.data.car_y - path[point_idx + 1][y])) / vision_slow_distance * (vision_medium_speed - vision_slow_speed);
            car_speed = vision_medium_speed - speed_right2, car_way = way;
        }
        else if (fabs(mv_packet.data.car_y - path[point_idx + 1][y]) < vision_stop_distance)
        {
            car_speed = vision_stop_speed, car_way = way;
            vision_error_dx = 0;
            vision_error_dy = 0;
            return 1;
        }
    }
    return 0;
}

void get_map_message() // 根据接收到的坐标判断箱子、炸弹、目的地的数量
{
    box_num = 0;
    destination_num = 0;
    bomb_num = 0;

    // 根据接收到的坐标判断箱子数量（最多3个，x和y都大于1）
    for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
    {
        if (mv_packet.data.box[i][x] > 1 && mv_packet.data.box[i][y] > 1)
        {
            box_num++;
        }
    }

    // 根据接收到的坐标判断炸弹数量（最多3个，x和y都大于1）
    for (int i = 0; i < BOMBS_NUM_MAX; i++)
    {
        if (mv_packet.data.bomb[i][x] > 1 && mv_packet.data.bomb[i][y] > 1)
        {
            bomb_num++;
        }
    }

    // 根据接收到的坐标判断目的地数量（最多3个，x和y都大于1）
    for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
    {
        if (mv_packet.data.destination[i][x] > 1 && mv_packet.data.destination[i][y] > 1)
        {
            destination_num++;
        }
    }
}

int target_box_idx = 0;                                            // 最近的箱子索引，假设最开始是第一个
int have_used_box[BOX_DESTINATION_NUM_MAX] = {-1, -1, -1, -1, -1}; // 已使用的箱子坐标，初始化为0.0f
int have_used_box_num = 0;                                         // 已使用的箱子数量
float level_one_car_x = 0.0f;
float level_one_car_y = 0.0f;
uint8_t level_one_path_idx = 0;              // 第一关的路径索引
uint8_t level_one_have_found_box_num = 0;    // 第一关已经找到的箱子数量，用于判断是否需要继续找箱子
uint8_t level_one_finish_find_path_flag = 0; // 第一关路径有没有找完的标志位
uint8_t box_have_used_flag = 0;              // 箱子是否被使用过标志位
float d;
void get_target_box(void) // 构建best序列存储箱子，best先后由astar_distance()决定
{
    if (box_num == 0)
        return;

    int best_dist = 999; // 初始化一个很大的数，当作最小值，用来比较距离
    level_one_car_x = mv_packet.data.car_x;
    level_one_car_y = mv_packet.data.car_y;

    for (int i = 0; i < box_num; i++)
    {
        float box_x = mv_packet.data.box[i][x];
        float box_y = mv_packet.data.box[i][y];
        if (box_x == 0 && box_y == 0)
            continue;

        d = astar_distance(level_one_car_x, level_one_car_y, box_x, box_y);
        if (d < best_dist)
        {
            best_dist = d;      // 更新最近距离
            target_box_idx = i; // 更新最近箱子索引
        }
    }

    if (best_dist == 999)
        return; // 没找到任何可达箱子

    target_box[x] = mv_packet.data.box[target_box_idx][x];
    target_box[y] = mv_packet.data.box[target_box_idx][y];
}

int have_used_destination[BOX_DESTINATION_NUM_MAX] = {-1, -1, -1, -1, -1}; // 已使用的目的地坐标，初始化为0.0f
uint8_t have_used_destination_num = 0;                                     // 已使用的目的地数量
int best_idx = 0;                                                          // 最近的目的地索引，假设最开始是第一个
uint8_t destination_have_used_flag = 0;
void get_target_destination(void) // 构建best序列存储目的地，best先后由astar_distance()决定
{
    if (destination_num == 0)
    {
        return;
    }
    int best_dist = 999; // 初始化一个很大的数，当作最小值，用来比较距离
    float box_x = target_box[x];
    float box_y = target_box[y];
    for (int i = 0; i < destination_num; i++)
    {
        float dst_x = mv_packet.data.destination[i][x]; // 列
        float dst_y = mv_packet.data.destination[i][y]; // 行
        if (dst_x == 0 && dst_y == 0)
            continue;
        float d = astar_distance(box_x, box_y, dst_x, dst_y);
        if (d < best_dist)
        {
            best_dist = d;
            best_idx = i;
        }
    }
    if (best_dist == 999)
        return; // 没找到任何可达目的地
    target_destination[x] = mv_packet.data.destination[best_idx][x];
    target_destination[y] = mv_packet.data.destination[best_idx][y];
}

int choose_box_with_destination()
{
    return 0;
}
int the_same_x_or_y(float x1, float y1, float x2, float y2) // 判断两个点是否在同一行或同一列,并且中间没有阻挡
{
    int sx = (int)x1;
    int sy = (int)y1;
    int tx = (int)x2;
    int ty = (int)y2;
    if (fabsf(x1 - x2) <= 0.3 && fabsf(y1 - y2) >= 0.8) // 同一列
    {
        for (int j = (int)(y1 < y2 ? y1 : y2); j <= (int)(y2 > y1 ? y2 : y1); j++)
        {
            if (j == sy) // 判断是否为起点
            {
                continue;
            }
            if (map_packet.map_data.map[j][(int)(x2)] != map_road && map_packet.map_data.map[j][(int)(x2)] != map_destination)
            {
                return 0; // 路径被阻挡，返回失败
            }
        }
        return 1; // 说明在同一列
    }
    else if (fabsf(x1 - x2) >= 0.8 && fabsf(y1 - y2) <= 0.3) // 同一行
    {
        for (int i = (int)(x1 < x2 ? x1 : x2); i <= (int)(x2 > x1 ? x2 : x1); i++)
        {
            if (i == sx) // 判断是否为起点
            {
                continue;
            }
            if (map_packet.map_data.map[(int)(y1)][i] != map_road && map_packet.map_data.map[(int)(y1)][i] != map_destination)
            {
                return 0; // 路径被阻挡，返回失败
            }
        }
        return 1; // 说明在同一行
    }
    return 0; // 说明不在同一行或同一列
}

volatile int car_speed = 0, car_way = 0; // 小车速度和方向
int move_path()
{
    float dx = path[point_idx + 1][x] - path[point_idx][x]; // x方向距离
    float dy = path[point_idx + 1][y] - path[point_idx][y]; // y方向距离
    if (fabsf(dx) > fabsf(dy) && fabs(dy) < 0.7)
    {
        // a = 1;
        if (dx > 0)
        {
            if (choose_vision_or_encoder_to_run_path == ENCODER)
            {
                if (encoder_move_distance(dx, go) == 1)
                {
                    return 1;
                }
            }
            else if (choose_vision_or_encoder_to_run_path == VISION)
            {
                // b = 4;
                if (vision_run_path(go) == 1)
                {
                    return 1;
                }
            }
        }
        else
        {
            if (choose_vision_or_encoder_to_run_path == ENCODER)
            {
                if (encoder_move_distance(-dx, back) == 1)
                {
                    return 1;
                }
            }
            else if (choose_vision_or_encoder_to_run_path == VISION)
            {
                if (vision_run_path(back) == 1)
                {
                    return 1;
                }
            }
        }
    }
    // else if (fabsf(dx) < 0.5 && fabsf(dy) > 0.3) // x方向距离小于0.5，y方向距离大于0.3
    else if (fabsf(dx) < fabsf(dy) && fabs(dx) < 0.7)
    {
        a = 2;
        if (dy > 0)
        {
            if (choose_vision_or_encoder_to_run_path == ENCODER)
            {
                if (encoder_move_distance(dy, right) == 1)
                {
                    return 1;
                }
            }
            else if (choose_vision_or_encoder_to_run_path == VISION)
            {
                // a = 12;
                if (vision_run_path(right) == 1)
                {
                    return 1;
                }
            }
        }
        else
        {
            if (choose_vision_or_encoder_to_run_path == ENCODER)
            {
                if (encoder_move_distance(-dy, left) == 1)
                {
                    return 1;
                }
            }
            else if (choose_vision_or_encoder_to_run_path == VISION)
            {
                if (vision_run_path(left) == 1)
                {
                    return 1;
                }
            }
        }
    }
    else
    {
        car_speed = 0;
    }
    return 0;
}

vision_calibrate_t vision_calibrate_state = CALIBRATE_ALIGN_X;
uint8_t vision_calibrate_ok_flag = 1; // 视觉校准标志位
int vision_calibrate(float dx, float dy)
{
    a = 10;
    // 根据视觉反馈进行校准，调整小车位置
    switch (vision_calibrate_state)
    {
    case CALIBRATE_ALIGN_X:
        if (dx > accept_distance_error)
        {
            if (encoder_move_distance(dx, go) == 1)
            {
                vision_calibrate_state = CALIBRATE_ALIGN_Y;
                return 0;
            }
        }
        else if (dx < -accept_distance_error)
        {
            if (encoder_move_distance(-dx, back) == 1)
            {
                vision_calibrate_state = CALIBRATE_ALIGN_Y;
                return 0;
            }
        }
        else
        {
            vision_calibrate_state = CALIBRATE_ALIGN_Y;
            return 0;
        }
        return 0;
    case CALIBRATE_ALIGN_Y:
        if (dy > accept_distance_error)
        {
            if (encoder_move_distance(dy, right) == 1)
            {
                vision_calibrate_state = CALIBRATE_FINISH;
                return 0;
            }
        }
        else if (dy < -accept_distance_error)
        {
            if (encoder_move_distance(-dy, left) == 1)
            {
                vision_calibrate_state = CALIBRATE_FINISH;
                return 0;
            }
        }
        else
        {
            vision_calibrate_state = CALIBRATE_FINISH;
            return 0;
        }
        return 0;

    case CALIBRATE_FINISH:

        if (fabsf(encoder_data_quaddec_L1) < 10 && fabsf(encoder_data_quaddec_R1) < 10) // 可看成停止
        {
            vision_calibrate_state = CALIBRATE_ALIGN_X;
            vision_calibrate_ok_flag = 1; // 标记校准完成
            return 1;
        }
        else
        {
            return 0;
        }
        return 0;
    }
}

// 极简曼哈顿路径实现
// 规则：先计算两条路径（先横再纵；先纵再横），逐格检查是否可走（只能走 map_road 或 map_destination），
// 只要其中一条完全无遮挡就直接返回该路径（若两条都可走，优先返回先横再纵的路径）。
// 返回值：曼哈顿拐点，失败返回0。
int manhattan_path(float x1, float y1, float x2, float y2)
{
    int sx = (int)x1;
    int sy = (int)y1;
    int tx = (int)x2;
    int ty = (int)y2;

    if (sx < 1 || sx >= 11 || tx < 1 || tx >= 11 || sy < 1 || sy >= 15 || ty < 1 || ty >= 15)
    {
        return 0;
    } // 坐标基本合法性判断
    if (the_same_x_or_y(x1, y1, x2, y2)) // 小车与箱子后面的坐标在同一行或者列
    {
        manhattan_turn[x] = x2, manhattan_turn[y] = y2; // 把目标点直接作为曼哈顿路径拐点返回
        return 3;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    int heng_shu = 1; // 先横后纵路径可行标志
    int shu_heng = 1; // 先纵后横路径可行标志
    if (heng_shu == 1)
    {
        for (int i = (int)(x1 < x2 ? x1 : x2); i <= (int)(x2 > x1 ? x2 : x1); i++)
        {
            if ((int)(y1) == sy && i == sx) // 判断是否为起点
            {
                continue;
            }
            if (map_packet.map_data.map[(int)(y1)][i] != map_road && map_packet.map_data.map[(int)(y1)][i] != map_destination)
            {
                heng_shu = 0;
                break; // 路径被阻挡，返回失败
            }
        }

        for (int j = (int)(y1 < y2 ? y1 : y2); j <= (int)(y2 > y1 ? y2 : y1); j++)
        {
            if (j == ty && (int)(x2) == tx) // 判断是否为终点
            {
                continue;
            }
            if (map_packet.map_data.map[j][(int)(x2)] != map_road && map_packet.map_data.map[j][(int)(x2)] != map_destination)
            {
                heng_shu = 0;
                break; // 路径被阻挡，返回失败
            }
        }
    }
    //////////////////////////////////////////////////////
    if (shu_heng == 1)
    {

        for (int j = (int)(y1 < y2 ? y1 : y2); j <= (int)(y2 > y1 ? y2 : y1); j++)
        {
            if (j == sy && (int)(x1) == sx) // 判断是否为起点
            {
                continue;
            }
            if (map_packet.map_data.map[j][(int)(x1)] != map_road && map_packet.map_data.map[j][(int)(x1)] != map_destination)
            {
                shu_heng = 0;
                break; // 路径被阻挡，返回失败
            }
        }
        for (int i = (int)(x1 < x2 ? x1 : x2); i <= (int)(x2 > x1 ? x2 : x1); i++)
        {
            if ((int)(y2) == ty && i == tx) // 判断是否为终点
            {
                continue;
            }
            if (map_packet.map_data.map[(int)(y2)][i] != map_road && map_packet.map_data.map[(int)(y2)][i] != map_destination)
            {
                shu_heng = 0;
                break; // 路径被阻挡，返回失败
            }
        }
    }

    if (heng_shu == 1)
    {
        // 先横后纵的拐点：(x2, y1)
        manhattan_turn[x] = x2;
        manhattan_turn[y] = y1;
        return hengshu;
    }
    else if (shu_heng == 1)
    {
        // 先纵后横的拐点：(x1, y2)
        manhattan_turn[x] = x1;
        manhattan_turn[y] = y2;
        return shuheng;
    }
    return 0; // 都不可行
}

uint8_t vision_calibrate_judge_flag = 0; // 视觉校准判断标志位
int vision_calibrate_judge()
{
    if ((path[target_idx + 1][x] < 1 && path[target_idx + 1][y] < 1) || (path[target_idx + 2][x] < 1 && path[target_idx + 2][y] < 1)) // 说明到了最后一个点，需要校准
    {                                                                                                                                 // 最后俩点需要校准
        return 1;
    }
    if ((fabs(path[target_idx + 1][x] - path[target_idx][x]) + fabs(path[target_idx + 1][y] - path[target_idx][y])) >= 5 || (fabs(path[target_idx + 1][x] - path[target_idx][x]) + fabs(path[target_idx + 1][y] - path[target_idx][y])) < 2)
    { // 长距离或者短距离，都需要校准
        return 1;
    }

    int dx = path[target_idx + 1][x] - path[target_idx][x];
    int dy = path[target_idx + 1][y] - path[target_idx][y];
    if (dx > 1 && dy < 0.5) // 往右
    {
        if (map_packet.map_data.map[(int)path[target_idx][y] + 1][(int)path[target_idx][x] + 1] != map_road || map_packet.map_data.map[(int)path[target_idx][y] - 1][(int)path[target_idx][x] + 1] != map_road)
        {
            return 1;
        }
    }
    else if (dx < -1 && dy < 0.5) // 往左
    {
        if (map_packet.map_data.map[(int)path[target_idx][y] + 1][(int)path[target_idx][x] - 1] != map_road || map_packet.map_data.map[(int)path[target_idx][y] - 1][(int)path[target_idx][x] - 1] != map_road)
        {
            return 1;
        }
    }
    else if (dx < 0.5 && dy > 1) // 往下
    {
        if (map_packet.map_data.map[(int)path[target_idx][y] + 1][(int)path[target_idx][x] - 1] != map_road || map_packet.map_data.map[(int)path[target_idx][y] + 1][(int)path[target_idx][x] + 1] != map_road)
        {
            return 1;
        }
    }
    else if (dx < 0.5 && dy < -1) // 往上
    {
        if (map_packet.map_data.map[(int)path[target_idx][y] - 1][(int)path[target_idx][x] - 1] != map_road || map_packet.map_data.map[(int)path[target_idx][y] - 1][(int)path[target_idx][x] + 1] != map_road)
        {
            return 1;
        }
    }

    return 0; // 都不满足，返回0，不需要校准
}

move_state_t move_state = MOVE_JUDGE;
float push_box_to_destination_dx, push_box_to_destination_dy;
int wait_sure_flag = 0; // 等待确认标志位，确保小车已经停止了才进入视觉校准状态
int car_run_path()      // 小车走path数组的每一个点
{
    if (can_run_flag == 0)
    {
        car_speed = 0, car_way = 0;
        return 0;
    }
    switch (move_state)
    {
    case MOVE_JUDGE:
        b = 1;
        if (path[point_idx][x] == 0 && path[point_idx][y] == 0) // 说明这个拐点可能不存在
        {
            if (point_idx == 0) // 第一个拐点是小车坐标，咋可能是0
            {
                point_idx = 0; // 要么没拐点，要么推完了，重置索引
                return 0;
            }
            else if (point_idx > 0) // 后面有的拐点要是坐标是0，就直接跳过
            {
                car_speed = 0, car_way = 0;
            }
        }
        else // 说明这个拐点存在，直接进入推箱子状态
        {
            move_state = MOVE_TO_POINT;
        }
        return 0;

    case MOVE_TO_POINT:
        b = 2;
        if (path[point_idx + 1][x] == path[point_idx][x] && path[point_idx + 1][y] == path[point_idx][y])
        { // 该点与下一个拐点一样，直接跳过
            move_state = MOVE_FINISH;
            return 0;
        }
        if (wait_sure_flag == 0 && vision_calibrate_ok_flag == 1) // 等待小车到达了拐点，确保小车已经停止了才进入视觉校准状态
        {

            if (move_path() == 1)
            {
                wait_sure_flag = 1; // 小车已经到达了拐点
            }
        }

        if (wait_sure_flag)
        {
            wait_sure_flag = 0; // 小车已经到达了拐点
            move_state = MOVE_FINISH;
        }

        return 0;

    case MOVE_CALIBRATE:

        push_box_to_destination_dx = path[point_idx + 1][x] - car_x; // 拐点与车的x误差
        push_box_to_destination_dy = path[point_idx + 1][y] - car_y; // 拐点与车的y误差
        vision_calibrate(push_box_to_destination_dx, push_box_to_destination_dy);
        if (vision_calibrate_ok_flag == 1) // 标记校准完成
        {
            move_state = MOVE_FINISH;
        }
        return 0;
    case MOVE_FINISH:
        b = 3;

        if (fabsf(encoder_data_quaddec_L1) < 20 && fabsf(encoder_data_quaddec_R1) < 20) // 可看成停止
        {
            // if((mv_packet.data.car_x - path[point_idx + 1][x]) < accept_distance_error && (mv_packet.data.car_y - path[point_idx + 1][y]) < accept_distance_error)
            // {
            move_state = MOVE_JUDGE;
            point_idx++; // 下一个拐点
            return 1;    // 说明到达目标点了
            // }
            // else
            // {
            //     car_x = mv_packet.data.car_x;
            //     car_y = mv_packet.data.car_y;
            //     move_state = MOVE_CALIBRATE;
            //     car_speed = 0, car_way = 0;
            //     return 0;
            // }
        }
    }
    return 0;
}

uint8_t level = HAVE_NOTHING;                 // 当前关卡,1:只有墙 2：有模型 3：有炸弹
uint8_t path_flag = 0;                        // 可以进行计算推箱子路线标志位
uint8_t go_to_get_map_flag = 0;               // 往前一格从而载入地图
uint8_t try_if_same_destination_x_y_flag = 0; // 尝试看看目的地是否同一行或者同一列
int push_flag = 0;                            // 推箱子标志，为1就说明算完了，正在推
int manhattan_get_path_error = 0;             // 曼哈顿路径错误标志
float box_manhattan_back_x = 0;               // 曼哈顿拐点后面坐标
float box_manhattan_back_y = 0;               // 曼哈顿拐点后面坐标
float manhattan_destination_back_x = 0;       // 曼哈顿拐点后面坐标
float manhattan_destination_back_y = 0;       // 曼哈顿拐点后面坐标
float destination_back_x = 0;                 // 目的地后面的坐标
float destination_back_y = 0;                 // 目的地后面的坐标
float path[50][2];                            // 计算出的推箱子路径，暂定最多50个拐点
int final_path_len = 0;                       // 最终路径长度
volatile int point_idx = 0;                   // 路径当前点的索引
float car_x, car_y;                           // 存放小车当前位置
get_path_t path_state = PATH_INIT;            // 设置初始状态为初始化状态
int push_dir = 0;                             // 小车要把箱子推到目标点的方向
float man_x, man_y;                           // 存放曼哈顿路径拐点坐标
uint8_t found_match = 0;                      // 是否找到匹配的目的地
int nearest_index = -1;                       // 最近的目的地索引
extern int b;
int last_destination_num = 0;

void turn_final_path()
{
    if (final_path_len < 2)
        return;
    float tmp_path[50][2];
    int tmp_len = 0;
    float last_dx = path[1][x] - path[0][x];
    float last_dy = path[1][y] - path[0][y];
    tmp_path[tmp_len][x] = path[0][x];
    tmp_path[tmp_len][y] = path[0][y];
    tmp_len++;

    for (int i = 1; i < final_path_len - 1; i++)
    {
        if (fabs(path[i][x] - path[i - 1][x]) < 0.3f &&
            fabs(path[i][y] - path[i - 1][y]) < 0.3f)
        {
            continue;
        }

        float dx = path[i + 1][x] - path[i][x];
        float dy = path[i + 1][y] - path[i][y];

        // 归一到方向符号（正→1, 负→-1, 零→0），比较符号而非数值大小
        int sx = (fabs(dx) > 0.3f) ? (dx > 0 ? 1 : -1) : 0;
        int sy = (fabs(dy) > 0.3f) ? (dy > 0 ? 1 : -1) : 0;
        int lx = (fabs(last_dx) > 0.3f) ? (last_dx > 0 ? 1 : -1) : 0;
        int ly = (fabs(last_dy) > 0.3f) ? (last_dy > 0 ? 1 : -1) : 0;

        if (sx != lx || sy != ly)
        {
            tmp_path[tmp_len][x] = path[i][x];
            tmp_path[tmp_len][y] = path[i][y];
            tmp_len++;
            last_dx = dx;
            last_dy = dy;
        }
    }

    if (tmp_len == 0 ||
        fabs(tmp_path[tmp_len - 1][x] - path[final_path_len - 1][x]) > 0.2f ||
        fabs(tmp_path[tmp_len - 1][y] - path[final_path_len - 1][y]) > 0.2f)
    {
        tmp_path[tmp_len][x] = path[final_path_len - 1][x];
        tmp_path[tmp_len][y] = path[final_path_len - 1][y];
        tmp_len++;
    }

    for (int i = 0; i < tmp_len; i++)
    {
        path[i][x] = tmp_path[i][x];
        path[i][y] = tmp_path[i][y];
    }

    for (int i = tmp_len; i < final_path_len; i++)
    {
        path[i][x] = 0.0f;
        path[i][y] = 0.0f;
    }

    final_path_len = tmp_len;
}

int deal_current_level() // 处理当前关卡
{
    if (destination_num == 0 && box_num == 0) // 不能推，因为啥都没有
    {
        path_flag = 0;
    }

    if (level == HAVE_NOTHING) // 只有墙
    {
        path_flag = 1;
        get_map_message();
        get_manhattan_path(); //   先看能不能直接找到曼哈顿路径
        get_bfs_path();       //   曼哈顿路径要是不行就上bfs，这俩函数完全不冲突
    }
    else if (level == HAVE_MODEL) // 有模型
    {
        if (have_recognized_nearest_box == 0)
        {
            path_flag = 0;
            model_recognization(); // 识别模型
        }

        else if (no_need_to_recognize_destination_model == 1 && have_recognized_nearest_box == 1) // 识别目的地模型和最近箱子模型已经完成
        {
            path_flag = 1;
            get_manhattan_path(); //   先看能不能直接找到曼哈顿路径
            get_bfs_path();       //   曼哈顿路径要是不行就上bfs，这俩函数完全不冲突
        }

        else
        {
            path_flag = 0;
            return 0;
        }
    }

    else if (level == HAVE_MODEL_AND_BOMBS) // 有模型和炸弹
    {
        run_bomb_path_plus();
        if (bomb_finish_flag == 1)
        {
            level = HAVE_MODEL;
        }
        path_flag = 1;
    }

    return 0;
}

void find_nearest_start_address() // 找到最近的起始地址，准备进入下一关
{
    point_idx = 0;
    path[0][x] = mv_packet.data.car_x;
    path[0][y] = mv_packet.data.car_y;
    path[1][x] = 6;
    path[1][y] = mv_packet.data.car_y;
    path[2][x] = 6;
    path[2][y] = 1.5;
}

uint8_t destination_used[BOX_DESTINATION_NUM_MAX] = {false, false, false, false, false};
uint8_t temp_num = 0;
uint8_t only_one_destination_flag = 0;
uint8_t can_run_flag = 0; // 是否可以运行路径
float last_destination_x = 0;
float last_destination_y = 0;
int current_model;
int encounter_bug_flag; // 这个东西吧，反正遇到bug了就会启用这个开始计时
int encounter_bug_cnt;  // 这个东西吧，反正遇到bug了就会启用这个开始计时
uint8_t get_better_path_flag = 0;
int get_manhattan_path() // 计算路径的函数，参数是当前状态
{

    if (path_flag == 0) // 如果是第二关，且识别目的地模型未完成
    {
        return 0; // 说明模型未识别完成，不能计算路径
    }
    float dx, dy; // 用来判断移动方向，本质是x或者y的误差
    switch (path_state)
    {
        // a++;
    case PATH_INIT: // “初始化”，得到目标箱子以及目标目的地

        if (!manhattan_get_path_error && map_rx_flag == 1 && mv_packet.data.car_x != 0 && mv_packet.data.car_y != 0) // 收到地图才开始初始化
        {
            get_better_path_flag = 0;
            b = 10;
            manhattan_get_path_error = 0; // 曼哈顿路径错误标志重置
            point_idx = 0;                // 索引重置
            push_flag = 0;
            if (level == HAVE_NOTHING)
            {
                a = 2;
                get_target_box();
                get_target_destination();
            } // 推箱子标志重置

            else if (no_need_to_recognize_destination_model == 1 && have_recognized_nearest_box == 1) // 如果需要识模型
            {

                if (only_one_destination_flag == 1) // 说明只有一个目的地，直接赋值
                {
                    path_state = PATH_TO_BOX_BACK;
                }
                else // 说明有多个目的地，需要找到最近的目的地并且符合对应关系
                {
                    float min_distance = 999.9f;
                    current_model = box_model;
                    int nearest_index = -1; // 建议初始化
                    int found_match = 0;
                    for (int i = 0; i < initial_destination_num; i++)
                    {
                        if (destination_used[i])
                            continue; // 跳过已被分配的目的地

                        if (destination_model[i] == current_model)
                        {
                            float distance = astar_distance(target_box[x], target_box[y], initial_destination_pos[i][x], initial_destination_pos[i][y]);
                            if (distance < min_distance)
                            {
                                min_distance = distance;
                                nearest_index = i;
                            }
                            found_match = 1;
                        }
                    }

                    if (found_match == 1 && nearest_index != -1) // 找到了对应关系的目的地并且是最近的那个
                    {
                        b = 11;
                        target_destination[x] = initial_destination_pos[nearest_index][x];
                        target_destination[y] = initial_destination_pos[nearest_index][y];
                        destination_used[nearest_index] = true; // 新增：标记已使用
                        have_used_destination_num++;            // 新增：已使用已使用目的地数量
                        nearest_index = -1;
                    }
                    else // 如果没有找到匹配的目的地，可能是这个目的地就是没有被识别的那个
                    {

                        for (int i = 0; i < initial_destination_num; i++)
                        {
                            if (destination_model[i] != -1 || destination_used[i])
                            {
                                continue; // 跳过已经识别过的目的地和用过的目的地
                            }
                            else // 找到未被识别的目的地，然后标记这个目的地
                            {
                                target_destination[x] = initial_destination_pos[i][x];
                                target_destination[y] = initial_destination_pos[i][y];
                                destination_used[i] = true;  // 标记已使用
                                have_used_destination_num++; // 已使用已使用目的地数量
                                break;                       // 找到第一个未使用的目的地后退出循环
                            }
                        }

                        // 新增：已使用已使用目的地数量
                    }

                    if (fabs(target_destination[x] - last_destination_x) < 0.2 && fabs(target_destination[y] - last_destination_y) < 0.2)
                    {
                        if (destination_num == 1)
                        {
                            target_destination[x] = mv_packet.data.destination[0][x];
                            target_destination[y] = mv_packet.data.destination[0][y];
                            have_used_destination_num++;
                        }
                        else
                        {
                            for (int i = 0; i < initial_destination_num; i++)
                            {
                                if (destination_used[i])
                                {
                                    continue;
                                }
                                else
                                {
                                    target_destination[x] = initial_destination_pos[i][x];
                                    target_destination[y] = initial_destination_pos[i][y];
                                    destination_used[i] = true;
                                    have_used_destination_num++;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                break;
            }

            car_x = mv_packet.data.car_x; // 获取小车初始坐标
            car_y = mv_packet.data.car_y; // 获取小车初始坐标
            memset(path, 0, sizeof(path));
            path[0][x] = car_x;     // 路径的第一个点是小车当前位置
            path[0][y] = car_y;     // 路径的第一个点是小车当前位置
            last_box_num = box_num; // 更新上一次箱子数量
            last_destination_num = destination_num;
            if (target_box[x] != 0 && target_box[y] != 0 && target_destination[x] != 0 && target_destination[y] != 0)
            {
                path_state = PATH_TO_BOX_BACK; // 进入到“移动到箱子后面”状态
                return 0;
            }
            // manhattan_get_path_error=1;
        }
        else
        {
            return 0;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////
    case PATH_TO_BOX_BACK: // 计算移动到箱子后面的路径

        if (manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 1 || manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 2 || manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 3)
        // 大条件，曼哈顿拐点需要存在，如果返回值是3，也就是把目标点作为曼哈顿拐点返回
        {
            a = 3;
            dx = manhattan_turn[x] - target_box[x]; // 曼哈顿拐点与箱子x坐标的差值，用来判断小车移动方向
            dy = manhattan_turn[y] - target_box[y]; // 曼哈顿拐点与箱子y坐标的差值，用来判断小车移动方向
            if (fabs(dx) < 0.5 && fabs(dy) > 0.8)   // 说明箱子和第一个曼哈顿拐点x坐标几乎一样(同一列)，要到上面/下面
            {
                if (dy > 0)
                {
                    push_dir = right; // 说明等会要往下推箱子
                    path[2][x] = target_box[x];
                    path[2][y] = target_box[y] - 1; // 到箱子上面一格
                }
                else
                {
                    push_dir = left; // 说明等会要往上推箱子
                    path[2][x] = target_box[x];
                    path[2][y] = target_box[y] + 1; // 到箱子下面一格
                }
            }
            else if (fabs(dy) < 0.3 && fabs(dx) > 0.5) // 说明箱子和第一个曼哈顿拐点y坐标几乎一样(同一行)，要到左面/右面
            {
                if (dx > 0)
                {
                    push_dir = go;                  // 说明等会要往前推箱子
                    path[2][x] = target_box[x] - 1; // 到箱子左边
                    path[2][y] = target_box[y];
                }
                else
                {
                    push_dir = back;                // 说明等会要往后推箱子
                    path[2][x] = target_box[x] + 1; // 到箱子右边
                    path[2][y] = target_box[y];
                }
            }
            else
            {
                return 0; // 大条件都不满足，直接退出
            }

            if ((manhattan_path(car_x, car_y, path[2][x], path[2][y]) == 1 || manhattan_path(car_x, car_y, path[2][x], path[2][y]) == 2 || manhattan_path(car_x, car_y, path[2][x], path[2][y]) == 3) && map_packet.map_data.map[(int)path[2][y]][(int)path[2][x]] == 0)
            {
                path[1][x] = manhattan_turn[x]; // 上面的函数计算出曼哈顿拐点坐标
                path[1][y] = manhattan_turn[y]; // 第一个拐点坐标，小车先到这个点
            }
            else
            {
                path_state = PATH_INIT;       // 进入初始化状态
                manhattan_get_path_error = 1; // 曼哈顿路径错误标志
                return 0;
            }

            path_state = PATH_TO_MANHATTAN; // 进入“把箱子推到曼哈顿拐点状态”
        }
        else
        {
            manhattan_get_path_error = 1; // 曼哈顿路径错误标志
            return 0;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////
    case PATH_TO_MANHATTAN: // 计算把箱子推到曼哈顿拐点的路径

        if (manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 1 || manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 2 || manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 3)
        // 大条件，曼哈顿拐点存在，如果返回值是3，也就是把目标点作为曼哈顿拐点返回
        {
            man_x = manhattan_turn[x];
            man_y = manhattan_turn[y];
            // 曼哈顿拐点更新，计算从箱子到目的地的曼哈顿拐点
            if (push_dir == right) // 说明等会要往右推箱子
            {
                box_manhattan_back_x = manhattan_turn[x]; // 小车把箱子推到曼哈顿拐点后，小车理论的坐标
                box_manhattan_back_y = manhattan_turn[y] - 1;
            }
            else if (push_dir == left) // 说明等会要往左推箱子
            {
                box_manhattan_back_x = manhattan_turn[x]; // 小车把箱子推到曼哈顿拐点后，小车理论的坐标
                box_manhattan_back_y = manhattan_turn[y] + 1;
            }
            else if (push_dir == go) // 说明等会要往前推箱子
            {
                box_manhattan_back_x = manhattan_turn[x] - 1; // 小车把箱子推到曼哈顿拐点后，小车理论的坐标
                box_manhattan_back_y = manhattan_turn[y];
            }
            else if (push_dir == back) // 说明等会要往后推箱子
            {
                box_manhattan_back_x = manhattan_turn[x] + 1; // 小车把箱子推到曼哈顿拐点后，小车理论的坐标
                box_manhattan_back_y = manhattan_turn[y];
            }
            path[3][x] = box_manhattan_back_x;
            path[3][y] = box_manhattan_back_y; // 把箱子推到曼哈顿点后，小车的位置
            if (manhattan_path(target_box[x], target_box[y], target_destination[x], target_destination[y]) == 3)
            {
                path_state = PATH_FINISH; // 直接进入目的地状态
                return 0;
            }
            path_state = PATH_TO_MANHATTAN_BACK; // 进入“计算小车到曼哈顿拐点后面的路径”
        }
        else
        {
            return 0;
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////
    case PATH_TO_MANHATTAN_BACK: // 计算小车到曼哈顿拐点后面的路径

        dx = target_destination[x] - man_x;
        dy = target_destination[y] - man_y;   // 目的地与曼哈顿拐点的误差
        if (fabs(dx) < 0.5 && fabs(dy) > 0.8) // 说明目的地和曼哈顿拐点的x坐标几乎一样(同一列)，要到上面/下面
        {
            if (dy > 0)
            {
                push_dir = right; // 说明等会要往下推箱子
                manhattan_destination_back_x = man_x;
                manhattan_destination_back_y = man_y - 1; // 到箱子上面的坐标
                path[4][x] = box_manhattan_back_x;        // 小车此时要移动到箱子后面，需要先经过这个拐点
                path[4][y] = box_manhattan_back_y - 1;
            }
            else
            {
                push_dir = left; // 说明等会要往上推箱子
                manhattan_destination_back_x = manhattan_turn[x];
                manhattan_destination_back_y = manhattan_turn[y] + 1; // 到箱子下面的坐标
                path[4][x] = box_manhattan_back_x;                    // 小车此时要移动到箱子后面，需要先经过这个拐点
                path[4][y] = box_manhattan_back_y + 1;
            }
        }
        else if (fabs(dy) < 0.3 && fabs(dx) > 0.8) // 说明目的地和曼哈顿拐点的y坐标几乎一样(同一行)，要到左面/右面
        {
            if (dx > 0)
            {
                push_dir = go; // 说明等会要往前推箱子
                manhattan_destination_back_x = manhattan_turn[x] - 1;
                manhattan_destination_back_y = manhattan_turn[y]; // 到箱子左面的坐标
                path[4][x] = box_manhattan_back_x - 1;            // 小车此时要移动到箱子后面，需要先经过这个拐点
                path[4][y] = box_manhattan_back_y;
            }
            else
            {
                push_dir = back; // 说明等会要往后推箱子
                manhattan_destination_back_x = manhattan_turn[x] + 1;
                manhattan_destination_back_y = manhattan_turn[y]; // 到箱子右面的坐标
                path[4][x] = box_manhattan_back_x + 1;            // 小车此时要移动到箱子后面，需要先经过这个拐点
                path[4][y] = box_manhattan_back_y;
            }
        }
        if (map_packet.map_data.map[(int)(path[4][y])][(int)(path[4][x])] != map_road && map_packet.map_data.map[(int)(path[4][y])][(int)(path[4][x])] != map_destination)
        {
            manhattan_get_path_error = 1; // 曼哈顿路径错误标志
            path_state = PATH_INIT;       // 进入初始化状态
            return 0;
        }
        path[5][x] = manhattan_destination_back_x;
        path[5][y] = manhattan_destination_back_y; // 算出小车要到达的箱子后面的坐标
        if (map_packet.map_data.map[(int)(path[5][y])][(int)(path[5][x])] != map_road && map_packet.map_data.map[(int)(path[5][y])][(int)(path[5][x])] != map_destination && map_packet.map_data.map[(int)(path[4][y])][(int)(path[4][x])] != map_road && map_packet.map_data.map[(int)(path[4][y])][(int)(path[4][x])] != map_destination)
        {
            manhattan_get_path_error = 1; // 曼哈顿路径错误标志
            path_state = PATH_INIT;       // 进入初始化状态
            return 0;
        }
        path_state = PATH_TO_DESTINATION; // 进入“计算小车把箱子推到目的地的路径”
        return 0;
        /////////////////////////////////////////////////////////////////////////////////////////////////////
    case PATH_TO_DESTINATION: // 计算小车把箱子推到目的地的路径

        if (push_dir == right) // 说明等会要往右推箱子
        {
            destination_back_x = target_destination[x]; // 小车把箱子推到目的地后，小车的理论坐标
            destination_back_y = target_destination[y] - 1;
        }
        else if (push_dir == left) // 说明等会要往左推箱子
        {
            destination_back_x = target_destination[x]; // 小车把箱子推到目的地后，小车的理论坐标
            destination_back_y = target_destination[y] + 1;
        }
        else if (push_dir == go) // 说明等会要往前推箱子
        {
            destination_back_x = target_destination[x] - 1; // 小车把箱子推到目的地后，小车的理论坐标
            destination_back_y = target_destination[y];
        }
        else if (push_dir == back) // 说明等会要往后推箱子
        {
            destination_back_x = target_destination[x] + 1; // 小车把箱子推到目的地后，小车的理论坐标
            destination_back_y = target_destination[y];
        }
        path[6][x] = destination_back_x;
        path[6][y] = destination_back_y; // 计算出小车把箱子推到目的地后，小车的理论坐标
        if (map_packet.map_data.map[(int)(path[6][y])][(int)(path[6][x])] != map_road && map_packet.map_data.map[(int)(path[6][y])][(int)(path[6][x])] != map_destination)
        {
            manhattan_get_path_error = 1; // 曼哈顿路径错误标志
            path_state = PATH_INIT;       // 进入初始化状态
            return 0;
        }
        int temp_len = 0;
        for (int i = 0; i < 20; i++)
        {
            if (path[i][x] > 1 && path[i][y] > 1)
            {
                temp_len++;
            }
        }

        final_path_len = temp_len;
        turn_final_path();
        turn_final_path();
        path_state = PATH_FINISH; // 进入“路径算完了，检测小车是否已经推完该箱子，然后回到初始化状态”
        return 0;
        /////////////////////////////////////////////////////////////////////////////////////////////////////
    case PATH_FINISH:
        if (get_better_path_flag == 0)
        {
            int temp_lens = 0;
            for (int i = 0; i < 20; i++)
            {
                if (path[i][x] > 1 && path[i][y] > 1)
                {
                    temp_lens++;
                }
            }
            final_path_len = temp_lens;
            turn_final_path();
            // clear_same_path();
            get_better_path_flag = 1;
        }
        can_run_flag = 1;
        if ((int)path[point_idx + 1][x] == 0 && (int)path[point_idx + 1][y] == 0 && point_idx != 0 && vision_calibrate_ok_flag == 1 && destination_num < last_destination_num && box_num < last_box_num) // 说明箱子已经被推走了，且视觉校准完成了，可以进行下一个箱子的路径计算了
        {
            can_run_flag = 0;
            b = 4;
            car_speed = 0;
            if (fabsf(encoder_data_quaddec_L1) < 10 && fabsf(encoder_data_quaddec_R1) < 10)
            {
                memset(path, 0, sizeof(path)); // 给数组清0
                // a++;
                // wait_ms(100);
                wait_ok_flag = 1;
                if (wait_ok_flag)
                {
                    last_destination_x = target_destination[x];
                    last_destination_y = target_destination[y];
                    if ((initial_destination_num - have_used_destination_num) == 1) // 说明只有一个目的地，直接赋值
                    {
                        only_one_destination_flag = 1;
                        for (int i = 0; i < initial_destination_num; i++)
                        {
                            if (destination_used[i])
                            {
                                continue; // 跳过已被分配的目的地
                            }
                            else
                            {
                                target_destination[x] = initial_destination_pos[i][x];
                                target_destination[y] = initial_destination_pos[i][y];
                                if (fabs(target_destination[x] - mv_packet.data.destination[0][x]) > 0.8 || fabs(target_destination[y] - mv_packet.data.destination[0][y]) > 0.8)
                                {
                                    target_destination[x] = mv_packet.data.destination[0][x];
                                    target_destination[y] = mv_packet.data.destination[0][y];
                                }
                                destination_used[i] = true; // 标记为已使用
                                have_used_destination_num++;
                                break; // 找到第一个未使用的目的地后退出循环
                            }
                        }
                    }
                    last_destination_num = destination_num;
                    last_box_num = box_num;
                    box_model = -1; // 重置当前箱子模型
                    wait_ok_flag = 0;
                    if (box_num == 0 || last_box_num == 0) // 所有箱子都被推到目的地
                    {
                        have_used_destination_num = 0;
                        c++;
                        point_idx = 0;                      // 索引重置
                        memset(path, 0, sizeof(path));      // 给数组清0
                        find_nearest_start_address();       // 找到最近的起始地址，准备进入下一关
                        path_state = PATH_READY_NEXT_LEVEL; // 回到发车区，然后进入下一关
                        return 0;
                    }
                    else
                    {
                        path_state = PATH_INIT;          // 回到初始化状态
                        model_state = MODEL_FIND_TARGET; // 重置识别模型状态机
                        have_recognized_nearest_box = 0; // 重置最近箱子识别标志
                        path_flag = 0;                   // 每次推完一个就把这个置0，等下一次选好了箱子和目的地，就会置1
                        push_flag = 1;
                        point_idx = 0;
                        return 1;
                    }
                }
            }
            else
            {
                break;
            }
        }
        else
        {
            car_run_path(); // 小车走path数组的每一个点
        }
        break;
    case PATH_READY_NEXT_LEVEL:
        a = 3;
        can_run_flag = 1;
        if ((int)path[point_idx + 1][x] == 0 && (int)path[point_idx + 1][y] == 0 && point_idx != 0 && vision_calibrate_ok_flag == 1) // 说明回到发车区了，且视觉校准完成了，可以进入下一关了
        {
            last_destination_x = 0;
            last_destination_y = 0;
            can_run_flag = 0;
            level++;       // 关卡增加1
            path_flag = 0; // 重置路径标志
            push_flag = 1;
            no_need_to_recognize_destination_model = 0; // 重置识别目的地模型标志
            path_state = PATH_INIT;                     // 回到初始化状态，准备下一关
            keep_destination_pos_flag = 0;              // 重置保持目的地位置标志
            go_to_get_map_flag = 0;                     // 往前一格从而载入地图标志重置
            point_idx = 0;
            map_rx_flag = 0;
            have_used_destination_num = 0;
            only_one_destination_flag = 0;
            have_recognized_nearest_box = 0;
            have_found_destination = 0;
            memset(destination_idx, -1, sizeof(destination_idx));
            for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
            {
                destination_model[i] = -1; // 重置目的地模型
                destination_used[i] = 0;   // 重置目的地使用标志
            }
            memset(path, 0, sizeof(path));
            return 1;
        }
        else
        {
            car_run_path(); // 小车走path数组的每一个点
        }
    }
    return 0;
}

// Point bfs_car_to_box_path[PATH_NODE];//小车到箱子旁边的数组
// Point push_path[PATH_NODE];//推箱子的数组
extern aPoint abox_path[APATH_NODE]; // 箱子路径数组，包含箱子路径的起点和终点
get_bfs_path_t bfs_path_state = BFS_INIT;
int bfs_flag = 0; // bfs路径标志
int get_bfs_path()
{
    if (path_flag == 0)
    {
        return 0;
    }
    if (!manhattan_get_path_error) // 曼哈顿路径错误标志
    {
        return 0;
    }
    switch (bfs_path_state)
    {
    case BFS_INIT:
        point_idx = 0;
        push_flag = 0;                                                                  // 初始化路径索引
        if (map_rx_flag == 1 && mv_packet.data.car_x != 0 && mv_packet.data.car_y != 0) // 曼哈顿路径条件不满足，无法计算，改用bfs
        {
            bfs_path_state = BFS_GET_PATH;
        }
        return 0;
    case BFS_GET_PATH:
        memset(path, 0, sizeof(path));
        if (amake_box_path() && bfs_flag == 0) // 得到箱子路径
        {
            if (amake_car_to_box_path(abox_path[0].ax, abox_path[0].ay, abox_path[1].ax, abox_path[1].ay)) // 得到小车到箱子的路径
            {

                bfs_flag = 1;
            }
        }
        if (bfs_flag == 1)
        {
            amake_push_box_path();
            get_astar_turn_path(apush_path, &apush_path_len);
            bfs_flag = 0;
            bfs_path_state = BFS_TRANSFER_PATH;
        }
        return 0;
    case BFS_TRANSFER_PATH:
        for (int i = 0; i < acar_to_box_path_len && point_idx < 30; i++)
        {
            path[point_idx][x] = (float)acar_to_box_path[i].ax + 0.5f;
            path[point_idx][y] = (float)acar_to_box_path[i].ay + 0.5f;
            point_idx++;
        }
        for (int k = 0; k < 30 && point_idx < 30; k++)
        {
            if (apush_path[k].ax != 0 && apush_path[k].ay != 0)
            {
                apush_path_len++;
            }
        }
        // 再复制推箱子到目的地的路径
        for (int j = 0; j < apush_path_len && point_idx < 50; j++)
        {
            path[point_idx][x] = (float)(apush_path[j].ax + 0.5f);
            path[point_idx][y] = (float)(apush_path[j].ay + 0.5f);
            point_idx++;
        }
        final_path_len = point_idx;
        turn_final_path();
        turn_final_path();
        point_idx = 0;
        bfs_path_state = BFS_FINISH;
        return 0;
    case BFS_FINISH:
        // a++;
        can_run_flag = 1;
        if ((int)path[point_idx + 1][x] == 0 && (int)path[point_idx + 1][y] == 0 && point_idx != 0 && box_num < last_box_num && destination_num < last_destination_num) // 说明箱子已经被推走了，且视觉校准完成了，可以进行下一个箱子的路径计算了
        {
            b = 5;
            can_run_flag = 0;
            car_speed = 0;
            if (fabsf(encoder_data_quaddec_L1) < 10 && fabsf(encoder_data_quaddec_R1) < 10)
            {

                // wait_ms(100);
                wait_ok_flag = 1;
                if (wait_ok_flag)
                {
                    last_destination_x = target_destination[x];
                    last_destination_y = target_destination[y];
                    if ((initial_destination_num - have_used_destination_num) == 1) // 说明只有一个目的地，直接赋值
                    {
                        only_one_destination_flag = 1;
                        for (int i = 0; i < initial_destination_num; i++)
                        {
                            if (destination_used[i])
                            {
                                continue; // 跳过已被分配的目的地
                            }
                            else
                            {
                                target_destination[x] = initial_destination_pos[i][x];
                                target_destination[y] = initial_destination_pos[i][y];
                                if (fabs(target_destination[x] - mv_packet.data.destination[0][x]) > 0.8 || fabs(target_destination[y] - mv_packet.data.destination[0][y]) > 0.8)
                                {
                                    target_destination[x] = mv_packet.data.destination[0][x];
                                    target_destination[y] = mv_packet.data.destination[0][y];
                                }
                                destination_used[i] = true; // 标记为已使用
                                have_used_destination_num++;
                                break; // 找到第一个未使用的目的地后退出循环
                            }
                        }
                    }
                    last_destination_num = destination_num;
                    last_box_num = box_num;
                    box_model = -1; // 重置当前箱子模型
                    wait_ok_flag = 0;
                    memset(acar_to_box_path, 0, sizeof(acar_to_box_path));
                    memset(apush_path, 0, sizeof(apush_path));
                    memset(path, 0, sizeof(path));
                    if (box_num == 0 || last_box_num == 0)
                    {
                        have_used_destination_num = 0;
                        c++;
                        point_idx = 0; // 索引重置
                        memset(path, 0, sizeof(path));
                        find_nearest_start_address();          // 找到最近的起始地址，准备进入下一关
                        bfs_path_state = BFS_READY_NEXT_LEVEL; // 回到发车区，然后进入下一关
                        return 0;
                    }
                    else
                    {
                        point_idx = 0;
                        bfs_path_state = BFS_INIT;       // 回到初始化状态
                        model_state = MODEL_FIND_TARGET; // 重置识别模型状态机
                        path_state = PATH_INIT;          // 进入初始化状态
                        manhattan_get_path_error = 0;
                        have_recognized_nearest_box = 0; // 重置最近箱子识别标志
                        path_flag = 0;
                        push_flag = 1;
                        memset(path, 0, sizeof(path));
                        return 1;
                    }
                }
            }
        }
        else
        {
            car_run_path(); // 小车走path数组的每一个点
        }
        break;

    case BFS_READY_NEXT_LEVEL:
        b = 19;
        can_run_flag = 1;
        if ((int)path[point_idx + 1][x] == 0 && (int)path[point_idx + 1][y] == 0 && point_idx != 0 && vision_calibrate_ok_flag == 1) // 说明回到发车区了，且视觉校准完成了，可以进入下一关了
        {
            last_destination_x = 0;
            last_destination_y = 0;
            can_run_flag = 0;
            manhattan_get_path_error = 0;
            level++;                                    // 关卡增加1
            path_flag = 0;                              // 重置路径标志
            no_need_to_recognize_destination_model = 0; // 重置识别目的地模型标志
            bfs_path_state = BFS_INIT;                  // 回到初始化状态，准备下一关
            path_state = PATH_INIT;
            keep_destination_pos_flag = 0; // 重置保持目的地位置标志
            go_to_get_map_flag = 0;        // 往前一格从而载入地图标志重置
            point_idx = 0;
            map_rx_flag = 0;
            have_used_destination_num = 0;
            only_one_destination_flag = 0;
            have_recognized_nearest_box = 0;
            have_found_destination = 0;
            memset(destination_idx, -1, sizeof(destination_idx));
            for (int i = 0; i < BOX_DESTINATION_NUM_MAX; i++)
            {
                destination_model[i] = -1; // 重置目的地模型
                destination_used[i] = 0;   // 重置目的地使用标志
            }
            memset(acar_to_box_path, 0, sizeof(acar_to_box_path));
            memset(apush_path, 0, sizeof(apush_path));
            memset(path, 0, sizeof(path));
            return 1;
        }
        else
        {
            car_run_path(); // 小车走path数组的每一个点
        }
    }
    return 0;
}

typedef enum
{
    BOMB_PATH_INIT, // 初始化
    BOMB_GET_STUCK_PATH,
    BOMB_GET_FREE_PATH, // 调用 make_free_bomb_path 规划炸弹+箱子路径
    BOMB_TRANSFER_PATH, // 将炸弹路径数据传到 path 数组
    BOMB_FINISH,        // 执行路径，检测是否推完了
} bomb_run_state_t;

bomb_run_state_t bomb_run_state = BOMB_PATH_INIT;
uint8_t bomb_path_flag = 0;

extern aPoint car_to_bomb_path[APATH_NODE];
extern int car_to_bomb_path_len;
extern aPoint push_bomb_path[APATH_NODE];
extern int push_bomb_len;
extern aPoint bomb_path[APATH_NODE];
extern int bomb_path_len;

extern int16_t pair_list_len;

extern int8_t nearlist_bomb_x;
extern int8_t nearlist_bomb_y;
int last_bomb_num = 0;
uint8_t state_a = 0;
int check_stuck_ret = -1;
uint8_t lock_flag = 0;
uint8_t free_flag = 0;
uint8_t ra = 0;
uint8_t rb = 0;
uint8_t lock = 0;
extern int stuck_x, stuck_y;

extern int8_t unlock_bomb_x;
extern int8_t unlock_bomb_y;
extern int8_t ul_debug, ul_wx, ul_wy, ul_bx, ul_by;

uint8_t nearlist_flag_a = 0;
uint8_t nearlist_box_index_flag = 0;

uint8_t game_a = 0;
static uint8_t game_init_flag = 0;
int run_bomb_path_plus(void)
{
    static uint8_t debug_path_type = 0;
    static uint8_t debug_bomb_x = 0;
    static uint8_t debug_bomb_y = 0;
    static uint8_t debug_explode_x = 0;
    static uint8_t debug_explode_y = 0;

    static int8_t executed_bomb_x = -1;
    static int8_t executed_bomb_y = -1;

    static uint8_t local_map_ready = 0;

    switch (bomb_run_state)
    {

    case BOMB_PATH_INIT:
    {
        if (local_map_ready == 0)
        {

            if (map_rx_flag != 1)
                return 0;

            if (mv_packet.data.car_x == 0 || mv_packet.data.car_y == 0)
                return 0;

            static int init_bomb_num = 0;

            if (init_bomb_num == 0)
            {
                last_bomb_num = bomb_num;
                init_bomb_num = 1;
            }

            if (game_init_flag == 0)
            {

                memcpy(bomb_map, map_packet.map_data.map, sizeof(bomb_map));
                game_a++;
                if (game_a > 5)
                {
                    game_init_flag = 1;
                }
            }
            state_a = 1;
            point_idx = 0;
            bomb_path_flag = 0;
            // 有了新视觉地图，允许后续走本地连炸模式
            if (game_init_flag == 1)
            {
                local_map_ready = 1;
            }
            return 0;
        }

        check_stuck_ret = check_bomb_lock(&stuck_x, &stuck_y);

        ips200_show_int(0, 0, check_stuck_ret, 1);
        if (check_stuck_ret == 1 && lock == 0)
        {
            int out_wall_x = -1, out_wall_y = -1;
            ra++;
            find_unlock_wall(stuck_x, stuck_y, &out_wall_x, &out_wall_y);

            bomb_pair();
            if (ra >= 5)
            {
                lock = 1;
            }
        }

        else if (check_stuck_ret != 1 && lock == 0)
        {
            rb++;
            bomb_pair();
            if (rb >= 5)
            {
                lock = 1;
            }
        }
    }

        if (nearlist_box_index_flag == 0)
        {
            // find_unlock_wall → make_push_bomb_path 会污染真实地图，
            // 必须在此处从 bomb_map 恢复，否则 nearlist_bomb_get 会找到错误的炸弹
            memcpy(map_packet.map_data.map, bomb_map, sizeof(map_packet.map_data.map));
            nearlist_bomb_get();
            nearlist_box_index_flag = 1;

            // ips200_show_int(0,180,nearlist_bomb_x,2);
            // ips200_show_int(30,180,nearlist_bomb_x,2);
        }

        if (mv_packet.data.car_x != 0 &&
            mv_packet.data.car_y != 0)
        {
            if (nearlist_bomb_x == unlock_bomb_x &&
                nearlist_bomb_y == unlock_bomb_y)
            {
                // 解锁炸弹

                bomb_run_state = BOMB_GET_STUCK_PATH;
            }

            else if (nearlist_bomb_x == stuck_x &&
                     nearlist_bomb_y == stuck_y)
            {

                // 死锁炸弹
                if (check_stuck_ret == 1)
                {
                    bomb_run_state = BOMB_GET_STUCK_PATH;
                }

                else
                {
                    bomb_run_state = BOMB_GET_FREE_PATH;
                }
            }

            else
            {
                // 普通炸弹
                bomb_run_state = BOMB_GET_FREE_PATH;
            }
        }

        else
        {
            return 0;
        }

        return 0;

    case BOMB_GET_STUCK_PATH:
    {
        state_a = 2;

        if (lock_flag == 0 && make_unlock_path())
        {
            lock_flag = 1;
            bomb_path_flag = 1;

            // 保存调试信息：STUCK路径 炸弹坐标 + 炸点坐标
            debug_path_type = 1;
            debug_bomb_x = nearlist_bomb_x;
            debug_bomb_y = nearlist_bomb_y;

            executed_bomb_x = nearlist_bomb_x;
            executed_bomb_y = nearlist_bomb_y;
            if (bomb_path_len > 0)
            {
                debug_explode_x = bomb_path[bomb_path_len - 1].ax;
                debug_explode_y = bomb_path[bomb_path_len - 1].ay;
            }

            // 路径规划完成后立即显示，保证是最新值
            // ips200_show_int(0, 300, debug_path_type, 1);
            // ips200_show_int(20, 300, debug_bomb_x, 2);
            // ips200_show_int(45, 300, debug_bomb_y, 2);
            // ips200_show_int(70, 300, debug_explode_x, 2);
            // ips200_show_int(95, 300, debug_explode_y, 2);

            bomb_run_state = BOMB_TRANSFER_PATH;
        }

        else
        {
            bomb_run_state = BOMB_PATH_INIT;
        }

        return 0;
    }

    case BOMB_GET_FREE_PATH:
        state_a = 3;
        int ok = make_free_bomb_path();

        // ips200_show_string(0, 260, "FREE");
        // ips200_show_int(0,280,ok,3);
        // ips200_show_int(60, 280, bomb_path_len, 3);
        // ips200_show_float(0,230,nearlist_bomb_x,3,2);
        // ips200_show_float(30,230,nearlist_bomb_y,3,2);

        if (free_flag == 0 && ok)
        {
            free_flag = 1;
            bomb_path_flag = 2;

            // 保存调试信息：FREE路径 炸弹坐标 + 炸点坐标
            debug_path_type = 2;
            debug_bomb_x = nearlist_bomb_x;
            debug_bomb_y = nearlist_bomb_y;

            executed_bomb_x = nearlist_bomb_x;
            executed_bomb_y = nearlist_bomb_y;
            if (bomb_path_len > 0)
            {
                debug_explode_x = bomb_path[bomb_path_len - 1].ax;
                debug_explode_y = bomb_path[bomb_path_len - 1].ay;
            }

            // 路径规划完成后立即显示，保证是最新值
            // ips200_show_int(0, 300, debug_path_type, 1);
            // ips200_show_int(20, 300, debug_bomb_x, 2);
            // ips200_show_int(45, 300, debug_bomb_y, 2);
            // ips200_show_int(70, 300, debug_explode_x, 2);
            // ips200_show_int(95, 300, debug_explode_y, 2);

            bomb_run_state = BOMB_TRANSFER_PATH;
        }

        else
        {
            bomb_run_state = BOMB_PATH_INIT;
        }

        return 0;

    case BOMB_TRANSFER_PATH:
        state_a = 4;
        {
            memset(path, 0, sizeof(path));

            point_idx = 0;
            // 1. 复制 car_to_bomb_path（小车 → 炸弹后面）
            for (int i = 0; i < car_to_bomb_path_len && point_idx < 30; i++)
            {
                path[point_idx][x] = (float)(car_to_bomb_path[i].ax + 0.5f);
                path[point_idx][y] = (float)(car_to_bomb_path[i].ay + 0.5f);
                point_idx++;
            }

            // 2. 复制 push_bomb_path（推炸弹到墙）
            for (int i = 0; i < push_bomb_len && point_idx < 50; i++)
            {
                if (push_bomb_path[i].ax == 0 && push_bomb_path[i].ay == 0)
                    continue;
                path[point_idx][x] = (float)(push_bomb_path[i].ax + 0.5f);
                path[point_idx][y] = (float)(push_bomb_path[i].ay + 0.5f);
                point_idx++;
            }

            final_path_len = point_idx; // 最终路径长度
            turn_final_path();
            turn_final_path();
            point_idx = 0;
            bomb_run_state = BOMB_FINISH;
            return 0;
        }

    case BOMB_FINISH:
    {
        state_a = 5;
        car_run_path();
        can_run_flag = 1;
        ips200_show_int(0, 300, debug_path_type, 1);
        ips200_show_int(20, 300, debug_bomb_x, 2);
        ips200_show_int(45, 300, debug_bomb_y, 2);
        ips200_show_int(70, 300, debug_explode_x, 2);
        ips200_show_int(95, 300, debug_explode_y, 2);
        if (bomb_num < last_bomb_num)
        {
            wait_ms(800);
            if (wait_ok_flag)
            {
                wait_ok_flag = 0;
                if (bomb_num == 0)
                {
                    bomb_finish_flag = 1;
                    return 0;
                }
                static int commit_backup[3][3];

                car_speed = 0;
                can_run_flag = 0;
                // 1. 把本轮执行的炸弹从“规划地图”中删掉
                if (executed_bomb_x >= 0 && executed_bomb_x < COLS &&
                    executed_bomb_y >= 0 && executed_bomb_y < ROWS)
                {
                    bomb_map[executed_bomb_y][executed_bomb_x] = 0;
                }

                // 2. 在炸点对规划地图施加真实爆炸效果（九宫格炸墙）
                bomb_apply(debug_explode_x,
                           debug_explode_y,
                           commit_backup,
                           bomb_map);

                last_bomb_num = bomb_num;

                bomb_path_len = 0;
                car_to_bomb_path_len = 0;
                push_bomb_len = 0;

                memset(path, 0, sizeof(path));
                memset(bomb_path, 0, sizeof(bomb_path));
                memset(car_to_bomb_path, 0, sizeof(car_to_bomb_path));
                memset(push_bomb_path, 0, sizeof(push_bomb_path));

                lock_flag = 0;
                free_flag = 0;
                nearlist_box_index_flag = 0;

                executed_bomb_x = -1;
                executed_bomb_y = -1;

                local_map_ready = 1;
                bomb_run_state = BOMB_PATH_INIT;
            }
        }
    }
    }
    return 0;
}