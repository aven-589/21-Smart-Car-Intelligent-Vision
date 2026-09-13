// #include "model.h"
// #include "math.h"
// extern OpenMV_Union_t mv_packet;
// extern OpenMV_Map_t map_packet;

// int target_idx = 0;                                                  // 最近的索引，假设最开始是第一个
// volatile float model_four_point[4][2] = {0};                         // 存储目标点上下左右四个点的坐标，分别是上下左右
// int destination_idx[BOX_DESTINATION_NUM_MAX] = {-1, -1, -1, -1, -1}; // 存储已经找到的目的地索引，最多3个目的地
// uint8_t have_found_destination = 0;                                  // 已经找过的目的地数量
// volatile float target_model[2];                                      // 目标模型的目的地坐标
// int destination_already_found = 0;                                   // 已经找到过了
// int no_need_to_recognize_destination_model = 0;                      // 是否需要识别目的地模型
// uint8_t model_back_or_go_flag = 0;
// void model_get_target(void) // 构建best序列存储箱子，best先后由bfs_distance()决定
// {
//     if (destination_num == 0)
//         return;          // 没有箱子也没有目的地，就直接返回
//     int best_dist = 999; // 初始化一个很大的数，当作最小值，用来比较距离
//     float car_x = mv_packet.data.car_x;
//     float car_y = mv_packet.data.car_y;

//     for (int k = 0; k < initial_destination_num; k++)
//     {
//         destination_already_found = 0;
//         for (int j = 0; j < initial_destination_num; j++)
//         {
//             if (destination_idx[j] == k) // 如果这个目的地已经找到过了，就跳过
//             {
//                 destination_already_found = 1;
//                 break;
//             }
//         }
//         if (destination_already_found == 1)
//         {
//             continue;
//         }
//         float destination_x = mv_packet.data.destination[k][x];
//         float destination_y = mv_packet.data.destination[k][y];

//         if (destination_x == 0 && destination_y == 0)
//             continue;
//         // float destination_d = fabsf(car_x - destination_x) + fabsf(car_y - destination_y);
//         float destination_d = astar_distance(car_x, car_y, destination_x, destination_y);
//         if (destination_d < best_dist)
//         {
//             best_dist = destination_d; // 更新最近距离
//             target_idx = k;            // 更新最近索引
//         }
//     }
//     if (best_dist == 999)
//         return; // 没找到任何可达目的地

//     target_model[x] = mv_packet.data.destination[target_idx][x];
//     target_model[y] = mv_packet.data.destination[target_idx][y];
//     for (int i = 0; i < 4; i++) // 得到目标点的四个点的坐标，分别是上下左右
//     {
//         if (i == LEFT)
//         {
//             model_four_point[i][x] = target_model[x] - 1;
//             model_four_point[i][y] = target_model[y];
//         }

//         else if (i == UP)
//         {
//             model_four_point[i][x] = target_model[x];
//             model_four_point[i][y] = target_model[y] - 1;
//         }
//         else if (i == RIGHT)
//         {
//             model_four_point[i][x] = target_model[x] + 1;
//             model_four_point[i][y] = target_model[y];
//         }
//         else if (i == DOWN)
//         {
//             model_four_point[i][x] = target_model[x];
//             model_four_point[i][y] = target_model[y] + 1;
//         }
//     }
// }

// Model_t model_state = MODEL_FIND_TARGET; // 识别模型状态机初始状态为寻找目标点
// aPoint tempt_path[APATH_NODE];           // 临时路径点数组
// int tempt_path_len = 0;                  // 临时路径长度
// uint8_t model_idx = 0;                   // 识别模型当前目标点索引
// volatile uint8_t uart1_flag = 0;         // 为1时就开始接收摄像头返回的数据
// volatile uint8_t uart1_rx_flag = 0;      // 为1时，说明已经接收到了摄像头返回的数据
// uint8_t have_recognized_nearest_box = 0; // 是否已经识别过最近的箱子了
// extern int a;
// extern int b;
// extern int box_become_wall_flag;
// uint8_t model_destination_num = 0;
// uint8_t model_destination_num_update_falg = 1;
// uint8_t recognizing_number_flag = 0, recognizing_person_flag = 0;
// int test_cnt_for_model = 0;
// int test_cnt_for_model_go_flag = 0;
// int test_cnt_for_model_back_flag = 0;
// int model_recognization() // 识别模型主函数状态机，根据状态机的不同状态执行不同的功能
// {

//     if (level <= 1) // 不需要识别模型，直接退出
//     {
//         return 0;
//     }
//     static uint8_t pleasant_sheep_count = 0;         // 模型为喜羊羊的次数
//     static uint8_t back_to_recognize_again_flag = 0; // 是否需要返回识别模型
//     if (map_rx_flag && initial_destination_num > 0 && initial_destination_num < 6)
//     {

//         float dx, dy = 0;
//         if (initial_destination_num - have_found_destination == 1) // 如果已经找到的数量大于等于总数量，说明已经把目标点都找完了
//         {
//             no_need_to_recognize_destination_model = 1; // 识别完了目的地模型
//         }
//         switch (model_state)
//         {
//         case MODEL_FIND_TARGET:
//             if (no_need_to_recognize_destination_model == 0)
//             {
//                 model_get_target(); // 获取目标目的地坐标
//             }
//             else if (no_need_to_recognize_destination_model == 1 && have_recognized_nearest_box == 0)
//             {
//                 a = 2;
//                 b = 2;
//                 if (box_num == 1)
//                 {
//                     target_model[x] = mv_packet.data.box[0][x];
//                     target_model[y] = mv_packet.data.box[0][y];
//                     target_box[x] = mv_packet.data.box[0][x];
//                     target_box[y] = mv_packet.data.box[0][y];
//                     model_state = MODEL_FINISH;
//                     return 0;
//                 }
//                 else
//                 {
//                     get_map_message();
//                     get_target_box(); // 获取目标箱子坐标
//                     target_model[x] = target_box[x];
//                     target_model[y] = target_box[y];
//                     for (int i = 0; i < 4; i++) // 得到目标点的四个点的坐标，分别是上下左右
//                     {
//                         if (i == LEFT)
//                         {
//                             model_four_point[i][x] = target_model[x] - 1;
//                             model_four_point[i][y] = target_model[y];
//                         }

//                         else if (i == UP)
//                         {
//                             model_four_point[i][x] = target_model[x];
//                             model_four_point[i][y] = target_model[y] - 1;
//                         }
//                         else if (i == RIGHT)
//                         {
//                             model_four_point[i][x] = target_model[x] + 1;
//                             model_four_point[i][y] = target_model[y];
//                         }
//                         else if (i == DOWN)
//                         {
//                             model_four_point[i][x] = target_model[x];
//                             model_four_point[i][y] = target_model[y] + 1;
//                         }
//                     }
//                 }
//             }

//             if (target_model[x] > 1 && target_model[y] > 1 && target_model[x] < 11 && target_model[y] < 15) // 如果找到了箱子,坐标在合理范围内，就转移到下一个状态，开始移动到箱子
//             {
//                 a = 3;
//                 point_idx = 0;                // 重置路径索引，从第一个点开始走
//                 car_x = mv_packet.data.car_x; // 获取小车初始坐标
//                 car_y = mv_packet.data.car_y; // 获取小车初始坐标
//                 float test_distance = 999.9;
//                 for (int i = 0; i < 4; i++)
//                 {
//                     if (map_packet.map_data.map[(int)model_four_point[i][y]][(int)model_four_point[i][x]] != 0 && map_packet.map_data.map[(int)model_four_point[i][y]][(int)model_four_point[i][x]] != 4)
//                     // 如果这个点不是路或者目的地，说明这个点是炸弹或者箱子，不能直接走过去
//                     {
//                         continue; // 这个点不是路或者目的地，跳过
//                     }
//                     if (i == 0)
//                     {
//                         model_idx = LEFT;
//                         break;
//                     }
//                     float distance = astar_distance(car_x, car_y, model_four_point[i][x], model_four_point[i][y]);
//                     if (distance < test_distance)
//                     {
//                         test_distance = distance;
//                         model_idx = i;
//                     }
//                     else
//                     {
//                         continue;
//                     }
//                 }
//                 //init_game_map();
//                 if (manhattan_path(car_x, car_y, model_four_point[model_idx][x], model_four_point[model_idx][y]) == 1 || manhattan_path(car_x, car_y, model_four_point[model_idx][x], model_four_point[model_idx][y]) == 2 || manhattan_path(car_x, car_y, model_four_point[model_idx][x], model_four_point[model_idx][y]) == 3)
//                 // 得到车到目标点的曼哈顿路径，顺便判断能不能直接走过去
//                 {
//                     memset(path, 0, sizeof(path));
//                     path[0][x] = car_x;                          // 路径的第一个点是小车当前位置
//                     path[0][y] = car_y;                          // 路径的第一个点
//                     path[1][x] = manhattan_turn[x];              // 路径的第二个点是曼哈顿拐点的坐标
//                     path[1][y] = manhattan_turn[y];              // 路径的第二个点是曼哈顿拐点的坐标
//                     path[2][x] = model_four_point[model_idx][x]; // 路径的第三个点是目标点对应方位的坐标
//                     path[2][y] = model_four_point[model_idx][y]; // 路径的第三个点是目标点对应方位的坐标
//                     model_state = MODEL_TO_TARGET;               // 状态转移到移动到目标点
//                     return 0;
//                 }

//                 else
//                 {
//                     a = 3;
//                     memset(acar_to_box_path, 0, sizeof(acar_to_box_path));
//                     memset(apush_path, 0, sizeof(apush_path));
//                     memset(path, 0, sizeof(path));
//                     //init_game_map();
//                     point_idx = 0; // 重置路径索引，从第一个点开始走
//                     start_x = car_x;
//                     start_y = car_y;
//                     end_x = model_four_point[model_idx][x];
//                     end_y = model_four_point[model_idx][y];

//                     if (AStar() != 1) // 计算BFS路径
//                     {
//                         break;
//                         ; // 如果BFS失败了，说明这个点不可达，跳过
//                     }

//                     get_astar_path_to(tempt_path, &tempt_path_len);   // 获取BFS路径到目标点的路径
//                     get_astar_turn_path(tempt_path, &tempt_path_len); // 对BFS路径进行转向处理，得到小车可行驶的路径
//                     // smooth_turn_path(tempt_path, &tempt_path_len);
//                     // smooth_turn_path(tempt_path, &tempt_path_len);
//                     int x_pos = 0;
//                     int y_pos = 0;
//                     int count = 0;

//                     for (int i = 0; i < tempt_path_len; i++)
//                     {
//                         int x1 = tempt_path[i].ax;
//                         int y1 = tempt_path[i].ay;

//                         x_pos += 20;
//                         count++;

//                         // 每行 9 个
//                         if (count == 9)
//                         {
//                             count = 0;
//                             x_pos = 0;

//                             // 换行：间距 30
//                             y_pos += 40;

//                             if (y_pos > 280)
//                                 break;
//                         }
//                     }

//                     for (int i = 0; i < tempt_path_len && point_idx < 30; i++)
//                     {
//                         path[point_idx][x] = (float)tempt_path[i].ax + 0.5f;
//                         path[point_idx][y] = (float)tempt_path[i].ay + 0.5f;
//                         point_idx++;
//                     }
//                     point_idx = 0;                 // 重置路径索引，从第一个点开始走
//                     model_state = MODEL_TO_TARGET; // 状态转移到移动到目标点
//                     return 0;
//                     // 用bfs算一下看看
//                     //}
//                 }
//             }
//             break;
//         case MODEL_TO_TARGET: // 向目标点移动
//             a = 4;
//             b = 5;
//             can_run_flag = 1;
//             if (path[point_idx + 1][x] == 0 && path[point_idx + 1][y] == 0 && point_idx != 0)
//             // 已经走到了最后一个点，此时索引加1后，path[point_idx]就会是0了，如果索引不是0了，说明确实是走完了路径
//             {
//                 can_run_flag = 0;
//                 car_speed = 0;                          // 停止小车
//                 model_state = MODEL_VISION_CALIBRATION; // 状态转移到视觉校准
//                 return 0;
//             }
//             else
//             {
//                 car_run_path(); // 小车走path数组的每一个点
//             }
//             break;
//         case MODEL_VISION_CALIBRATION: // 视觉校准
//             if (model_idx == LEFT)     // 如果是左方,不需要转向
//             {
//                 target_angle = 0;
//             }
//             else if (model_idx == UP) // 如果是上方,需要顺时针转90度
//             {
//                 target_angle = -45;
//             }
//             else if (model_idx == RIGHT) // 如果是右方,需要顺时针转180度
//             {
//                 target_angle = 90;
//             }
//             else if (model_idx == DOWN) // 如果是下方,需要顺时针转90度
//             {
//                 target_angle = 45;
//             }

//             if (model_idx == LEFT)
//             {
//                 wait_ms(300); // 等待300ms，确保小车完全停下来
//                 if (wait_ok_flag)
//                 {
//                     wait_ok_flag = 0;          // 重置等待完成标志位，为下一次使用做准备
//                     model_state = MODEL_JUDGE; // 就转移到识别状态
//                 }
//             }
//             else
//             {
//                 wait_ms(400); // 等待400ms，确保小车完全停下来
//                 if (wait_ok_flag)
//                 {
//                     wait_ok_flag = 0;          // 重置等待完成标志位，为下一次使用做准备
//                     model_state = MODEL_JUDGE; // 就转移到识别状态
//                 }
//             }

//             break;

//         case MODEL_JUDGE: // 开始识别，看是啥人物或者数字
//             a = 6;
//             b++;
//             if (test_cnt_for_model_back_flag || test_cnt_for_model_go_flag)
//             {
//                 model_state = MODEL_GO_BACK_OR_GO;
//                 return 0;
//             }
//             if (destination_model[target_idx] == -1 && no_need_to_recognize_destination_model != 1) // 识别数字
//             {
//                 uart1_flag = 1;                  // 打开uart1_flag，开始接收摄像头返回的数据
//                 LPUART_WriteByte(LPUART1, 0x02); // 发送信号，告诉摄像头可以开始发送数据了
//             }
//             else if (no_need_to_recognize_destination_model == 1 && box_model == -1) // 识别人物
//             {
//                 if(recognizing_person_flag == 0)
//                 {
//                     model_state = MODEL_GO_BACK_OR_GO;
//                 }
//                 else
//                 {
//                     wait_ms(100);
//                     if(wait_ok_flag)
//                     {
//                         uart1_flag = 1;                  // 打开uart1_flag，开始接收摄像头返回的数据
//                         LPUART_WriteByte(LPUART1, 0x01); // 发送信号，告诉摄像头可以开始发送数据了
//                     }
//                 }
//             }
//             else // 识别完成，状态转移到结束
//             {
//                 wait_ok_flag=0;
//                 recognizing_person_flag = 0;
//                 test_cnt_for_model = 0;
//                 test_cnt_for_model_go_flag = 0;
//                 test_cnt_for_model_back_flag = 0;
//                 target_angle = 0;  // 重置目标角度，准备找下一个目标点
//                 uart1_rx_flag = 0; // 接收到了摄像头返回的数据，重置标志位，为下一次接收做准备
//                 uart1_flag = 0;    // 重置已经找到标志位，为下一次使用做准备
//                 if (model_idx != LEFT)
//                 {
//                     wait_ms(400); // 等待小车转回初始角度
//                     if (wait_ok_flag)
//                     {
//                         wait_ok_flag = 0;           // 重置等待完成标志位，为下一次使用做准备
//                         model_state = MODEL_FINISH; // 识别完成，状态转移到结束
//                     }
//                 }
//                 else
//                 {
//                     model_state = MODEL_FINISH; // 识别完成，状态转移到结束
//                 }
//             }
//             break;

//         case MODEL_GO_BACK_OR_GO:
//             if(recognizing_person_flag == 0&&no_need_to_recognize_destination_model==1)
//             {
//                 if(encoder_move_distance(0.1, back) == 1)
//                 {
//                     recognizing_person_flag = 1;
//                     model_state = MODEL_JUDGE;
//                 }
//                 return 0;
//             }
//             if (model_back_or_go_flag == 0)
//             {
//                 if (test_cnt_for_model_go_flag == 1)
//                 {
//                     if (encoder_move_distance(0.1, go) == 1)
//                     {
//                         model_back_or_go_flag = 1;
//                     }
//                 }
//                 else if (test_cnt_for_model_back_flag == 1)
//                 {
//                     if (encoder_move_distance(0.1, back) == 1)
//                     {
//                         model_back_or_go_flag = 1;
//                     }
//                 }
//             }
//             else if (model_back_or_go_flag == 1)
//             {
//                 model_back_or_go_flag = 0;
//                 test_cnt_for_model = 0;
//                 test_cnt_for_model_go_flag = 0;
//                 test_cnt_for_model_back_flag = 0;
//                 model_state = MODEL_JUDGE; // 就转移到识别状态
//             }
//             break;
//         case MODEL_FINISH: // 识别完成，开始识别下一个目标点
//             recognizing_number_flag = 0;
//             recognizing_person_flag = 0;
//             a = 7;
//             if (no_need_to_recognize_destination_model == 0)
//             {
//                 destination_idx[have_found_destination] = target_idx; // 记录最近的目的地索引
//                 have_found_destination++;                             // 更新已找到目的地的数量
//                 model_state = MODEL_FIND_TARGET;                      // 状态转移到找目标点
//                 break;
//             }
//             else if (no_need_to_recognize_destination_model == 1 && have_recognized_nearest_box == 0)
//             {
//                 model_state = MODEL_FIND_TARGET; // 状态转移到找目标点
//                 have_recognized_nearest_box = 1;
//                 break;
//             }
//             else
//             {
//                 model_state = MODEL_FIND_TARGET; // 状态转移到找目标点
//                 break;
//             }
//         }
//     }
//     return 0;
// }

// uint8_t together_destination_and_can_push_flag = 0; // 箱子是否在一起
// float smart_target_destination[2][2] = {0};         // 俩目标点被选中的那个
// float the_other_destination[2] = {0, 0};            // 俩目标点中的另一个
// uint8_t destination_same_x_flag = 0, destination_same_y_flag = 0;
// uint8_t single_destination_flag = 0;
// int judge_all_destination_is_same_x_or_y() // 判断所有目的地是否在同一列或同一行
// {
//     if (destination_num > 2)
//     {

//         if ((fabs(initial_destination_pos[0][x] - initial_destination_pos[destination_num - 1][x]) < 0.5) && (fabs(initial_destination_pos[1][x] - initial_destination_pos[destination_num - 1][x]) < 0.5) && (fabs(initial_destination_pos[0][y] - initial_destination_pos[destination_num - 1][y]) < 1.5) && (fabs(initial_destination_pos[1][y] - initial_destination_pos[destination_num - 1][y]) < 1.5))
//         { // x坐标是一样的，二/三 个目的地在同一列

//             if (map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 2 && map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 2)
//             { // 第一个目的地上面不可走，最后一个目的地下面不可走，两不可
//                 together_destination_and_can_push_flag = 0;
//                 destination_same_x_flag = 0;
//                 destination_same_y_flag = 0;
//                 return 0;
//             }

//             else if ((map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 0 || map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 4) && (map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 0 || map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 4))
//             { // 第一个目的地上面可走，最后一个目的地下面可走，两可
//                 smart_target_destination[0][x] = initial_destination_pos[0][x];
//                 smart_target_destination[0][y] = initial_destination_pos[0][y];
//                 smart_target_destination[1][x] = initial_destination_pos[destination_num - 1][x];
//                 smart_target_destination[1][y] = initial_destination_pos[destination_num - 1][y];
//                 destination_same_x_flag = 1;
//                 together_destination_and_can_push_flag = 1;
//             }

//             else if ((map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 0 || map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 4))
//             { // 最后一个目的地下面可走，下可
//                 target_destination[x] = initial_destination_pos[destination_num - 1][x];
//                 target_destination[y] = initial_destination_pos[destination_num - 1][y];
//                 the_other_destination[x] = initial_destination_pos[0][x];
//                 the_other_destination[y] = initial_destination_pos[0][y];
//                 single_destination_flag = 1;
//                 destination_same_x_flag = 1;
//                 together_destination_and_can_push_flag = 1;
//             }

//             else if ((map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 0 || map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 4))
//             { // 第一个目的地上面可走，上可
//                 target_destination[x] = initial_destination_pos[0][x];
//                 target_destination[y] = initial_destination_pos[0][y];
//                 the_other_destination[x] = initial_destination_pos[destination_num - 1][x];
//                 the_other_destination[y] = initial_destination_pos[destination_num - 1][y];
//                 single_destination_flag = 1;
//                 destination_same_x_flag = 1;
//                 together_destination_and_can_push_flag = 1;
//             }

//             else
//             {
//                 together_destination_and_can_push_flag = 0;
//                 destination_same_x_flag = 0;
//                 destination_same_y_flag = 0;
//                 return 0;
//             }
//         }
//         else if ((fabs(mv_packet.data.destination[0][y] - mv_packet.data.destination[destination_num - 1][y]) < 0.5) && (fabs(mv_packet.data.destination[1][y] - mv_packet.data.destination[destination_num - 1][y]) < 0.5) && (fabs(initial_destination_pos[0][x] - initial_destination_pos[destination_num - 1][x]) < 1.5) && (fabs(initial_destination_pos[1][x] - initial_destination_pos[destination_num - 1][x]) < 1.5))
//         { // y坐标是一样的，三个目的地在同一行
//             if (map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 2 && map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 2)
//             { // 第一个目的地左面不可走，最后一个目的地右面不可走，两不可
//                 together_destination_and_can_push_flag = 0;
//                 destination_same_y_flag = 0;
//                 destination_same_x_flag = 0;
//                 return 0;
//             }

//             else if ((map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 0 || map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 4) && (map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 0 || map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 4))
//             { // 第一个目的地左面可走，最后一个目的地右面可走，两可
//                 smart_target_destination[0][x] = initial_destination_pos[0][x];
//                 smart_target_destination[0][y] = initial_destination_pos[0][y];
//                 smart_target_destination[1][x] = initial_destination_pos[destination_num - 1][x];
//                 smart_target_destination[1][y] = initial_destination_pos[destination_num - 1][y];
//                 destination_same_y_flag = 1;
//                 together_destination_and_can_push_flag = 1;
//             }

//             else if ((map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 0 || map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 4))
//             { // 最后一个目的地右面可走，右可
//                 target_destination[x] = initial_destination_pos[destination_num - 1][x];
//                 target_destination[y] = initial_destination_pos[destination_num - 1][y];
//                 the_other_destination[x] = initial_destination_pos[0][x];
//                 the_other_destination[y] = initial_destination_pos[0][y];
//                 single_destination_flag = 1;
//                 destination_same_y_flag = 1;
//                 together_destination_and_can_push_flag = 1;
//             }

//             else if ((map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 0 || map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 4))
//             { // 第一个目的地左面可走，左可
//                 target_destination[x] = initial_destination_pos[0][x];
//                 target_destination[y] = initial_destination_pos[0][y];
//                 the_other_destination[x] = initial_destination_pos[destination_num - 1][x];
//                 the_other_destination[y] = initial_destination_pos[destination_num - 1][y];
//                 single_destination_flag = 1;
//                 destination_same_y_flag = 1;
//                 together_destination_and_can_push_flag = 1;
//             }

//             else
//             {
//                 together_destination_and_can_push_flag = 0;
//                 destination_same_y_flag = 0;
//                 destination_same_x_flag = 0;
//                 return 0;
//             }
//         }

//         else
//         {
//             together_destination_and_can_push_flag = 0;
//             destination_same_y_flag = 0;
//             destination_same_x_flag = 0;
//             return 0;
//         }

//         return 0;
//     }
//     else
//     {
//         together_destination_and_can_push_flag = 0;
//         destination_same_y_flag = 0;
//         destination_same_x_flag = 0;
//         return 0;
//     }
// }

// SmartSolution_t smart_solution_state = SMART_CHOOOSE_DESTINATION; // 初始状态为选择目的地
// int smart_solution_to_combined_destination()
// {
//     static float dx = 0.0, dy = 0.0;
//     if (map_rx_flag == 1 && destination_num > 1)
//     {
//         switch (smart_solution_state)
//         {
//         case SMART_CHOOOSE_DESTINATION:
//             if (together_destination_and_can_push_flag == 1)
//             {
//                 smart_solution_state = SMART_FIND_TARGET_BOX_AND_DESTINATION;
//             }
//             break;
//         case SMART_FIND_TARGET_BOX_AND_DESTINATION:
//             path_flag = 1;
//             get_target_box();
//             if (single_destination_flag == 1)
//             {
//                 smart_solution_state = SMART_RUN_PATH;
//                 break;
//             }
//             if (astar_distance(target_box[x], target_box[y], smart_target_destination[0][x], smart_target_destination[0][y]) >= astar_distance(target_box[x], target_box[y], smart_target_destination[1][x], smart_target_destination[1][y]))
//             {
//                 target_destination[x] = smart_target_destination[1][x];
//                 target_destination[y] = smart_target_destination[1][y];
//                 the_other_destination[x] = smart_target_destination[0][x];
//                 the_other_destination[y] = smart_target_destination[0][y];
//             }
//             else
//             {
//                 target_destination[x] = smart_target_destination[0][x];
//                 target_destination[y] = smart_target_destination[0][y];
//                 the_other_destination[x] = smart_target_destination[1][x];
//                 the_other_destination[y] = smart_target_destination[1][y];
//             }
//             smart_solution_state = SMART_RUN_PATH;
//             break;
//         case SMART_RUN_PATH:
//             get_manhattan_path();
//             get_bfs_path();
//             if (path[point_idx + 1][x] == 0 && path[point_idx + 1][y] == 0 && point_idx != 0)
//             {
//                 wait_ms(1000);
//                 if (wait_ok_flag == 1)
//                 {
//                     wait_ok_flag = 0;
//                     if (box_num < last_box_num)
//                     {
//                         smart_solution_state = SMART_FINISH;
//                         break;
//                     }
//                     else
//                     {
//                         point_idx = 0;
//                         memset(path, 0, sizeof(path));
//                         smart_solution_state = SMART_TEST_ONE_BY_ONE;
//                         dx = the_other_destination[x] - target_destination[x];
//                         dy = the_other_destination[y] - target_destination[y];
//                         break;
//                     }
//                 }
//             }
//             break;
//         case SMART_TEST_ONE_BY_ONE:
//             if (destination_same_y_flag == 1) // 目的地在同一行
//             {
//                 if (fabs(mv_packet.data.car_x - target_destination[x]) > 0.8)
//                 { // 小车跟目的地同一行
//                     path[0][x] = mv_packet.data.car_x;
//                     path[0][y] = mv_packet.data.car_y;
//                     path[1][x] = the_other_destination[x];
//                     path[1][y] = the_other_destination[y];
//                     smart_solution_state = SMART_RUN_TEST_PATH;
//                     break;
//                 }
//                 else // 那就是在目的地上面或者下面
//                 {

//                     if (map_packet.map_data.map[(int)target_destination[y]][(int)target_destination[x] + 1] == 4)
//                     { // 目标目的地右边还是目的地
//                         path[0][x] = mv_packet.data.car_x;
//                         path[0][y] = mv_packet.data.car_y;
//                         path[1][x] = mv_packet.data.car_x - 1; // 小车向左移动一格
//                         path[1][y] = mv_packet.data.car_y;
//                         if (mv_packet.data.car_y > target_destination[y]) // 小车在目的地上面
//                         {
//                             path[2][x] = mv_packet.data.car_x - 1; // 再向下一格
//                             path[2][y] = mv_packet.data.car_y + 1;
//                         }
//                         else // 小车在目的下面
//                         {
//                             path[2][x] = mv_packet.data.car_x - 1; // 再向上一格
//                             path[2][y] = mv_packet.data.car_y - 1;
//                         }
//                         path[3][x] = the_other_destination[x] - 1; // 横着走到另一个目的地前一格
//                         path[3][y] = the_other_destination[y];
//                         smart_solution_state = SMART_RUN_TEST_PATH;
//                         break;
//                     }
//                     else // 目标目的地左边还是目的地
//                     {
//                         path[0][x] = mv_packet.data.car_x;
//                         path[0][y] = mv_packet.data.car_y;
//                         path[1][x] = mv_packet.data.car_x + 1; // 小车向右移动一格
//                         path[1][y] = mv_packet.data.car_y;
//                         if (mv_packet.data.car_y > target_destination[y]) // 小车在目的地上面
//                         {
//                             path[2][x] = mv_packet.data.car_x + 1; // 再向下一格
//                             path[2][y] = mv_packet.data.car_y + 1;
//                         }
//                         else // 小车在目的下面
//                         {
//                             path[2][x] = mv_packet.data.car_x + 1; // 再向上一格
//                             path[2][y] = mv_packet.data.car_y - 1;
//                         }
//                         path[3][x] = the_other_destination[x] + 1; // 横着走到另一个目的地后一格
//                         path[3][y] = the_other_destination[y];
//                         smart_solution_state = SMART_RUN_TEST_PATH;
//                         break;
//                     }
//                 }
//             }
//             else if (destination_same_x_flag == 1) // 目的地在同一列
//             {
//                 if (fabs(mv_packet.data.car_y - target_destination[y]) > 0.8)
//                 {
//                     path[0][x] = mv_packet.data.car_x;
//                     path[0][y] = mv_packet.data.car_y;
//                     path[1][x] = the_other_destination[x];
//                     path[1][y] = the_other_destination[y];
//                     smart_solution_state = SMART_RUN_TEST_PATH;
//                     break;
//                 }
//                 else // 那就是在目的地左面或者右边
//                 {

//                     if (map_packet.map_data.map[(int)target_destination[y] + 1][(int)target_destination[x]] == 4)
//                     { // 目标目的地下面还是目的地
//                         path[0][x] = mv_packet.data.car_x;
//                         path[0][y] = mv_packet.data.car_y;
//                         path[1][x] = mv_packet.data.car_x; // 小车向上移动一格
//                         path[1][y] = mv_packet.data.car_y - 1;
//                         if (mv_packet.data.car_x > target_destination[x]) // 小车在目的左边
//                         {
//                             path[2][x] = mv_packet.data.car_x + 1; // 再往右移动一格
//                             path[2][y] = mv_packet.data.car_y - 1;
//                         }
//                         else // 小车在目的右边
//                         {
//                             path[2][x] = mv_packet.data.car_x - 1; // 再往左移动一格
//                             path[2][y] = mv_packet.data.car_y - 1;
//                         }
//                         path[3][x] = the_other_destination[x];
//                         path[3][y] = the_other_destination[y] - 1;
//                         smart_solution_state = SMART_RUN_TEST_PATH;
//                         break;
//                     }
//                     else // 目标目的地上面还是目的地
//                     {
//                         path[0][x] = mv_packet.data.car_x;
//                         path[0][y] = mv_packet.data.car_y;
//                         path[1][x] = mv_packet.data.car_x; // 小车向下移动一格
//                         path[1][y] = mv_packet.data.car_y + 1;
//                         if (mv_packet.data.car_x > target_destination[x]) // 小车在目的左边
//                         {
//                             path[2][x] = mv_packet.data.car_x + 1; // 再往右移动一格
//                             path[2][y] = mv_packet.data.car_y + 1;
//                         }
//                         else // 小车在目的右边
//                         {
//                             path[2][x] = mv_packet.data.car_x - 1; // 再往左移动一格
//                             path[2][y] = mv_packet.data.car_y + 1;
//                         }
//                         path[3][x] = the_other_destination[x];
//                         path[3][y] = the_other_destination[y] + 1;
//                         smart_solution_state = SMART_RUN_TEST_PATH;
//                         break;
//                     }
//                 }
//             }
//             break;
//         case SMART_RUN_TEST_PATH:
//             if (path[point_idx + 1][x] == 0 && path[point_idx + 1][y] == 0 && point_idx != 0)
//             {
//                 if (box_num < last_box_num)
//                 {
//                     smart_solution_state = SMART_FINISH;
//                     break;
//                 }
//             }
//             else
//             {
//                 car_run_path();
//             }
//             break;
//         case SMART_FINISH:
//             if (box_num == 0) // 所有箱子都被推到目的地
//             {
//                 together_destination_and_can_push_flag = 0;
//                 destination_same_x_flag = 0;
//                 destination_same_y_flag = 0;

//                 memset(path, 0, sizeof(path));
//                 point_idx = 0;
//                 path_flag = 0;
//                 level++;
//             }
//             smart_solution_state = SMART_FIND_TARGET_BOX_AND_DESTINATION;
//             break;
//         }
//     }
//     return 0;
// }

#include "model.h"
#include "math.h"
extern OpenMV_Union_t mv_packet;
extern OpenMV_Map_t map_packet;

int target_idx = 0;                                                  // 最近的索引，假设最开始是第一个
volatile float model_four_point[4][2] = {0};                         // 存储目标点上下左右四个点的坐标，分别是上下左右
int destination_idx[BOX_DESTINATION_NUM_MAX] = {-1, -1, -1, -1, -1}; // 存储已经找到的目的地索引，最多3个目的地
uint8_t have_found_destination = 0;                                  // 已经找过的目的地数量
volatile float target_model[2];                                      // 目标模型的目的地坐标
int destination_already_found = 0;                                   // 已经找到过了
int no_need_to_recognize_destination_model = 0;                      // 是否需要识别目的地模型
uint8_t model_back_or_go_flag = 0;
void model_get_target(void) // 构建best序列存储箱子，best先后由bfs_distance()决定
{
    if (destination_num == 0)
        return;          // 没有箱子也没有目的地，就直接返回
    int best_dist = 999; // 初始化一个很大的数，当作最小值，用来比较距离
    float car_x = mv_packet.data.car_x;
    float car_y = mv_packet.data.car_y;

    for (int k = 0; k < initial_destination_num; k++)
    {
        destination_already_found = 0;
        for (int j = 0; j < initial_destination_num; j++)
        {
            if (destination_idx[j] == k) // 如果这个目的地已经找到过了，就跳过
            {
                destination_already_found = 1;
                break;
            }
        }
        if (destination_already_found == 1)
        {
            continue;
        }
        float destination_x = mv_packet.data.destination[k][x];
        float destination_y = mv_packet.data.destination[k][y];

        if (destination_x == 0 && destination_y == 0)
            continue;
        // float destination_d = fabsf(car_x - destination_x) + fabsf(car_y - destination_y);
        float destination_d = astar_distance(car_x, car_y, destination_x, destination_y);
        if (destination_d < best_dist)
        {
            best_dist = destination_d; // 更新最近距离
            target_idx = k;            // 更新最近索引
        }
    }
    if (best_dist == 999)
        return; // 没找到任何可达目的地

    target_model[x] = mv_packet.data.destination[target_idx][x];
    target_model[y] = mv_packet.data.destination[target_idx][y];
    for (int i = 0; i < 4; i++) // 得到目标点的四个点的坐标，分别是上下左右
    {
        if (i == LEFT)
        {
            model_four_point[i][x] = target_model[x] - 1;
            model_four_point[i][y] = target_model[y];
        }

        else if (i == UP)
        {
            model_four_point[i][x] = target_model[x];
            model_four_point[i][y] = target_model[y] - 1;
        }
        else if (i == RIGHT)
        {
            model_four_point[i][x] = target_model[x] + 1;
            model_four_point[i][y] = target_model[y];
        }
        else if (i == DOWN)
        {
            model_four_point[i][x] = target_model[x];
            model_four_point[i][y] = target_model[y] + 1;
        }
    }
}

Model_t model_state = MODEL_FIND_TARGET; // 识别模型状态机初始状态为寻找目标点
aPoint tempt_path[APATH_NODE];           // 临时路径点数组
int tempt_path_len = 0;                  // 临时路径长度
uint8_t model_idx = 0;                   // 识别模型当前目标点索引
volatile uint8_t uart1_flag = 0;         // 为1时就开始接收摄像头返回的数据
volatile uint8_t uart1_rx_flag = 0;      // 为1时，说明已经接收到了摄像头返回的数据
uint8_t have_recognized_nearest_box = 0; // 是否已经识别过最近的箱子了
extern int a;
extern int b;
extern int box_become_wall_flag;
uint8_t model_destination_num = 0;
uint8_t model_destination_num_update_falg = 1;
uint8_t recognizing_number_flag = 0, recognizing_person_flag = 0;
int test_cnt_for_model = 0;
int test_cnt_for_model_go_flag = 0;
int test_cnt_for_model_back_flag = 0;
uint8_t go_flag = 0;
float error_distance = 0.1;
int model_recognization() // 识别模型主函数状态机，根据状态机的不同状态执行不同的功能
{

    if (level <= 1) // 不需要识别模型，直接退出
    {
        return 0;
    }
    static uint8_t pleasant_sheep_count = 0;         // 模型为喜羊羊的次数
    static uint8_t back_to_recognize_again_flag = 0; // 是否需要返回识别模型
    if (map_rx_flag && initial_destination_num > 0 && initial_destination_num < 6)
    {

        float dx, dy = 0;
        if (initial_destination_num - have_found_destination == 1) // 如果已经找到的数量大于等于总数量，说明已经把目标点都找完了
        {
            no_need_to_recognize_destination_model = 1; // 识别完了目的地模型
        }
        switch (model_state)
        {
        case MODEL_FIND_TARGET:
            if (no_need_to_recognize_destination_model == 0)
            {
                model_get_target(); // 获取目标目的地坐标
            }
            else if (no_need_to_recognize_destination_model == 1 && have_recognized_nearest_box == 0)
            {
                a = 2;
                b = 2;
                if (box_num == 1)
                {
                    target_model[x] = mv_packet.data.box[0][x];
                    target_model[y] = mv_packet.data.box[0][y];
                    target_box[x] = mv_packet.data.box[0][x];
                    target_box[y] = mv_packet.data.box[0][y];
                    model_state = MODEL_FINISH;
                    return 0;
                }
                else
                {
                    get_map_message();
                    get_target_box(); // 获取目标箱子坐标
                    target_model[x] = target_box[x];
                    target_model[y] = target_box[y];
                    for (int i = 0; i < 4; i++) // 得到目标点的四个点的坐标，分别是上下左右
                    {
                        if (i == LEFT)
                        {
                            model_four_point[i][x] = target_model[x] - 1;
                            model_four_point[i][y] = target_model[y];
                        }

                        else if (i == UP)
                        {
                            model_four_point[i][x] = target_model[x];
                            model_four_point[i][y] = target_model[y] - 1;
                        }
                        else if (i == RIGHT)
                        {
                            model_four_point[i][x] = target_model[x] + 1;
                            model_four_point[i][y] = target_model[y];
                        }
                        else if (i == DOWN)
                        {
                            model_four_point[i][x] = target_model[x];
                            model_four_point[i][y] = target_model[y] + 1;
                        }
                    }
                }
            }

            if (target_model[x] > 1 && target_model[y] > 1 && target_model[x] < 11 && target_model[y] < 15) // 如果找到了箱子,坐标在合理范围内，就转移到下一个状态，开始移动到箱子
            {
                a = 3;
                point_idx = 0;                // 重置路径索引，从第一个点开始走
                car_x = mv_packet.data.car_x; // 获取小车初始坐标
                car_y = mv_packet.data.car_y; // 获取小车初始坐标
                float test_distance = 999.9;
                for (int i = 0; i < 4; i++)
                {
                    if (map_packet.map_data.map[(int)model_four_point[i][y]][(int)model_four_point[i][x]] != 0 && map_packet.map_data.map[(int)model_four_point[i][y]][(int)model_four_point[i][x]] != 4)
                    // 如果这个点不是路或者目的地，说明这个点是炸弹或者箱子，不能直接走过去
                    {
                        continue; // 这个点不是路或者目的地，跳过
                    }
                    if (i == 0)
                    {
                        model_idx = LEFT;
                        break;
                    }
                    float distance = astar_distance(car_x, car_y, model_four_point[i][x], model_four_point[i][y]);
                    if (distance < test_distance)
                    {
                        test_distance = distance;
                        model_idx = i;
                    }
                    else
                    {
                        continue;
                    }
                }
                if (manhattan_path(car_x, car_y, model_four_point[model_idx][x], model_four_point[model_idx][y]) == 1 || manhattan_path(car_x, car_y, model_four_point[model_idx][x], model_four_point[model_idx][y]) == 2 || manhattan_path(car_x, car_y, model_four_point[model_idx][x], model_four_point[model_idx][y]) == 3)
                // 得到车到目标点的曼哈顿路径，顺便判断能不能直接走过去
                {
                    memset(path, 0, sizeof(path));
                    path[0][x] = car_x;                          // 路径的第一个点是小车当前位置
                    path[0][y] = car_y;                          // 路径的第一个点
                    path[1][x] = manhattan_turn[x];              // 路径的第二个点是曼哈顿拐点的坐标
                    path[1][y] = manhattan_turn[y];              // 路径的第二个点是曼哈顿拐点的坐标
                    path[2][x] = model_four_point[model_idx][x]; // 路径的第三个点是目标点对应方位的坐标
                    path[2][y] = model_four_point[model_idx][y]; // 路径的第三个点是目标点对应方位的坐标
                    model_state = MODEL_TO_TARGET;               // 状态转移到移动到目标点
                    return 0;
                }

                else
                {
                    a = 3;
                    memset(acar_to_box_path, 0, sizeof(acar_to_box_path));
                    memset(apush_path, 0, sizeof(apush_path));
                    memset(path, 0, sizeof(path));
                    init_game_map();
                    point_idx = 0; // 重置路径索引，从第一个点开始走
                    start_x = car_x;
                    start_y = car_y;
                    end_x = model_four_point[model_idx][x];
                    end_y = model_four_point[model_idx][y];

                    if (AStar() != 1) // 计算BFS路径
                    {
                        break;
                        ; // 如果BFS失败了，说明这个点不可达，跳过
                    }

                    get_astar_path_to(tempt_path, &tempt_path_len);   // 获取BFS路径到目标点的路径
                    get_astar_turn_path(tempt_path, &tempt_path_len); // 对BFS路径进行转向处理，得到小车可行驶的路径
                    // smooth_turn_path(tempt_path, &tempt_path_len);
                    // smooth_turn_path(tempt_path, &tempt_path_len);
                    int x_pos = 0;
                    int y_pos = 0;
                    int count = 0;

                    for (int i = 0; i < tempt_path_len; i++)
                    {
                        int x1 = tempt_path[i].ax;
                        int y1 = tempt_path[i].ay;

                        x_pos += 20;
                        count++;

                        // 每行 9 个
                        if (count == 9)
                        {
                            count = 0;
                            x_pos = 0;

                            // 换行：间距 30
                            y_pos += 40;

                            if (y_pos > 280)
                                break;
                        }
                    }

                    for (int i = 0; i < tempt_path_len && point_idx < 30; i++)
                    {
                        path[point_idx][x] = (float)tempt_path[i].ax + 0.5f;
                        path[point_idx][y] = (float)tempt_path[i].ay + 0.5f;
                        point_idx++;
                    }
                    point_idx = 0;                 // 重置路径索引，从第一个点开始走
                    model_state = MODEL_TO_TARGET; // 状态转移到移动到目标点
                    return 0;
                    // 用bfs算一下看看
                    //}
                }
            }
            break;
        case MODEL_TO_TARGET: // 向目标点移动
            a = 4;
            b = 5;
            can_run_flag = 1;
            if (path[point_idx + 1][x] == 0 && path[point_idx + 1][y] == 0 && point_idx != 0)
            // 已经走到了最后一个点，此时索引加1后，path[point_idx]就会是0了，如果索引不是0了，说明确实是走完了路径
            {
                can_run_flag = 0;
                car_speed = 0;                          // 停止小车
                model_state = MODEL_VISION_CALIBRATION; // 状态转移到视觉校准
                return 0;
            }
            else
            {
                car_run_path(); // 小车走path数组的每一个点
            }
            break;
        case MODEL_VISION_CALIBRATION: // 视觉校准
            if (model_idx == LEFT)     // 如果是左方,不需要转向
            {
                target_angle = 0;
            }
            else if (model_idx == UP) // 如果是上方,需要顺时针转90度
            {
                target_angle = -45;
            }
            else if (model_idx == RIGHT) // 如果是右方,需要顺时针转180度
            {
                target_angle = 90;
            }
            else if (model_idx == DOWN) // 如果是下方,需要顺时针转90度
            {
                target_angle = 45;
            }

            wait_ms(400); // 等待400ms，确保小车完全停下来
            if (wait_ok_flag)
            {
                wait_ok_flag = 0;          // 重置等待完成标志位，为下一次使用做准备
                model_state = MODEL_JUDGE; // 就转移到识别状态
            }
            break;

        case MODEL_JUDGE: // 开始识别，看是啥人物或者数字
            if (test_cnt_for_model_back_flag || test_cnt_for_model_go_flag)
            {
                model_state = MODEL_GO_BACK_OR_GO;
                return 0;
            }
            if (destination_model[target_idx] == -1 && no_need_to_recognize_destination_model != 1) // 识别数字
            {
                if (recognizing_number_flag == 0)
                {
                    model_state = MODEL_GO_BACK_OR_GO;
                }
                else
                {
                    wait_ms(200);
                    if (wait_ok_flag)
                    {
                        uart1_flag = 1;                  // 打开uart1_flag，开始接收摄像头返回的数据
                        LPUART_WriteByte(LPUART1, 0x02); // 发送信号，告诉摄像头可以开始发送数据了
                    }
                }
            }
            else if (no_need_to_recognize_destination_model == 1 && box_model == -1) // 识别人物
            {
                if (recognizing_person_flag == 0)
                {
                    model_state = MODEL_GO_BACK_OR_GO;
                }
                else
                {
                    wait_ms(200);
                    if (wait_ok_flag)
                    {
                        uart1_flag = 1;                  // 打开uart1_flag，开始接收摄像头返回的数据
                        LPUART_WriteByte(LPUART1, 0x01); // 发送信号，告诉摄像头可以开始发送数据了
                    }
                }
            }
            else // 识别完成，状态转移到结束
            {
                if (go_flag == 0 && destination_model[target_idx] != -1 && no_need_to_recognize_destination_model != 1)
                {
                    if (encoder_move_distance(error_distance, go) == 1)
                    {
                        go_flag = 1;
                        car_speed = 0;
                    }
                }
                else
                {
                    wait_ok_flag = 0;
                    recognizing_person_flag = 0;
                    test_cnt_for_model = 0;
                    test_cnt_for_model_go_flag = 0;
                    test_cnt_for_model_back_flag = 0;
                    target_angle = 0;  // 重置目标角度，准备找下一个目标点
                    uart1_rx_flag = 0; // 接收到了摄像头返回的数据，重置标志位，为下一次接收做准备
                    uart1_flag = 0;    // 重置已经找到标志位，为下一次使用做准备
                    if (model_idx != LEFT)
                    {
                        wait_ms(400); // 等待小车转回初始角度
                        if (wait_ok_flag)
                        {
                            wait_ok_flag = 0;           // 重置等待完成标志位，为下一次使用做准备
                            model_state = MODEL_FINISH; // 识别完成，状态转移到结束
                            go_flag = 0;
                        }
                    }
                    else
                    {
                        model_state = MODEL_FINISH; // 识别完成，状态转移到结束
                        go_flag = 0;
                    }
                }
            }
            break;

        case MODEL_GO_BACK_OR_GO:
            if (recognizing_number_flag == 0 && no_need_to_recognize_destination_model == 0)
            {
                if (encoder_move_distance(0.1, back) == 1)
                {
                    recognizing_number_flag = 1;
                    model_state = MODEL_JUDGE;
                }
                return 0;
            }
            if (recognizing_person_flag == 0 && no_need_to_recognize_destination_model == 1)
            {
                if (encoder_move_distance(0.1, back) == 1)
                {
                    recognizing_person_flag = 1;
                    model_state = MODEL_JUDGE;
                }
                return 0;
            }
            if (model_back_or_go_flag == 0)
            {
                if (test_cnt_for_model_go_flag == 1)
                {
                    if (encoder_move_distance(0.1, go) == 1)
                    {
                        model_back_or_go_flag = 1;
                    }
                }
                else if (test_cnt_for_model_back_flag == 1)
                {
                    if (encoder_move_distance(0.1, back) == 1)
                    {
                        model_back_or_go_flag = 1;
                    }
                }
            }
            else if (model_back_or_go_flag == 1)
            {
                model_back_or_go_flag = 0;
                test_cnt_for_model = 0;
                test_cnt_for_model_go_flag = 0;
                test_cnt_for_model_back_flag = 0;
                model_state = MODEL_JUDGE; // 就转移到识别状态
            }
            break;
        case MODEL_FINISH: // 识别完成，开始识别下一个目标点
            recognizing_number_flag = 0;
            recognizing_person_flag = 0;
            a = 7;
            if (no_need_to_recognize_destination_model == 0)
            {
                destination_idx[have_found_destination] = target_idx; // 记录最近的目的地索引
                have_found_destination++;                             // 更新已找到目的地的数量
                model_state = MODEL_FIND_TARGET;                      // 状态转移到找目标点
                break;
            }
            else if (no_need_to_recognize_destination_model == 1 && have_recognized_nearest_box == 0)
            {
                model_state = MODEL_FIND_TARGET; // 状态转移到找目标点
                have_recognized_nearest_box = 1;
                break;
            }
            else
            {
                model_state = MODEL_FIND_TARGET; // 状态转移到找目标点
                break;
            }
        }
    }
    return 0;
}

uint8_t together_destination_and_can_push_flag = 0; // 箱子是否在一起
float smart_target_destination[2][2] = {0};         // 俩目标点被选中的那个
float the_other_destination[2] = {0, 0};            // 俩目标点中的另一个
uint8_t destination_same_x_flag = 0, destination_same_y_flag = 0;
uint8_t single_destination_flag = 0;
int judge_all_destination_is_same_x_or_y() // 判断所有目的地是否在同一列或同一行
{
    if (destination_num > 2)
    {

        if ((fabs(initial_destination_pos[0][x] - initial_destination_pos[destination_num - 1][x]) < 0.5) && (fabs(initial_destination_pos[1][x] - initial_destination_pos[destination_num - 1][x]) < 0.5) && (fabs(initial_destination_pos[0][y] - initial_destination_pos[destination_num - 1][y]) < 1.5) && (fabs(initial_destination_pos[1][y] - initial_destination_pos[destination_num - 1][y]) < 1.5))
        { // x坐标是一样的，二/三 个目的地在同一列

            if (map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 2 && map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 2)
            { // 第一个目的地上面不可走，最后一个目的地下面不可走，两不可
                together_destination_and_can_push_flag = 0;
                destination_same_x_flag = 0;
                destination_same_y_flag = 0;
                return 0;
            }

            else if ((map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 0 || map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 4) && (map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 0 || map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 4))
            { // 第一个目的地上面可走，最后一个目的地下面可走，两可
                smart_target_destination[0][x] = initial_destination_pos[0][x];
                smart_target_destination[0][y] = initial_destination_pos[0][y];
                smart_target_destination[1][x] = initial_destination_pos[destination_num - 1][x];
                smart_target_destination[1][y] = initial_destination_pos[destination_num - 1][y];
                destination_same_x_flag = 1;
                together_destination_and_can_push_flag = 1;
            }

            else if ((map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 0 || map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y] + 1][(int)initial_destination_pos[destination_num - 1][x]] == 4))
            { // 最后一个目的地下面可走，下可
                target_destination[x] = initial_destination_pos[destination_num - 1][x];
                target_destination[y] = initial_destination_pos[destination_num - 1][y];
                the_other_destination[x] = initial_destination_pos[0][x];
                the_other_destination[y] = initial_destination_pos[0][y];
                single_destination_flag = 1;
                destination_same_x_flag = 1;
                together_destination_and_can_push_flag = 1;
            }

            else if ((map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 0 || map_packet.map_data.map[(int)initial_destination_pos[0][y] - 1][(int)initial_destination_pos[0][x]] == 4))
            { // 第一个目的地上面可走，上可
                target_destination[x] = initial_destination_pos[0][x];
                target_destination[y] = initial_destination_pos[0][y];
                the_other_destination[x] = initial_destination_pos[destination_num - 1][x];
                the_other_destination[y] = initial_destination_pos[destination_num - 1][y];
                single_destination_flag = 1;
                destination_same_x_flag = 1;
                together_destination_and_can_push_flag = 1;
            }

            else
            {
                together_destination_and_can_push_flag = 0;
                destination_same_x_flag = 0;
                destination_same_y_flag = 0;
                return 0;
            }
        }
        else if ((fabs(mv_packet.data.destination[0][y] - mv_packet.data.destination[destination_num - 1][y]) < 0.5) && (fabs(mv_packet.data.destination[1][y] - mv_packet.data.destination[destination_num - 1][y]) < 0.5) && (fabs(initial_destination_pos[0][x] - initial_destination_pos[destination_num - 1][x]) < 1.5) && (fabs(initial_destination_pos[1][x] - initial_destination_pos[destination_num - 1][x]) < 1.5))
        { // y坐标是一样的，三个目的地在同一行
            if (map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 2 && map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 2)
            { // 第一个目的地左面不可走，最后一个目的地右面不可走，两不可
                together_destination_and_can_push_flag = 0;
                destination_same_y_flag = 0;
                destination_same_x_flag = 0;
                return 0;
            }

            else if ((map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 0 || map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 4) && (map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 0 || map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 4))
            { // 第一个目的地左面可走，最后一个目的地右面可走，两可
                smart_target_destination[0][x] = initial_destination_pos[0][x];
                smart_target_destination[0][y] = initial_destination_pos[0][y];
                smart_target_destination[1][x] = initial_destination_pos[destination_num - 1][x];
                smart_target_destination[1][y] = initial_destination_pos[destination_num - 1][y];
                destination_same_y_flag = 1;
                together_destination_and_can_push_flag = 1;
            }

            else if ((map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 0 || map_packet.map_data.map[(int)initial_destination_pos[destination_num - 1][y]][(int)initial_destination_pos[destination_num - 1][x] + 1] == 4))
            { // 最后一个目的地右面可走，右可
                target_destination[x] = initial_destination_pos[destination_num - 1][x];
                target_destination[y] = initial_destination_pos[destination_num - 1][y];
                the_other_destination[x] = initial_destination_pos[0][x];
                the_other_destination[y] = initial_destination_pos[0][y];
                single_destination_flag = 1;
                destination_same_y_flag = 1;
                together_destination_and_can_push_flag = 1;
            }

            else if ((map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 0 || map_packet.map_data.map[(int)initial_destination_pos[0][y]][(int)initial_destination_pos[0][x] - 1] == 4))
            { // 第一个目的地左面可走，左可
                target_destination[x] = initial_destination_pos[0][x];
                target_destination[y] = initial_destination_pos[0][y];
                the_other_destination[x] = initial_destination_pos[destination_num - 1][x];
                the_other_destination[y] = initial_destination_pos[destination_num - 1][y];
                single_destination_flag = 1;
                destination_same_y_flag = 1;
                together_destination_and_can_push_flag = 1;
            }

            else
            {
                together_destination_and_can_push_flag = 0;
                destination_same_y_flag = 0;
                destination_same_x_flag = 0;
                return 0;
            }
        }

        else
        {
            together_destination_and_can_push_flag = 0;
            destination_same_y_flag = 0;
            destination_same_x_flag = 0;
            return 0;
        }

        return 0;
    }
    else
    {
        together_destination_and_can_push_flag = 0;
        destination_same_y_flag = 0;
        destination_same_x_flag = 0;
        return 0;
    }
}

SmartSolution_t smart_solution_state = SMART_CHOOOSE_DESTINATION; // 初始状态为选择目的地
int smart_solution_to_combined_destination()
{
    static float dx = 0.0, dy = 0.0;
    if (map_rx_flag == 1 && destination_num > 1)
    {
        switch (smart_solution_state)
        {
        case SMART_CHOOOSE_DESTINATION:
            if (together_destination_and_can_push_flag == 1)
            {
                smart_solution_state = SMART_FIND_TARGET_BOX_AND_DESTINATION;
            }
            break;
        case SMART_FIND_TARGET_BOX_AND_DESTINATION:
            path_flag = 1;
            get_target_box();
            if (single_destination_flag == 1)
            {
                smart_solution_state = SMART_RUN_PATH;
                break;
            }
            if (astar_distance(target_box[x], target_box[y], smart_target_destination[0][x], smart_target_destination[0][y]) >= astar_distance(target_box[x], target_box[y], smart_target_destination[1][x], smart_target_destination[1][y]))
            {
                target_destination[x] = smart_target_destination[1][x];
                target_destination[y] = smart_target_destination[1][y];
                the_other_destination[x] = smart_target_destination[0][x];
                the_other_destination[y] = smart_target_destination[0][y];
            }
            else
            {
                target_destination[x] = smart_target_destination[0][x];
                target_destination[y] = smart_target_destination[0][y];
                the_other_destination[x] = smart_target_destination[1][x];
                the_other_destination[y] = smart_target_destination[1][y];
            }
            smart_solution_state = SMART_RUN_PATH;
            break;
        case SMART_RUN_PATH:
            get_manhattan_path();
            get_bfs_path();
            if (path[point_idx + 1][x] == 0 && path[point_idx + 1][y] == 0 && point_idx != 0)
            {
                wait_ms(1000);
                if (wait_ok_flag == 1)
                {
                    wait_ok_flag = 0;
                    if (box_num < last_box_num)
                    {
                        smart_solution_state = SMART_FINISH;
                        break;
                    }
                    else
                    {
                        point_idx = 0;
                        memset(path, 0, sizeof(path));
                        smart_solution_state = SMART_TEST_ONE_BY_ONE;
                        dx = the_other_destination[x] - target_destination[x];
                        dy = the_other_destination[y] - target_destination[y];
                        break;
                    }
                }
            }
            break;
        case SMART_TEST_ONE_BY_ONE:
            if (destination_same_y_flag == 1) // 目的地在同一行
            {
                if (fabs(mv_packet.data.car_x - target_destination[x]) > 0.8)
                { // 小车跟目的地同一行
                    path[0][x] = mv_packet.data.car_x;
                    path[0][y] = mv_packet.data.car_y;
                    path[1][x] = the_other_destination[x];
                    path[1][y] = the_other_destination[y];
                    smart_solution_state = SMART_RUN_TEST_PATH;
                    break;
                }
                else // 那就是在目的地上面或者下面
                {

                    if (map_packet.map_data.map[(int)target_destination[y]][(int)target_destination[x] + 1] == 4)
                    { // 目标目的地右边还是目的地
                        path[0][x] = mv_packet.data.car_x;
                        path[0][y] = mv_packet.data.car_y;
                        path[1][x] = mv_packet.data.car_x - 1; // 小车向左移动一格
                        path[1][y] = mv_packet.data.car_y;
                        if (mv_packet.data.car_y > target_destination[y]) // 小车在目的地上面
                        {
                            path[2][x] = mv_packet.data.car_x - 1; // 再向下一格
                            path[2][y] = mv_packet.data.car_y + 1;
                        }
                        else // 小车在目的下面
                        {
                            path[2][x] = mv_packet.data.car_x - 1; // 再向上一格
                            path[2][y] = mv_packet.data.car_y - 1;
                        }
                        path[3][x] = the_other_destination[x] - 1; // 横着走到另一个目的地前一格
                        path[3][y] = the_other_destination[y];
                        smart_solution_state = SMART_RUN_TEST_PATH;
                        break;
                    }
                    else // 目标目的地左边还是目的地
                    {
                        path[0][x] = mv_packet.data.car_x;
                        path[0][y] = mv_packet.data.car_y;
                        path[1][x] = mv_packet.data.car_x + 1; // 小车向右移动一格
                        path[1][y] = mv_packet.data.car_y;
                        if (mv_packet.data.car_y > target_destination[y]) // 小车在目的地上面
                        {
                            path[2][x] = mv_packet.data.car_x + 1; // 再向下一格
                            path[2][y] = mv_packet.data.car_y + 1;
                        }
                        else // 小车在目的下面
                        {
                            path[2][x] = mv_packet.data.car_x + 1; // 再向上一格
                            path[2][y] = mv_packet.data.car_y - 1;
                        }
                        path[3][x] = the_other_destination[x] + 1; // 横着走到另一个目的地后一格
                        path[3][y] = the_other_destination[y];
                        smart_solution_state = SMART_RUN_TEST_PATH;
                        break;
                    }
                }
            }
            else if (destination_same_x_flag == 1) // 目的地在同一列
            {
                if (fabs(mv_packet.data.car_y - target_destination[y]) > 0.8)
                {
                    path[0][x] = mv_packet.data.car_x;
                    path[0][y] = mv_packet.data.car_y;
                    path[1][x] = the_other_destination[x];
                    path[1][y] = the_other_destination[y];
                    smart_solution_state = SMART_RUN_TEST_PATH;
                    break;
                }
                else // 那就是在目的地左面或者右边
                {

                    if (map_packet.map_data.map[(int)target_destination[y] + 1][(int)target_destination[x]] == 4)
                    { // 目标目的地下面还是目的地
                        path[0][x] = mv_packet.data.car_x;
                        path[0][y] = mv_packet.data.car_y;
                        path[1][x] = mv_packet.data.car_x; // 小车向上移动一格
                        path[1][y] = mv_packet.data.car_y - 1;
                        if (mv_packet.data.car_x > target_destination[x]) // 小车在目的左边
                        {
                            path[2][x] = mv_packet.data.car_x + 1; // 再往右移动一格
                            path[2][y] = mv_packet.data.car_y - 1;
                        }
                        else // 小车在目的右边
                        {
                            path[2][x] = mv_packet.data.car_x - 1; // 再往左移动一格
                            path[2][y] = mv_packet.data.car_y - 1;
                        }
                        path[3][x] = the_other_destination[x];
                        path[3][y] = the_other_destination[y] - 1;
                        smart_solution_state = SMART_RUN_TEST_PATH;
                        break;
                    }
                    else // 目标目的地上面还是目的地
                    {
                        path[0][x] = mv_packet.data.car_x;
                        path[0][y] = mv_packet.data.car_y;
                        path[1][x] = mv_packet.data.car_x; // 小车向下移动一格
                        path[1][y] = mv_packet.data.car_y + 1;
                        if (mv_packet.data.car_x > target_destination[x]) // 小车在目的左边
                        {
                            path[2][x] = mv_packet.data.car_x + 1; // 再往右移动一格
                            path[2][y] = mv_packet.data.car_y + 1;
                        }
                        else // 小车在目的右边
                        {
                            path[2][x] = mv_packet.data.car_x - 1; // 再往左移动一格
                            path[2][y] = mv_packet.data.car_y + 1;
                        }
                        path[3][x] = the_other_destination[x];
                        path[3][y] = the_other_destination[y] + 1;
                        smart_solution_state = SMART_RUN_TEST_PATH;
                        break;
                    }
                }
            }
            break;
        case SMART_RUN_TEST_PATH:
            if (path[point_idx + 1][x] == 0 && path[point_idx + 1][y] == 0 && point_idx != 0)
            {
                if (box_num < last_box_num)
                {
                    smart_solution_state = SMART_FINISH;
                    break;
                }
            }
            else
            {
                car_run_path();
            }
            break;
        case SMART_FINISH:
            if (box_num == 0) // 所有箱子都被推到目的地
            {
                together_destination_and_can_push_flag = 0;
                destination_same_x_flag = 0;
                destination_same_y_flag = 0;

                memset(path, 0, sizeof(path));
                point_idx = 0;
                path_flag = 0;
                level++;
            }
            smart_solution_state = SMART_FIND_TARGET_BOX_AND_DESTINATION;
            break;
        }
    }
    return 0;
}