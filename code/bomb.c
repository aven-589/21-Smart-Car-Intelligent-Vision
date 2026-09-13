#include "bomb.h"
#include "Astar.h"
#include "zf_common_headfile.h"
#include <math.h>

extern OpenMV_Union_t mv_packet;
extern OpenMV_Map_t map_packet;

extern int apush_path_len;
extern int abox_path_len;
extern uint8_t bomb_num;
float target_bomb[2];

extern aPoint apush_path[APATH_NODE];

aPoint bomb_path[APATH_NODE];
int bomb_path_len = 0;
int bomb_path_cost = 0;

aPoint car_to_bomb_path[APATH_NODE];
int car_to_bomb_path_len = 0;

aPoint push_bomb_path[APATH_NODE];
int push_bomb_len = 0;

int car_to_bomb_path_cost = 0;
int push_bomb_cost = 0;

int is_boundary(int bfs_x, int bfs_y) // 越界返回1
{
    if (bfs_x == 0 || bfs_y == 0 || bfs_x == COLS - 1 || bfs_y == ROWS - 1)
        return 1;
    return 0;
}

void bomb_apply(int cx, int cy, int backup[3][3], uint16_t map[16][12])
{
    for (int dy = -1; dy <= 1; dy++)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            int bfs_x = cx + dx;
            int bfs_y = cy + dy;

            if (bfs_x < 0 || bfs_y < 0 || bfs_x >= COLS || bfs_y >= ROWS)
            {
                backup[dy + 1][dx + 1] = -1;
                continue;
            }

            backup[dy + 1][dx + 1] = map[bfs_y][bfs_x];

            if (bfs_x == 0 || bfs_y == 0 || bfs_x == COLS - 1 || bfs_y == ROWS - 1)
                continue;

            if (map[bfs_y][bfs_x] == 2)
                map[bfs_y][bfs_x] = 0;
        }
    }
}

void bomb_restore(int cx, int cy, int backup[3][3], uint16_t map[ROWS][COLS])
{
    for (int dy = -1; dy <= 1; dy++)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            int bfs_x = cx + dx;
            int bfs_y = cy + dy;

            if (bfs_x < 0 || bfs_y < 0 || bfs_x >= COLS || bfs_y >= ROWS)
                continue;

            if (backup[dy + 1][dx + 1] != -1)
                map[bfs_y][bfs_x] = backup[dy + 1][dx + 1];
        }
    }
}

int is_bomb_stuck(int bomb_x, int bomb_y)
{
    uint8_t car_x = (uint8_t)roundf(mv_packet.data.car_x - 0.5f);
    uint8_t car_y = (uint8_t)roundf(mv_packet.data.car_y - 0.5f);
    int8_t dir_x[4] = {1, 0, -1, 0};
    int8_t dir_y[4] = {0, 1, 0, -1};

    for (int d = 0; d < 4; d++)
    {
        // 炸弹目标位置
        int tx = bomb_x + dir_x[d];
        int ty = bomb_y + dir_y[d];

        // 小车占位
        int sx = bomb_x - dir_x[d];
        int sy = bomb_y - dir_y[d];

        if (tx < 0 || ty < 0 || tx >= COLS || ty >= ROWS) // 越界跳过
            continue;

        if (map_packet.map_data.map[ty][tx] == 3 ||
            map_packet.map_data.map[ty][tx] == 6)
            continue;

        if (sx < 0 || sy < 0 || sx >= COLS || sy >= ROWS) // 越界跳过
            continue;

        if (map_packet.map_data.map[sy][sx] == 2 ||
            map_packet.map_data.map[sy][sx] == 3 ||
            map_packet.map_data.map[sy][sx] == 6)
            continue;

        int old = map_packet.map_data.map[bomb_y][bomb_x];
        map_packet.map_data.map[bomb_y][bomb_x] = 2;

        aPoint _tmp_path[APATH_NODE];
        int _tmp_len;
        int dist = astar_integrate(car_x, car_y, sx, sy, _tmp_path, &_tmp_len);

        map_packet.map_data.map[bomb_y][bomb_x] = old;

        if (dist == 1)
            return 0;
    }
    return 1;
}

// 检查地图上有没有卡死炸弹，如果有：把这个炸弹坐标返回出去，函数返回1
int check_bomb_lock(int *out_stuck_x, int *out_stuck_y)
{
    for (int bomb_y = 0; bomb_y < ROWS; bomb_y++)
    {
        for (int bomb_x = 0; bomb_x < COLS; bomb_x++)
        {
            if (map_packet.map_data.map[bomb_y][bomb_x] != 6) // 寻找地图上的炸弹
                continue;

            if (is_bomb_stuck(bomb_x, bomb_y)) // 不能推返回1
            {
                *out_stuck_x = bomb_x;
                *out_stuck_y = bomb_y;
                return 1;
            }
        }
    }
    return 0;
}

int8_t unlock_bomb_x = -1, unlock_bomb_y = -1;

int find_unlock_wall(int stuck_x, int stuck_y,
                     int *out_wall_x, int *out_wall_y)
{
    int car_x = (int)roundf(mv_packet.data.car_x - 0.5f);
    int car_y = (int)roundf(mv_packet.data.car_y - 0.5f);

    float saved_tb0 = target_bomb[0];
    float saved_tb1 = target_bomb[1];

    // 1. 只做一次 stuck 判断
    int base_stuck = is_bomb_stuck(stuck_x, stuck_y);

    // 2. 收集 free bombs（不 BFS）
    int free_bombs[10][2];
    int free_count = 0;

    for (int dy = 0; dy < ROWS; dy++)
        for (int dx = 0; dx < COLS; dx++)
        {
            if (map_packet.map_data.map[dy][dx] == 6 &&
                !(dx == stuck_x && dy == stuck_y))
            {
                free_bombs[free_count][0] = dx;
                free_bombs[free_count][1] = dy;
                free_count++;
            }
        }

    // 3. 合并候选墙（3x3 + spiral 一个列表）
    int walls[200][2];
    int wall_count = 0;

    for (int dy = 1; dy < ROWS - 1; dy++)
        for (int dx = 1; dx < COLS - 1; dx++)
        {
            if (map_packet.map_data.map[dy][dx] != 2)
                continue;

            if (is_boundary(dx, dy))
                continue;

            // 超轻过滤（只看邻接）
            int adj = 0;
            int dir_x[4] = {1, 0, -1, 0};
            int dir_y[4] = {0, 1, 0, -1};

            for (int i = 0; i < 4; i++)
            {
                int nx = dx + dir_x[i], ny = dy + dir_y[i];
                if (map_packet.map_data.map[ny][nx] != 2) // 四周都是墙的墙
                    adj = 1;
            }

            if (!adj)
                continue;

            walls[wall_count][0] = dx;
            walls[wall_count][1] = dy;
            wall_count++;
        }

    int best_len = 9999;
    int best_wx = -1;
    int best_wy = -1;
    // 4. 只对候选墙做验证
    for (int i = 0; i < wall_count; i++)
    {
        int wx = walls[i][0];
        int wy = walls[i][1];

        int backup[3][3];
        bomb_apply(wx, wy, backup, map_packet.map_data.map);

        int stuck_after = is_bomb_stuck(stuck_x, stuck_y);

        if (stuck_after) // 炸弹仍然死锁，跳过
        {
            bomb_restore(wx, wy, backup, map_packet.map_data.map);
            continue;
        }
        bomb_restore(wx, wy, backup, map_packet.map_data.map);
        // 炸弹不死锁了，找自由炸弹
        for (int bi = 0; bi < free_count; bi++)
        {
            target_bomb[0] = free_bombs[bi][0] + 0.5f;
            target_bomb[1] = free_bombs[bi][1] + 0.5f;

            if (make_bomb_path(wx, wy))
            {
                if (make_car_to_bomb_path(bomb_path[0].ax, bomb_path[0].ay,
                                          bomb_path[1].ax, bomb_path[1].ay))
                {
                    make_push_bomb_path();
                    int cost = push_bomb_cost;
                    if (cost < best_len)
                    {
                        best_len = cost;

                        best_wx = wx;
                        best_wy = wy;

                        unlock_bomb_x = free_bombs[bi][0];
                        unlock_bomb_y = free_bombs[bi][1];
                    }
                }
            }
        }
        target_bomb[0] = saved_tb0;
        target_bomb[1] = saved_tb1;
    }
    if (best_wx == -1)
    {
        return 0;
    }
    if (best_wy == -1)
    {
        return 0;
    }
    *out_wall_x = best_wx;
    *out_wall_y = best_wy;
    // ips200_show_int(0,160,best_wx,2);
    // ips200_show_int(30,160,best_wy,2);
    // ips200_show_int(60,160,best_len,2);
    return 1;
}

PairNode pair_list[MAX_PAIR];
int16_t pair_list_len = 0;

static float manhattan(int x1, int y1, int x2, int y2)
{
    return fabsf(x1 - x2) + fabsf(y1 - y2);
}

int8_t bomb_pair_map[ROWS][COLS];
int bomb_pair(void)
{
    int box_list[10][2];
    int goal_list[10][2];
    int bomb_list[10][2];

    pair_list_len = 0;

    int box_cnt = 0;
    int goal_cnt = 0;
    int bomb_cnt = 0;

    // 初始化列表
    for (int dy = 0; dy < ROWS; dy++)
    {
        for (int dx = 0; dx < COLS; dx++)
        {
            bomb_pair_map[dy][dx] = -1;
        }
    }

    // 收集物体坐标
    for (int dy = 0; dy < ROWS; dy++)
    {
        for (int dx = 0; dx < COLS; dx++)
        {
            switch (map_packet.map_data.map[dy][dx])
            {
            case 3:
                if (box_cnt < MAX_BOX_NUM)
                {
                    box_list[box_cnt][0] = dx;
                    box_list[box_cnt][1] = dy;
                    box_cnt++;
                }
                break;

            case 4:
                if (goal_cnt < MAX_GOAL_NUM)
                {
                    goal_list[goal_cnt][0] = dx;
                    goal_list[goal_cnt][1] = dy;
                    goal_cnt++;
                }
                break;

            case 6:
                /* 解死锁炸弹过滤 */
                if (dx == unlock_bomb_x && dy == unlock_bomb_y)
                    break;

                if (bomb_cnt < MAX_BOMB_NUM)
                {
                    bomb_list[bomb_cnt][0] = dx;
                    bomb_list[bomb_cnt][1] = dy;
                    bomb_cnt++;
                }
                break;
            default:
                break;
            }
        }
    }

    if (box_cnt == 0 || goal_cnt == 0)
        return 0;

    // 使用标记数组，配对成功标记已配对炸弹和目的地，避免重复配对
    int used_goal[MAX_GOAL_NUM] = {0};
    int used_bomb[MAX_BOMB_NUM] = {0};

    // 配对，以箱子为中心
    for (int i = 0; i < box_cnt; i++)
    {
        int box_x = box_list[i][0];
        int box_y = box_list[i][1];

        int best_goal = -1;
        int best_goal_dist = 999999;

        for (int j = 0; j < goal_cnt; j++)
        {
            if (used_goal[j])
                continue;

            int d = manhattan(
                box_x, box_y,
                goal_list[j][0],
                goal_list[j][1]);

            if (d < best_goal_dist)
            {
                best_goal_dist = d;
                best_goal = j;
            }
        }

        if (best_goal == -1)
            continue;

        /* 找炸弹 */
        int best_bomb = -1;
        int best_bomb_dist = 999999;

        for (int j = 0; j < bomb_cnt; j++)
        {
            if (used_bomb[j])
                continue;

            int d = manhattan(
                box_x, box_y,
                bomb_list[j][0],
                bomb_list[j][1]);

            if (d < best_bomb_dist)
            {
                best_bomb_dist = d;
                best_bomb = j;
            }
        }

        used_goal[best_goal] = 1;
        if (best_bomb != -1)          // 有炸弹才标记，可能没有炸弹
            used_bomb[best_bomb] = 1; // 标记数组置一

        // 写入结构体
        int idx = pair_list_len;

        pair_list[idx].box_x = box_x;
        pair_list[idx].box_y = box_y;

        pair_list[idx].goal_x = goal_list[best_goal][0];
        pair_list[idx].goal_y = goal_list[best_goal][1];

        if (best_bomb != -1)
        {
            pair_list[idx].bomb_x = bomb_list[best_bomb][0];
            pair_list[idx].bomb_y = bomb_list[best_bomb][1];

            // 建立二维映射，方便取用
            bomb_pair_map[bomb_list[best_bomb][1]]
                         [bomb_list[best_bomb][0]] = idx;
        }
        else
        {
            pair_list[idx].bomb_x = -1;
            pair_list[idx].bomb_y = -1;
        }

        pair_list[idx].dist = best_goal_dist;
        pair_list_len++;
    }

    return pair_list_len;
}

int8_t nearlist_bomb_x = -1, nearlist_bomb_y = -1;
void nearlist_bomb_get()
{
    uint8_t car_x = (uint8_t)roundf(mv_packet.data.car_x - 0.5f);
    uint8_t car_y = (uint8_t)roundf(mv_packet.data.car_y - 0.5f);

    int best_d = 999999;
    int16_t idx = -1;
     for (int dy = 0; dy < ROWS; dy++)
    {
        for (int dx = 0; dx < COLS; dx++)
        {
            if (map_packet.map_data.map[dy][dx] != 6)
                continue;

            int16_t d = manhattan(car_x, car_y, dx, dy);
            if (d < best_d)
            {
                best_d = d;
                nearlist_bomb_x = dx;
                nearlist_bomb_y = dy;
            }
        }
    }

}
int nearlist_box_index()
{
    uint8_t car_x = (uint8_t)roundf(mv_packet.data.car_x - 0.5f);
    uint8_t car_y = (uint8_t)roundf(mv_packet.data.car_y - 0.5f);

    int16_t best_d = 9999;
    int16_t idx = -1;

    for (int dy = 0; dy < ROWS; dy++)
    {
        for (int dx = 0; dx < COLS; dx++)
        {
            if (map_packet.map_data.map[dy][dx] != 6)
                continue;

            int8_t pair_idx = bomb_pair_map[dy][dx];

            if (pair_idx == -1)
                continue;

            int16_t d = manhattan(car_x, car_y, dx, dy);
            if (d < best_d)
            {
                best_d = d;
                idx = pair_idx;
                nearlist_bomb_x = dx;
                nearlist_bomb_y = dy;
            }
        }
    }
    return idx;
}
extern int8_t check_stuck_ret;
int stuck_x = -1, stuck_y = -1;
int8_t ul_debug = 0;
int8_t ul_wx = -1, ul_wy = -1;
int8_t ul_bx = -1, ul_by = -1;

int make_unlock_path()
{
    int out_wall_x = -1, out_wall_y = -1;
    ul_debug = 88; // 进入标志，若未被调用则保持初始值0
    if (check_stuck_ret)
    {
        if (find_unlock_wall(stuck_x, stuck_y, &out_wall_x, &out_wall_y))
        {
            ul_wx = out_wall_x; ul_wy = out_wall_y;
            ul_bx = unlock_bomb_x; ul_by = unlock_bomb_y;
            target_bomb[0] = unlock_bomb_x;
            target_bomb[1] = unlock_bomb_y;
            if (make_bomb_path(out_wall_x, out_wall_y))
            {
                if (make_car_to_bomb_path(bomb_path[0].ax, bomb_path[0].ay, bomb_path[1].ax, bomb_path[1].ay))
                {
                    make_push_bomb_path();
                    ul_debug = 5;
                    return 1;
                }
                ul_debug = 4; // make_car_to失败
                return 0;
            }
            ul_debug = 3; // make_bomb_path失败
            return 0;
        }
        ul_debug = 2; // find_unlock失败
        return 0;
    }
    else
    {
        ul_debug = 1; // stuck_ret=0
        return 0;
    }
    return 0;
}

// box_path有返回1，没有返回0
int8_t check_box_path_ok(int8_t wall_x, int8_t wall_y)
{
    memcpy(map_packet.map_data.map, bomb_map, sizeof(map_packet.map_data.map));
    int backup[3][3];
    bomb_apply(wall_x, wall_y, backup, map_packet.map_data.map);
    int old = map_packet.map_data.map[(int)roundf(target_bomb[1] - 0.5f)][(int)roundf(target_bomb[0] - 0.5f)];
    map_packet.map_data.map[(int)roundf(target_bomb[1] - 0.5f)][(int)roundf(target_bomb[0] - 0.5f)] = 0;

    // 直接调 sokoban_search 检查箱推路径，不用 amake_box_path
    // （amake_box_path 内部会调 init_game_map，覆盖掉 bomb_apply 的效果）
    int car_x = (int)roundf(mv_packet.data.car_x - 0.5f);
    int car_y = (int)roundf(mv_packet.data.car_y - 0.5f);
    int box_x = (int)roundf(target_box[0] - 0.5f);
    int box_y = (int)roundf(target_box[1] - 0.5f);
    int dst_x = (int)roundf(target_destination[0] - 0.5f);
    int dst_y = (int)roundf(target_destination[1] - 0.5f);

    if (sokoban_search(car_x, car_y, box_x, box_y, dst_x, dst_y) == 1)
    {
        bomb_restore(wall_x, wall_y, backup, map_packet.map_data.map);
        map_packet.map_data.map[(int)roundf(target_bomb[1] - 0.5f)][(int)roundf(target_bomb[0] - 0.5f)] = old;
        return 1;
    }

    bomb_restore(wall_x, wall_y, backup, map_packet.map_data.map);
    map_packet.map_data.map[(int)roundf(target_bomb[1] - 0.5f)][(int)roundf(target_bomb[0] - 0.5f)] = old;
    return 0;
}

extern uint8_t best_idx;
aPoint best_car_to_bomb_path[APATH_NODE];
int best_car_to_bomb_path_len = 0;
aPoint best_bomb_path[APATH_NODE];
int best_bomb_path_len = 0;
aPoint best_push_bomb_path[APATH_NODE];
int best_push_bomb_len = 0;

int16_t bba = 0, bbb = 0, bbc = 0, bbd = 0, bbe = 0, bbf = 0;

int make_free_bomb_path()
{
    // ===== DWT 周期计数器计时 =====
    static int dwt_ready = 0;
    if (!dwt_ready)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        dwt_ready = 1;
    }
    uint32_t t_start = DWT->CYCCNT;
    // ============================

    memcpy(map_packet.map_data.map, bomb_map, sizeof(map_packet.map_data.map));

    int target_index = nearlist_box_index();
    if (target_index < 0)
        return 0;
    if (nearlist_bomb_x == -1 || nearlist_bomb_y == -1)
        return 0;

    int found = 0;
    int best_cost = 999999;
    float best_wall_x = -1, best_wall_y = -1;

    target_box[0] = pair_list[target_index].box_x;
    target_box[1] = pair_list[target_index].box_y;

    target_destination[0] = pair_list[target_index].goal_x;
    target_destination[1] = pair_list[target_index].goal_y;

    target_bomb[0] = pair_list[target_index].bomb_x;
    target_bomb[1] = pair_list[target_index].bomb_y;

    int bomb_x = (int)roundf(target_bomb[0] - 0.5f);
    int bomb_y = (int)roundf(target_bomb[1] - 0.5f);
    for (int dy = 1; dy < ROWS - 1; dy++)
    {
        for (int dx = 1; dx < COLS - 1; dx++)
        {
            if (map_packet.map_data.map[dy][dx] != 2)
                continue;
            bba++;
            int manhattan =
                abs(dx - bomb_x) +
                abs(dy - bomb_y);

            if (manhattan > 6)
                continue;

            if (!check_box_path_ok(dx, dy))
                continue;

            if (!make_bomb_path(dx, dy))
                continue;
            bbb++;
            if (bomb_path_len < 2)
                continue;
            bbc++;
            if (!make_car_to_bomb_path(
                    bomb_path[0].ax,
                    bomb_path[0].ay,
                    bomb_path[1].ax,
                    bomb_path[1].ay))
                continue;
            bbd++;
            if (!make_push_bomb_path())
                continue;
            bbe++;

            bbf++;
            int cost = car_to_bomb_path_cost + push_bomb_cost;
            if (cost < best_cost)
            {
                found = 1;
                best_cost = cost;
                best_wall_x = dx;
                best_wall_y = dy;

                memcpy(best_car_to_bomb_path,
                       car_to_bomb_path,
                       sizeof(aPoint) * car_to_bomb_path_len);
                best_car_to_bomb_path_len = car_to_bomb_path_len;
                memcpy(best_bomb_path,
                       bomb_path,
                       sizeof(aPoint) * bomb_path_len);
                best_bomb_path_len = bomb_path_len;
                memcpy(best_push_bomb_path,
                       push_bomb_path,
                       sizeof(aPoint) * push_bomb_len);
                best_push_bomb_len = push_bomb_len;
            }
        }
    }

    uint32_t t_end = DWT->CYCCNT;
    static uint32_t bomb_path_ms = 0;
    bomb_path_ms = (t_end - t_start) / (SystemCoreClock / 1000);
    ips200_show_int(80, 180, bomb_path_ms, 5);
    // ips200_show_int(80, 90, bba, 3);
    // ips200_show_int(110, 90, bbb, 3);
    // ips200_show_int(140, 90, bbc, 3);
    // ips200_show_int(0, 110, bbd, 3);
    // ips200_show_int(40, 110, bbe, 3);
    // ips200_show_int(80, 110, bbf, 3);
    // ===== DWT 计时结束 =====

    // 输出结果
    if (!found)
    {
        bomb_path_len = 0;
        car_to_bomb_path_len = 0;
        push_bomb_len = 0;

        return 0;
    }

    memcpy(car_to_bomb_path,
           best_car_to_bomb_path,
           sizeof(aPoint) * best_car_to_bomb_path_len);
    car_to_bomb_path_len = best_car_to_bomb_path_len;
    memcpy(bomb_path,
           best_bomb_path,
           sizeof(aPoint) * best_bomb_path_len);
    bomb_path_len = best_bomb_path_len;
    memcpy(push_bomb_path,
           best_push_bomb_path,
           sizeof(aPoint) * best_push_bomb_len);
    push_bomb_len = best_push_bomb_len;

    // ips200_show_int(0, 280, best_wall_x, 2);
    // ips200_show_int(30, 280, best_wall_y, 2);
    return 1;
}

/* ========== 小车走到炸弹后面 ========== */
int make_car_to_bomb_path(int bomb_x, int bomb_y, int first_dst_x, int first_dst_y)
{
    // 推炸弹方向直接从 bomb_path 相邻节点得出（类比 make_push_box_path 用 box_path 节点算 stand）
    int dx = first_dst_x - bomb_x;
    int dy = first_dst_y - bomb_y;
    int push_dx = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
    int push_dy = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
    int stand_x = bomb_x - push_dx;
    int stand_y = bomb_y - push_dy;

    int car_x = (int)roundf(mv_packet.data.car_x - 0.5f);
    int car_y = (int)roundf(mv_packet.data.car_y - 0.5f);

    memcpy(map_packet.map_data.map, bomb_map, sizeof(map_packet.map_data.map));
    if (astar_integrate(car_x, car_y, stand_x, stand_y,
                        car_to_bomb_path, &car_to_bomb_path_len) != 1)
        return 0;
    car_to_bomb_path_cost = car_to_bomb_path_len;

    get_astar_turn_path(car_to_bomb_path, &car_to_bomb_path_len);
    return 1;
}

int make_bomb_path(int wall_x, int wall_y)
{
    int car_x = (int)roundf(mv_packet.data.car_x - 0.5f);
    int car_y = (int)roundf(mv_packet.data.car_y - 0.5f);
    int bomb_x = (int)roundf(target_bomb[0] - 0.5f);
    int bomb_y = (int)roundf(target_bomb[1] - 0.5f);

    if (bomb_x < 0 || bomb_y < 0 || bomb_x >= COLS || bomb_y >= ROWS)
        return 0;

    memcpy(map_packet.map_data.map, bomb_map, sizeof(map_packet.map_data.map));

    int ret = sokoban_search(car_x, car_y, bomb_x, bomb_y, wall_x, wall_y);
    if (ret != 1)
    {
        bomb_path_len = 0;
        return 0;
    }

    get_box_path(bomb_path, &bomb_path_len);
    // 不对 bomb_path 做 turn_path 压缩 —— 炸弹必须逐格推，
    // 压缩后 make_push_bomb_path 会少算 push 步数，导致 cost 比较失准
    bomb_path_cost = bomb_path_len;
    return 1;
}

// 根据 bomb_path 生成 push_bomb_path，返回1，路径保存在 push_bomb_path 里，路径长度保存在 push_bomb_len 里
int make_push_bomb_path(void)
{
    push_bomb_len = 0;
    push_bomb_cost = 0;
    int car_x = car_to_bomb_path[car_to_bomb_path_len - 1].ax;
    int car_y = car_to_bomb_path[car_to_bomb_path_len - 1].ay;

    for (int i = 0; i < bomb_path_len - 1; i++)
    {
        int bx = bomb_path[i].ax;
        int by = bomb_path[i].ay;
        int nx = bomb_path[i + 1].ax;
        int ny = bomb_path[i + 1].ay;

        int dx = nx - bx;
        int dy = ny - by;
        int push_dx = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
        int push_dy = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
        int stand_x = bx - push_dx;
        int stand_y = by - push_dy;

        if (i > 0)
        {
            map_packet.map_data.map[bomb_path[0].ay][bomb_path[0].ax] = 0;
        }

        uint16_t old = map_packet.map_data.map[by][bx];
        map_packet.map_data.map[by][bx] = 2;

        if (car_x != stand_x || car_y != stand_y)
        {
            aPoint change_path[APATH_NODE];
            int change_len = 0;

            if (astar_integrate(car_x, car_y, stand_x, stand_y,
                                change_path, &change_len) == 1)
            {
                push_bomb_cost += change_len;
                get_astar_turn_path(change_path, &change_len);
                for (int k = 1; k < change_len; k++)
                {
                    push_bomb_path[push_bomb_len++] = change_path[k];
                }
            }
        }

        map_packet.map_data.map[by][bx] = old;

        if (i < bomb_path_len - 2)
        {
            map_packet.map_data.map[ny][nx] = 3;
        }

        // 站位
        if (push_bomb_len == 0 ||
            push_bomb_path[push_bomb_len - 1].ax != stand_x ||
            push_bomb_path[push_bomb_len - 1].ay != stand_y)
        {
            push_bomb_path[push_bomb_len].ax = stand_x;
            push_bomb_path[push_bomb_len].ay = stand_y;
            push_bomb_len++;
            push_bomb_cost += 1;
        }

        push_bomb_path[push_bomb_len].ax = nx - push_dx;
        push_bomb_path[push_bomb_len].ay = ny - push_dy;
        push_bomb_len++;
        push_bomb_cost += 1;

        car_x = nx - push_dx;
        car_y = ny - push_dy;
    }

    return 1;
}

// typedef struct
// {
//     uint8_t bomb_x;
//     uint8_t bomb_y;
//     uint8_t car_x;
//     uint8_t car_y;
//     uint8_t dst_x;
//     uint8_t dst_y;
//     int8_t dir;
//     int16_t cost;
// } BombState;

// typedef struct
// {
//     int8_t bomb_idx[MAX_BOMB_NUM];
//     uint8_t len;
// } BombOrder;

// typedef struct
// {
//     int16_t total_cost;
//     BombOrder order;
// } BombPlanResult;

// typedef struct
// {
//     uint16_t map[ROWS][COLS];
//     uint8_t car_x;
//     uint8_t car_y;
//     uint8_t used[MAX_BOMB_NUM];
//     int16_t cost;
//     uint8_t depth;
//     int8_t order[MAX_BOMB_NUM];
// } BombSearchNode;

// static int8_t get_dir_from_step(int dx, int dy)
// {
//     if (dx == 0 && dy == -1) return 0;
//     if (dx == 0 && dy == 1)  return 1;
//     if (dx == -1 && dy == 0) return 2;
//     if (dx == 1 && dy == 0)  return 3;
//     return -1;
// }

// static uint8_t wall_has_open_neighbor(const uint16_t map[ROWS][COLS],
//                                   int wx, int wy)
// {
//     int dx[4] = {1,0,-1,0};
//     int dy[4] = {0,1,0,-1};

//     for(int i=0;i<4;i++)
//     {
//         int nx=wx+dx[i];
//         int ny=wy+dy[i];

//         if(map[ny][nx] != 2)
//             return 1;
//     }

//     return 0;
// }

// static int bomb_can_push(const uint16_t map[ROWS][COLS],
//                          int bx,int by)
// {
//     int dx[4]={1,0,-1,0};
//     int dy[4]={0,1,0,-1};


//     for(int i=0;i<4;i++)
//     {
//         int nx=bx+dx[i];
//         int ny=by+dy[i];

//         int sx=bx-dx[i];
//         int sy=by-dy[i];


//         // 炸弹目标位置
//         if(map[ny][nx]==2 ||
//            map[ny][nx]==3 ||
//            map[ny][nx]==6)
//             continue;


//         // 小车站位
//         if(map[sx][sy]==2 ||
//            map[sx][sy]==3 ||
//            map[sx][sy]==6)
//             continue;


//         return 1;
//     }

//     return 0;
// }

// int collect_bomb_states(const uint16_t map[ROWS][COLS],
//                         int car_x, int car_y,
//                         int bomb_x, int bomb_y,
//                         BombState states[], int *count)
// {
//     int found = 0;
//     *count = 0;

//     if (bomb_x < 0 || bomb_y < 0 || bomb_x >= COLS || bomb_y >= ROWS)
//         return 0;

//     if (!bomb_can_push(map, bomb_x, bomb_y))
//         return 0;
//     /* 只搜集“有意义”的目标 */
//     for (int ty = 1; ty < ROWS - 1; ty++)
//     {
//         for (int tx = 1; tx < COLS - 1; tx++)
//         {

//             /* 不能去炸弹自身 */
//             if (tx == bomb_x && ty == bomb_y)
//                 continue;

//             /* 不能撞到别的箱子/炸弹 */
//             if (map[ty][tx] == 3 || map[ty][tx] == 6)
//                 continue;

//             /* 目标是墙：允许炸 */
//             /* 目标是空地/目标点：也允许停 */
//             if (map[ty][tx] != 2)
//                 continue;

//             if (is_boundary(tx, ty))
//                 continue;

//             if(!wall_has_open_neighbor(map, tx, ty))
//                 continue;            /* 到此说明能停放炸弹 */
                

//             uint16_t saved_map[ROWS][COLS];
//             memcpy(saved_map, game_map, sizeof(saved_map));
//             memcpy(game_map, map, sizeof(saved_map));

//             // 同步 game_map → 真实地图，让 sokoban_search 看到模拟状态
//             uint16_t _real_save[ROWS][COLS];
//             memcpy(_real_save, map_packet.map_data.map, sizeof(_real_save));
//             memcpy(map_packet.map_data.map, game_map, sizeof(map_packet.map_data.map));

//             int sr = sokoban_search(car_x, car_y, bomb_x, bomb_y, tx, ty);

//             memcpy(map_packet.map_data.map, _real_save, sizeof(_real_save));

//             if (sr != 1)
//             {
//                 memcpy(game_map, saved_map, sizeof(saved_map));
//                 continue;
//             }

//             aPoint path[APATH_NODE];
//             int path_len = 0;
//             get_box_path(path, &path_len);

//             memcpy(game_map, saved_map, sizeof(saved_map));

//             if (path_len < 2)
//                 continue;

//             int ldx = path[path_len - 1].ax - path[path_len - 2].ax;
//             int ldy = path[path_len - 1].ay - path[path_len - 2].ay;
//             int dir = get_dir_from_step(ldx, ldy);
//             if (dir < 0)
//                 continue;

//             if (*count >= MAX_STATE_PER_BOMB)
//                 return 1;

//             states[*count].bomb_x = bomb_x;
//             states[*count].bomb_y = bomb_y;
//             states[*count].car_x  = path[path_len - 1].ax - ldx;
//             states[*count].car_y  = path[path_len - 1].ay - ldy;
//             states[*count].dst_x  = tx;
//             states[*count].dst_y  = ty;
//             states[*count].dir    = dir;
//             states[*count].cost   = path_len - 1;

//             (*count)++;
//             found = 1;
//         }
//     }

//     return found;
// }
