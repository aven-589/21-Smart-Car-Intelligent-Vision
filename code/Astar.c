#include "Astar.h"
#include "zf_common_headfile.h"
#include <math.h>

#define stra_value 10

int16_t open_count = 0;
ANode open_list[MAX_NODE];
int16_t close_count = 0;
ANode close_list[MAX_NODE];
int8_t start_x, start_y;
int8_t end_x, end_y;
int8_t end_dir;
extern OpenMV_Union_t mv_packet;
extern OpenMV_Map_t map_packet;
extern uint8_t box_num;
extern uint8_t destination_num;
extern float target_box[2];
extern float target_destination[2];

uint16_t game_map[ROWS][COLS];
uint16_t base_map[ROWS][COLS];
uint16_t bomb_map[ROWS][COLS];
uint8_t astar_allow_dest_box = 0;

void show_map_number(void)
{
    for (int dy = 0; dy < ROWS; dy++)
    {
        for (int dx = 0; dx < COLS; dx++)
        {
            ips200_show_int(dx * 16, dy * 16, bomb_map[dy][dx], 2);
        }
    }
}

void init_game_map(void)
{
    memcpy(game_map,
           map_packet.map_data.map,
           sizeof(game_map));
}

int16_t afind_in_close(uint8_t ax, uint8_t ay, int8_t dir)
{
    for (int16_t i = 0; i < close_count; i++)
    {
        if (close_list[i].ax == ax &&
            close_list[i].ay == ay &&
            close_list[i].dir == dir)
        {
            return i;
        }
    }
    return -1;
}

int16_t calc_h(uint8_t ax, uint8_t ay)
{
    int dx = abs(ax - end_x);
    int dy = abs(ay - end_y);

    return 10 * (dx + dy);
}

int16_t find_min_f(void)
{
    int16_t min = -1;
    for (int16_t i = 0; i < open_count; i++)
    {
        if (afind_in_close(open_list[i].ax,
                           open_list[i].ay,
                           open_list[i].dir) >= 0)
            continue;
        if (min == -1 ||
            open_list[i].f < open_list[min].f ||
            (open_list[i].f == open_list[min].f &&
             open_list[i].h < open_list[min].h))
        {
            min = i;
        }
    }
    return min;
}

int16_t find_in_open(uint8_t ax, uint8_t ay, int8_t dir)
{
    int16_t i;
    for (i = 0; i < open_count; i++)
        if (open_list[i].ax == ax &&
            open_list[i].ay == ay &&
            open_list[i].dir == dir)
            return i;
    return -1;
}

int8_t dir_x[4] = {0, 0, -1, 1};
int8_t dir_y[4] = {-1, 1, 0, 0};

/* ==================================A*?????==================================*/
int AStar(void)
{
    int16_t cur;
    int nx, ny;
    open_count = 1;
    close_count = 0;
    open_list[0].ax = start_x;
    open_list[0].ay = start_y;
    open_list[0].dir = -1;
    open_list[0].g = 0;
    open_list[0].h = calc_h(start_x, start_y);
    open_list[0].f = open_list[0].h;
    open_list[0].parent_ax = start_x;
    open_list[0].parent_ay = start_y;
    open_list[0].parent_dir = -1;

    while (open_count > 0)
    {
        cur = find_min_f();
        if (cur < 0)
            break;

        ANode current = open_list[cur];
        close_list[close_count++] = current;
        open_list[cur] = open_list[--open_count];

        for (int i = 0; i < 4; i++)
        {
            nx = current.ax + dir_x[i];
            ny = current.ay + dir_y[i];
            int new_dir = i;

            if (nx < 0 || ny < 0 || nx >= COLS || ny >= ROWS)
                continue;

            if (map_packet.map_data.map[ny][nx] == 2)
                continue;

            if(map_packet.map_data.map[ny][nx] == 3&& !(astar_allow_dest_box && nx==end_x&&ny==end_y))
                continue;

            if (map_packet.map_data.map[ny][nx] == 6)
                continue;

            if (afind_in_close(nx, ny, new_dir) >= 0)
                continue;

            int cost = stra_value;
            if (current.dir != -1 && current.dir != new_dir)
            {
                cost += 2;
            }

            int new_g = current.g + cost;
            int16_t idx = find_in_open(nx, ny, new_dir);
            if (idx >= 0)
            {
                if (new_g < open_list[idx].g)
                {
                    open_list[idx].g = new_g;
                    open_list[idx].f = new_g + open_list[idx].h;
                    open_list[idx].parent_ax = current.ax;
                    open_list[idx].parent_ay = current.ay;
                    open_list[idx].parent_dir = current.dir;
                }
            }
            else
            {
                open_list[open_count].ax = nx;
                open_list[open_count].ay = ny;
                open_list[open_count].dir = new_dir;
                open_list[open_count].g = new_g;
                open_list[open_count].h = calc_h(nx, ny);
                open_list[open_count].f = new_g + open_list[open_count].h;
                open_list[open_count].parent_ax = current.ax;
                open_list[open_count].parent_ay = current.ay;
                open_list[open_count].parent_dir = current.dir;
                open_count++;
            }
        }

        if (current.ax == end_x && current.ay == end_y)
        {
            end_dir = current.dir;
            return 1;
        }
    }
    return -1;
}

int astar_distance(float sx, float sy, float ex, float ey)
{
    start_x = (int8_t)roundf(sx - 0.5f);
    start_y = (int8_t)roundf(sy - 0.5f);
    end_x   = (int8_t)roundf(ex - 0.5f);
    end_y   = (int8_t)roundf(ey - 0.5f);

    open_count  = 0;
    close_count = 0;
    astar_allow_dest_box = 1;
    if(AStar() != 1)
    {
        astar_allow_dest_box = 0;
        return 999;
    }
        astar_allow_dest_box = 0;
    int16_t index = afind_in_close(end_x, end_y, end_dir);
    if(index == -1)
        return 999;
    return close_list[index].g;
}

/* ==================== A* ???? ==================== */
void get_astar_path_to(aPoint *out_path, int *out_len)
{
    int cur_x = end_x;
    int cur_y = end_y;
    int cur_dir = end_dir;
    int len = 0;

    while (1)
    {
        out_path[len].ax = (uint8_t)cur_x;
        out_path[len].ay = (uint8_t)cur_y;
        len++;

        if (cur_x == start_x && cur_y == start_y)

            break;

        uint8_t found = 0;
        for (int i = 0; i < close_count; i++)
        {
            if (close_list[i].ax == cur_x &&
                close_list[i].ay == cur_y &&
                close_list[i].dir == cur_dir)
            {
                cur_x = close_list[i].parent_ax;
                cur_y = close_list[i].parent_ay;
                cur_dir = close_list[i].parent_dir;
                found = 1;
                break;
            }
        }

        if (!found || len >= APATH_NODE)
        {
            *out_len = 0;
            return;
        }
    }

    // reverse
    for (int i = 0; i < len / 2; i++)
    {
        aPoint tmp = out_path[i];
        out_path[i] = out_path[len - 1 - i];
        out_path[len - 1 - i] = tmp;
    }

    *out_len = len;
}

/* ==================== A* ???? ==================== */
void get_astar_turn_path(aPoint *path, int *path_len)
{
    if (*path_len < 2)
        return;

    aPoint tmp[APATH_NODE];
    int16_t len = 0;
    int8_t last_dx = path[1].ax - path[0].ax;
    int8_t last_dy = path[1].ay - path[0].ay;
    tmp[len++] = path[0];

    for (int16_t i = 1; i < (*path_len) - 1; i++)
    {
        int8_t dx = path[i + 1].ax - path[i].ax;
        int8_t dy = path[i + 1].ay - path[i].ay;
        if (dx != last_dx || dy != last_dy)
        {
            tmp[len++] = path[i];
            last_dx = dx;
            last_dy = dy;
        }
    }

    tmp[len++] = path[(*path_len) - 1];
    for (int16_t i = 0; i < len; i++)
    {
        path[i] = tmp[i];
    }
    *path_len = len;
}

/* ==================== A* ????:??????,?????? ==================== */
int astar_integrate(uint8_t sx, uint8_t sy, uint8_t ex, uint8_t ey, aPoint *path, int *path_len)
{
    start_x = sx;
    start_y = sy;
    end_x = ex;
    end_y = ey;
    if (AStar() != 1)
        return -1;

    get_astar_path_to(path, path_len);
    if (*path_len == 0)
        return -1;
    return 1;
}

BoxNode box_open[BOX_STATE_MAX];
BoxNode box_close[BOX_STATE_MAX];
int16_t box_open_count;
int16_t box_close_count;
int8_t box_dir_x[4] = {0, 0, -1, 1};
int8_t box_dir_y[4] = {-1, 1, 0, 0};

int16_t box_find_close(uint8_t car_x, uint8_t car_y, uint8_t box_x, uint8_t box_y)
{
    for (int16_t i = 0; i < box_close_count; i++)
    {
        if (box_close[i].car_x == car_x &&
            box_close[i].car_y == car_y &&
            box_close[i].box_x == box_x &&
            box_close[i].box_y == box_y)
        {
            return i;
        }
    }
    return -1;
}

int16_t box_calc_h(uint8_t box_x, uint8_t box_y, uint8_t dst_x, uint8_t dst_y)
{
    return abs(box_x - dst_x) + abs(box_y - dst_y);
}

int16_t box_find_open(uint8_t car_x, uint8_t car_y, uint8_t box_x, uint8_t box_y)
{
    for (int16_t i = 0; i < box_open_count; i++)
    {
        if (box_open[i].car_x == car_x &&
            box_open[i].car_y == car_y &&
            box_open[i].box_x == box_x &&
            box_open[i].box_y == box_y)
            return i;
    }
    return -1;
}

int16_t box_find_min()
{
    int16_t min = -1;
    for (int16_t i = 0; i < box_open_count; i++)
    {
        if (min == -1 || box_open[i].f < box_open[min].f)
        {
            min = i;
        }
    }
    return min;
}

uint8_t box_can_move(int ax, int ay, uint8_t dst_x, uint8_t dst_y)
{
    if (ax < 0 || ay < 0 || ax >= COLS || ay >= ROWS)
        return 0;
    if (map_packet.map_data.map[ay][ax] == 2 && !(ax == dst_x && ay == dst_y))
        return 0;
    if (map_packet.map_data.map[ay][ax] == 3)
        return 0;
    if (map_packet.map_data.map[ay][ax] == 6)
        return 0;
    return 1;
}

int sokoban_search(uint8_t car_x, uint8_t car_y,
                   uint8_t box_x, uint8_t box_y,
                   uint8_t dst_x, uint8_t dst_y)
{
    box_open_count = 1;
    box_close_count = 0;

    box_open[0].car_x = car_x;
    box_open[0].car_y = car_y;
    box_open[0].box_x = box_x;
    box_open[0].box_y = box_y;
    box_open[0].g = 0;
    box_open[0].h = box_calc_h(box_x, box_y, dst_x, dst_y);
    box_open[0].f = box_open[0].h;
    box_open[0].parent = -1;

    memcpy(base_map, map_packet.map_data.map, sizeof(base_map));

    while (box_open_count)
    {

        int16_t cur = box_find_min();

        if (cur < 0)
            break;


        BoxNode now = box_open[cur];

        if (now.box_x == dst_x && now.box_y == dst_y)
        {
            memcpy(map_packet.map_data.map, base_map, sizeof(map_packet.map_data.map));
            box_close[box_close_count++] = now;
            return 1;
        }

        // 姣忔杩唬浠? base_map 鎭㈠锛岄伩鍏嶈法鍒嗘敮鏃剁瀛愭畫鐣?
        memcpy(map_packet.map_data.map, base_map, sizeof(map_packet.map_data.map));
        if (now.parent != -1)
        {
            map_packet.map_data.map[box_y][box_x] = 0;
            map_packet.map_data.map[now.box_y][now.box_x] = 3;
        }

        if (box_close_count >= BOX_STATE_MAX)
        {
            memcpy(map_packet.map_data.map, base_map, sizeof(map_packet.map_data.map));
            return -1;
        }

        box_close[box_close_count++] = now;
        box_open[cur] = box_open[--box_open_count];

        for (int dir = 0; dir < 4; dir++)
        {
            int new_box_x = now.box_x + box_dir_x[dir];
            int new_box_y = now.box_y + box_dir_y[dir];

            if (!box_can_move(new_box_x, new_box_y, dst_x, dst_y))
                continue;

            int need_car_x = now.box_x - box_dir_x[dir];
            int need_car_y = now.box_y - box_dir_y[dir];

            if (need_car_x < 0 || need_car_y < 0 ||
                need_car_x >= COLS || need_car_y >= ROWS)
                continue;

            if (map_packet.map_data.map[need_car_y][need_car_x] == 2 ||
                map_packet.map_data.map[need_car_y][need_car_x] == 3 ||
                map_packet.map_data.map[need_car_y][need_car_x] == 6)
            {
                continue;
            }

            start_x = now.car_x;
            start_y = now.car_y;
            end_x = need_car_x;
            end_y = need_car_y;

            if (AStar() != 1)
                continue;

            aPoint temp_path[APATH_NODE];
            int path_len = 0;
            get_astar_path_to(temp_path, &path_len);

            BoxNode next;
            next.car_x = now.box_x;
            next.car_y = now.box_y;
            next.box_x = new_box_x;
            next.box_y = new_box_y;

            next.g = now.g + path_len;
            next.h = box_calc_h(new_box_x, new_box_y, dst_x, dst_y);
            next.f = next.g + next.h;
            next.parent = box_close_count - 1;

            if (box_find_close(next.car_x, next.car_y,
                               next.box_x, next.box_y) >= 0)
                continue;

            int16_t idx = box_find_open(next.car_x, next.car_y,
                                    next.box_x, next.box_y);
            if (idx < 0)
            {
                if (box_open_count >= BOX_STATE_MAX)
                    continue;
                box_open[box_open_count++] = next;
            }

            else
            {
                if (next.g < box_open[idx].g)
                {
                    box_open[idx] = next;
                }
            }
        }
    }

    memcpy(map_packet.map_data.map, base_map, sizeof(map_packet.map_data.map));
    return -1;
}

aPoint abox_path[APATH_NODE];
int abox_path_len;

void get_box_path(aPoint *path, int *len)
{
    int16_t cur = box_close_count - 1;
    aPoint temp[APATH_NODE];
    int16_t count = 0;

    while (box_close[cur].parent != -1)
    {
        temp[count].ax = box_close[cur].box_x;
        temp[count].ay = box_close[cur].box_y;
        count++;
        cur = box_close[cur].parent;
        if (count >= APATH_NODE)
            break;
    }
	
    temp[count].ax = box_close[cur].box_x;
    temp[count].ay = box_close[cur].box_y;
    count++;

    // ??
    for (int16_t i = 0; i < count / 2; i++)
    {
        aPoint t = temp[i];
        temp[i] = temp[count - 1 - i];
        temp[count - 1 - i] = t;
    }
    for (int16_t i = 0; i < count; i++)
    {
        path[i] = temp[i];
    }
    *len = count;
}

/* ==================== 闁规亽鍔庨鍫??娑欏姃鐎靛矂宕楅妷銉ョ稉(A*闁???) ==================== */
int amake_box_path(void)
{
    int8_t car_x = (int8_t)roundf(mv_packet.data.car_x - 0.5f);
    int8_t car_y = (int8_t)roundf(mv_packet.data.car_y - 0.5f);
    int8_t box_x = (int8_t)roundf(target_box[0] - 0.5f);
    int8_t box_y = (int8_t)roundf(target_box[1] - 0.5f);
    int8_t dst_x = (int8_t)roundf(target_destination[0] - 0.5f);
    int8_t dst_y = (int8_t)roundf(target_destination[1] - 0.5f);

    if (sokoban_search(car_x, car_y, box_x, box_y, dst_x, dst_y) != 1)
        return -1;

    get_box_path(abox_path, &abox_path_len);
    get_astar_turn_path(abox_path, &abox_path_len);
    return 1;
}

aPoint acar_to_box_path[APATH_NODE];
int acar_to_box_path_len = 0;

int amake_car_to_box_path(uint8_t box_x, uint8_t box_y, uint8_t dst_x, uint8_t dst_y)
{
    int dx = dst_x - box_x;
    int dy = dst_y - box_y;
    int push_dx = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
    int push_dy = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
    uint8_t stand_x = box_x - push_dx;
    uint8_t stand_y = box_y - push_dy;

    uint8_t car_x = (uint8_t)roundf(mv_packet.data.car_x - 0.5f);
    uint8_t car_y = (uint8_t)roundf(mv_packet.data.car_y - 0.5f);

    if (astar_integrate(car_x, car_y, stand_x, stand_y,
                        acar_to_box_path, &acar_to_box_path_len) != 1)
        return 0;

    get_astar_turn_path(acar_to_box_path, &acar_to_box_path_len);
    return 1;
}

aPoint apush_path[APATH_NODE];
int apush_path_len = 0;

int amake_push_box_path(void)
{
    apush_path_len = 0;
    uint8_t car_x = acar_to_box_path[acar_to_box_path_len - 1].ax;
    uint8_t car_y = acar_to_box_path[acar_to_box_path_len - 1].ay;

    for (int16_t i = 0; i < abox_path_len - 1; i++)
    {
        uint8_t bx = abox_path[i].ax;
        uint8_t by = abox_path[i].ay;
        uint8_t nx = abox_path[i + 1].ax;
        uint8_t ny = abox_path[i + 1].ay;

        int dx = nx - bx;
        int dy = ny - by;
        int push_dx = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
        int push_dy = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
        uint8_t stand_x = bx - push_dx;
        uint8_t stand_y = by - push_dy;

        if (i > 0)
        {
            map_packet.map_data.map[abox_path[0].ay][abox_path[0].ax] = 0;
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
                get_astar_turn_path(change_path, &change_len);
                for (int16_t k = 1; k < change_len; k++)
                {
                    apush_path[apush_path_len++] = change_path[k];
                }
            }
        }
            map_packet.map_data.map[by][bx] = old;

        if (i < abox_path_len - 2)
        {
            map_packet.map_data.map[ny][nx] = 3;
        }

        // ????
        if (apush_path_len == 0 ||
            apush_path[apush_path_len - 1].ax != stand_x ||
            apush_path[apush_path_len - 1].ay != stand_y)
        {
            apush_path[apush_path_len].ax = stand_x;
            apush_path[apush_path_len].ay = stand_y;
            apush_path_len++;
        }

        apush_path[apush_path_len].ax = nx - push_dx;
        apush_path[apush_path_len].ay = ny - push_dy;
        apush_path_len++;

        car_x = nx - push_dx;
        car_y = ny - push_dy;
    }

    return 1;
}

/* ==================== amake_box_path 婵炴潙顑堥惁顖炲礄閼恒儲娈? ==================== */
/* 闂侇剙绉村濠氬箥閻愵剚浠橀弶鈺傚灩椤斿牏???+闁???閺夆晜鍨瑰ú浼存儍閸曨偅鍕鹃柨娑樻湰婢э箓宕℃０浣虹憦闁哄銈囩唴鐎??? */
void abox_test(void)
{
	static uint8_t done = 0;
	if (done)
		return;

	init_game_map();

	float car_x = mv_packet.data.car_x;
	float car_y = mv_packet.data.car_y;

	int16_t best_box_idx = -1;
	int16_t best_box_dist = 999;
	for (int i = 0; i < box_num; i++)
	{
		float bx = mv_packet.data.box[i][0];
		float by = mv_packet.data.box[i][1];
		if (bx == 0 && by == 0)
			continue;
		int16_t d = astar_distance(car_x, car_y, bx, by);
		if (d < best_box_dist)
		{
			best_box_dist = d;
			best_box_idx = i;
		}
	}

	if (best_box_idx == -1)
	{
		done = 1;
		return;
	}

	target_box[0] = mv_packet.data.box[best_box_idx][0];
	target_box[1] = mv_packet.data.box[best_box_idx][1];

	int16_t best_dst_idx = -1;
	int16_t best_dst_dist = 999;
	for (int i = 0; i < destination_num; i++)
	{
		float dx = mv_packet.data.destination[i][0];
		float dy = mv_packet.data.destination[i][1];
		if (dx == 0 && dy == 0)
			continue;
		int16_t d = astar_distance(target_box[0], target_box[1], dx, dy);
		if (d < best_dst_dist)
		{
			best_dst_dist = d;
			best_dst_idx = i;
		}
	}

	if (best_dst_idx == -1)
	{
		done = 1;
		return;
	}

	target_destination[0] = mv_packet.data.destination[best_dst_idx][0];
	target_destination[1] = mv_packet.data.destination[best_dst_idx][1];

	/* 3. 闁规亽鍔庨鍫??娑欏姃鐎靛瞼鎲撮崟顐㈢亰 */
	if (amake_box_path() != 1)
	{
		ips200_show_string(0, 0, "box_path FAIL");
		done = 1;
		return;
	}

	/* 4. 閺夌儑绠戦崺宀?绮婚崡鐐垫憤闁汇劌瀚惌鎯ь嚗? */
	if (amake_car_to_box_path(abox_path[0].ax, abox_path[0].ay,
	                          abox_path[1].ax, abox_path[1].ay) != 1)
	{
		ips200_show_string(0, 0, "car2box FAIL");
		done = 1;
		return;
	}

	/* 5. 闁汇垻鍠愰崹姘跺箳閵娧屽敻閻庢稒鍔曠花顓㈠礆? */
	if (amake_push_box_path() != 1)
	{
		ips200_show_string(0, 0, "push_path FAIL");
		done = 1;
		return;
	}
	get_astar_turn_path(apush_path, &apush_path_len);

	/* ===== 濞寸姰鍎扮粭鍛磼閻斿墎顏遍柡鍕⒔閵?? ===== */

	/* 缂???0閻???: 闁硅姤枪椤?? BX:x,y D:x,y b:N c:N p:N */
	ips200_show_string(0, 0, "BX");
	ips200_show_int(22, 0, (int)target_box[0], 2);
	ips200_show_int(44, 0, (int)target_box[1], 2);
	ips200_show_string(66, 0, "D");
	ips200_show_int(78, 0, (int)target_destination[0], 2);
	ips200_show_int(100, 0, (int)target_destination[1], 2);
	ips200_show_string(125, 0, "b");
	ips200_show_int(138, 0, abox_path_len, 2);
	ips200_show_string(160, 0, "c");
	ips200_show_int(172, 0, acar_to_box_path_len, 2);
	ips200_show_string(195, 0, "p");
	ips200_show_int(207, 0, apush_path_len, 2);

	/* 缂???1閻???: 閻犱警鍨扮欢?1 abox_path 缂佺姴宕悺娆戠矓鐠囨彃袟閻犱警鍨扮欢? */
	ips200_show_string(0, 18, "1");
	{
		int x_pos = 14;
		for (int i = 0; i < abox_path_len && i < 15; i++)
		{
			ips200_show_int(x_pos, 18, abox_path[i].ax, 2);
			ips200_show_int(x_pos, 36, abox_path[i].ay, 2);
			x_pos += 20;
		}
	}

	/* 缂???3閻???: 閻犱警鍨扮欢?2 acar_to_box_path 閺夌儑绠戦崺宀?绮婚崡鐐垫憤 */
	ips200_show_string(0, 56, "2");
	{
		int x_pos = 14;
		for (int i = 0; i < acar_to_box_path_len && i < 15; i++)
		{
			ips200_show_int(x_pos, 56, acar_to_box_path[i].ax, 2);
			ips200_show_int(x_pos, 74, acar_to_box_path[i].ay, 2);
			x_pos += 20;
		}
	}

	/* 缂???5閻???: 閻犱警鍨扮欢?3 apush_path 闁规亽鍔庨鍫??娑欏姇缁參宕?? */
	ips200_show_string(0, 94, "3");
	{
		int x_pos = 14;
		for (int i = 0; i < apush_path_len && i < 15; i++)
		{
			ips200_show_int(x_pos, 94, apush_path[i].ax, 2);
			ips200_show_int(x_pos, 112, apush_path[i].ay, 2);
			x_pos += 20;
		}
	}

	done = 1;
}

aPoint test_path[APATH_NODE];
int test_path_len = 0;
void a_test()
{
    init_game_map();
    start_x =(int)round(mv_packet.data.car_x - 0.5);
    start_y =(int)round(mv_packet.data.car_y - 0.5);
    end_x = 6;
    end_y = 9;
    AStar();
    get_astar_path_to(test_path, &test_path_len);
    get_astar_turn_path(test_path, &test_path_len);
    		int x_pos = 14;
		for (int i = 0; i < test_path_len && i < 15; i++)
		{
			ips200_show_int(x_pos, 280, test_path[i].ax, 2);
			ips200_show_int(x_pos, 300, test_path[i].ay, 2);
			x_pos += 20;
		}

}
