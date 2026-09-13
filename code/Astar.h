#ifndef _Astar_H
#define _Astar_H

#include "zf_common_headfile.h"

#define ROWS 16
#define COLS 12
#define MAX_NODE 400
#define APATH_NODE 300

typedef struct
{
    uint8_t ax;
    uint8_t ay;
    int8_t dir;
    int16_t g;
    int16_t h;
    int16_t f;
    uint8_t parent_ax;
    uint8_t parent_ay;
    int8_t parent_dir;
} ANode;

typedef struct
{
    uint8_t ax;
    uint8_t ay;
} aPoint;

#define BOX_STATE_MAX 2000

typedef struct
{
    uint8_t car_x;
    uint8_t car_y;
    uint8_t box_x;
    uint8_t box_y;
    int16_t g;
    int16_t h;
    int16_t f;
    int16_t parent;
} BoxNode;

extern int8_t start_x, start_y;
extern int8_t end_x, end_y;

extern int acar_to_box_path_len;
extern int apush_path_len;
extern int push_box_cost;
extern aPoint apush_path[APATH_NODE];
extern aPoint acar_to_box_path[APATH_NODE];

extern uint16_t game_map[ROWS][COLS];
extern uint16_t base_map[ROWS][COLS];
extern uint16_t bomb_map[ROWS][COLS];
extern int16_t open_count;
extern int16_t close_count;
extern int8_t end_dir;
void init_game_map(void);
void show_map_number(void);
int AStar(void);
int astar_integrate(uint8_t sx, uint8_t sy, uint8_t ex, uint8_t ey, aPoint *path, int *path_len);
void get_astar_path_to(aPoint *out_path, int *out_len);
void get_astar_turn_path(aPoint *path, int *path_len);
void get_box_path(aPoint *path, int *len);
int astar_distance(float sx, float sy, float ex, float ey);
int sokoban_search(uint8_t car_x, uint8_t car_y, uint8_t box_x, uint8_t box_y, uint8_t dst_x, uint8_t dst_y);
int amake_box_path(void);
int amake_car_to_box_path(uint8_t box_x, uint8_t box_y, uint8_t dst_x, uint8_t dst_y);
int amake_push_box_path(void);

void abox_test(void);

#endif
