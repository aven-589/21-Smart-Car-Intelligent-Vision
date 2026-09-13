#ifndef _bomb_H
#define _bomb_H
#include "zf_common_headfile.h"
#include "Astar.h"
#define MAX_BOX_NUM   10
#define MAX_GOAL_NUM  10
#define MAX_BOMB_NUM  10
#define MAX_PAIR 10

#define MAX_STATE_PER_BOMB  50
typedef struct
{
    float box_x;
    float box_y;
    float goal_x;
    float goal_y;
    float bomb_x;
    float bomb_y;
    uint8_t dist;
    uint8_t bomb_dist;
} PairNode;

extern int16_t bomb_seq_len;
extern PairNode pair_list[MAX_PAIR];
extern int push_bomb_len;
extern int bomb_path_len;
extern int bomb_path_cost;
extern int bfs_car_to_bomb_path_len;
extern int final_bomb_path_len;
int check_bomb_lock(int *out_stuck_x, int *out_stuck_y);
int is_bomb_stuck(int bomb_x, int bomb_y);
int find_unlock_wall(int stuck_x, int stuck_y, int *out_wall_x, int *out_wall_y);
int make_bomb_path(int wall_x, int wall_y);
int make_push_bomb_path();
int make_car_to_bomb_path(int bomb_x, int bomb_y, int first_dst_x, int first_dst_y);

int bomb_pair();
int nearlist_box_index();
int make_unlock_path();
int make_free_bomb_path();

void bomb_apply(int cx, int cy, int backup[3][3], uint16_t map[16][12]);

void nearlist_bomb_get();

int make_new_bomb_path(void);

#endif
