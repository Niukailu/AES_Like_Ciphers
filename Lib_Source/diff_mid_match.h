#ifndef __DIFF_MID_MATCH_CPP__
#define __DIFF_MID_MATCH_CPP__

#include "astbb.h"
#include <iostream>
#include <map>
#include <vector>

/*Variables only for the current c code region*/
static BIT_SIZE_t sbox_O_WORD_BIT_SIZE;
static NUM_t num_SBOX_IN_A_STATE;

void assign_values(BIT_SIZE_t sbox_o_word_bit_size, NUM_t num_sbox_in_a_state);

PROB_t can_trans_sbox(char in, char out);

// 获取road的活跃S盒的分布位置
std::vector<int> get_road_active_sboxes(std::string road);

struct Road {
    std::string begin;
    std::string end;
    PROB_t prob;
    Road(std::string begin, std::string end, PROB_t prob) : begin(begin), end(end), prob(prob) {}
};

struct RoadsByActive {
    int len;        // 一个状态的长度(包含字的个数)
    std::vector<int> acts;
    std::vector<std::pair<Road, std::vector<int>>> roads;

    RoadsByActive(int len, std::vector<int> acts) : len(len) {
        this->acts = acts;
    }

    std::vector<int> is_same_class(std::vector<int> acts);

    //这个函数和上面一样，为了更快返回
    int get_one_offset(std::vector<int> acts);

    void push_back(Road road, std::vector<int> offsets);
};


PROB_t trans_sboxes(std::string a, std::string b, int offset, const std::vector<int>& active_sboxes, int len);

//预处理左边的路线，分类
void preTreat_left_trails(const std::map<std::string, std::map<std::string, PROB_t>>& left_roads, std::vector<RoadsByActive>* classified_left_roads);

//右边的一条路径和左边的路径集合碰撞，找匹配
std::vector<Road> find_collision(std::string right_tail, std::string right_head, PROB_t right_trail_prob, std::vector<RoadsByActive>* classified_left_roads);

#endif // !__DIFF_MID_MATCH_CPP__
