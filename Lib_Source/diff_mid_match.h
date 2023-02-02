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

// ��ȡroad�Ļ�ԾS�еķֲ�λ��
std::vector<int> get_road_active_sboxes(std::string road);

struct Road {
    std::string begin;
    std::string end;
    PROB_t prob;
    Road(std::string begin, std::string end, PROB_t prob) : begin(begin), end(end), prob(prob) {}
};

struct RoadsByActive {
    int len;        // һ��״̬�ĳ���(�����ֵĸ���)
    std::vector<int> acts;
    std::vector<std::pair<Road, std::vector<int>>> roads;

    RoadsByActive(int len, std::vector<int> acts) : len(len) {
        this->acts = acts;
    }

    std::vector<int> is_same_class(std::vector<int> acts);

    //�������������һ����Ϊ�˸��췵��
    int get_one_offset(std::vector<int> acts);

    void push_back(Road road, std::vector<int> offsets);
};


PROB_t trans_sboxes(std::string a, std::string b, int offset, const std::vector<int>& active_sboxes, int len);

//Ԥ������ߵ�·�ߣ�����
void preTreat_left_trails(const std::map<std::string, std::map<std::string, PROB_t>>& left_roads, std::vector<RoadsByActive>* classified_left_roads);

//�ұߵ�һ��·������ߵ�·��������ײ����ƥ��
std::vector<Road> find_collision(std::string right_tail, std::string right_head, PROB_t right_trail_prob, std::vector<RoadsByActive>* classified_left_roads);

#endif // !__DIFF_MID_MATCH_CPP__
