#include "diff_mid_match.h"
#include "dif_prob.h"


//declare in prep_dc.c
extern PROB_t* SBOX_DDT;


void assign_values(BIT_SIZE_t sbox_o_word_bit_size, NUM_t num_sbox_in_a_state) {
    sbox_O_WORD_BIT_SIZE = sbox_o_word_bit_size;
    num_SBOX_IN_A_STATE = num_sbox_in_a_state;
}


PROB_t can_trans_sbox(char in, char out) {
    auto convert = [](char c) {
        return (c >= '0' && c <= '9') ? c - '0' : c - 'A' + 10;
    };
    int sbox_in = convert(in);
    int sbox_out = convert(out);
    SBOX_O_CNT_t max_out = (SBOX_O_CNT_t)(1 << sbox_O_WORD_BIT_SIZE) - 1;
    return SBOX_DDT[(SBOX_I_WRD_t)sbox_in * (SBOX_O_CNT_t)(max_out + 1) + (SBOX_O_WRD_t)sbox_out];
}


// ��ȡroad�Ļ�ԾS�еķֲ�λ��
std::vector<int> get_road_active_sboxes(std::string road) {
    std::vector<int> acts;
    for (int i = 0; i < road.size(); i++) {
        if (road[i] != '0') acts.push_back(i);
    }
    return acts;
}


std::vector<int> RoadsByActive::is_same_class(std::vector<int> acts) {
    std::vector<int> offsets;
    if (this->acts.size() != acts.size()) return offsets; // ��������������
    for (int i = 0; i < acts.size(); i++) {
        int offset = (acts[i] - this->acts[0] + len) % len;
        // ����Ƿ��ǺϷ���offset(ÿ��S�е����ƫ��Ӧһ��)
        bool flag = true;
        for (int j = 1; j < acts.size(); j++) {
            int offset_ = (acts[(i + j) % acts.size()] - this->acts[j] + len) % len;
            if (offset_ != offset) {
                flag = false;
                break;
            }
        }
        // ����Ϸ������¼
        if (flag) offsets.push_back(offset);
    }
    return offsets;
}


//�������������һ����Ϊ�˸��췵��
int RoadsByActive::get_one_offset(std::vector<int> acts) {
    if (this->acts.size() != acts.size()) return -1; // ��������������
    for (int i = 0; i < acts.size(); i++) {
        int offset = (acts[i] - this->acts[0] + len) % len;
        // ����Ƿ��ǺϷ���offset
        bool flag = true;
        for (int j = 1; j < acts.size(); j++) {
            int offset_ = (acts[(i + j) % acts.size()] - this->acts[j] + len) % len;
            if (offset_ != offset) {
                flag = false;
                break;
            }
        }
        // ����Ϸ������¼
        if (flag) return offset;
    }
    return -1;
}

void RoadsByActive::push_back(Road road, std::vector<int> offsets) {
    roads.push_back({ road, offsets });
}


PROB_t trans_sboxes(std::string a, std::string b, int offset, const std::vector<int>& active_sboxes, int len) {
    PROB_t mid_round_prob = ZERO_PROB;
    for (auto& sbox_idx : active_sboxes) {
        PROB_t one_sbox_prob = can_trans_sbox(a[(sbox_idx + offset) % len], b[sbox_idx]);
        if (one_sbox_prob == ZERO_PROB) return ZERO_PROB;
        else {
            if (mid_round_prob == ZERO_PROB) mid_round_prob = one_sbox_prob;
            else mid_round_prob += one_sbox_prob;
        }
    }
    return mid_round_prob; // ���ظ���
}


//Ԥ������ߵ�·�ߣ�����
void preTreat_left_trails(const std::map<std::string, std::map<std::string, PROB_t>>& left_roads, std::vector<RoadsByActive>* classified_left_roads) {
    for (auto& start : left_roads) {
        for (auto& end : start.second) {
            // ��ȡ��ԾS�еķֲ�λ��
            auto active_sboxes = get_road_active_sboxes(end.first);
            // �������ڶ�Ӧ�����е���һ��
            bool flag = false;
            for (auto& roadsByActive : classified_left_roads[active_sboxes.size()]) {
                // �����ͬһ�࣬offsets�ǿ�
                auto offsets = roadsByActive.is_same_class(active_sboxes);
                if (offsets.size()) {
                    roadsByActive.push_back(Road(start.first, end.first, end.second), offsets);
                    flag = true;
                    break;
                }
            }
            // ����������κ�һ�࣬�ʹ����µ�һ�ಢ�洢
            if (!flag) {
                RoadsByActive roadsByActive(num_SBOX_IN_A_STATE, active_sboxes);
                auto offsets = roadsByActive.is_same_class(active_sboxes);
                roadsByActive.push_back(Road(start.first, end.first, end.second), offsets);
                classified_left_roads[active_sboxes.size()].push_back(roadsByActive);
            }
        }
    }
}


//�ұߵ�һ��·������ߵ�·��������ײ����ƥ��
std::vector<Road> find_collision(std::string right_tail, std::string right_head, PROB_t right_trail_prob, std::vector<RoadsByActive>* classified_left_roads) {
    std::vector<Road> roads;
    // ��ȡ��ԾS�еķֲ�λ��
    auto active_sboxes = get_road_active_sboxes(right_tail);
    // ȥ��Ӧ�Ļ�ԾS�и�������𼯺��������λ��ƥ���
    for (auto& roadsByActive : classified_left_roads[active_sboxes.size()]) {
        auto one_offset = roadsByActive.get_one_offset(active_sboxes);
        // ����ҵ���
        if (one_offset != -1) {
            // ����������������
            for (auto& road : roadsByActive.roads) {
                // �������е�ƥ��������ҵ������ʣ��ǵ��жϸ���Ϊ0�������
                PROB_t max_prob = ZERO_PROB;
                for (auto offset : road.second) {
                    offset = (offset - one_offset + num_SBOX_IN_A_STATE) % num_SBOX_IN_A_STATE;
                    PROB_t mid_round_prob = trans_sboxes(road.first.end, right_tail, offset, active_sboxes, num_SBOX_IN_A_STATE);
                    if (mid_round_prob == ZERO_PROB) continue;
                    PROB_t one_meet_trail_prob = mid_round_prob + road.first.prob + right_trail_prob;
                    if (max_prob == ZERO_PROB || max_prob < one_meet_trail_prob) max_prob = one_meet_trail_prob;
                }
                // ����ҵ�
                if (max_prob != ZERO_PROB) {
                    roads.push_back(Road(road.first.begin, right_head, max_prob));
                }
            }
            // �����ٺ�������ƥ���ˣ�ֱ��break
            break;
        }
    }
    return roads;
}