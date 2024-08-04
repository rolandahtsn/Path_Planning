#include <iostream>
#include <fstream>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>

#include "GroundedCondition.hpp"

typedef unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> State;

struct state_comparison {
    bool operator()(const State& input_state, const State& input_state_1) const {
        auto size_state1 = input_state.size();
        auto size_state2 = input_state_1.size();
        
        if (size_state1 != size_state2)
            return false;

        return std::all_of(input_state.begin(), input_state.end(), [&](const GroundedCondition& gc) {
            return input_state_1.count(gc) > 0;
        });
    }
};

struct hashState {
    size_t operator()(const State& state) const {
        size_t hashValue = 0;
        for (const GroundedCondition& gc : state) {
                                                                   //Constant; ratio
            hashValue ^= std::hash<std::string>{}(gc.toString()) + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
        }
        return hashValue;
    }
};



//Changed Function --> Need to change names
struct Node {
    Node() {}
    Node(State s, Node* p, int g, int h) : curr_state(s), b_node(p), g_value(g), h_value(h) {}

    bool operator!=(const State& goal_state) const {
    return !std::all_of(goal_state.begin(), goal_state.end(), [&](const GroundedCondition& gc) {
        return curr_state.find(gc) != curr_state.end();
        });
    }
    
    bool compare_goal_start(const State& start_state, const State& goal_state) const;

    public:
        Node* b_node;
        State curr_state;
        GroundedAction action;
        int g_value;
        int h_value;
        int f_value;
        int parent_x;
        int parent_y;
};

bool compare_goal_start(const State& start_state, const State& goal_state){
    return std::all_of(goal_state.begin(), goal_state.end(), [&](const GroundedCondition& gc) {
        return start_state.find(gc) != start_state.end();
    });
}

State set_state(GroundedAction grounded_action, State curr_state) {
    GroundedAction g_actionCopy = grounded_action;
    bool operator_ = true;
    if (operator_){
        for (const auto& effects_state : grounded_action.getEffects()) {
            if (effects_state.get_truth() == operator_) {
                curr_state.insert(effects_state);
            } else {
                GroundedCondition negated_eff = effects_state;
                negated_eff.set_truth(operator_);
                curr_state.erase(negated_eff);
            }
        }
    }
    return curr_state;
}

struct CompareStates {
    bool operator()(Node* p1, Node* p2)
    {
        return p1->f_value > p2->f_value || (p1->f_value == p2->f_value && p1->h_value > p2->h_value);
    }
};



int is_move_possible(State curr_state, const GroundedAction& grounded_action1) {
    GroundedAction g_actionCopy = grounded_action1;
    return std::all_of(g_actionCopy.getPreconditions().begin(), g_actionCopy.getPreconditions().end(), [&](const GroundedCondition& a) {
        return curr_state.count(a) != 0;
    });
}




