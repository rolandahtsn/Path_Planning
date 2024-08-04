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
#include <chrono>
#include <numeric>
#include "planner_functions.hpp"

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class Condition;
class Action;
class Env;

using namespace std;

bool print_status = true;

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_conditions() const
    {
        return this->initial_conditions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_conditions() const
    {
        return this->goal_conditions;
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_actions() const
    {
        return this->actions;
    }


    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

auto get_permutations(const std::unordered_set<std::string>& symbols)  {
    vector<vector<list<string>>> get_perm;

    function<void(int)> generate_permutations = [&](int num) {
        auto list_permutations = std::list<string> {};
        auto permutations = std::vector<list<string>> {};

        function<void(int)> generate = [&](int remaining) {
            if (remaining == 0) {
                permutations.push_back(list_permutations);
            } else {
                for (const auto& symbol : symbols) {
                    if (find(list_permutations.begin(), list_permutations.end(), symbol) == list_permutations.end()) {
                        list_permutations.push_back(symbol);
                        generate(remaining - 1);
                        list_permutations.pop_back();
                    }
                }
            }
        };

        generate(num);
        get_perm.push_back(permutations);
    };

    // for (int i = 1; i <= symbols.size(); i++) {
    //     generate_permutations(i);
    // }
    std::vector<int> indices(symbols.size());
    std::iota(indices.begin(), indices.end(), 1);

    for (int i : indices) {
        generate_permutations(i);
    }
    return get_perm;
}

auto get_grounded_actions(const unordered_set<Action, ActionHasher, ActionComparator>& actions, const unordered_set<string>& symbols) {
    
    list<GroundedAction> grounded_actions;
    auto list_of_permutations = get_permutations(symbols);

    for (auto actionIt = actions.begin(); actionIt != actions.end(); ++actionIt) {
        const auto& action = *actionIt;
        auto arg_size = action.get_args().size();

        int size_iteration = arg_size - 1;
        int num_args = size_iteration;
        list<string> g_names = action.get_args();
        
        auto argsBegin = std::begin(list_of_permutations[num_args]);
        auto argsEnd = std::end(list_of_permutations[num_args]);
        auto eff = unordered_set<Condition, ConditionHasher, ConditionComparator> {};
        eff = action.get_effects();

        auto cond = unordered_set<Condition, ConditionHasher, ConditionComparator> {};
        cond = action.get_preconditions();

        for (auto argsIter = argsBegin; argsIter != argsEnd; ++argsIter) {
            auto& arguments_ = *argsIter;
            std::string name_actions = action.get_name();
            auto un_maps = std::unordered_map<string, string> {};
            GroundedAction ga(name_actions, arguments_);

            

            auto it = symbols.begin();
            while (it != symbols.end()) {
                const auto& s = *it;
                un_maps[s] = s;
                ++it;
            }

            auto arg_iter = arguments_.begin();
            std::transform(
                g_names.begin(), g_names.end(),
                std::inserter(un_maps, un_maps.begin()),
                [&arg_iter](const auto& g_names) { return std::make_pair(g_names, *arg_iter++); }
            );

            for (const auto& conditions : {std::cref(cond), std::cref(eff)}) {
                for (const auto& i : conditions.get()) {
                    list<string> cond_args;
                    for (const auto& k : i.get_args()) {
                        cond_args.push_back(un_maps[k]);
                    }
                    auto predicate_ = i.get_predicate();
                    auto truth_statement = i.get_truth();
                    GroundedCondition g(predicate_, cond_args, truth_statement);
                    (&conditions.get() == &cond) ? ga.add_gpre(g) : ga.add_geff(g);
                }
            }
            grounded_actions.insert(grounded_actions.end(), ga);
        }
    }

    return grounded_actions;
}

std::list<GroundedAction> performTraceback(const Node* goal_state) {
    std::list<GroundedAction> path_of_actions;

    // Define a lambda function to avoid repeating the loop logic
    auto trace = [&]() {
        path_of_actions.push_back(goal_state->action);
        goal_state = goal_state->b_node;
    };

    while (goal_state != nullptr) {
        trace();
    }

    path_of_actions.reverse();
    return path_of_actions;
}

// Option 2 Heuristic: Astar Heuristic Need to change look into
int h_astar(list<GroundedAction>grounded_actions, State start_state, State goal_state)
{
    auto open_list = std::priority_queue<Node*, vector<Node*>, CompareStates> {};
    auto closed_list = std::unordered_map<State, Node*, hashState, state_comparison> {}; //Use a map to avoid searching through a list

    auto start_time = std::chrono::high_resolution_clock::now();

    int found_goal = 0;
    Node* goal_node = new Node();
    Node* start_state_1 = new Node(start_state, nullptr, found_goal, found_goal); //G_value to start state is 0

    open_list.emplace(start_state_1);
    bool found = false;


    for (; !open_list.empty(); open_list.pop()) {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = current_time - start_time;

        Node *min_f_state = open_list.top();

        auto closed_list_iter = closed_list.find(min_f_state->curr_state);
        if (closed_list_iter == closed_list.end()) {
            closed_list[min_f_state->curr_state] = min_f_state;

            double min_f = std::numeric_limits<double>::max();
            std::string state_identifier = std::to_string(min_f_state->parent_x) + "," + std::to_string(min_f_state->parent_y);

            for (auto ga_it = grounded_actions.begin(); ga_it != grounded_actions.end(); ++ga_it) {
                const auto &ga_1 = *ga_it;
                int move_valid = is_move_possible(min_f_state->curr_state, ga_1);

                if (move_valid) {
                    int updated_g_value = min_f_state->g_value + 1;
                    auto State_update = min_f_state->curr_state;
                    State updated_state = set_state(ga_1, State_update);
                    Node *updated_node = new Node(updated_state, min_f_state, updated_g_value, 0);

                    updated_node->f_value = updated_node->g_value;

                    updated_node->action = ga_1;
                    open_list.emplace(new Node(updated_state, min_f_state, updated_g_value, 0));

                    if (!(updated_node->curr_state != goal_state)) {
                        int updated_g = updated_node->g_value;
                        return updated_g;
                    }
                }
            }
        }
    }
    return 0;
}


// Heuristic Function
int h_astar_(State start_state, State goal_state) {
    return std::count_if(goal_state.begin(), goal_state.end(), [&start_state](const GroundedCondition& gc) {
        return start_state.find(gc) == start_state.end();
    });
}

list<GroundedAction> planner(Env* env)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    list<GroundedAction>grounded_actions;
    auto env_actions = env->get_actions();
    auto env_symbols = env->get_symbols();

    list<GroundedAction>all_gA = get_grounded_actions(env_actions, env_symbols);

    
    std::priority_queue<Node*, vector<Node*>, CompareStates> open_list;
    std::unordered_map<State, Node*, hashState, state_comparison> closed_list; //Use a map to avoid searching through a list
    unordered_map<State, Node*, hashState, state_comparison>previous_states_map;


    
    

    State goal_conditions = env->get_goal_conditions();
    State start_conditions = env->get_initial_conditions();

    Node* goal_state = new Node();
    Node* start_state = new Node(start_conditions, NULL, 0, 0);

    start_state->f_value = start_state->g_value;

    open_list.emplace(start_state);
    previous_states_map.insert({ start_conditions, start_state });
    int found_goal = false;

    while (!open_list.empty() && !found_goal) {
        Node* current = open_list.top();
        Node *min_f_state = current;
        auto closed_list_iter = closed_list.find(min_f_state->curr_state);

        open_list.pop();

        if (closed_list_iter == closed_list.end()) {
            closed_list[min_f_state->curr_state] = min_f_state;
            for (auto ga_it = all_gA.begin(); ga_it != all_gA.end(); ++ga_it) {
                const auto &ga_1 = *ga_it;
                int move_valid = is_move_possible(min_f_state->curr_state, ga_1);

                if (move_valid) {
                    auto current_state = current->curr_state;
                    State new_state = set_state(ga_1, current_state);

                    if (!previous_states_map.count(new_state)) {
                        Node* updated_node = new Node();
                        
                        // else set h-value = 0; meaning no heuristic is used
                        int h_value = h_astar_(new_state, goal_conditions);
                        int update_g = current->g_value + 1;
                        updated_node = new Node(new_state, current, update_g,h_value); //This line too
                        
                        // Comment out these lines to run w/o heuristics
                        int updated_f = updated_node->g_value+updated_node->h_value;
                        updated_node->f_value = updated_f;

                        // int h_value = 10 * h_astar(all_gA, new_state, goal_conditions);
                        // updated_node = new Node(new_state, current, update_g, h_value);

                        //-- Uncomment these lines to run w/o heuristics
                        // updated_node = new Node(new_state, current, update_g, 0);
                        // int updated_f = updated_node->g_value;
                        // updated_node->f_value = updated_f;

                        updated_node->action = ga_1;
                        previous_states_map.insert({ new_state, updated_node });
                        open_list.push(updated_node);

                        if (compare_goal_start(updated_node->curr_state, goal_conditions)) {
                            found_goal = true;
                            goal_state = updated_node;
                            break;
                        }
                    }
                    else {

                        if (closed_list.count(new_state) == 0) {

                            if (previous_states_map[new_state]->g_value > current->g_value + 1) {
                                int g_value = current->g_value + 1;
                                auto b_node = current;
                                previous_states_map[new_state]->action = ga_1;
                                
                                //Comment out this line to run w/o heuristics
                                auto h_val = previous_states_map[new_state]->h_value;
                                int f_value = g_value + h_val;
                                previous_states_map[new_state] = new Node(new_state, current, g_value, h_val);

                                //Comment out this line to run w/o heuristics
                                previous_states_map[new_state]->f_value = previous_states_map[new_state]->g_value + 10*previous_states_map[new_state]->h_value;

                                //-- Uncomment these lines to run w/o heuristics
                                // previous_states_map[new_state] = new Node(new_state, current, g_value, 0);
                                // previous_states_map[new_state]->f_value = previous_states_map[new_state]->g_value; 
                                // open_list.emplace(std::make_unique<Node>(new_state, current, g_value, 0).get());
                            }
                        }

                    }

                }


            }
            if (found_goal) {
                
                auto path_actions = performTraceback(goal_state);
                // Uncomment these lines to print results

                auto current_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed_seconds = current_time - start_time;

                std::cout << "Elapsed time: " << elapsed_seconds.count() << std::endl;
                std::cout << "Number of States Expanded: " << closed_list.size()<< std::endl;
                return grounded_actions = path_actions;
            }

        }

    }
    return grounded_actions;
}





























int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("FireExtinguisher.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}