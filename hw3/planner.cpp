#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <chrono>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6
#ifndef ENVS_DIR
#define ENVS_DIR "../envs"
#endif
class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(const string &predicate, const list<string> &arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth; // fixed
        for (const string &l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition &gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth; // fixed
        for (const string &l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const GroundedCondition &pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition &rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        temp += this->predicate;
        temp += "(";
        for (const string &l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition &lhs, const GroundedCondition &rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition &gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(const string &pred, const list<string> &args, const bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (const string &ar : args)
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

    friend ostream &operator<<(ostream &os, const Condition &cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition &rhs) const // fixed
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
        string temp;
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (const string &l : this->args)
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
    bool operator()(const Condition &lhs, const Condition &rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition &cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(const string &name, const list<string> &args,
           const unordered_set<Condition, ConditionHasher, ConditionComparator> &preconditions,
           const unordered_set<Condition, ConditionHasher, ConditionComparator> &effects)
    {
        this->name = name;
        for (const string &l : args)
        {
            this->args.push_back(l);
        }
        for (const Condition &pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (const Condition &pc : effects)
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

    bool operator==(const Action &rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream &operator<<(ostream &os, const Action &ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (const Condition &precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (const Condition &effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->get_name();
        temp += "(";
        for (const string &l : this->get_args())
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
    bool operator()(const Action &lhs, const Action &rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action &ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(const GroundedCondition &gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(const GroundedCondition &gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(const GroundedCondition &gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(const GroundedCondition &gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(const string &symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(const list<string> &symbols)
    {
        for (const string &l : symbols)
            this->symbols.insert(l);
    }
    void add_action(const Action &action)
    {
        this->actions.insert(action);
    }

    Action get_action(const string &name) const
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

    friend ostream &operator<<(ostream &os, const Env &w)
    {
        os << "***** Environment *****" << endl
           << endl;
        os << "Symbols: ";
        for (const string &s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (const GroundedCondition &s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (const GroundedCondition &g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (const Action &g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }

    // add getters
    auto get_initial_conditions() const
    {
        return this->initial_conditions;
    }
    auto get_goal_conditions() const
    {
        return this->goal_conditions;
    }
    auto get_actions() const
    {
        return this->actions;
    }
};

class GroundedAction
{
    string name;
    list<string> arg_values;

public:
    GroundedAction(const string &name, const list<string> &arg_values)
    {
        this->name = name;
        for (const string &ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction &rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream &operator<<(ostream &os, const GroundedAction &gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->name;
        temp += "(";
        for (const string &l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
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

Env *create_env(char *filename)
{
    ifstream input_file(filename);
    Env *env = new Env();
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

            if (line.empty())
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str())); // fixed

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
                const char *line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
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
                const char *line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
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
                const char *line_c = line.c_str();
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
                const char *line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
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
                const char *line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = {1, 2};
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
                const char *line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = {1, 2};
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

// list<GroundedAction> planner(Env* env)
// {
//     //////////////////////////////////////////
//     ///// TODO: INSERT YOUR PLANNER HERE /////
//     //////////////////////////////////////////

//     // Blocks World example (TODO: CHANGE THIS)
//     cout << endl << "CREATING DEFAULT PLAN" << endl;
//     list<GroundedAction> actions;
//     actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
//     actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
//     actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

//     return actions;
// }

// Helper class to represent a search state
class PlannerState
{
public:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> conditions;
    list<GroundedAction> actions_to_reach;
    int g_cost; // Path cost
    int h_cost; // Heuristic cost

    PlannerState(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &conditions,
                 const list<GroundedAction> &actions,
                 int g = 0, int h = 0) : conditions(conditions), actions_to_reach(actions), g_cost(g), h_cost(h)
    {
    }

    int f_cost() const { return g_cost + h_cost; }

    bool operator==(const PlannerState &other) const
    {
        return conditions == other.conditions;
    }
};

// Custom hash function for PlannerState
struct PlannerStateHasher
{
    size_t operator()(const PlannerState &state) const
    {
        size_t hash_val = 0;
        for (const auto &cond : state.conditions)
        {
            hash_val ^= GroundedConditionHasher()(cond);
        }
        return hash_val;
    }
};

// Comparison function for priority queue
struct PlannerStateComparator
{
    bool operator()(const PlannerState &a, const PlannerState &b) const
    {
        return a.f_cost() > b.f_cost(); // Min-heap
    }
};

// 
// Check if a grounded action is applicable in the current state
bool is_applicable(const Action &action, const GroundedAction &ground_action,
                   const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &state)
{
    // Map action parameters to grounded values

    // // cout << "debug statement 2.1"<<endl;
    unordered_map<string, string> param_map;

    list<string> params = action.get_args();

    // Print out parameters for debugging
    // cout << "Action parameters:" << endl;
    // for (const string& param : params) {
    //     cout << "  " << param << endl;
    // }

    list<string> grounded_args = ground_action.get_arg_values();
    // Print the grounded arguments for debugging
    // cout << "Grounded arguments:" << endl;
    // for (const string& arg : grounded_args) {
    //     cout << "  " << arg << endl;
    // }

    // Check for duplicate arguments in grounded_action
    auto args = ground_action.get_arg_values();
    set<string> unique_args;
    for (const string &arg : args)
    {
        if (!unique_args.insert(arg).second)
        {
            return false; // Found a duplicate argument
        }
    }

    while (!params.empty())
    {
        if (grounded_args.front() == "")
        {
            param_map[params.front()] = params.front();
        }
        else
        {
            param_map[params.front()] = grounded_args.front();
        }
        params.pop_front();
        grounded_args.pop_front();
    }

    for (const Condition &precond : action.get_preconditions())
    {
        // Ground the precondition
        list<string> grounded_args;
        list<GroundedCondition> grounded_preconditions;
        for (const string &arg : precond.get_args())
        {
            if (param_map.find(arg) == param_map.end())
            {
                grounded_args.push_back(arg);
            }
            else
            {
                grounded_args.push_back(param_map[arg]);
            }
        }
        // for (const string &arg : grounded_args)
        // {
        //     cout << "  " << arg;
        // }
        // cout << endl;
        // cout << "grounded_args: "<<grounded_args.size()<<endl;

        // cout << "debug statement 2.3" << endl;
        GroundedCondition ground_precond(precond.get_predicate(), grounded_args, precond.get_truth());

        // cout << "Ground precondition: " << ground_precond << endl;

        if (precond.get_truth() && state.find(ground_precond) == state.end())
        {
            // cout << "Precondition not found: " << ground_precond << endl;
            return false;
        }
        else if (!precond.get_truth() && state.find(ground_precond) != state.end())
        {
            // cout << "Precondition found: " << ground_precond << endl;
            return false;
        }
        // // Check if precondition holds in current state
        // bool found = (state.find(ground_precond) != state.end());
        // cout << "Found: "<<found<<endl;
        // if (precond.get_truth() != found) {
        //     return false;
        // }

        // break;
    }

    // // cout << "debug statement 2.4" << endl;

    return true;
}

// Apply a grounded action to generate the successor state
unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>
apply_action(const Action &action, const GroundedAction &ground_action,
             const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &state)
{

    // cout << "Applying action: " << ground_action << endl;

    auto new_state = state;

    // Map action parameters to grounded values
    unordered_map<string, string> param_map;
    auto param_it = action.get_args().begin();
    auto val_it = ground_action.get_arg_values().begin();

    // // cout << "debug statement 1" << endl;

    list<string> grounded_args = ground_action.get_arg_values();
    list<string> params = action.get_args();
    while (!params.empty())
    {
        if (grounded_args.front() == "")
        {
            param_map[params.front()] = params.front();
        }
        else
        {
            param_map[params.front()] = grounded_args.front();
        }
        params.pop_front();
        grounded_args.pop_front();
    }

    // // cout << "debug statement 2" << endl;
    // Apply effects
    for (const Condition &effect : action.get_effects())
    {
        // Ground the effect
        list<string> grounded_args;
        for (const string &arg : effect.get_args())
        {
            if (param_map.find(arg) == param_map.end())
            {
                grounded_args.push_back(arg);
            }
            else
            {
                grounded_args.push_back(param_map[arg]);
            }
        }

        // // cout << "debug statement 3" << endl;
        GroundedCondition ground_effect(effect.get_predicate(), grounded_args, effect.get_truth());
        // // cout << "debug statement 4" << endl;
        if (!effect.get_truth())
        {
            // Remove negated conditions
            new_state.erase(GroundedCondition(effect.get_predicate(), grounded_args, true));
        }
        else
        {
            new_state.insert(ground_effect);
        }
    }
    // // cout << "debug statement 5" << endl;

    return new_state;
}

// Heuristic function: count unsatisfied goals
int calculate_heuristic(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &state,
                        const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &goals)
{
    int unsatisfied = 0;
    for (const auto &goal : goals)
    {
        if (state.find(goal) == state.end())
        {
            unsatisfied++;
        }
    }
    return unsatisfied;
}
// Function to generate all possible grounded actions
vector<GroundedAction> generate_all_possible_ground_actions(
    const unordered_set<Action, ActionHasher, ActionComparator> &actions,
    const unordered_set<string> &symbols)
{

    vector<GroundedAction> all_ground_actions;

    vector<Action> action_grounded_action_map;

    // Iterate through each action
    for (const Action &action : actions)
    {
        // Get number of parameters
        int num_params = action.get_args().size();

        // Generate all possible combinations of symbols
        vector<vector<string>> param_combinations;
        vector<string> current_combo;

        function<void(int)> generate_combinations = [&](int depth)
        {
            if (depth == num_params)
            {
                param_combinations.push_back(current_combo);
                return;
            }

            for (const string &symbol : symbols)
            {
                current_combo.push_back(symbol);
                generate_combinations(depth + 1);
                current_combo.pop_back();
            }
        };

        generate_combinations(0);

        // Create grounded actions from combinations
        for (const auto &combo : param_combinations)
        {
            list<string> params(combo.begin(), combo.end());
            all_ground_actions.push_back(GroundedAction(action.get_name(), params));
        }
    }

    return all_ground_actions;
}
// Main planning function

// bool is_applicable(const Action& action, const GroundedAction& ground_action,
//                   const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& state) {

//     unordered_map<string, string> param_map;

//     // Create parameter mapping from action parameters to grounded values
//     auto param_it = action.get_args().begin();
//     auto val_it = ground_action.get_arg_values().begin();
//     while (param_it != action.get_args().end()) {
//         param_map[*param_it] = *val_it;
//         ++param_it;
//         ++val_it;
//     }
//     // Check each precondition
//     for (const Condition& precond : action.get_preconditions()) {
//         // Ground the precondition arguments
//         list<string> grounded_args;
//         for (const string& arg : precond.get_args()) {
//             grounded_args.push_back(param_map[arg]);
//         }

//         // Create grounded precondition
//         GroundedCondition ground_precond(precond.get_predicate(), grounded_args, precond.get_truth());

//         // Check if precondition holds in current state
//         bool found = (state.find(ground_precond) != state.end());
//         if (precond.get_truth() != found) {
//             return false;
//         }
//     }

//     return true;
// }

list<GroundedAction> planner(Env *env)
{
    int nodes_expanded = 0;
    // Initialize search
    // cout << "debug statement 1" << endl;

    priority_queue<PlannerState, vector<PlannerState>, PlannerStateComparator> open_list;
    unordered_set<PlannerState, PlannerStateHasher> closed_list;

    // Create initial state
    PlannerState initial_state(env->get_initial_conditions(), list<GroundedAction>(), 0,
                               calculate_heuristic(env->get_initial_conditions(), env->get_goal_conditions()));

    // cout << "debug statement 2" << endl;

    open_list.push(initial_state);

    vector<GroundedAction> all_possible_actions = generate_all_possible_ground_actions(env->get_actions(), env->get_symbols());

    // cout << "debug statement 3" << endl;

    while (!open_list.empty())
    {
        // Get the state with lowest f-cost
        PlannerState current = open_list.top();
        open_list.pop();

        // Check if we've reached the goal
        bool is_goal = true;
        for (const auto &goal : env->get_goal_conditions())
        {
            if (current.conditions.find(goal) == current.conditions.end())
            {
                is_goal = false;
                break;
            }
        }
        // cout << "debug statement 3.1" << endl;
        if (is_goal)
        {
            if (print_status)
            {
                cout << "Goal found! Plan length: " << current.actions_to_reach.size() << endl;
                cout << "Nodes expanded: " << nodes_expanded << endl;

            }
            return current.actions_to_reach;
        }

        // Skip if we've seen this state

        if (!closed_list.empty() && closed_list.find(current) != closed_list.end())
        {
            continue;
        }
        closed_list.insert(current);
        nodes_expanded++;

        // cout << "debug statement 4 " << endl;

        for (const GroundedAction &ground_action : all_possible_actions)
        {

            // cout << "debug statement 4.1 " << endl;
            const Action &action = env->get_action(ground_action.get_name());

            // cout << "debug statement 5 " << endl;

            if (!is_applicable(action, ground_action, current.conditions))
            {
                continue;
            }

            // cout << "debug statement 6 " << endl;

            // Generate successor state
            auto successor_conditions = apply_action(action, ground_action, current.conditions);

            // cout << "debug statement 7 " << endl;

            // Create new state with updated path
            auto successor_actions = current.actions_to_reach;
            successor_actions.push_back(ground_action);

            // cout << "debug statement 8 " << endl;

            PlannerState successor(successor_conditions, successor_actions,
                                   current.g_cost + 1,
                                   calculate_heuristic(successor_conditions, env->get_goal_conditions()));

            // cout << "debug statement 9 " << endl;
            // Add to open list if we haven't seen it
            if (closed_list.find(successor) == closed_list.end())
            {
                open_list.push(successor);
            }

            // cout << "debug statement 7 " << endl;
        }

        // // Try all possible actions
        // for (const Action& action : env->get_actions()) {
        //     // Generate all possible groundings of this action
        //     auto ground_actions = generate_ground_actions(action, env->get_symbols());

        //     unordered_map<string, string> param_map;
        //     for (const GroundedAction& ground_action : ground_actions) {

        //         list<string> params = action.get_args();
        //         list<string>::iterator param_it = params.begin();
        //         // auto param_it = action.get_args().begin();
        //         auto val_it = ground_action.get_arg_values().begin();

        //         cout<< "length of params: "<<params.size()<<endl;

        //         // for (; param_it != action.get_args().end(); next(param_it), next(val_it))
        //         // for (; param_it != params.end(); next(param_it), next(val_it))
        //         for (int i=0; i<params.size(); i++)

        //         {
        //             param_map[*param_it] = *val_it;
        //             next(param_it);
        //             next(val_it);
        //             // cout << "param_map[*param_it] = *val_it"<<endl;
        //         }

        //     }

        //     for (const GroundedAction& ground_action : ground_actions) {
        //         // Check if action is applicable
        //         if (!is_applicable(action, ground_action, current.conditions)) {
        //             continue;
        //         }

        //         // cout << "debug statement 5 "<<endl;

        //         // Generate successor state
        //         auto successor_conditions = apply_action(action, ground_action, current.conditions);

        //         // Create new state with updated path
        //         auto successor_actions = current.actions_to_reach;
        //         successor_actions.push_back(ground_action);

        //         PlannerState successor(successor_conditions, successor_actions,
        //                              current.g_cost + 1,
        //                              calculate_heuristic(successor_conditions, env->get_goal_conditions()));

        //         // cout << "debug statement 6 "<<endl;
        //         // Add to open list if we haven't seen it
        //         if (closed_list.find(successor) == closed_list.end()) {
        //             open_list.push(successor);
        //         }

        //         // cout << "debug statement 7 "<<endl;
        //     }
        // }
    }

    cout << "No solution found!" << endl;
    cout << "Nodes expanded: " << nodes_expanded << endl;
    return list<GroundedAction>();
}

int main(int argc, char *argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char *env_file = static_cast<char *>("example.txt");
    if (argc > 1)
        env_file = argv[1];
    std::string envsDirPath = ENVS_DIR;
    char *filename = new char[envsDirPath.length() + strlen(env_file) + 2];
    strcpy(filename, envsDirPath.c_str());
    strcat(filename, "/");
    strcat(filename, env_file);

    cout << "Environment: " << filename << endl;
    Env *env = create_env(filename);
    // cout << "debug statement 2" << endl;

    if (print_status)
    {
        cout << *env;
    }

    // cout << "debug statement 3" << endl;

    
    auto start = chrono::high_resolution_clock::now();
    list<GroundedAction> actions = planner(env);
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> duration = end - start;
    cout << "Time taken: " << duration.count() << " seconds" << endl;

    cout << "\nPlan: " << endl;
    for (const GroundedAction &gac : actions)
    {
        cout << gac << endl;
    }

    delete env;
    return 0;
}