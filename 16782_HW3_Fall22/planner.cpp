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
#include <bits/stdc++.h>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
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

    void change_truth(){
        this->truth=!this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
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
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
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
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

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
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_condition()
    {
        return this->initial_conditions;
    }
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
    unordered_set<Action, ActionHasher, ActionComparator> list_of_actions()
    {
        return this->actions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_condition()
    {
        return this->goal_conditions;
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

class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gPreconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gEffects;
public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }
    GroundedAction(string name, list<string> arg_values,
                   unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> g_precon,
                   unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> g_effect)
    {
        this->name = name;
        for (string ar : arg_values)
            this->arg_values.push_back(ar);
        for (GroundedCondition gc : g_precon)
            this->gPreconditions.insert(gc);
        for (GroundedCondition gc : g_effect)
            this->gEffects.insert(gc);
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_preconditions()
    {
        return this->gPreconditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_effects()
    {
        return this->gEffects;
    }
    bool operator==(const GroundedAction& rhs) const
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

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
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

void get_combinations(int i, int k, vector<vector<string>>&combinations,vector<string> temp, int numOfSym,vector<string>symbols){
   
    if (k == 0) 
    {
        combinations.push_back(temp);

        return ;
    }

    for (int j = i; j <= numOfSym - k; ++j) 
    {
        temp.push_back(symbols[j]);
        get_combinations(j+1, k-1,combinations,temp,numOfSym,symbols);
        temp.pop_back();
    }

    return ;
}

void get_permutations(vector<vector<string>>& permutations,vector<string> &comb){
    sort(comb.begin(),comb.end());

    do{
        permutations.push_back(comb);
    }while(next_permutation(comb.begin(),comb.end()));


}
void getGroundedActions(Action &action,vector<vector<string>>&perm,vector<GroundedAction> &allgactions){
    unordered_set<Condition, ConditionHasher, ConditionComparator> action_effect = action.get_effects();    
    unordered_set<Condition, ConditionHasher, ConditionComparator> action_pcond = action.get_preconditions();

    for(vector<string> singleaction:perm){
        unordered_map<string,string> argsmap;
        list<string> action_arg=action.get_args();
        list<string> g_action_arg(singleaction.begin(),singleaction.end());
        for( list<string>::const_iterator a_ = action_arg.begin(),g_ = g_action_arg.begin();a_!=action_arg.end();++a_,++g_){
                argsmap[*a_] = *g_;
        }

        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> GC_pre;
        unordered_set<GroundedCondition,GroundedConditionHasher,GroundedConditionComparator> GC_eff;

        for(Condition pre_cond : action_pcond){
            list<string> g_pre_cond_arg;
            list<string> pre_cond_arg = pre_cond.get_args();

            for(list<string>::const_iterator p_ = pre_cond_arg.begin();p_ != pre_cond_arg.end();++p_){

                if(argsmap[*p_] == ""){
                    g_pre_cond_arg.push_back(*p_);
                }
                else{
                    g_pre_cond_arg.push_back(argsmap[*p_]);
                }
            }
            GroundedCondition groundc (pre_cond.get_predicate(),g_pre_cond_arg,pre_cond.get_truth());
            GC_pre.insert(groundc);
        }

        for(Condition eff : action_effect)
      {
            list<string> gceffects_arg;
            list<string> effectArgs = eff.get_args();  // Make direct
             
            for(list<string>::const_iterator e_ = effectArgs.begin(); e_ != effectArgs.end() ; ++e_)
            {
                if(argsmap[*e_] == "")
                {
                    gceffects_arg.push_back(*e_);
                }
                else
                {
                    gceffects_arg.push_back(argsmap[*e_]);
                }
                // cout<<argMap[*e_it]<<" -> "<<*e_it;
            }

            GroundedCondition gc(eff.get_predicate(), gceffects_arg, eff.get_truth());
            GC_eff.insert(gc);
        }
        GroundedAction ga(action.get_name(),g_action_arg,GC_pre,GC_eff);
        allgactions.push_back(ga);

  

    
    }   

}
string hashst(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& stateset)
{
    set<string> strings;
    string hashedS = "";

    for(GroundedCondition gc : stateset)
    {
        strings.insert(gc.toString());
    }
    for (auto it = strings.begin(); it != strings.end(); it++) 
        hashedS += *it; 
    
    return hashedS;
}
struct node
    {
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state;

        double f = DBL_MAX;
        double g = DBL_MAX;
        double h = DBL_MAX;
        
        int parentIndex = -1;

        string parentNodeState = "";
    };
bool goalReached(node & curNode,Env* env)
{
    for(GroundedCondition gc : env->get_goal_condition())
    {   
        if(curNode.state.find(gc) == curNode.state.end())
            return 0;
    }
    return 1;
}

list<GroundedAction> planner(Env* env)
{
    // this is where you insert your planner

    // blocks world example
    // list<GroundedAction> actions;
    // actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    // actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    // actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

    // starting to code

    std::unordered_set<Action,ActionHasher,ActionComparator> listofActions;
    listofActions = env->list_of_actions();
    std::unordered_set<string> listofSymbols = env->get_symbols();
    vector<string> symbols(listofSymbols.begin(), listofSymbols.end());
    vector<GroundedAction> AllGactions;
    vector<vector<string>> combinations;
    vector<vector<string>> permutations;
    int numofSym = listofSymbols.size();
    vector<string> temp;
    for(Action action:listofActions){
        int args = action.get_args().size();
        
        get_combinations(0, args,combinations,temp,numofSym,symbols);
        for(int i=0;i<combinations.size();i++){
            get_permutations(permutations,combinations[i]);
 

        }
    
    getGroundedActions(action,permutations,AllGactions);
    // cout<<permutations.size()<<endl;
    permutations.clear();
    combinations.clear();
    temp.clear();
    }


// Starting ASTAR////////////////////////////////

    unordered_map<string, bool> closedList;     
    unordered_map<string, node> nodeInfo;
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> openList;    
    node firstNode;
    firstNode.state = env->get_initial_condition();
    string firstNodestring;
    // firstNodestring = hashst(firstNode.state);
    firstNode.g = 0;
    firstNode.h = 0;
    nodeInfo[hashst(firstNode.state)] = firstNode;
    openList.push(make_pair(firstNode.f,hashst(firstNode.state)));
    int n_states = 0;
    while(!openList.empty()){

        pair<double,string> curNodestring = openList.top();
        openList.pop();

        if(closedList[curNodestring.second]==true){
            continue;
        }
        closedList[curNodestring.second]= true;
        n_states++;
        
        node current_node = nodeInfo[curNodestring.second];

        if(goalReached(current_node,env)){

            cout<<"DONEEEE"<<endl;
              

        }

        int count_action = -1;
        bool possibleaction = 1;

        string newhnode;

        for(GroundedAction gaction : AllGactions){
            count_action++;
            possibleaction = true;

            for(GroundedCondition gcond : gaction.get_preconditions()){
                if(current_node.state.find(gcond) == current_node.state.end())
                {
                    possibleaction = false;
                    break;
                }                
            }

            if(possibleaction){
                node n_node;
                n_node.state = current_node.state;
                for(GroundedCondition effect : gaction.get_effects())
                {
                    if(effect.get_truth())
                    {
                        n_node.state.insert(effect);
                    }
                    else
                    {
                        // Remove from state
                        effect.change_truth();
                        n_node.state.erase(n_node.state.find(effect));
                    }
                }  
                newhnode = hashst(n_node.state);
                if (closedList[newhnode])
                    continue;

                n_node.g = current_node.g + 1;
                n_node.h = 1;
                n_node.f = n_node.g + n_node.h;

                // If the node has not been seen before (or) if the node now has a lesser cost path
                if(nodeInfo.find(newhnode) == nodeInfo.end() || n_node.g < nodeInfo[newhnode].g)
                {
                    n_node.parentIndex = count_action;
                    n_node.parentNodeState = curNodestring.second;
                    nodeInfo[newhnode] = n_node;
                    openList.push(make_pair(n_node.f, newhnode));
                }    
            }

          

        }
        






    }
    




    
    
    
    
    // return actions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("BlocksTriangle.txt");
    // if (argc > 1)
    //     filename = argv[1];

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