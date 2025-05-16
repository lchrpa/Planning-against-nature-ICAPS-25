#ifndef __relax__
#define __relax__

#include <vector>
#include <deque>
#include <list>
#include <queue>
#include <SAS_problem.hpp>
#include <SAS_plan.hpp>
#include <SAS_operator.hpp>
#include <SAS_event.hpp>
#include <RPG.hpp>
#include <DTG.hpp>
#include <relaxed_heuristics.hpp>

#define HMAX 1
#define HADD 2
#define HFF 3

struct search_node{
    //search_node(SAS_State s, SAS_Relaxed_Assignment f,std::vector<std::reference_wrapper<SAS_Operator>> pi) : _s(s), _f(f), _pi(pi){};
    search_node(SAS_Relaxed_Assignment f,std::vector<std::reference_wrapper<SAS_Operator>> pi,size_t g=0, size_t h=0) : _f(f), _pi(pi), _g(g), _h(h), _c(g+h){};
    //SAS_State _s;
    SAS_Relaxed_Assignment _f;
    std::vector<std::reference_wrapper<SAS_Operator>> _pi;
    size_t _g,_h,_c;
};

struct fullcost   {
        bool operator()(const search_node& l, const search_node& r) const { return l._c > r._c; }
} ;

//struct    {
//        bool operator()(const search_node l, const search_node r) const { return l._h > r._h; }
//} heurcost;

class relax {
public:
    relax(SAS_Problem prob, SAS_Plan plan) : _problem(prob), _plan(plan){};
    relax(SAS_Problem prob) : _problem(prob), _plan(NULL){};
    bool check_robustness();
    bool generate_robust_plan_bfs();
    bool generate_robust_plan_astar();
    bool generate_robust_plan_greedy();
    bool generate_les_greedy();
    inline SAS_Plan &get_plan(){return _plan;}

private:

    SAS_Problem _problem;
    SAS_Plan _plan;
    bool is_duplicite(const SAS_Relaxed_Assignment &s, std::list<SAS_Relaxed_Assignment> &states);
    std::set<size_t> only_effect_variables;
    void look_for_only_effect_variables(const std::vector<SAS_Action> &actions);
    bool is_deadend(const SAS_Relaxed_Assignment &s);

    int heur(const SAS_Relaxed_Assignment &f,std::vector<SAS_Operator>& acts, char type = HMAX, bool skip = false);

    bool is_fact_safe_lemma(size_t var, size_t value);
    std::unordered_map<size_t, std::vector<bool> > _safe_facts;
};


#endif
