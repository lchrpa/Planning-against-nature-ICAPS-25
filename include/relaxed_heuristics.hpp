#ifndef __relaxed_heuristics__
#define __relaxed_heuristics__

#include <vector>
#include <deque>
#include <list>
#include <queue>
#include <SAS_problem.hpp>
#include <SAS_plan.hpp>
#include <SAS_operator.hpp>
#include <RPG.hpp>

#define HMAX 1
#define HADD 2
#define HFF 3

struct op_data{
    op_data(size_t op_id, bool goal = false) : _op_id(op_id), _appl(false), _processed(false), _goal(goal), _val(INFTY){};
    //SAS_State _s;
    size_t _op_id;
    bool _appl;
    bool _processed;
    bool _goal;
    int _val;
};

struct   {
        bool operator()(const op_data& l, const op_data& r) const {
           // if (!l._appl) return true;
          //  if (l._val == r._val){
          //      return !l._goal;
          //  }
          //  return r._appl &&
            return l._val < r._val;
        }
} op_data_compare ;

class relaxed_heuristics {

public:
    relaxed_heuristics(const SAS_Relaxed_Assignment &ra, std::vector<SAS_Operator>& acts, char type, bool no_affected = true, std::unordered_map<size_t, std::vector<bool> > *skip = NULL);
    int compute_heur(const SAS_Assignment &goal);

private:
    char _type;
    std::vector<SAS_Operator> _acts;
    std::unordered_map<size_t, std::vector<int> > _assignment_vals;
};

#endif
