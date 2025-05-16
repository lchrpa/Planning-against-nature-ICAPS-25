#ifndef ___SAS_PLAN___
#define ___SAS_PLAN___

#include <vector>
#include <functional>
#include <algorithm>
#include <fstream>

#include "SAS_operator.hpp"
#include "SAS_problem.hpp"
#include "SAS_state.hpp"

class SAS_Plan
{

public:
    SAS_Plan(SAS_Problem *problem, std::vector<std::reference_wrapper<SAS_Operator>> plan) : _problem(problem), _plan(plan) {}
    SAS_Plan(SAS_Problem *problem) : _problem(problem) {}
    inline size_t get_length() const { return _plan.size(); }
    inline SAS_Problem *get_problem() { return _problem; }
    unsigned int get_cost() const;
    const SAS_Operator &get_operator(size_t index) const;

    bool is_applicable() const;                                 // check if aplicable from init state of corresponding problem
    bool is_applicable(SAS_State init) const;                   // check if aplicable from given state
    bool is_valid() const;                                      // check goal of corresponding problem is reached after applying all actions
    bool is_valid(SAS_State init, const SAS_State &goal) const; // check if given goal is reached from given init state
    void remove(size_t index);
    void remove(std::vector<size_t> indeces);

    void to_file(std::string path) const;

    // iterator definition
    std::vector<std::reference_wrapper<SAS_Operator>>::const_iterator begin() const { return _plan.cbegin(); }
    std::vector<std::reference_wrapper<SAS_Operator>>::const_iterator end() const { return _plan.cend(); }

    // reverse iterator definition
    std::vector<std::reference_wrapper<SAS_Operator>>::const_reverse_iterator rbegin() const { return _plan.crbegin(); }
    std::vector<std::reference_wrapper<SAS_Operator>>::const_reverse_iterator rend() const { return _plan.crend(); }

private:
    SAS_Problem *_problem;
    std::vector<std::reference_wrapper<SAS_Operator>> _plan;
};

#endif
