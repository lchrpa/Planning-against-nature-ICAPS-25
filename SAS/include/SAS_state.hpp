#ifndef ___SAS_STATE___
#define ___SAS_STATE___

#include <assert.h>
#include <string>
#include <vector>

#include<fstream>

class SAS_Problem;
class SAS_Operator;

#include "SAS_assignment.hpp"
#include "SAS_axiom.hpp"
#include "SAS_operator.hpp"

class SAS_State
{

public:
    SAS_State(SAS_Assignment assignment) : _assignment(assignment){};
    SAS_State(SAS_Assignment assignments, SAS_Problem *parent_problem) : _assignment(assignments), _problem(parent_problem){};
    SAS_State(SAS_Problem *parent_problem) : _problem(parent_problem) {}

    const std::string dump() const;

    void set_problem(SAS_Problem *parent_problem);
    SAS_Problem *get_problem() const;

    const SAS_Assignment &get_assignment() const { return _assignment; };

    size_t get(size_t variable) const;
    void set(size_t variable, size_t value);
    bool is_applicable(const SAS_Axiom &axiom) const;
    bool is_applicable(const SAS_Operator &o) const;
    bool check_and_apply(const SAS_Operator &o);
    bool is_goal() const;
    bool is_goal(const SAS_State &goal) const;
    bool satisfied_by(const SAS_Assignment &assignment) const;
    bool satisfied_by(const SAS_State &state) const;
    bool satisfy_all(const SAS_Assignment &assignment) const;
    bool satisfy_all(const SAS_State &state) const;
    void apply(const SAS_Operator &o);
    void apply(const SAS_Assignment &to_assign);

    bool is_regressively_applicable(SAS_Operator &o);
    void regressivlvely_apply(SAS_Operator &o,bool with_waitfor = false);
    bool check_and_regressively_apply(SAS_Operator &o);

    bool operator==(const SAS_State &other) const
    {
        return satisfied_by(other.get_assignment()) && other.satisfied_by(get_assignment());
    };

    void to_sas_file(std::ofstream &file) const;

private:
    SAS_Assignment _assignment;
    SAS_Problem *_problem;

    void evaluate_axioms();

    void apply(const SAS_Operator &o, bool ignore_preconditions);
};

#endif
