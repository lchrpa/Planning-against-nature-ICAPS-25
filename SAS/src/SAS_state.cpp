#include "SAS_state.hpp"

#include "SAS_problem.hpp"

const std::string SAS_State::dump() const
{
    std::string s;
    bool first = true;
    for (auto &assignment : _assignment)
    {
        if (!first)
        {
            s.append(", ");
        }
        else
        {
            first = false;
        }
        s.append(std::to_string(assignment.first) + ": " + std::to_string(assignment.second));
    }
    return s;
}

void SAS_State::set_problem(SAS_Problem *parent_problem)
{
    _problem = parent_problem;
}

SAS_Problem *SAS_State::get_problem() const
{
    return _problem;
}

void SAS_State::evaluate_axioms()
{

    assert(_problem != nullptr);

    //init derived variables
    for (auto &axiom : _problem->get_axioms())
    {
        size_t variable = axiom.get_affected_variable();
        set(variable, _problem->get_init().get(variable));
    }

    //for each layel, check for applicable axioms and apply them
    for (auto &layer : _problem->get_grouped_axioms())
    {
        for (const SAS_Axiom &axiom : layer)
        {
            if (is_applicable(axiom))
            {
                apply(axiom.get_effect());
            }
        }
    }
}

void SAS_State::apply(const SAS_Operator &o, bool ignore_preconditions = false)
{
    //TODO: evaluate_axioms() should i evaluate axioms before?
    if (ignore_preconditions || is_applicable(o))
    {
        SAS_Assignment _assignment_backup = _assignment;
        for (auto &p : o.get_effects())
        {
            if (_assignment_backup.satisfy_all(p.first))
            {
                apply(p.second);
            }
        }
        evaluate_axioms();
    }
}

void SAS_State::apply(const SAS_Operator &o)
{
    apply(o, false);
}

bool SAS_State::check_and_apply(const SAS_Operator &o)
{
    if (is_applicable(o))
    {
        apply(o, true);
        return true;
    }
    else
    {
        return false;
    }
}

bool SAS_State::is_goal() const
{
    return is_goal(_problem->get_goal());
}

bool SAS_State::is_goal(const SAS_State &goal) const
{
    return goal.satisfied_by(_assignment);
}

bool SAS_State::satisfied_by(const SAS_State &other_state) const
{
    return _assignment.satisfied_by(other_state.get_assignment());
}

bool SAS_State::satisfied_by(const SAS_Assignment &other_assignment) const
{
    return _assignment.satisfied_by(other_assignment);
}

bool SAS_State::satisfy_all(const SAS_State &other_state) const
{
    return _assignment.satisfy_all(other_state.get_assignment());
}

bool SAS_State::satisfy_all(const SAS_Assignment &other_assignment) const
{
    return _assignment.satisfy_all(other_assignment);
}

size_t SAS_State::get(size_t variable) const
{
    return _assignment.get(variable);
}

void SAS_State::set(size_t variable, size_t value)
{
    _assignment.set(variable, value);
}

void SAS_State::apply(const SAS_Assignment &to_assign)
{
    _assignment.apply(to_assign);
}

bool SAS_State::is_applicable(const SAS_Axiom &axiom) const
{
    return _assignment.satisfy_all(axiom.get_preconditions());
}

bool SAS_State::is_applicable(const SAS_Operator &o) const
{
    return _assignment.satisfy_all(o.get_preconditions());
}

bool SAS_State::is_regressively_applicable(SAS_Operator& o)
{
    return _assignment.satisfy_all(o.hold_after());
}

void SAS_State::regressivlvely_apply(SAS_Operator& o, bool with_waitfor)
{
    for (auto eff : o.get_minimal_effects()) if (!o.get_preconditions().contains(eff.first)||(with_waitfor && !o.get_waitfor().contains(eff.first))) _assignment.erase(eff.first);
    _assignment.apply(o.get_preconditions());
    if (with_waitfor) _assignment.apply(o.get_waitfor());
}

bool SAS_State::check_and_regressively_apply( SAS_Operator& o)
{
    if (is_regressively_applicable(o)){
        regressivlvely_apply(o);
        return true;
    } else return false;
}

void SAS_State::to_sas_file(std::ofstream& file) const
{
    file << "begin_state" << std::endl;
    int n=_assignment.size();
    for (int i=0;i<n;i++){
        file << _assignment.get(i) << std::endl;
    }
    file << "end_state" << std::endl;
}
