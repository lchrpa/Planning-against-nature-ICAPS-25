#ifndef ___SAS_PROBLEM___
#define ___SAS_PROBLEM___

#include <functional>
#include <vector>
#include <string>
#include <fstream>

class SAS_State;

#include "SAS_state.hpp"
#include "SAS_variable_info.hpp"
#include "SAS_mutex.hpp"
#include "SAS_operator.hpp"
#include "SAS_action.hpp"
#include "SAS_event.hpp"
#include "SAS_axiom.hpp"
#include "SAS_exit_codes.hpp"
#include "SAS_exceptions.hpp"

class SAS_Problem
{

public:
    SAS_Problem(unsigned int version, bool metric, std::vector<SAS_Variable_Info> variables, std::vector<SAS_Mutex> mutexes, SAS_State init, SAS_State goal, std::vector<SAS_Action> actions, std::vector<SAS_Event> events, std::vector<SAS_Axiom> axioms);

    inline const std::vector<SAS_Axiom> &get_axioms() const { return _axioms; }
    inline const SAS_State &get_init() const { return _init; }
    inline const SAS_State &get_goal() const { return _goal; }
    inline const std::vector<SAS_Variable_Info> &get_variables() const { return _variables; }
    inline const std::vector<SAS_Event> &get_events() const { return _events; }
    inline const std::vector<SAS_Action> &get_actions() const { return _actions; }
    std::vector<std::vector<std::reference_wrapper<SAS_Axiom>>> &get_grouped_axioms();
    SAS_Operator &find_operator_by_name(std::string name);
    const SAS_Action &find_action_by_name(const std::string &name) const;
    const SAS_Event &find_event_by_name(const std::string &name) const;
    size_t get_number_of_variables() const;

    std::vector<std::reference_wrapper<const SAS_Event>> get_applicable_events(const SAS_State &state) const;

    /* experimental */
    // this may cause problems with negative preconditions and many other things..
    std::string _strips_text;
    void set_strips_text(const std::string &strips_text) { _strips_text = strips_text; };
    void to_strips_file(std::string filepath) const;
    void to_strips_file(std::string filepath, const SAS_State &init, const SAS_State &goal) const;

    void to_sas_file(std::string filepath);

private:
    unsigned int _version;
    bool _metric;
    std::vector<SAS_Variable_Info> _variables;
    std::vector<SAS_Mutex> _mutexes;
    SAS_State _init;
    SAS_State _goal;
    std::vector<std::reference_wrapper<SAS_Operator>> _operators;
    std::vector<SAS_Action> _actions;
    std::vector<SAS_Event> _events;
    std::vector<SAS_Axiom> _axioms;
    std::vector<std::vector<std::reference_wrapper<SAS_Axiom>>> _grouped_axioms;

    void link_problem_to_states();
    void group_axioms(); // group axioms to the layers

    std::string get_variable_state(std::pair<unsigned int, unsigned int> assignment) const;
};

#endif
