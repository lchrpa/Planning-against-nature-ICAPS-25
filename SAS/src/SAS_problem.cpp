#include "SAS_problem.hpp"

SAS_Problem::SAS_Problem(unsigned int version, bool metric, std::vector<SAS_Variable_Info> variables, std::vector<SAS_Mutex> mutexes, SAS_State init, SAS_State goal, std::vector<SAS_Action> actions, std::vector<SAS_Event> events, std::vector<SAS_Axiom> axioms) : _version(version), _metric(metric), _variables(variables), _mutexes(mutexes), _init(init), _goal(goal), _actions(actions), _events(events), _axioms(axioms)
{
    _operators.clear();
    _operators.insert(_operators.end(), actions.begin(), actions.end());
    _operators.insert(_operators.end(), events.begin(), events.end());
    link_problem_to_states();
    group_axioms();
}

void SAS_Problem::link_problem_to_states()
{
    _init.set_problem(this);
    _goal.set_problem(this);
}

void SAS_Problem::group_axioms()
{
    _grouped_axioms.clear();
    for (auto &axiom : _axioms)
    {
        int layer = _variables[axiom.get_affected_variable()].get_axiom_layer();
        assert(layer >= -1);
        unsigned int index = (unsigned int)(layer + 1);
        while (index >= _grouped_axioms.size())
        {
            std::vector<std::reference_wrapper<SAS_Axiom>> v;
            _grouped_axioms.emplace_back(v);
        }
        _grouped_axioms[index].emplace_back(axiom);
    }
}

std::vector<std::vector<std::reference_wrapper<SAS_Axiom>>> &SAS_Problem::get_grouped_axioms()
{
    if (_grouped_axioms.empty())
    {
        group_axioms();
    }
    return _grouped_axioms;
}

SAS_Operator &SAS_Problem::find_operator_by_name(const std::string name)
{
    if (name == SAS_Action::noop_action.get_name())
    {
        return SAS_Action::noop_action;
    }
    for (auto &a : _actions)
    {
        if (a.get_name() == name)
        {
           return a;
        }
    }
    for (auto &e : _events)
    {
        if (e.get_name() == name)
        {
           return e;
        }
    }
    throw SAS_not_found("No operator with name '" + name + "' found.");
}

const SAS_Action &SAS_Problem::find_action_by_name(const std::string &name) const
{
    if (name == SAS_Action::noop_action.get_name())
    {
        return SAS_Action::noop_action;
    }
    for (auto &a : _actions)
    {
        if (a.get_name() == name)
        {
            return a;
        }
    }
    throw SAS_not_found("No action with name '" + name + "' found.");
}

const SAS_Event &SAS_Problem::find_event_by_name(const std::string &name) const
{
   // if (name == SAS_Action::noop_action.get_name())
   // {
   //     return SAS_Action::noop_action;
   // }
    for (auto &e : _events)
    {
        if (e.get_name() == name)
        {
            return e;
        }
    }
    throw SAS_not_found("No event with name '" + name + "' found.");
}


void SAS_Problem::to_sas_file(std::string filepath)
{
    // write to file
    std::ofstream file(filepath, std::ofstream::out);
    if (!file.is_open())
    {
        throw SAS_file_error("File '" + filepath + "' cannot be created.");
    }
    file << "begin_version" << std::endl;
    file << "3" << std::endl;
    file << "end_version" << std::endl;
    file << "begin_metric" << std::endl;
    file << "0" << std::endl;
    file << "end_metric" << std::endl;
    //variables
    file << _variables.size() << std::endl;
    for (auto v : _variables) v.to_sas_file(file);
    file << "0" << std::endl;
    _init.to_sas_file(file);
    file << "begin_goal" << std::endl;
    _goal.get_assignment().to_sas_file(file);
    file << "end_goal" << std::endl;
    file << _actions.size() << std::endl;
    for (auto a : _actions) a.to_sas_file(file);
    file << "0" << std::endl;
    file.close();
}


/* experimental */

void SAS_Problem::to_strips_file(std::string filepath) const
{

    // write to file
    std::ofstream file(filepath, std::ofstream::out);
    if (!file.is_open())
    {
        throw SAS_file_error("File '" + filepath + "' cannot be created.");
    }

    file << _strips_text << std::endl;

    file.close();
}

void SAS_Problem::to_strips_file(std::string filepath, const SAS_State &init, const SAS_State &goal) const
{

    std::string text = _strips_text;

    // change init
    // for each variable add current value and remove all others
    auto variables = init.get_problem()->get_variables();

    size_t init_index = text.find("(:init");

    for (auto assignment : init.get_assignment())
    {

        size_t variable = assignment.first;
        size_t value = assignment.second;

        // value string
        std::string value_string = get_variable_state(assignment);
        if (value_string.substr(0, 4) != "(not" && text.find(value_string) == std::string::npos)
        {
            text.insert(init_index + 6, "\n" + value_string + "\n");
        }

        size_t range = variables.at(variable).get_range();
        for (size_t i = 0; i < range; i += 1)
        {
            if (i != value)
            {
                std::string value_string_to_remove = get_variable_state(std::make_pair(variable, i));
                size_t found_index = text.find(value_string_to_remove);
                if (found_index != std::string::npos)
                {
                    text.erase(found_index, value_string_to_remove.length());
                }
            }
        }
    }

    /* change goal */

    // construct a new goal
    std::string new_goal;
    new_goal = "(:goal (and\n";
    for (auto assignment : goal.get_assignment())
    {
        new_goal += get_variable_state(assignment) + "\n";
    }
    new_goal += "))\n";

    // find first goal parenthesis
    size_t goal_start = text.find("(:goal");
    if (goal_start == std::string::npos)
    {
        throw SAS_not_found("Given string doesn't contain searched string '(:goal'.");
    }

    // find second goal parenthesis
    // TODO: DOESN'T WORK WITH COMMENTS
    size_t index = goal_start;
    int counter = 0;
    do
    {
        if (text.at(index) == ')')
        {
            counter -= 1;
        }
        else if (text.at(index) == '(')
        {
            counter += 1;
        }
        index += 1;
    } while (counter != 0 && index != std::string::npos);
    size_t goal_end = index;

    text.replace(goal_start, goal_end - goal_start, new_goal);

    // write to file
    std::ofstream file(filepath, std::ofstream::out);
    if (!file.is_open())
    {
        throw SAS_file_error("File '" + filepath + "' cannot be created.");
    }

    file << text << std::endl;

    file.close();
}

std::string SAS_Problem::get_variable_state(std::pair<unsigned int, unsigned int> assignment) const
{
    return _variables.at(assignment.first).to_strips_string_atom(assignment.second);
}

std::vector<std::reference_wrapper<const SAS_Event>> SAS_Problem::get_applicable_events(const SAS_State &state) const
{
    std::vector<std::reference_wrapper<const SAS_Event>> applicable_events;
    applicable_events.reserve(_events.size() / 4);
    std::copy_if(_events.begin(), _events.end(), back_inserter(applicable_events), [state](const SAS_Event &event)
                 { return state.is_applicable(event); });
    applicable_events.shrink_to_fit();
    return applicable_events;
}

size_t SAS_Problem::get_number_of_variables() const
{
    return _variables.size();
}
