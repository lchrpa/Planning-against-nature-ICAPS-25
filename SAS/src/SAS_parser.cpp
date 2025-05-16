#include "SAS_parser.hpp"

SAS_Problem SAS_Parser::parse(std::string path_to_sas_file, std::string event_prefix, bool verbose)
{

    if (verbose)
        std::cout << "Parsing file '" << path_to_sas_file << "' started." << std::endl;

    std::ifstream f(path_to_sas_file);

    if (!f.is_open())
    {
        std::cerr << "File " + path_to_sas_file + " cannot be opened!" << std::endl;
        exit(SAS_FILE_NOT_FOUND);
    }

    unsigned int version = parse_version(f);
    if (verbose)
        std::cout << "Version: " << version << std::endl;

    bool metric = parse_metric(f);
    if (verbose)
        std::cout << "Metric: " << metric << std::endl;

    std::vector<SAS_Variable_Info> variables = parse_variables(f);
    if (verbose)
        std::cout << "Number of variables: " << variables.size() << std::endl;

    std::vector<SAS_Mutex> mutexes = parse_mutexes(f);
    if (verbose)
        std::cout << "Number of mutexes: " << mutexes.size() << std::endl;

    SAS_State init = parse_init(f, variables.size());
    if (verbose)
        std::cout << "Init: " << init.dump() << std::endl;

    SAS_State goal = parse_goal(f);
    if (verbose)
        std::cout << "Goal: " << goal.dump() << std::endl;

    std::pair<std::vector<SAS_Action>, std::vector<SAS_Event>> operators = parse_operators(f, metric, event_prefix);
    std::vector<SAS_Action> actions = operators.first;
    std::vector<SAS_Event> events = operators.second;
    if (verbose)
        std::cout << "Number of operators: " << actions.size() + events.size() << std::endl;
    if (verbose && events.size() > 0)
        std::cout << "Number of actions: " << actions.size() << std::endl;
    if (verbose && events.size() > 0)
        std::cout << "Number of events: " << events.size() << std::endl;

    std::vector<SAS_Axiom> axioms = parse_axioms(f);
    if (verbose)
        std::cout << "Number of axioms: " << axioms.size() << std::endl;

    SAS_Problem problem(version, metric, variables, mutexes, init, goal, actions, events, axioms);

    if (verbose)
        std::cout << "Parsing of SAS file finished." << std::endl;
    return problem;
}

SAS_Plan SAS_Parser::parse_plan(std::string path_to_file, SAS_Problem &problem, std::string event_prefix, bool skip_events, bool verbose)
{

    if (verbose)
        std::cout << "Parsing of plan file '" << path_to_file << "' started." << std::endl;

    std::ifstream f(path_to_file);

    if (!f.is_open())
    {
        std::cerr << "File " + path_to_file + " cannot be opened!" << std::endl;
        exit(PLAN_FILE_NOT_FOUND);
    }

    std::vector<std::reference_wrapper<SAS_Operator>> plan;

    std::string line;
    while (std::getline(f, line))
    {

        //remove comments
        size_t index = line.find(";");
        if (index != std::string::npos)
        {
            line = line.substr(0, index);
        }

        if (line != "")
        {
            if (line.front() == '(' && line.back() == ')')
            {
                std::string operator_string = line.substr(1, line.size() - 2);
                  if (event_prefix == "" || operator_string.rfind(event_prefix, 0) == std::string::npos)
                    plan.emplace_back(problem.find_operator_by_name(operator_string));
                  else
                      if (!skip_events) plan.emplace_back(problem.find_operator_by_name(operator_string.substr(event_prefix.size())));
            }
            else
            {
                std::cerr << "Invalid line in problem file." << std::endl;
                std::cerr << line << std::endl;
                exit(PLAN_PARSING_FAILED);
            }
        }
    }

    SAS_Plan p(&problem, plan);

    if (verbose)
        std::cout << "Plan length: " << p.get_length() << std::endl;
    if (verbose)
        std::cout << "Plan cost: " << p.get_cost() << std::endl;

    if (verbose)
        std::cout << "Parsing file plan file finished." << std::endl;
    return p;
}

unsigned int SAS_Parser::parse_version(std::ifstream &f)
{
    assert(next_line_equal(f, "begin_version"));
    std::string version_line;
    std::getline(f, version_line);
    unsigned int version = std::stol(version_line);
    assert(version == 3); // only SAS version 3 allowed
    assert(next_line_equal(f, "end_version"));
    return version;
}

bool SAS_Parser::parse_metric(std::ifstream &f)
{
    assert(next_line_equal(f, "begin_metric"));
    std::string metric_line;
    std::getline(f, metric_line);
    int metric = std::stoi(metric_line);
    assert(metric == 0 || metric == 1);
    bool metric_bool = (bool)metric;
    assert(next_line_equal(f, "end_metric"));
    return metric_bool;
}

std::vector<SAS_Variable_Info> SAS_Parser::parse_variables(std::ifstream &f)
{
    std::string count_line;
    std::getline(f, count_line);
    int count = std::stoi(count_line);
    assert(count >= 0);
    std::vector<SAS_Variable_Info> variables;
    for (size_t i = 0; i < (unsigned int)count; i += 1)
    {
        assert(next_line_equal(f, "begin_variable"));
        std::string name;
        std::getline(f, name);
        std::string axiom_layer_line;
        std::getline(f, axiom_layer_line);
        int axiom_layer = std::stoi(axiom_layer_line);
        assert(axiom_layer >= -1);
        std::string domain_size_line;
        std::getline(f, domain_size_line);
        int domain_size = std::stoi(domain_size_line);
        assert(domain_size >= 0); // TODO: is 0 possible?
        std::vector<std::string> values;
        for (size_t j = 0; j < (unsigned int)domain_size; j += 1)
        {
            std::string value;
            std::getline(f, value);
            values.emplace_back(value);
        }
        assert(next_line_equal(f, "end_variable"));
        SAS_Variable_Info v(i, name, axiom_layer, (unsigned int)domain_size, values);
        variables.emplace_back(v);
    }
    return variables;
}

std::vector<SAS_Mutex> SAS_Parser::parse_mutexes(std::ifstream &f)
{
    std::string count_line;
    std::getline(f, count_line);
    int count = std::stoi(count_line);
    assert(count >= 0);
    std::vector<SAS_Mutex> mutexes;
    for (size_t i = 0; i < (unsigned int)count; i += 1)
    {
        assert(next_line_equal(f, "begin_mutex_group"));
        std::string domain_size_line;
        std::getline(f, domain_size_line);
        int size = std::stoi(domain_size_line);
        assert(size >= 0); // TODO: is 0 possible?
        std::unordered_map<size_t, size_t> group;
        for (size_t j = 0; j < (unsigned int)size; j += 1)
        {
            std::string line;
            std::getline(f, line);
            unsigned int variable;
            unsigned int value;
            std::stringstream ss;
            ss << line;
            ss >> variable >> value;
            group[variable] = value;
        }
        assert(next_line_equal(f, "end_mutex_group"));
        SAS_Assignment assignment(group);
        SAS_Mutex m(assignment);
        mutexes.emplace_back(m);
    }
    return mutexes;
}

SAS_State SAS_Parser::parse_init(std::ifstream &f, size_t number_of_variables)
{
    assert(next_line_equal(f, "begin_state"));
    std::unordered_map<size_t, size_t> map;
    for (size_t i = 0; i < number_of_variables; i += 1)
    {
        std::string line;
        std::getline(f, line);
        unsigned int value = std::stoul(line);
        map[i] = value;
    }
    assert(next_line_equal(f, "end_state"));
    SAS_Assignment a(map);
    SAS_State s(a);
    return s;
}

SAS_State SAS_Parser::parse_goal(std::ifstream &f)
{
    assert(next_line_equal(f, "begin_goal"));
    std::string count_line;
    std::getline(f, count_line);
    unsigned int count = std::stoul(count_line);
    std::unordered_map<size_t, size_t> map;
    for (size_t i = 0; i < count; i += 1)
    {
        std::string line;
        std::getline(f, line);
        std::stringstream ss;
        unsigned int variable;
        unsigned int value;
        ss << line;
        ss >> variable >> value;
        map[variable] = value;
    }
    assert(next_line_equal(f, "end_goal"));
    SAS_Assignment a(map);
    SAS_State g(a);
    return g;
}

std::pair<std::vector<SAS_Action>, std::vector<SAS_Event>> SAS_Parser::parse_operators(std::ifstream &f, const bool metric, std::string event_prefix)
{
    std::string count_line;
    std::getline(f, count_line);
    unsigned int count = std::stoul(count_line);
    std::vector<SAS_Action> actions;
    std::vector<SAS_Event> events;
    for (size_t i = 0; i < count; i += 1)
    {
        assert(next_line_equal(f, "begin_operator"));
        std::string name;
        std::getline(f, name);
        std::string line;
        std::getline(f, line);
        unsigned int number_of_prevail = std::stoul(line);
        std::unordered_map<size_t, size_t> preconditions;
        for (size_t j = 0; j < number_of_prevail; j += 1)
        {
            std::string prevail_line;
            std::getline(f, prevail_line);
            std::stringstream ss;
            unsigned int variable;
            unsigned int value;
            ss << prevail_line;
            ss >> variable >> value;
            preconditions[variable] = value;
        }
        std::getline(f, line);
        unsigned int number_of_effects = std::stoul(line);
        std::vector<std::pair<SAS_Assignment, SAS_Assignment>> effects;
        for (size_t j = 0; j < number_of_effects; j += 1)
        {
            std::string operator_line;
            std::getline(f, operator_line);
            std::stringstream ss;
            ss << operator_line;
            unsigned int number_of_effect_conditions;
            ss >> number_of_effect_conditions;
            std::unordered_map<size_t, size_t> effect_conditions;
            for (size_t k = 0; k < number_of_effect_conditions; k += 1)
            {
                unsigned int e_variable;
                int e_value;
                ss >> e_variable >> e_value;
                assert(e_value >= -1);
                if (e_value >= 0)
                {
                    effect_conditions[e_variable] = e_value;
                }
            }
            unsigned int variable;
            int precondition_value;
            ss >> variable >> precondition_value;
            assert(precondition_value >= -1);
            unsigned int effect_value;
            ss >> effect_value;
            if (precondition_value >= 0)
            {
                preconditions[variable] = precondition_value;
            }
            std::unordered_map<size_t, size_t> effect_map;
            effect_map[variable] = effect_value;
            SAS_Assignment c_effect_precondition_assignment(effect_conditions);
            SAS_Assignment effect_assignment(effect_map);
            std::pair<SAS_Assignment, SAS_Assignment> effect = std::make_pair(c_effect_precondition_assignment, effect_map);
            effects.emplace_back(effect);
        }
        std::getline(f, line);
        unsigned int cost = std::stoul(line);
        if (!metric)
        {
            cost = 1;
        }
        assert(next_line_equal(f, "end_operator"));
        SAS_Assignment preconditions_assignment(preconditions);
        if (event_prefix == "" || name.rfind(event_prefix, 0) == std::string::npos)
        {
            SAS_Action a(name, preconditions_assignment, effects, cost);
            actions.emplace_back(a);
        }
        else
        {
            // TODO: can events have a cost?
            SAS_Event e(name.substr(event_prefix.size()), preconditions_assignment, effects, cost);
            events.emplace_back(e);
        }
    }
    return std::make_pair(actions, events);
}

std::vector<SAS_Axiom> SAS_Parser::parse_axioms(std::ifstream &f)
{
    std::string axiom_line;
    std::getline(f, axiom_line);
    unsigned int number_of_axioms = std::stoul(axiom_line);
    std::vector<SAS_Axiom> axioms;
    for (size_t i = 0; i < number_of_axioms; i += 1)
    {
        assert(next_line_equal(f, "begin_rule"));
        std::string count_line;
        std::getline(f, count_line);
        std::unordered_map<size_t, size_t> preconditions_map;
        unsigned int number_of_preconditions = std::stoul(count_line);
        for (size_t j = 0; j < number_of_preconditions; j += 1)
        {
            std::string line;
            std::getline(f, line);
            std::stringstream ss;
            unsigned int variable;
            unsigned int value;
            ss << line;
            ss >> variable >> value;
            preconditions_map[variable] = value;
        }
        std::string head_line;
        std::getline(f, head_line);
        std::stringstream ss;
        ss << head_line;
        unsigned int variable;
        unsigned int precondition;
        unsigned int effect;
        ss >> variable >> precondition >> effect;
        preconditions_map[variable] = precondition;
        std::unordered_map<size_t, size_t> effect_map;
        effect_map[variable] = effect;
        assert(next_line_equal(f, "end_rule"));
        SAS_Assignment preconditions(preconditions_map);
        SAS_Assignment e(effect_map);
        SAS_Axiom axiom(preconditions, e);
        axioms.emplace_back(axiom);
    }
    return axioms;
}
