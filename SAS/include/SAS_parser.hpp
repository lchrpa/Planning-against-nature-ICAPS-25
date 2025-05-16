#ifndef ___SAS_PARSER___
#define ___SAS_PARSER___

#include <assert.h>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <string>
#include <iostream>
#include <utility>
#include <sstream>

#include "utils.hpp"
#include "SAS_problem.hpp"
#include "SAS_variable_info.hpp"
#include "SAS_mutex.hpp"
#include "SAS_assignment.hpp"
#include "SAS_state.hpp"
#include "SAS_axiom.hpp"
#include "SAS_exit_codes.hpp"
#include "SAS_plan.hpp"
#include "SAS_action.hpp"
#include "SAS_event.hpp"

class SAS_Parser
{

public:
    SAS_Problem parse(std::string path_to_sas_file, std::string event_prefix = "", bool verbose = false);
    SAS_Plan parse_plan(std::string path_to_plan, SAS_Problem &problem, std::string event_prefix = "", bool skip_events = false, bool verbose = false);

private:
    unsigned int parse_version(std::ifstream &f);
    bool parse_metric(std::ifstream &f);
    std::vector<SAS_Variable_Info> parse_variables(std::ifstream &f);
    std::vector<SAS_Mutex> parse_mutexes(std::ifstream &f);
    SAS_State parse_init(std::ifstream &f, size_t number_of_variables);
    SAS_State parse_goal(std::ifstream &f);
    std::pair<std::vector<SAS_Action>, std::vector<SAS_Event>> parse_operators(std::ifstream &f, const bool metric, std::string event_prefix = "");
    std::vector<SAS_Axiom> parse_axioms(std::ifstream &f);
};

#endif
