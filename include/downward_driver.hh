#pragma once

#include <iostream>
#include <string>

#include "paths.h"

#define DEFAULT_DRIVER_TIME_LIMIT "5m"

class DownwardDriver
{

public:
    DownwardDriver(std::string search_setting, std::string logging);
    int plan_domain_problem(std::string domain_filename, std::string problem_filename, std::string plan_filename, std::string time_limit = DEFAULT_DRIVER_TIME_LIMIT);
    int plan_sas(std::string sas_filepath, std::string plan_filename, std::string time_limit = DEFAULT_DRIVER_TIME_LIMIT);

private:
    int handle_result(int system_call_return);

    std::string _search_setting;
    std::string _logging;
};
