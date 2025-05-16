#include "downward_driver.hh"

DownwardDriver::DownwardDriver(std::string search_setting, std::string logging) : _search_setting(search_setting)
{
    if (logging != "")
    {
        if (logging.find(">>") == std::string::npos && logging.find(">") == std::string::npos)
        {
            _logging = ">> " + logging;
        }
        else
        {
            _logging = logging;
        }
    }
}

int DownwardDriver::plan_domain_problem(std::string domain_filename, std::string problem_filename, std::string plan_filename, std::string time_limit)
{
    char command[500];
    sprintf(command, "python3 %sfast-downward.py %s --plan-file %s --search-time-limit %s %s %s %s", FD_PATH, _search_setting.c_str(), plan_filename.c_str(), time_limit.c_str(), domain_filename.c_str(), problem_filename.c_str(), _logging.c_str());
    int result = std::system(command);
    return handle_result(result);
}

int DownwardDriver::plan_sas(std::string sas_filename, std::string plan_filename, std::string time_limit)
{
    char command[500];
    sprintf(command, "python3 %sfast-downward.py %s --plan-file %s --search-time-limit %s %s %s", FD_PATH, _search_setting.c_str(), plan_filename.c_str(), time_limit.c_str(), sas_filename.c_str(), _logging.c_str());
    int result = std::system(command);
    return handle_result(result);
}

int DownwardDriver::handle_result(int system_call_result)
{
    int downward_result = WEXITSTATUS(system_call_result);
    if (downward_result != 0 && downward_result != 10 && downward_result != 11 && downward_result != 12)
    {
        std::cerr << "Error " << downward_result << " occured while Fast Downward planning." << std::endl;
    }
    return downward_result;
}