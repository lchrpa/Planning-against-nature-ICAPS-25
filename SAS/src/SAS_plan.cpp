#include "SAS_plan.hpp"

unsigned int SAS_Plan::get_cost() const
{
    unsigned int cost = 0;
    for (SAS_Operator &o : _plan)
    {
        cost += o.get_cost();
    }
    return cost;
}

bool SAS_Plan::is_applicable() const
{
    return is_applicable(_problem->get_init());
}

bool SAS_Plan::is_applicable(SAS_State init) const
{
    for (SAS_Operator &o : _plan)
    {
        if (!init.check_and_apply(o))
        {
            return false;
        }
    }
    return true;
}

bool SAS_Plan::is_valid() const
{
    return is_valid(_problem->get_init(), _problem->get_goal());
}

bool SAS_Plan::is_valid(SAS_State init, const SAS_State &goal) const
{
    for (SAS_Operator &o : _plan)
    {
        if (!init.check_and_apply(o))
        {
            return false;
        }
    }
    return init.is_goal(goal);
}

const SAS_Operator &SAS_Plan::get_operator(size_t index) const
{
    if (index < get_length())
    {
        return _plan[index];
    }
    else
    {
        std::string error = "Invalid index " + std::to_string(index) + ". Plan is only " + std::to_string(get_length()) + " operators long.";
        throw std::out_of_range(error.c_str());
    }
}

void SAS_Plan::remove(size_t index)
{
    if (index < get_length())
    {
        _plan.erase(_plan.begin() + index);
    }
    else
    {
        std::string error = "Invalid index " + std::to_string(index) + ". Plan is only " + std::to_string(get_length()) + " actions long.";
        throw std::out_of_range(error.c_str());
    }
}

void SAS_Plan::remove(std::vector<size_t> indeces)
{
    assert(indeces.size() <= get_length());
    if (!std::is_sorted(indeces.begin(), indeces.end(), std::greater<size_t>()))
    {
        std::sort(indeces.begin(), indeces.end(), std::greater<size_t>());
    }
    for (auto index : indeces)
    {
        remove(index);
    }
}

void SAS_Plan::to_file(std::string filepath) const
{

    std::ofstream f(filepath);

    if (!f.is_open())
    {
        std::cerr << "File " + filepath + " cannot be created!" << std::endl;
        exit(PLAN_WRITING_FAILED);
    }

    for (SAS_Operator &o : _plan)
    {
        f << "(" << o.get_name() << ")" << std::endl;
    }

    f << "; cost = " << get_cost() << std::endl;

    f.close();
}