#include "SAS_assignment.hpp"

SAS_Assignment::SAS_Assignment(size_t variable, size_t value)
{
    std::unordered_map<size_t, size_t> a;
    a[variable] = value;
    _assignment = a;
}

void SAS_Assignment::set(size_t variable, size_t value)
{
    _assignment[variable] = value;
}

size_t SAS_Assignment::get(size_t variable) const
{
    auto it = _assignment.find(variable);
    if (it != _assignment.end())
    {
        return it->second;
    }
    else
    {
        throw std::out_of_range("Assignment does not contain given key " + std::to_string(it->first) + ".");
    }
}

bool SAS_Assignment::empty() const
{
    return _assignment.empty();
}

bool SAS_Assignment::contains(size_t variable) const
{
    auto it = _assignment.find(variable);
    return it != _assignment.end();
}

void SAS_Assignment::apply(const SAS_Assignment &a)
{
    for (auto &it : a)
    {
        set(it.first, it.second);
    }
}

size_t SAS_Assignment::size() const
{
    return _assignment.size();
}

bool SAS_Assignment::satisfy(size_t variable, size_t value) const
{
    auto it = _assignment.find(variable);
    if (it != _assignment.end())
    {
        return it->second == value;
    }
    else
    {
        return false;
    }
}

bool SAS_Assignment::satisfy_all(const SAS_Assignment &assignment) const
{
    for (auto &it : assignment)
    {
        if (!satisfy(it.first, it.second))
        {
            return false;
        }
    }
    return true;
}

bool SAS_Assignment::satisfied_by(const SAS_Assignment &other_assignment) const
{
    return other_assignment.satisfy_all(_assignment);
}

bool SAS_Assignment::any_mismatch(const SAS_Assignment& assignment) const
{
    for (auto &it : assignment)
    {
        if (contains(it.first)&&get(it.first)!=it.second)
        {
            return true;
        }
    }
    return false;
}

SAS_Assignment SAS_Assignment::get_mismatch(const SAS_Assignment& assignment) const
{
    SAS_Assignment ret;
    for (auto &it : assignment)
    {
        if (!contains(it.first)||get(it.first)!=it.second) ret.set(it.first,get(it.first));
    }
    return ret;

}


void SAS_Assignment::to_sas_file(std::ofstream& file) const
{

        file << _assignment.size() << std::endl;
        for (auto x : _assignment){
            file << x.first << " " << x.second << std::endl;
        }


}
