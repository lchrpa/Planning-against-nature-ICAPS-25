#include "SAS_variable_info.hpp"

SAS_Variable_Info::SAS_Variable_Info(unsigned int id, std::string name, int axiom_layer, size_t range, std::vector<std::string> values) : _id(id), _name(name), _axiom_layer(axiom_layer), _range(range), _values(values) {}

std::string SAS_Variable_Info::get_value(size_t index) const
{
    if (index < _range)
    {
        return _values[index];
    }
    else
    {
        throw SAS_out_of_range("Out of range. Domain of variable has only " + std::to_string(_range) + " values.");
    }
}

std::string SAS_Variable_Info::to_strips_string_atom(size_t index) const
{
    std::string value = get_value(index);
    std::string atom;
    if (value.substr(0, 11) == "NegatedAtom")
    {
        atom = value.substr(12);
    }
    else
    {
        atom = value.substr(5);
    }
    size_t parenthesis_index = atom.find("(");
    if (atom.at(parenthesis_index + 1) == ')')
    {
        atom = atom.substr(0, parenthesis_index);
    }
    else
    {
        atom = atom.substr(0, atom.size() - 1);
        atom.replace(parenthesis_index, 1, " ");
    }
    atom.erase(std::remove(atom.begin(), atom.end(), ','), atom.end());
    if (value.substr(0, 11) == "NegatedAtom")
    {
        return "(not (" + atom + "))";
    }
    else
    {
        return "(" + atom + ")";
    }
}

void SAS_Variable_Info::to_sas_file(std::ofstream &file)
{
    file << "begin_variable" << std::endl;
    file << "var" << _id <<std::endl;
    file << "-1" << std::endl;
    file << _range << std::endl;
    for (auto s : _values){
        file << s << std::endl;
    }
    file << "end_variable" << std::endl;
}

