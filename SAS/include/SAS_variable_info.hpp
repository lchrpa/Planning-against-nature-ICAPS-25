#ifndef ___SAS_VARIABLE_INFO___
#define ___SAS_VARIABLE_INFO___

#include <assert.h>
#include <string>
#include <vector>
#include <algorithm>

#include <fstream>

#include "SAS_exceptions.hpp"

class SAS_Variable_Info
{

public:
    SAS_Variable_Info(unsigned int id, std::string name, int axiom_layer, size_t range, std::vector<std::string> values);
    inline std::string get_name() const { return _name; }
    inline unsigned int get_id() const { return _id; }
    inline unsigned int get_range() const { return _range; }
    inline int get_axiom_layer() const { return _axiom_layer; }

    std::string get_value(size_t index) const;

    /* ! Experimental ! */
    std::string to_strips_string_atom(size_t index) const;

    void to_sas_file(std::ofstream &file);

private:
    unsigned int _id;
    std::string _name;
    int _axiom_layer;
    unsigned int _range;
    std::vector<std::string> _values;
};

#endif
