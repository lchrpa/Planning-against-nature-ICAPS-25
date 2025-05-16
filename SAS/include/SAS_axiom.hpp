#ifndef ___SAS_AXIOM___
#define ___SAS_AXIOM___

#include <vector>
#include <utility>
#include <assert.h>

#include "SAS_assignment.hpp"

class SAS_Axiom
{

public:
    SAS_Axiom(SAS_Assignment preconditions, SAS_Assignment effect) : _preconditions(preconditions), _effect(effect) {}

    size_t get_affected_variable() const; // return varible from _effect

    inline const SAS_Assignment &get_preconditions() const { return _preconditions; }
    inline const SAS_Assignment &get_effect() const { return _effect; }

private:
    SAS_Assignment _preconditions;
    SAS_Assignment _effect; // there is only one effect allowed - SAS_Assignment is implemented with unordered_map, which is redundant in this case; however SAS_Assignment is kept for simplicity
};

#endif