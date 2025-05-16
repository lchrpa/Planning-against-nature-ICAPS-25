#include "SAS_axiom.hpp"

size_t SAS_Axiom::get_affected_variable() const
{
    assert(_effect.size() == 1);
    return _effect.begin()->first;
}