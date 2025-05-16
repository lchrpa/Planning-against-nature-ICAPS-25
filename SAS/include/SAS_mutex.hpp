#ifndef ___SAS_MUTEX___
#define ___SAS_MUTEX___

#include <vector>

#include "SAS_assignment.hpp"

class SAS_Mutex
{

public:
    SAS_Mutex(SAS_Assignment group) : _group(group) {}

private:
    SAS_Assignment _group;
};

#endif