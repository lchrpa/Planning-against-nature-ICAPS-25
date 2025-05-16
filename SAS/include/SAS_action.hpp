#ifndef ___SAS_ACTION___
#define ___SAS_ACTION___

#include "SAS_operator.hpp"

#include <string>

class SAS_Action : public SAS_Operator
{

public:
    SAS_Action(std::string name, SAS_Assignment preconditions, std::vector<std::pair<SAS_Assignment, SAS_Assignment>> effects, unsigned int cost) : SAS_Operator(name, preconditions, effects, cost) {}

    static SAS_Action noop_action;
};

#endif
