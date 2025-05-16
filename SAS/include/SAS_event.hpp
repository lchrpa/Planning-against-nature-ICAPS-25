#ifndef ___SAS_EVENT___
#define ___SAS_EVENT___

#include "SAS_operator.hpp"

class SAS_Event : public SAS_Operator
{

public:
    SAS_Event(std::string name, SAS_Assignment preconditions, std::vector<std::pair<SAS_Assignment, SAS_Assignment>> effects, unsigned int cost) : SAS_Operator(name, preconditions, effects, cost) {}

    inline bool isEvent(){return true;}
};

#endif
