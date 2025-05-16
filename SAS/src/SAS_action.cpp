#include "SAS_action.hpp"

SAS_Action SAS_Action::noop_action = SAS_Action("<noop-action>", SAS_Assignment(), std::vector<std::pair<SAS_Assignment, SAS_Assignment>>(), 0);
