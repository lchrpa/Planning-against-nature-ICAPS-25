#ifndef __DTG__
#define __DTG__

#define EVENTS_ONLY 1
#define ACTIONS_ONLY 2
#define EVENTS_ACTIONS 3

#include <vector>
#include <stack>
#include <list>

#include <iostream>

#include "SAS_problem.hpp"

struct DTG_matrix_element {
  bool edge,path,leaf;
};

class DTG {
    public:
        DTG(SAS_Problem prob, char type_operators) : _problem(prob), _type_operators(type_operators){};

        inline size_t getNumberOfDTGs(){return _problem.get_variables().size();}

        void BuildDTGs();

        //can be used after BuildDTGs()
        inline bool isLeaf(size_t variable, size_t value){return _DTGs[variable][value][value].leaf;}
        inline bool isPath(size_t variable, size_t value1, size_t value2){return _DTGs[variable][value1][value2].path;}

        bool unreachable(const SAS_Assignment &to, const SAS_Assignment &from);

        std::vector<size_t> leavesOF(size_t variable, size_t value);

        void outputDTGInfo();

        std::vector<size_t> connected_from(size_t variable, size_t value);

    private:
        SAS_Problem _problem;
        char _type_operators;

        std::vector<std::vector<std::vector<DTG_matrix_element>>> _DTGs;

        void process_SAS_Operator(SAS_Operator&);
        void detect_Paths(std::vector<std::vector<DTG_matrix_element>>&);
};

#endif
