#include <iostream>

#include "DTG.hpp"


void DTG::process_SAS_Operator ( SAS_Operator &op)
{
   for (auto eff : op.get_minimal_effects()){
       if (op.get_preconditions().contains(eff.first)) _DTGs[eff.first][op.get_preconditions().get(eff.first)][eff.second].edge=true;
           else for (size_t j=0;j<_problem.get_variables()[eff.first].get_range();j++)if (j!=eff.second) _DTGs[eff.first][j][eff.second].edge=true;
   }

   //debug
   //std::cout << "processed Operator: " << op.get_name() << std::endl;
}

void DTG::detect_Paths(std::vector<std::vector<DTG_matrix_element>> &graph)
{
   size_t n=graph.size();

   for (size_t i=0;i<n;i++) if (!graph[i][i].leaf){
     std::stack<int> to_process;
     std::list<int> expanded[n];
     std::vector<bool> visited(n,false);

     to_process.push(i);
     while (!to_process.empty()){
          //look for neighbours of node i
        size_t curr=to_process.top();
        to_process.pop();
        visited[curr]=true;
       // std::cout << "processing " << curr << std::endl;
        expanded[curr].push_back(curr);
        bool possible_leaf=true;
        for (size_t j=0;j<n;j++) if (j!=curr && graph[curr][j].edge){
            possible_leaf=false;
            if (!visited[j]) to_process.push(j);
            expanded[j]=expanded[curr];
            for (auto k : expanded[j]) graph[k][j].path=true;
        }
        if (possible_leaf) graph[curr][curr].leaf=true;

    }
  }
}


void DTG::BuildDTGs(){

    //initialize _DTGs
    for (size_t i=0;i<getNumberOfDTGs();i++){
        DTG_matrix_element empty_cell;
        empty_cell.edge=empty_cell.path=empty_cell.leaf=false;
        std::vector<DTG_matrix_element> temp(_problem.get_variables()[i].get_range(),empty_cell);
        std::vector<std::vector<DTG_matrix_element>> temp2(_problem.get_variables()[i].get_range(),temp);
        _DTGs.push_back(temp2);
    }

   //process Operators (Action and Events)

   if (_type_operators!=ACTIONS_ONLY)
     for (auto ev : _problem.get_events()) process_SAS_Operator(ev);
   if (_type_operators!=EVENTS_ONLY)
     for (auto act : _problem.get_actions()) process_SAS_Operator(act);

  for (size_t i=0;i<getNumberOfDTGs();i++){
   // std::cout << "processing " << i << std::endl;
    detect_Paths(_DTGs[i]);
  }

}

bool DTG::unreachable(const SAS_Assignment& to, const SAS_Assignment& from)
{
  for (auto fact : to) {
    if (from.contains(fact.first)&&(fact.second!=from.get(fact.first))&&!isPath(fact.first,from.get(fact.first),fact.second)) return true;
  }
  return false;
}

std::vector<size_t> DTG::leavesOF(size_t variable, size_t value)
{
  std::vector<size_t> leaves;

  if (isLeaf(variable,value)) {leaves.push_back(value);}

  else
  for (size_t i=0;i<_problem.get_variables()[variable].get_range(); i++){
    if (isPath(variable,value,i)&&isLeaf(variable,i)) leaves.push_back(i);
  }

  return leaves;
}

std::vector<size_t> DTG::connected_from(size_t variable, size_t value)
{
  std::vector<size_t> ret;

  for (size_t i=0;i<_problem.get_variables()[variable].get_range(); i++){
    if (i==value || isPath(variable,value,i)) ret.push_back(i);
  }
  return ret;
}


void DTG::outputDTGInfo()
{
    for (size_t i=0;i<getNumberOfDTGs();i++){
       std::cout << "Variable: " << _problem.get_variables()[i].get_name() << std::endl;
       for (size_t j=0;j<_problem.get_variables()[i].get_range();j++)
         if (isLeaf(i,j)) std::cout << _problem.get_variables()[i].get_value(j) << " is a leaf node" <<std::endl;
         else
           for (size_t k=0;k<_problem.get_variables()[i].get_range();k++) if (j!=k && isPath(i,j,k)) std::cout << _problem.get_variables()[i].get_value(j) << "-->" << _problem.get_variables()[i].get_value(k) << std::endl;
    }
}
