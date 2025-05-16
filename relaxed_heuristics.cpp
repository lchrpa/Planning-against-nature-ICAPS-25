#include "relaxed_heuristics.hpp"

relaxed_heuristics::relaxed_heuristics(const SAS_Relaxed_Assignment &ra, std::vector<SAS_Operator>& acts, char type, bool no_affected, std::unordered_map<size_t, std::vector<bool> > *safe){
    _type = type;
    _acts = acts;

    bool no_skip = safe == NULL;
    for (auto v : ra){
        _assignment_vals[v.first]=std::vector<int>(v.second.size());
        if (no_affected && ra.multiple_values(v.first)){
            if (no_skip) std::fill(_assignment_vals[v.first].begin(),_assignment_vals[v.first].end(),INFTY);
            else {
                for (size_t i=0;i<v.second.size();i++) _assignment_vals[v.first][i]= v.second[i]&&safe->at(v.first)[i] ? 0 : INFTY;
            }
        } else {
            for (size_t i=0;i<v.second.size();i++) _assignment_vals[v.first][i]= v.second[i] ? 0 : INFTY;
        }
    }
}

int relaxed_heuristics::compute_heur(const SAS_Assignment& goal)
{
    std::deque<op_data> data;
    size_t n=_acts.size();
    for (size_t i=0;i<n;i++){
        data.push_back(op_data(i));
    }
    data.push_back(op_data(n+1,true));

    while(!data.empty()){
        //process action applicability
        for (auto x = data.begin(); x!=data.end(); x++){
            const SAS_Assignment &temp = x->_goal ? goal : _acts[x->_op_id].get_preconditions();
            int val = 0;
            for (auto fact : temp){
                if (_assignment_vals[fact.first][fact.second]==INFTY){val=INFTY;break;}
                switch(_type){
                    case HMAX:
                        val = val < _assignment_vals[fact.first][fact.second] ? _assignment_vals[fact.first][fact.second] : val;
                        break;
                    case HADD:
                        val+=_assignment_vals[fact.first][fact.second];
                        break;
                    default:
                        val=0;
                }
            }
            x->_val=val;
            //std::cout << "Current val: " << val << std::endl;
            x->_appl=(val!=INFTY);
        }

       // std::cout << "Got here (before sort) " << std::endl;
        std::sort(data.begin(),data.end(),op_data_compare);
       // std::cout << "Got here (after sort) " << std::endl;
        //goal reached or no further action is applicable (so the goal cannot be reached)
        int min_val = data.front()._val;
        //std::cout << "Current min val: " << min_val << std::endl;
        if (min_val==INFTY) return min_val;
        //process action results
        do {
            //std::cout << "Current min val: " << min_val << std::endl;
            auto x = data.front();
            if (x._val!=min_val) break;
            if (x._goal) return min_val;
            const SAS_Assignment &temp = _acts[x._op_id].get_minimal_effects();
            for (auto fact : temp) if (_assignment_vals[fact.first][fact.second]> min_val+1 || _assignment_vals[fact.first][fact.second]==INFTY) _assignment_vals[fact.first][fact.second]=min_val+1;
            data.pop_front();
            //std::cout << "data size: " << data.size() << std::endl;
        } while (!data.empty());
    }
    return INFTY;
}

