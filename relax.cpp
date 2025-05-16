#include<deque>

#include "relax.hpp"

bool relax::check_robustness(){

    //get events into ops (vector of SAS_Operator)
    std::vector<SAS_Event> events=_problem.get_events();
    std::vector<SAS_Operator> ops;
    for (std::vector<SAS_Event>::iterator it=events.begin();it!=events.end();it++){
        //std::cout << it->get_name() << ", ";
        ops.push_back(SAS_Operator(*it));
    }

     //RPG
    SAS_Relaxed_Assignment f(_problem.get_init().get_assignment(),_problem.get_variables());
    //RPG rpg(rinit,ops);
    //SAS_Relaxed_Assignment f=rinit;

   //check actions
    for (auto a : _plan){
        RPG rpg(f,ops);
        rpg.buildRPG();
        //rpg.debug_print_layers();
        SAS_Assignment pre=a.get().get_preconditions();
        f=rpg.getLastAtomLayer();
        if (!f.satisfy_all(pre)||f.exist_multiple_value_variable(pre)) return false;
        f.strict_apply(a.get().get_minimal_effects());
    }
    //check goal
     RPG rpg(f,ops);
        rpg.buildRPG();
        //rpg.debug_print_layers();
        f=rpg.getLastAtomLayer();
        if (!f.satisfy_all(_problem.get_goal().get_assignment())||f.exist_multiple_value_variable(_problem.get_goal().get_assignment())) return false;

    return true;
}

void relax::look_for_only_effect_variables(const std::vector<SAS_Action>& actions)
{
    for (auto act : actions){
        for (auto e : act.get_minimal_effects()){
            if (!act.get_preconditions().contains(e.first)) only_effect_variables.insert(e.first);
        }
    }
}


bool relax::is_duplicite(const SAS_Relaxed_Assignment& s, std::list<SAS_Relaxed_Assignment>& states)
{
    for (auto st : states){
        if (s.match_all(st)) return true;
    }
    return false;
}

bool relax::is_deadend(const SAS_Relaxed_Assignment& s)
{
    for (auto g : _problem.get_goal().get_assignment()){
        if (only_effect_variables.find(g.first)==only_effect_variables.end() && s.multiple_values(g.first)) return true;
    }
    return false;
}

int relax::heur(const SAS_Relaxed_Assignment& f, std::vector<SAS_Operator>& acts,char type, bool skip)
{
    /*
    SAS_Relaxed_Assignment a=f;
    a.clean_duplicite_variables();

    RPG rg(a,acts);
    return rg.hmax(_problem.get_goal().get_assignment());
    */
    if (skip) {
        relaxed_heuristics h(f,acts,type,true,&_safe_facts);
        return h.compute_heur(_problem.get_goal().get_assignment());
    }   else {
            relaxed_heuristics h(f,acts,type);
            return h.compute_heur(_problem.get_goal().get_assignment());
    }
}

bool relax::generate_robust_plan_bfs()
{
        //get events into ops (vector of SAS_Operator)
    std::vector<SAS_Event> events=_problem.get_events();
    std::vector<SAS_Operator> ops;
    for (std::vector<SAS_Event>::iterator it=events.begin();it!=events.end();it++){
        //std::cout << it->get_name() << ", ";
        ops.push_back(SAS_Operator(*it));
    }

    //actions
    std::vector<SAS_Action> actions=_problem.get_actions();
    look_for_only_effect_variables(actions);

    size_t nodes = 0;

     //RPG
    SAS_Relaxed_Assignment init(_problem.get_init().get_assignment(),_problem.get_variables());

    std::deque<search_node> search_queue;
    std::list<SAS_Relaxed_Assignment> expanded;
    search_queue.push_back(search_node(init,std::vector<std::reference_wrapper<SAS_Operator>>()));
    while (!search_queue.empty()){
        search_node s=search_queue.front();
        search_queue.pop_front();
        if (is_deadend(s._f) || is_duplicite(s._f,expanded)) continue;
        expanded.push_back(s._f);
         RPG rpg(s._f,ops);
        rpg.buildRPG();

        nodes++;

        SAS_Relaxed_Assignment f=rpg.getLastAtomLayer();


        //check goal
        if (f.satisfy_all(_problem.get_goal().get_assignment())&&!f.exist_multiple_value_variable(_problem.get_goal().get_assignment())){
            //debug
            for (auto a : s._pi){
                std::cout << a.get().get_name() << std::endl;
            }
            std::cout << "Expanded nodes: " <<nodes <<std::endl;
            _plan=SAS_Plan(&_problem,s._pi);
            return true;
        }

        //expanding
        for (auto a : actions){

            //check applicability
            if (f.satisfy_all(a.get_preconditions())&&!f.exist_multiple_value_variable(a.get_preconditions())){
                /*
                std::vector<std::reference_wrapper<SAS_Operator>> seq=s._pi;
                seq.emplace_back(a);
                SAS_Relaxed_Assignment fx=f;
                fx.strict_apply(a.get_minimal_effects());
                search_queue.push_back(search_node(fx,seq));
                std::cout << search_queue.back()._pi.front().get().get_name() << std::endl;
                */

                search_node node(f,s._pi);
                node._f.strict_apply(a.get_minimal_effects());
                //node._s.apply(a.get_minimal_effects());
//                std::cout << "depth: " << s._pi.size() << ", action: " << a.get_name() << std::endl;
                node._pi.emplace_back(_problem.find_operator_by_name(a.get_name()));
                //std::cout << node._pi.back().get().get_name() << std::endl;
                search_queue.push_back(node);

                //debug
  /*
                for (auto x : search_queue.back()._pi){
                  std::cout << x.get().get_name() << ", ";
                }
                std::cout << std::endl;
                //std::cout << search_queue.back()._pi.front().get().get_name() << std::endl;
*/
            }

        }

    }
    return false;
}


bool relax::generate_robust_plan_astar()
{
        //get events into ops (vector of SAS_Operator)
    std::vector<SAS_Event> events=_problem.get_events();
    std::vector<SAS_Operator> ops;
    for (std::vector<SAS_Event>::iterator it=events.begin();it!=events.end();it++){
        //std::cout << it->get_name() << ", ";
        ops.push_back(SAS_Operator(*it));
    }

    //actions
    std::vector<SAS_Action> actions=_problem.get_actions();
    std::vector<SAS_Operator> acts;
    for (std::vector<SAS_Action>::iterator it=actions.begin();it!=actions.end();it++){
        //std::cout << it->get_name() << ", ";
        acts.push_back(SAS_Operator(*it));
    }
    look_for_only_effect_variables(actions);

    size_t nodes = 0;

     //RPG
    SAS_Relaxed_Assignment init(_problem.get_init().get_assignment(),_problem.get_variables());

    std::priority_queue<search_node,std::deque<search_node>,fullcost> search_queue;
    std::list<SAS_Relaxed_Assignment> expanded;
    size_t h=heur(init,acts);
    if (h==INFTY) return false;
    search_queue.push(search_node(init,std::vector<std::reference_wrapper<SAS_Operator>>(),0,h));
    std::cout << "initial heuristic: " <<search_queue.top()._h << std::endl;
    while (!search_queue.empty()){
        search_node s=search_queue.top();
        search_queue.pop();
        if (/*is_deadend(s._f) ||*/ is_duplicite(s._f,expanded)) continue;
        //std::cout << "expanded node heuristic: " <<s._h << " (and cost): " <<s._c <<  std::endl;
        expanded.push_back(s._f);
         RPG rpg(s._f,ops);
        rpg.buildRPG();

        SAS_Relaxed_Assignment f=rpg.getLastAtomLayer();

        nodes++;

        //check goal
        if (f.satisfy_all(_problem.get_goal().get_assignment())&&!f.exist_multiple_value_variable(_problem.get_goal().get_assignment())){
            //debug
            for (auto a : s._pi){
                std::cout << a.get().get_name() << std::endl;
            }
            std::cout << "Expanded nodes: " <<nodes <<std::endl;
            _plan=SAS_Plan(&_problem,s._pi);

            return true;
        }

        //expanding
        for (auto a : actions){

            //check applicability
            if (f.satisfy_all(a.get_preconditions())&&!f.exist_multiple_value_variable(a.get_preconditions())){
                /*
                std::vector<std::reference_wrapper<SAS_Operator>> seq=s._pi;
                seq.emplace_back(a);
                SAS_Relaxed_Assignment fx=f;
                fx.strict_apply(a.get_minimal_effects());
                search_queue.push_back(search_node(fx,seq));
                std::cout << search_queue.back()._pi.front().get().get_name() << std::endl;
                */

                search_node node(f,s._pi,s._g+1,0);
                node._f.strict_apply(a.get_minimal_effects());
                h=heur(node._f,acts);
                if (h<INFTY){
                    //node._s.apply(a.get_minimal_effects());
//                std::cout << "depth: " << s._pi.size() << ", action: " << a.get_name() << std::endl;
                    node._h=h;
                    node._c=node._g+h;
                    node._pi.emplace_back(_problem.find_operator_by_name(a.get_name()));
                //std::cout << node._pi.back().get().get_name() << std::endl;
                    search_queue.push(node);
                }
                //debug
  /*
                for (auto x : search_queue.back()._pi){
                  std::cout << x.get().get_name() << ", ";
                }
                std::cout << std::endl;
                //std::cout << search_queue.back()._pi.front().get().get_name() << std::endl;
*/
            }

        }

    }
    return false;
}

bool relax::generate_robust_plan_greedy()
{
        //get events into ops (vector of SAS_Operator)
    std::vector<SAS_Event> events=_problem.get_events();
    std::vector<SAS_Operator> ops;
    for (std::vector<SAS_Event>::iterator it=events.begin();it!=events.end();it++){
        //std::cout << it->get_name() << ", ";
        ops.push_back(SAS_Operator(*it));
    }

    //actions
    std::vector<SAS_Action> actions=_problem.get_actions();
    std::vector<SAS_Operator> acts;
    for (std::vector<SAS_Action>::iterator it=actions.begin();it!=actions.end();it++){
        //std::cout << it->get_name() << ", ";
        acts.push_back(SAS_Operator(*it));
    }
    look_for_only_effect_variables(actions);

    size_t nodes = 0;

     //RPG
    SAS_Relaxed_Assignment init(_problem.get_init().get_assignment(),_problem.get_variables());

    std::priority_queue<search_node,std::deque<search_node>,fullcost> search_queue;
    std::list<SAS_Relaxed_Assignment> expanded;
    size_t h=heur(init,acts,HADD);
    if (h==INFTY) return false;
    search_queue.push(search_node(init,std::vector<std::reference_wrapper<SAS_Operator>>(),0,h));
    std::cout << "initial heuristic: " <<search_queue.top()._h << std::endl;
    while (!search_queue.empty()){
        search_node s=search_queue.top();
        search_queue.pop();
        if (/*is_deadend(s._f) ||*/ is_duplicite(s._f,expanded)) continue;
        //std::cout << "expanded node heuristic: " <<s._h << " (and cost): " <<s._c <<  std::endl;
        expanded.push_back(s._f);
         RPG rpg(s._f,ops);
        rpg.buildRPG();

        SAS_Relaxed_Assignment f=rpg.getLastAtomLayer();

        nodes++;

        //check goal
        if (f.satisfy_all(_problem.get_goal().get_assignment())&&!f.exist_multiple_value_variable(_problem.get_goal().get_assignment())){
            //debug
            for (auto a : s._pi){
                std::cout << a.get().get_name() << std::endl;
            }
            std::cout << "Expanded nodes: " <<nodes <<std::endl;
            _plan=SAS_Plan(&_problem,s._pi);

            return true;
        }

        //expanding
        for (auto a : actions){

            //check applicability
            if (f.satisfy_all(a.get_preconditions())&&!f.exist_multiple_value_variable(a.get_preconditions())){
                /*
                std::vector<std::reference_wrapper<SAS_Operator>> seq=s._pi;
                seq.emplace_back(a);
                SAS_Relaxed_Assignment fx=f;
                fx.strict_apply(a.get_minimal_effects());
                search_queue.push_back(search_node(fx,seq));
                std::cout << search_queue.back()._pi.front().get().get_name() << std::endl;
                */

                search_node node(f,s._pi,0,0);
                node._f.strict_apply(a.get_minimal_effects());
                h=heur(node._f,acts,HADD);
                if (h<INFTY){
                    //node._s.apply(a.get_minimal_effects());
//                std::cout << "depth: " << s._pi.size() << ", action: " << a.get_name() << std::endl;
                    node._h=node._c=h;
                    node._pi.emplace_back(_problem.find_operator_by_name(a.get_name()));
                //std::cout << node._pi.back().get().get_name() << std::endl;
                    search_queue.push(node);
                }
                //debug
  /*
                for (auto x : search_queue.back()._pi){
                  std::cout << x.get().get_name() << ", ";
                }
                std::cout << std::endl;
                //std::cout << search_queue.back()._pi.front().get().get_name() << std::endl;
*/
            }

        }

    }
    return false;
}

bool relax::is_fact_safe_lemma(size_t var, size_t value)
{

    for (auto ev : _problem.get_events()) if (ev.deletesFact(var,value)){
        bool e2_found=false;
        for (auto ev2 : _problem.get_events()) if (ev.hold_after().satisfy_all(ev2.get_preconditions())){
               for (auto ev3 : _problem.get_events()) if (ev2.get_name()!=ev3.get_name()&&ev3.isClobbererFor(ev2)&&!ev2.get_preconditions().any_mismatch(ev3.get_preconditions())){
                       if ((!ev3.get_preconditions().contains(var)||ev3.get_preconditions().get(var)!=value)&&(!ev3.get_minimal_effects().contains(var)||ev3.get_minimal_effects().get(var)!=value)){
                               return false;
                            }
                        }
                        //if (!is_safe) break;
                        e2_found=true;
                    }
                if (!e2_found) return false;
    }
    return true;
}




bool relax::generate_les_greedy()
{
        //get events into ops (vector of SAS_Operator)
    std::vector<SAS_Event> events=_problem.get_events();
    std::vector<SAS_Operator> ops;
    std::vector<SAS_Event> ev_exp;
    DTG ndtg(_problem,EVENTS_ONLY);
    ndtg.BuildDTGs();
    for (std::vector<SAS_Event>::iterator it=events.begin();it!=events.end();it++){
        //std::cout << it->get_name() << ", ";
        ops.push_back(SAS_Operator(*it));
        bool fnd;
        if (ndtg.unreachable(it->get_preconditions(),it->get_minimal_effects())) ev_exp.push_back(*it);
        /*
        for (auto e : it->get_minimal_effects()){
            SAS_Assignment p = it->get_preconditions();
            fnd=false;
            if (p.contains(e.first) && e.second!=p.get(e.first))
                for (auto ev : events) if (ev.get_minimal_effects().contains(e.first) && ev.get_minimal_effects().get(e.first)==p.get(e.first)){fnd=true;break;}
            if (!fnd) ev_exp.push_back(*it);
        }
        */
    }

    //actions
    std::vector<SAS_Action> actions=_problem.get_actions();
    std::vector<SAS_Operator> acts;
    for (std::vector<SAS_Action>::iterator it=actions.begin();it!=actions.end();it++){
        //std::cout << it->get_name() << ", ";
        acts.push_back(SAS_Operator(*it));
    }
    look_for_only_effect_variables(actions);

    //safe facts
    for (auto var : _problem.get_variables()){
        _safe_facts[var.get_id()]=std::vector<bool>(var.get_range(),false);
        for (size_t i=0;i<var.get_range();i++) {_safe_facts[var.get_id()][i]=is_fact_safe_lemma(var.get_id(),i);} //if (_safe_facts[var.get_id()][i]) std::cout << "safe: " << var.get_name() << var.get_value(i) << std::endl;}

    }

    size_t nodes = 0;

     //RPG
    SAS_Relaxed_Assignment init(_problem.get_init().get_assignment(),_problem.get_variables());

    std::priority_queue<search_node,std::deque<search_node>,fullcost> search_queue;
    std::list<SAS_Relaxed_Assignment> expanded;
    size_t h=heur(init,acts,HADD,true);
    if (h==INFTY) return false;
    search_queue.push(search_node(init,std::vector<std::reference_wrapper<SAS_Operator>>(),0,h));
    std::cout << "initial heuristic: " <<search_queue.top()._h << std::endl;
    while (!search_queue.empty()){
        search_node s=search_queue.top();
        search_queue.pop();
        if (/*is_deadend(s._f) ||*/ is_duplicite(s._f,expanded)) continue;
        //std::cout << "expanded node heuristic: " <<s._h << " (and cost): " <<s._c <<  std::endl;
        expanded.push_back(s._f);

        //expand events
        bool event_triggered = false;
        do{
            event_triggered = false;
        for (auto ev : ev_exp){
            if (s._f.satisfy_all(ev.get_preconditions())){
                RPG rpg(s._f,ops,(SAS_Operator*)&ev);
                rpg.buildRPG();
                SAS_Relaxed_Assignment fe=rpg.getLastAtomLayer();
                //std::cout << "processed event: " << ev.get_name() << std::endl;
                if (fe.check_applicability_with_safe(ev.get_preconditions(),_safe_facts)){
                    s._f.strict_apply(ev.get_minimal_effects());
                    s._pi.emplace_back(_problem.find_operator_by_name(ev.get_name()));
                    //std::cout <<ev.get_name() << std::endl;
                    event_triggered=true;
                    break;
                }
            }
        }
        } while (event_triggered);

        RPG rpg(s._f,ops);
        rpg.buildRPG();

        SAS_Relaxed_Assignment f=rpg.getLastAtomLayer();

        nodes++;

        //check goal
        if (f.satisfy_all(_problem.get_goal().get_assignment())&&f.check_applicability_with_safe(_problem.get_goal().get_assignment(),_safe_facts)){
            //debug
            SAS_Assignment q1(_problem.get_init().get_assignment());
            SAS_Assignment q2(_problem.get_init().get_assignment());

            for (auto a : s._pi){
                if (a.get().isEvent()){
                    q2.apply(a.get().get_minimal_effects());
                } else {
                    SAS_Assignment diff = q2.get_mismatch(q1);
                    std::cout << "Waitfor (apart of precondition): ";
                    for (auto z : diff){
                        std::vector<size_t> vv = ndtg.connected_from(z.first,z.second);
                        std::cout << _problem.get_variables()[z.first].get_name() << "-";
                        for (auto z2 : vv) std::cout << _problem.get_variables()[z.first].get_value(z2) << ",";
                        std::cout << " -- ";
                    }
                    std::cout <<std::endl;
                    std::cout << a.get().get_name() << std::endl;
                    q2.apply(a.get().get_minimal_effects());
                    q1=q2;
                }
            }
            std::cout << "Expanded nodes: " <<nodes <<std::endl;
            _plan=SAS_Plan(&_problem,s._pi);

            return true;
        }

        //expanding
        for (auto a : actions){

            //check applicability
            if (f.satisfy_all(a.get_preconditions())&&f.check_applicability_with_safe(a.get_preconditions(),_safe_facts)){
                /*
                std::vector<std::reference_wrapper<SAS_Operator>> seq=s._pi;
                seq.emplace_back(a);
                SAS_Relaxed_Assignment fx=f;
                fx.strict_apply(a.get_minimal_effects());
                search_queue.push_back(search_node(fx,seq));
                std::cout << search_queue.back()._pi.front().get().get_name() << std::endl;
                */

                search_node node(f,s._pi,0,0);
                node._f.strict_apply(a.get_minimal_effects());
                h=heur(node._f,acts,HADD,true);
                if (h<INFTY){
                    //node._s.apply(a.get_minimal_effects());
//                std::cout << "depth: " << s._pi.size() << ", action: " << a.get_name() << std::endl;
                    node._h=node._c=h;
                    node._pi.emplace_back(_problem.find_operator_by_name(a.get_name()));
                //std::cout << node._pi.back().get().get_name() << std::endl;
                    search_queue.push(node);
                }
                //debug
  /*
                for (auto x : search_queue.back()._pi){
                  std::cout << x.get().get_name() << ", ";
                }
                std::cout << std::endl;
                //std::cout << search_queue.back()._pi.front().get().get_name() << std::endl;
*/
            }

        }

    }
    return false;
}
