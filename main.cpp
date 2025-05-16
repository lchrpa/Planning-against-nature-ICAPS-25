#include <iostream>
#include <vector>
#include "SAS_problem.hpp"
#include "SAS_plan.hpp"
#include "SAS_parser.hpp"
#include "SAS_relaxed_assignment.hpp"
#include "utility.h"
#include "downward_driver.hh"
#include "RPG.hpp"
#include <ctime>

#include <string>
#include <stdio.h>

#include "DTG.hpp"
#include <relax.hpp>

using namespace std;


void create_invalidating_task(SAS_Problem &prob, SAS_Plan &plan,string filename){

    vector <SAS_Action> acts;
    vector <SAS_Variable_Info> vars(prob.get_variables());

    //get events into acts (vector of SAS_Action)
    std::vector<SAS_Event> events=prob.get_events();
    for (std::vector<SAS_Event>::iterator it=events.begin();it!=events.end();it++){
        //std::cout << it->get_name() << ", ";
        acts.push_back(SAS_Action(it->get_name(),it->get_preconditions(),it->get_effects(),it->get_cost()));
    }

    size_t i=prob.get_number_of_variables()+1;
    vector<string> vals={"false", "true"};

    vars.push_back(SAS_Variable_Info(i,"goal",0,2,vector<string>({"NegatedAtom goal()","Atom goal()`"})));

    vars.push_back(SAS_Variable_Info(i,"applied"+to_string(i),0,2,vector<string>({"NegatedAtom appl(pos"+to_string(i)+")","Atom appl(pos"+to_string(i)+")"})));

    //action in plan
    for (auto a : plan){
        i++;
        SAS_Assignment prec(a.get().get_preconditions());
        SAS_Assignment effs(a.get().get_minimal_effects());

        vars.push_back(SAS_Variable_Info(i,"applied"+to_string(i),0,2,vector<string>({"NegatedAtom appl(pos"+to_string(i)+")","Atom appl(pos"+to_string(i)+")"})));

        prec.set(i-1,1);
        prec.set(i,0);
        effs.set(i,1);


        vector <pair<SAS_Assignment,SAS_Assignment>> cond_effs;

        cond_effs.push_back(pair<SAS_Assignment,SAS_Assignment>(SAS_Assignment(),effs));

        acts.push_back(SAS_Action(a.get().get_name()+" pos"+to_string(i),prec,cond_effs,a.get().get_cost()));

        //goal actions
        for (auto x : a.get().get_preconditions()){
            for (size_t val=0;val<prob.get_variables()[x.first].get_range();val++){
                SAS_Assignment apre(i-1,1);
                apre.set(i,0);
                apre.set(prob.get_number_of_variables(),0);
                if (val!=x.second){
                    apre.set(x.first,val);
                    vector <pair<SAS_Assignment,SAS_Assignment>> aeffs;
                    aeffs.push_back(pair<SAS_Assignment,SAS_Assignment>(SAS_Assignment(),SAS_Assignment(prob.get_number_of_variables(),1)));
                    acts.push_back(SAS_Action("agoal pos"+to_string(i-1)+" v"+to_string(x.first)+ " val"+to_string(val), apre, aeffs ,0));
                }
            }
        }
    }

    //goal actions (for goal)
        for (auto x : prob.get_goal().get_assignment()){
            for (size_t val=0;val<prob.get_variables()[x.first].get_range();val++){
                SAS_Assignment apre(i,1);
                apre.set(prob.get_number_of_variables(),0);
                if (val!=x.second){
                    apre.set(x.first,val);
                    vector <pair<SAS_Assignment,SAS_Assignment>> aeffs;
                    aeffs.push_back(pair<SAS_Assignment,SAS_Assignment>(SAS_Assignment(),SAS_Assignment(prob.get_number_of_variables(),1)));
                    acts.push_back(SAS_Action("agoal pos"+to_string(i)+" v"+to_string(x.first)+ " val"+to_string(val), apre, aeffs ,0));
                }
            }
        }

    //initial state
    SAS_State init(prob.get_init());
    size_t j=prob.get_number_of_variables();
    init.set(j,0);j++;
    init.set(j,1);j++;
    for (j;j<=i;j++) init.set(j,0);j++;

    //goal
    SAS_Assignment goal(prob.get_number_of_variables(),1);

    SAS_Problem(3,0,vars,std::vector<SAS_Mutex>(),init,SAS_State(goal), acts, vector<SAS_Event>(),vector<SAS_Axiom>()).to_sas_file(filename);
}


int main(int argc, char **argv)
{

    //std::clock_t c_start = std::clock();

    SAS_Parser parser;

    if (argc<5) {cout << "Arguments <domain> <problem> <plan> [-v] [-g] [-c] are required for Robust Plan verification/generation." << endl; return -1;}

    //translate
    //fd_translate_with_events("event_","domains/auv/domain-no-destroy.pddl","domains/auv/problem1.pddl");
    fd_translate_with_events("event_",argv[1],argv[2]);

    SAS_Problem prob=parser.parse("output.sas","event_");

    string type=argv[4];

    //prob.to_sas_file("output-act.sas");

    if (type=="-v"){//vefification
        //parse plan
        SAS_Plan plan=parser.parse_plan(argv[3],prob,"event_",true);

        relax v(prob,plan);
        if (v.check_robustness()) cout << "Success" << endl; else cout << "Fail" << endl;
    }
    if (type=="-g") {//generation
        string heur;
        if (argc<6) heur="bfs"; else heur=argv[5];
        relax g(prob);
        if (heur=="bfs") {
            if (g.generate_robust_plan_bfs()){
                cout << "Robust Plan Generation Success" << endl;
                g.get_plan().to_file(argv[3]);
            } else {
                cout << "Robust Plan Generation Fail" << endl;
            }
        } else if (heur=="hmax"){
            if (g.generate_robust_plan_astar()){
                cout << "Robust Plan Generation Success" << endl;
                g.get_plan().to_file(argv[3]);
            } else {
                cout << "Robust Plan Generation Fail" << endl;
            }
        } else {
            if (g.generate_robust_plan_greedy()){
                cout << "Robust Plan Generation Success" << endl;
                g.get_plan().to_file(argv[3]);
            } else {
                cout << "Robust Plan Generation Fail" << endl;
            }
        }
    }
    if (type=="-l"){//generationcof LES
        relax g(prob);
        if (g.generate_les_greedy()){
                cout << "LES Generation Success" << endl;
                g.get_plan().to_file(argv[3]);
            } else {
                cout << "LES Generation Fail" << endl;
            }

    }
    if (type=="-c"){//vefification by compilation
        SAS_Plan plan=parser.parse_plan(argv[3],prob,"event_",true);
        create_invalidating_task(prob,plan,"output-inval.sas");
       // string filename=argv[3];
       // inval.to_sas_file(filename);
    }
/*
    //print all events
    vector<SAS_Event> events=prob.get_events();
    vector<SAS_Operator> ops;

    for (vector<SAS_Event>::iterator it=events.begin();it!=events.end();it++){
        cout << it->get_name() << ", ";
        ops.push_back(SAS_Operator(*it));
    }

    cout << endl;

    cout << events[0].isAchieverFor(events[1]) << endl;
    cout << events[1].isClobbererFor(events[2]) << endl;

    //print all applicable events
    vector<reference_wrapper<const SAS_Event>> app_events=prob.get_applicable_events(prob.get_init());

    for (vector<reference_wrapper<const SAS_Event>>::iterator it=app_events.begin();it!=app_events.end();it++){
        cout << it->get().get_name() << ", ";
        for (auto var : it->get().get_variables()) cout << var <<", ";
    }

    cout << endl;

    //RPG
    SAS_Relaxed_Assignment rinit(prob.get_init().get_assignment(),prob.get_variables());
    RPG rpg(rinit,ops);
    rpg.buildRPG();
    rpg.debug_print_layers();
*/


    //planning
    //DownwardDriver d("--alias \"lama-first\"","log.x");

    //d.plan_sas("output.sas","plan.sol");

    //parse plan
    //SAS_Plan plan=parser.parse_plan("plan.sol",prob,"event_",true);

    //std::clock_t c_planning = std::clock();

    //cout << "PLAN: ";
    //print plan
    //for(vector<reference_wrapper<SAS_Operator>>::const_iterator it=plan.begin();it!=plan.end();it++){
     //   cout << it->get().get_name() << (it->get().isEvent() ? " (event) " : "") << ", ";
    //}

    //cout << endl;

    //verify plan
   // DTG prob_DTG(prob,EVENTS_ONLY);
    //prob_DTG.BuildDTGs();


    //prob_DTG.outputDTGInfo();





    //verify plan (old)
/*  verifier ver(prob,plan);
    if (ver.verify_RPG_plain()){
        cout << "Plan verified successfully" << endl;
    } else {
        cout << "Plan verification failed at step " <<ver.get_step() << endl;
    }
*/
    //std::clock_t c_end = std::clock();

/*
    double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
    double time_elapsed_planning_ms = 1000.0 * (c_planning-c_start) / CLOCKS_PER_SEC;
    double time_elapsed_verification_ms = 1000.0 * (c_end-c_planning) / CLOCKS_PER_SEC;
    std::cout << "CPU time used: " << time_elapsed_ms << " ms" << endl;
    std::cout << "CPU time used (planning): " << time_elapsed_planning_ms << " ms" << endl;
    std::cout << "CPU time used (verification): " << time_elapsed_verification_ms << " ms" << endl;
*/
    //delete all output files
  //  system("rm output.sas log.x plan.sol");
    return 0;
}
