//
//  CCEA.hpp
//  UTM_Monoploy_V2
//
//  Created by Scott S Forer on 8/8/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//


#ifndef CCEA_hpp
#define CCEA_hpp

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <ctime>
#include <sstream>


using namespace std;


class CCEA
{
    friend class Parameters;
    friend class Team;
    friend class Individual;
    friend class Waypoint;
    friend class Simulator;
    
protected:
    
    
public:
    vector<Team> corp;
    Parameters* pP;
    vector<Policy> sim_team;
    
    //CCEA build funcitons
    void create_population();
    void create_starting_telem();
    void create_checkpoints();
    void create_target_telem();
    void set_conflict_counter();
    void build_world();
    int loadwaypoints();
    
    //Functions for Simulation
    void set_up_experiment_parameters();
    void clone_policies();
    void build_team(int gen);
    void reset_selction_counter();
    void reset_selection_identifiers();
    void build_sim_team(int po);
    void get_fair_statistics();
    void get_policy_fitness(int po, int len);
    void simulate_team(vector<Policy>* sim_team, int gen, vector<double> &conflict_counter);
    
    //Binary Selection Funcitons
    struct less_than_policy_fitness;
    void sort_agent_policies();
    int binary_selection(int team, int indv);
    void natural_selection();
    int kill;
    void mutation(Policy &MP);
    void check_new_telem(Policy &MP, int w);
    
    
    //CCEA main
    void run_CCEA(int sr, int stat_run);
    
    //Statistics
    int fcount = 0;
    vector<double> min_team_0_fitness;
    vector<double> ave_team_0_fitness;
    vector<double> max_team_0_fitness;
    vector<double> min_team_1_fitness;
    vector<double> ave_team_1_fitness;
    vector<double> max_team_1_fitness;
    vector<double> conflict_counter;          //0_0, 0_1, 1_0, 1_1
    vector<double> ave_0_0_conflict;
    vector<double> ave_0_1_conflict;
    vector<double> ave_1_0_conflict;
    vector<double> ave_1_1_conflict;
    vector<double> ave_0_conflict;
    vector<double> ave_1_conflict;
    
    void store_ave_conflict_data();
    void get_statistics();
    void write_statistics_to_file(int sr);
    void write_conflict_data_to_file(int sr);
    void write_parameters_to_file(float seconds);
    void write_policies_to_file();
    
    
    //Simulator Test Functions
    //read the test fucntion parameters
    void Simulator_test_functions();
    void two_agents_one_policy_same_team_collide();
    void two_agents_one_policy_same_team_near_miss();
    void two_agents_one_policy_same_team_exact_miss();
    void two_agents_one_policy_same_team_close_parallel();
    void two_agents_one_policy_same_team_exact_parallel();
    void two_agents_one_policy_same_team_far_parallel();
    void two_agents_two_policy_same_team_collide();
    void two_agents_two_policy_same_team_near_miss();
    void two_agents_two_policy_same_team_exact_miss();
    void two_agents_two_policy_same_team_close_parallel();
    void two_agents_two_policy_same_team_exact_parallel();
    void two_agents_two_policy_same_team_far_parallel();
    void two_agents_one_policy_different_team_collide();
    void two_agents_one_policy_different_team_near_miss();
    void two_agents_one_policy_different_team_exact_miss();
    void two_agents_one_policy_different_team_close_parallel();
    void two_agents_one_policy_different_team_exact_parallel();
    void two_agents_one_policy_different_team_far_parallel();
    void four_agents_one_policy_different_team_collide_same_team();
    void four_agents_one_policy_different_team_near_miss_same_team();
    void four_agents_one_policy_different_team_exact_miss_same_team();
    void four_agents_one_policy_different_team_close_parallel_same_team();
    void four_agents_one_policy_different_team_exact_parallel_same_team();
    void four_agents_one_policy_different_team_far_parallel_same_team();
    void four_agents_one_policy_different_team_all_collide();
    
private:
    
    
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// Current Issues
//
//////// Resloved Issues
//Constructing a temporary vector containing a team for simulation
//Setting up pointers for the simulator
//Getting the fitness for each policy of a team for simulation
//Clean up build_team
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
//Creates Population for each simulation
//creates an instance for each waypoint for each policy for each agent in each team
void CCEA::create_population()
{
    for (int ii=0; ii < pP->num_teams; ii++)
    {
        Team T;
        corp.push_back(T);
        for (int jj=0; jj < pP->team_sizes.at(ii); jj++)
        {
            Individual I;
            corp.at(ii).agents.push_back(I);
            for (int pp=0;  pp < pP->num_policies; pp++)
            {
                Policy Po;
                corp.at(ii).agents.at(jj).policies.push_back(Po);
                corp.at(ii).agents.at(jj).policies.at(pp).policy_id = pp; //sets the policy id for that policy
                corp.at(ii).agents.at(jj).policies.at(pp).corp_id = ii; //sets the corporation id for that policy
                
                for (int ww=0; ww < pP->num_waypoints+2; ww++)
                {
                    Waypoint W;
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.push_back(W);
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(ww).waypoint_telem.resize(3);
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Starting Coordinates
//sets the starting telemetry for each policy for each agent in each team
void CCEA::create_starting_telem()
{
    for (int ii=0; ii<pP->num_teams; ii++)
    {
        for (int jj=0; jj<pP->team_sizes.at(ii); jj++)
        {
            for (int po=0; po<1; po++)
            {
                double x_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_x_dim);
                double y_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_y_dim);
                //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
                //int x_waypoint = 2;
                //int y_waypoint = 4;
                double z_waypoint = 0;
                
                //comment out for test fucntions
                //{
                corp.at(ii).agents.at(jj).policies.at(po).check_points.at(0).waypoint_telem.at(0)=x_waypoint;
                corp.at(ii).agents.at(jj).policies.at(po).check_points.at(0).waypoint_telem.at(1)=y_waypoint;
                corp.at(ii).agents.at(jj).policies.at(po).check_points.at(0).waypoint_telem.at(2)=z_waypoint;
            }
        }
    }
    
    //makes each starting telemetry for each policy for each agent the same
    for (int ii=0; ii < pP->num_teams; ii++)
    {
        for (int jj=0; jj<pP->team_sizes.at(ii); jj++)
        {
            for (int po=1;  po<pP->num_policies; po++)
            {
                corp.at(ii).agents.at(jj).policies.at(po).check_points.at(0).waypoint_telem.at(0) = corp.at(ii).agents.at(jj).policies.at(0).check_points.at(0).waypoint_telem.at(0);
                corp.at(ii).agents.at(jj).policies.at(po).check_points.at(0).waypoint_telem.at(1) = corp.at(ii).agents.at(jj).policies.at(0).check_points.at(0).waypoint_telem.at(1);
                corp.at(ii).agents.at(jj).policies.at(po).check_points.at(0).waypoint_telem.at(2) = corp.at(ii).agents.at(jj).policies.at(0).check_points.at(0).waypoint_telem.at(2);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Intermediate Waypoints
//sets the telemetry for each intermediate waypoint for each policy for each agent in each team
void CCEA::create_checkpoints()
{
    for (int ii=0; ii<pP->num_teams; ii++)
    {
        for (int jj=0; jj<pP->team_sizes.at(ii); jj++)
        {
            for (int po=0;  po<pP->num_policies; po++)
            {
                for (int ww=0; ww<pP->num_waypoints; ww++)
                {
                    double x_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_x_dim);
                    double y_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_y_dim);
                    double z_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_z_dim);
                    //int x_waypoint = 5;
                    //int y_waypoint = 6;
                    //int z_waypoint = 8;
                    
                    //comment out for test fucntions
                    //{
                    corp.at(ii).agents.at(jj).policies.at(po).check_points.at(ww+1).waypoint_telem.at(0) = x_waypoint;
                    corp.at(ii).agents.at(jj).policies.at(po).check_points.at(ww+1).waypoint_telem.at(1) = y_waypoint;
                    corp.at(ii).agents.at(jj).policies.at(po).check_points.at(ww+1).waypoint_telem.at(2) = z_waypoint;
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Target Coordinates
//sets the final telemetry for each policy for each agent in each team
void CCEA::create_target_telem()
{
    for (int ii=0; ii<pP->num_teams; ii++)
    {
        for (int jj=0; jj<pP->team_sizes.at(ii); jj++)
        {
            double x_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_x_dim);
            double y_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_y_dim);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 2;
            //int y_waypoint = 4;
            double z_waypoint = 0;
            
            //comment out for test fucntions
            //{
            corp.at(ii).agents.at(jj).policies.at(0).check_points.at(pP->num_waypoints+1).waypoint_telem.at(0)=x_waypoint;
            corp.at(ii).agents.at(jj).policies.at(0).check_points.at(pP->num_waypoints+1).waypoint_telem.at(1)=y_waypoint;
            corp.at(ii).agents.at(jj).policies.at(0).check_points.at(pP->num_waypoints+1).waypoint_telem.at(2)=z_waypoint;
        }
    }
    
    //makes each ending telemetry for each policy for each agent the same
    for (int ii=0; ii<pP->num_teams; ii++)
    {
        for (int jj=0; jj<pP->team_sizes.at(ii); jj++)
        {
            for (int po=1;  po<pP->num_policies; po++)
            {
                corp.at(ii).agents.at(jj).policies.at(po).check_points.at(pP->num_waypoints+1).waypoint_telem.at(0) = corp.at(ii).agents.at(jj).policies.at(0).check_points.at(pP->num_waypoints+1).waypoint_telem.at(0);
                corp.at(ii).agents.at(jj).policies.at(po).check_points.at(pP->num_waypoints+1).waypoint_telem.at(1) = corp.at(ii).agents.at(jj).policies.at(0).check_points.at(pP->num_waypoints+1).waypoint_telem.at(1);
                corp.at(ii).agents.at(jj).policies.at(po).check_points.at(pP->num_waypoints+1).waypoint_telem.at(2) = corp.at(ii).agents.at(jj).policies.at(0).check_points.at(pP->num_waypoints+1).waypoint_telem.at(2);
            }
        }
    }
}



/////////////////////////////////////////////////////////////////
//loads the waypoints for a static case
int CCEA::loadwaypoints()
{
    if (pP->stat_full_malicious_with_loaded_wp_with_len == 1)
    {
        ifstream ifile("static_waypoints.txt", ios::in);
        vector<double> sw;
        
        //check to see that the file was opened correctly:
        if (!ifile.is_open())
        {
            cerr << "There was a problem opening the input file!\n";
            exit(1);//exit or do additional error checking
        }
        
        double num = 0.0;
        //keep storing values from the text file so long as data exists:
        while (ifile >> num)
        {
            sw.push_back(num);
        }
        
        //verify that the scores were stored correctly:
        for (int i = 0; i < sw.size(); ++i)
        {
            //cout << sw[i] << endl;
        }
        int counter = 0;
        for (int inv=0; inv<pP->team_sizes.at(1); inv++)
        {
            for (int w=0; w<pP->num_waypoints+2; w++)
            {
                for (int i=0; i<3; i++)
                {
                    corp.at(1).agents.at(inv).policies.at(0).check_points.at(w).waypoint_telem.at(i) = sw.at(counter);
                    counter += 1;
                }
            }
        }
        clone_policies();
    }
    if (pP->stat_coop_with_loaded_wp_with_len == 1)
    {
        ifstream ifile("static_waypoints.txt", ios::in);
        vector<double> sw;
        
        //check to see that the file was opened correctly:
        if (!ifile.is_open())
        {
            cerr << "There was a problem opening the input file!\n";
            exit(1);//exit or do additional error checking
        }
        
        double num = 0.0;
        //keep storing values from the text file so long as data exists:
        while (ifile >> num)
        {
            sw.push_back(num);
        }
        
        //verify that the scores were stored correctly:
        for (int i = 0; i < sw.size(); ++i)
        {
            //cout << sw[i] << endl;
        }
        int counter = 0;
        for (int inv=0; inv<pP->team_sizes.at(1); inv++)
        {
            for (int w=0; w<pP->num_waypoints+2; w++)
            {
                for (int i=0; i<3; i++)
                {
                    corp.at(1).agents.at(inv).policies.at(0).check_points.at(w).waypoint_telem.at(i) = sw.at(counter);
                    counter += 1;
                }
            }
        }
        clone_policies();
    }
    if (pP->stat_domino_with_loaded_wp_with_len == 1)
    {
        ifstream ifile("static_waypoints.txt", ios::in);
        vector<double> sw;
        
        //check to see that the file was opened correctly:
        if (!ifile.is_open())
        {
            cerr << "There was a problem opening the input file!\n";
            exit(1);//exit or do additional error checking
        }
        
        double num = 0.0;
        //keep storing values from the text file so long as data exists:
        while (ifile >> num)
        {
            sw.push_back(num);
        }
        
        //verify that the scores were stored correctly:
        for (int i = 0; i < sw.size(); ++i)
        {
            //cout << sw[i] << endl;
        }
        int counter = 0;
        for (int inv=0; inv<pP->team_sizes.at(1); inv++)
        {
            for (int w=0; w<pP->num_waypoints+2; w++)
            {
                for (int i=0; i<3; i++)
                {
                    corp.at(1).agents.at(inv).policies.at(0).check_points.at(w).waypoint_telem.at(i) = sw.at(counter);
                    counter += 1;
                }
            }
        }
        clone_policies();
    }
    if (pP->stat_uncoop_with_loaded_wp_with_len == 1)
    {
        ifstream ifile("static_waypoints.txt", ios::in);
        vector<double> sw;
        
        //check to see that the file was opened correctly:
        if (!ifile.is_open())
        {
            cerr << "There was a problem opening the input file!\n";
            exit(1);//exit or do additional error checking
        }
        
        double num = 0.0;
        //keep storing values from the text file so long as data exists:
        while (ifile >> num)
        {
            sw.push_back(num);
        }
        
        //verify that the scores were stored correctly:
        for (int i = 0; i < sw.size(); ++i)
        {
            //cout << sw[i] << endl;
        }
        int counter = 0;
        for (int inv=0; inv<pP->team_sizes.at(1); inv++)
        {
            for (int w=0; w<pP->num_waypoints+2; w++)
            {
                for (int i=0; i<3; i++)
                {
                    corp.at(1).agents.at(inv).policies.at(0).check_points.at(w).waypoint_telem.at(i) = sw.at(counter);
                    counter += 1;
                }
            }
        }
        clone_policies();
    }
    return 0;
}



/////////////////////////////////////////////////////////////////
//Sets the conflict counters for the stat run to 0
void CCEA::set_conflict_counter()
{
    conflict_counter.resize(4);
    //cout << conflict_counter.size() << endl;
    for (int i=0; i<conflict_counter.size(); i++)
    {
        conflict_counter.at(i) = 0;
    }
}


/////////////////////////////////////////////////////////////////
//Build World for CCEA
//creates the instaces for the enitre population with memory alocation and creates the waypoint paths for each policy of each agent of each team
void CCEA::build_world()
{
    create_population();
    create_starting_telem();
    create_checkpoints();
    create_target_telem();
}


//clones the policies for each agent in team 1
void CCEA::clone_policies()
{
    for (int i=0; i<pP->team_sizes.at(1); i++)
    {
        for (int j=0; j<pP->num_policies; j++)
        {
            corp.at(1).agents.at(i).policies.at(j) = corp.at(1).agents.at(i).policies.at(0);
        }
    }
}


/////////////////////////////////////////////////////////////////
//changes the parameters based on which experiment is being ran
void CCEA::set_up_experiment_parameters()
{
    //ccop no leniency
    if (pP->coop_no_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 0;
        pP->amount_lenient = 1;
        pP->fair_trial = 0;
    }
    //coop with leniency
    if (pP->coop_with_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
    }
    //coop with leniency
    if (pP->stat_coop_with_loaded_wp_with_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
        clone_policies();
    }
    //coop fair trial
    if (pP->coop_fair == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 1;
    }
    //uncoop no leniency
    if (pP->uncoop_with_len == 1)
    {
        pP->uncoop = 1;
        pP->leniency = 0;
        pP->amount_lenient = 1;
        pP->fair_trial = 0;
    }
    //uncoop with leniency
    if (pP->uncoop_with_len == 1)
    {
        pP->uncoop = 1;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
    }
    if (pP->stat_uncoop_with_loaded_wp_with_len == 1)
    {
        pP->uncoop = 1;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
        clone_policies();
    }
    //uncoop behavioral switch with leniency
    if (pP->uncoop_behavioral_switch_with_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
    }
    //domino effect with leniency
    if (pP->domino_with_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
    }
    if (pP->stat_domino_with_loaded_wp_with_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
        clone_policies();
    }
    //domino behavioral switch with leniency
    if (pP->domino_behavioral_switch_with_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
    }
    //malicious with leniency
    if (pP->malicious_with_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
    }
    //static full malicious with leniency
    if (pP->stat_full_malicious_with_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
        clone_policies();
    }
    if (pP->stat_full_malicious_with_loaded_wp_with_len == 1)
    {
        pP->uncoop = 0;
        pP->leniency = 1;
        pP->amount_lenient = 5;
        pP->fair_trial = 0;
        clone_policies();
    }
}


/////////////////////////////////////////////////////////////////
//simulates the radomly generated sim_team
void CCEA::simulate_team(vector<Policy>* psim_team, int gen, vector<double> &conflict_counter)
{
    Simulator S;
    Parameters P;
    vector<double>* pconflict_counter = &conflict_counter;
    S.pP = &P;
    S.run_simulation(psim_team, gen, pconflict_counter);
}


/////////////////////////////////////////////////////////////////
//resets the selection identifier
void CCEA::reset_selection_identifiers()
{
    //resets each agents policy selection identifier
    for (int team=0; team<pP->num_teams; team++)
    {
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            for (int p=0; p<pP->num_policies; p++)
            {
                corp.at(team).agents.at(indv).policies.at(p).selected = -1;
                corp.at(team).agents.at(indv).policies.at(p).simulation_id = -1;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//resets the selection identifier
void CCEA::reset_selction_counter()
{
    for (int team=0; team<pP->num_teams; team++)
    {
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            for (int po=0; po<pP->num_policies; po++)
            {
                corp.at(team).agents.at(indv).policies.at(po).selection_counter = 0;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//build the randomly generated sim_team
void CCEA::build_sim_team(int po)
{
    //cout << "check corp ids" << endl;
    int ph = 0;     //place holder for assigning the corp id to the correct policy in sim_team
    for (int team=0; team<pP->num_teams; team++)
    {
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            int rand_select = 0;
            rand_select = rand () % pP->num_policies;       //radomly picks a policy
            //cout << corp.at(team).agents.at(indv).policies.at(rand_select).selected << endl;
            while (corp.at(team).agents.at(indv).policies.at(rand_select).selected != -1)
            {
                rand_select = rand () % pP->num_policies;       //randomly selects another policy if previous selected policy has already been selected
                //if (corp.at(team).agents.at(indv).policies.at(rand_select).selected == 1)
                //{
                //cout << corp.at(team).agents.at(indv).policies.at(rand_select).selected << endl;
                //}
            }
            //cout << po << endl;
            corp.at(team).agents.at(indv).policies.at(rand_select).simulation_id = po;    //sets team selection identifier
            corp.at(team).agents.at(indv).policies.at(rand_select).selection_counter += 1;
            corp.at(team).agents.at(indv).policies.at(rand_select).selected = 1;    //changes selection identifier to selected
            
            //cout << corp.at(team).agents.at(indv).policies.at(rand_select).corp_id << "\t";
            sim_team.push_back(corp.at(team).agents.at(indv).policies.at(rand_select));
            //cout << sim_team.at(ph).corp_id << endl;
            //select that policy for team
            //build team vector for simulation
            ph += 1;
        }
    }
    //cout << sim_team.size() << endl;
}


/////////////////////////////////////////////////////////////////
//gets the fair trial statistical data for the trial
void CCEA::get_fair_statistics()
{
    vector<double> temp;
    temp.resize(pP->team_sizes.at(0)*pP->num_policies);
    for (int indv=0; indv<pP->team_sizes.at(0); indv++)
    {
        for (int pp=0; pp<pP->num_policies; pp++)
        {
            temp.at(indv*pP->num_policies+pp) = corp.at(0).agents.at(indv).policies.at(pp).policy_fitness;
        }
    }
    
    //check sort
    /*
     cout << "temp size" << "\t" << temp.size() << endl;
     for (int i=0; i<pP->team_sizes.at(0)*pP->num_policies; i++)
     {
     cout << temp.at(i) << endl;
     }
     */
    
    double min = 1000000;
    for (int i=0; i<temp.size(); i++)
    {
        if (temp.at(i) < min)
        {
            min = temp.at(i);
        }
    }
    min_team_0_fitness.push_back(min);
    //cout << endl;
    //cout << min << endl;
    
    double ave_sum = 0;
    for (int indv=0; indv<pP->team_sizes.at(0); indv++)
    {
        for (int pp=0; pp<pP->num_policies; pp++)
        {
            ave_sum += temp.at(indv*pP->num_policies+pp);
        }
    }
    ave_team_0_fitness.push_back(ave_sum/(pP->team_sizes.at(0)*pP->num_policies));
    
    
    double max = -1;
    for (int i=0; i<temp.size(); i++)
    {
        if (temp.at(i) > max)
        {
            max = temp.at(i);
        }
    }
    max_team_0_fitness.push_back(max);
    //cout << max << endl;
    temp.clear();
}


/////////////////////////////////////////////////////////////////
//build the randomly generated sim_team
void CCEA::get_policy_fitness(int po, int len)
{
    for (int team=0; team<pP->num_teams; team++)
    {
        int cc = 0;
        if (team == 1)
        {
            cc = pP->team_sizes.at(team-1);
        }
        double sum = 0;
        for (int indv=0+cc; indv<cc+pP->team_sizes.at(team); indv++)
        {
            sum += sim_team.at(indv).policy_fitness;
        }
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            for (int p=0; p<pP->num_policies; p++)
            {
                if (corp.at(team).agents.at(indv).policies.at(p).simulation_id == po)
                {
                    //without leniency
                    if (pP->leniency == 0)
                    {
                        corp.at(team).agents.at(indv).policies.at(p).policy_fitness = sum;
                    }
                    
                    //with leniency
                    if (pP->leniency == 1)
                    {
                        if(len==0)
                        {
                            corp.at(team).agents.at(indv).policies.at(p).policy_fitness = sum;
                        }
                        if (len>0)
                        {
                            if(corp.at(team).agents.at(indv).policies.at(p).policy_fitness>sum)
                            {
                                corp.at(team).agents.at(indv).policies.at(p).policy_fitness = sum;
                            }
                        }
                    }
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//runs the entire build and simulation for each sim_team
void CCEA::build_team(int gen)
{
    reset_selction_counter();
    
    int ll = 0;
    if (pP->leniency == 1)
    {
        ll = pP->amount_lenient;
    }
    if (pP->leniency == 0)
    {
        pP->amount_lenient = 1;
        ll = pP->amount_lenient;
    }
    
    
    if (pP->fair_trial == 0)
    {
        for (int len=0; len<ll; len++)
        {
            reset_selection_identifiers();
            
            //builds teams for simulation of randomly selected policies from each agent
            for (int po=0; po<pP->num_policies; po++)
            {
                //sim_team.resize(pP->team_sizes.at(0)+pP->team_sizes.at(1));
                //cout << "-----------------------------------------------------------------------------------" << endl;
                //cout << "-----------------------------------------------------------------------------------" << endl;
                
                build_sim_team(po);
                
                vector<Policy>* psim_team = &sim_team;
                simulate_team(psim_team, gen, conflict_counter);
                
                get_policy_fitness(po, len);
                
                sim_team.erase(sim_team.begin(), sim_team.end());
            }
        }
    }
    
    
    if (pP->fair_trial == 1)
    {
        if (fcount<pP->gen_max)
        {
            for (int len=0; len<ll; len++)
            {
                if (fcount == pP->gen_max)
                {
                    exit(1);
                }
                
                reset_selection_identifiers();
                
                //builds teams for simulation of randomly selected policies from each agent
                for (int po=0; po<pP->num_policies; po++)
                {
                    //sim_team.resize(pP->team_sizes.at(0)+pP->team_sizes.at(1));
                    //cout << "-----------------------------------------------------------------------------------" << endl;
                    //cout << "-----------------------------------------------------------------------------------" << endl;
                    
                    build_sim_team(po);
                    
                    vector<Policy>* psim_team = &sim_team;
                    simulate_team(psim_team, gen, conflict_counter);
                    
                    get_policy_fitness(po, len);
                    
                    sim_team.erase(sim_team.begin(), sim_team.end());
                }
                
                get_fair_statistics();
                fcount += 1;
                //cout << "check counter" << endl;
                //cout << fcount << endl;
            }
        }
    }
    
    
    /*
     for (int team=0; team<pP->num_teams; team++)
     {
         for (int indv=0; indv<pP->team_sizes.at(team); indv++)
         {
         cout << "agent" << "\t" << indv << endl;
             for (int p=0; p<pP->num_policies; p++)
             {
                 cout << "policy" << "\t" << corp.at(team).agents.at(indv).policies.at(p).policy_id << "\t" << "selected team" << "\t" << corp.at(team).agents.at(indv).policies.at(p).simulation_id << endl;
                 cout << "\t" << "fitness" << "\t" <<  corp.at(team).agents.at(indv).policies.at(p).policy_fitness << endl;
             }
             cout << endl;
         }
     }
     */
     cout << endl;
     cout << endl;
}


/////////////////////////////////////////////////////////////////
//sorts the population based on their fitness from lowest to highest
struct CCEA::less_than_policy_fitness
{
    inline bool operator() (const Policy& struct1, const Policy& struct2)
    {
        return (struct1.policy_fitness < struct2.policy_fitness);
    }
};


/////////////////////////////////////////////////////////////////
//sorts each agents policy based on their fitness
void CCEA::sort_agent_policies()
{
    for (int team=0; team<pP->num_teams; team++)
    {
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            sort(corp.at(team).agents.at(indv).policies.begin(), corp.at(team).agents.at(indv).policies.end(), less_than_policy_fitness());
        }
    }
    
    /*
     for (int team=0; team<pP->num_teams; team++)
     {
     for (int indv=0; indv<pP->team_sizes.at(team); indv++)
     {
     cout << "agent" << "\t" << indv << endl;
     for (int p=0; p<pP->num_policies; p++)
     {
     cout << "policy" << "\t" << corp.at(team).agents.at(indv).policies.at(p).policy_id << "\t" << "selected team" << "\t" << corp.at(team).agents.at(indv).policies.at(p).simulation_id << endl;
     cout << "\t" << "fitness" << "\t" <<  corp.at(team).agents.at(indv).policies.at(p).policy_fitness << endl;
     }
     cout << endl;
     }
     }
     */
}


/////////////////////////////////////////////////////////////////
//Randomly picks two policies for each agent and compares their fitness
int CCEA::binary_selection(int team, int indv)
{
    int loser;
    int index_1 = rand() % corp.at(team).agents.at(indv).policies.size();
    int index_2 = rand() % corp.at(team).agents.at(indv).policies.size();
    while (index_1 == index_2)
    {
        index_2 = rand() % corp.at(team).agents.at(indv).policies.size();
    }
    int f1 = corp.at(team).agents.at(indv).policies.at(index_1).policy_fitness;
    int f2 = corp.at(team).agents.at(indv).policies.at(index_2).policy_fitness;
    if (f1 < f2)
    {
        loser = index_2;
    }
    else
    {
        loser = index_1;
    }
    return loser;
}


/////////////////////////////////////////////////////////////////
//checks that the mutate telemetry is within the bounds of the simulator map parameters
void CCEA::check_new_telem(Policy &MP, int w)
{
    for (int ww=0; ww<3; ww++)
    {
        if (ww == 0)
        {
            if (MP.check_points.at(w).waypoint_telem.at(ww) < pP->min_x_dim)
            {
                MP.check_points.at(w).waypoint_telem.at(ww) = pP->min_x_dim;
            }
            if (MP.check_points.at(w).waypoint_telem.at(ww) > pP->max_x_dim)
            {
                MP.check_points.at(w).waypoint_telem.at(ww) = pP->max_x_dim;
            }
        }
        if (ww == 1)
        {
            if (MP.check_points.at(w).waypoint_telem.at(ww) < pP->min_y_dim)
            {
                MP.check_points.at(w).waypoint_telem.at(ww) = pP->min_y_dim;
            }
            if (MP.check_points.at(w).waypoint_telem.at(ww) > pP->max_y_dim)
            {
                MP.check_points.at(w).waypoint_telem.at(ww) = pP->max_y_dim;
            }
        }
        if (ww == 2)
        {
            if (MP.check_points.at(w).waypoint_telem.at(ww) < pP->min_z_dim)
            {
                MP.check_points.at(w).waypoint_telem.at(ww) = pP->min_z_dim;
            }
            if (MP.check_points.at(w).waypoint_telem.at(ww) > pP->max_z_dim)
            {
                MP.check_points.at(w).waypoint_telem.at(ww) = pP->max_z_dim;
            }
        }
    }
}



/////////////////////////////////////////////////////////////////
//Runs the mutation for a selected policy
void CCEA::mutation(Policy &MP)
{
    //will only mutate the intermediate waypoints (not the staring or ending waypoints)
    for (int w=1; w<pP->num_waypoints+1; w++)
    {
        double rand_num = 0;
        rand_num = ((double) rand() / (RAND_MAX));       //double between 0 and 1
        //will only mutate the number of intermediate waypoints with a likelyhood = mutate_percentage
        if (rand_num<=pP->mutate_percentage/100)
        {
            for (int ww=0; ww<3; ww++)
            {
                double rand_sign = 0;
                rand_sign = ((double) rand() / (RAND_MAX));       //double between 0 and 1
                //if true will switch rand_sign to -1 else rand_sign = 1
                if (rand_sign<=0.5)
                {
                    rand_sign = -1;
                }
                else
                {
                    rand_sign = 1;
                }
                double original = 0;
                original = MP.check_points.at(w).waypoint_telem.at(ww);
                original = original + (rand_sign)*(pP->mutation_range);
                MP.check_points.at(w).waypoint_telem.at(ww) = original;
            }
            check_new_telem(MP, w);
        }
    }
}


/////////////////////////////////////////////////////////////////
//Runs the entire selection and mutation process
void CCEA::natural_selection()
{
    for (int team=0; team<pP->num_teams; team++)
    {
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            //will only run for half the num_policies
            for (int p=0; p<pP->to_kill; p++)
            {
                kill = 0;
                kill = binary_selection(team, indv);
                //will erase the losing policy
                corp.at(team).agents.at(indv).policies.erase(corp.at(team).agents.at(indv).policies.begin() + kill);
            }
        }
    }
    
    for (int team=0; team<pP->num_teams; team++)
    {
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            //will only run for half the num_policies
            for (int p=0; p<pP->to_replicate; p++)
            {
                Policy MP;
                int spot = 0;
                spot = rand() % corp.at(team).agents.at(indv).policies.size();
                MP = corp.at(team).agents.at(indv).policies.at(spot);
                if (pP->stat_full_malicious_with_len == 0)
                {
                    if (pP->stat_full_malicious_with_loaded_wp_with_len == 0)
                    {
                        if (pP->stat_coop_with_loaded_wp_with_len == 0)
                        {
                            if (pP->stat_domino_with_loaded_wp_with_len == 0)
                            {
                                if (pP->stat_uncoop_with_loaded_wp_with_len == 0)
                                {
                                    mutation(MP);
                                }
                            }
                        }
                    }
                }
                if (pP->stat_full_malicious_with_len == 1)
                {
                    if (team == 0)
                    {
                        mutation(MP);
                    }
                }
                if (pP->stat_full_malicious_with_loaded_wp_with_len == 1)
                {
                    if (team == 0)
                    {
                        mutation(MP);
                    }
                }
                if (pP->stat_coop_with_loaded_wp_with_len == 1)
                {
                    if (team == 0)
                    {
                        mutation(MP);
                    }
                }
                if (pP->stat_domino_with_loaded_wp_with_len == 1)
                {
                    if (team == 0)
                    {
                        mutation(MP);
                    }
                }
                if (pP->stat_uncoop_with_loaded_wp_with_len == 1)
                {
                    if (team == 0)
                    {
                        mutation(MP);
                    }
                }
                corp.at(team).agents.at(indv).policies.push_back(MP);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//gets and sotres the average conflict data
void CCEA::store_ave_conflict_data()
{

    double a = 0;
    double b = 0;
    double c = 0;
    double d = 0;
    double e = 0;
    double f = 0;
    a = (conflict_counter.at(0)/(pP->num_policies*pP->amount_lenient))/2;
    b = conflict_counter.at(1)/(pP->num_policies*pP->amount_lenient);
    c = conflict_counter.at(2)/(pP->num_policies*pP->amount_lenient);
    d = (conflict_counter.at(3)/(pP->num_policies*pP->amount_lenient))/2;
    e = a+b;
    f = c+d;
    //cout << a << endl;
    //cout << b << endl;
    //cout << e << endl;
    //cout << c << endl;
    //cout << d << endl;
    //cout << f << endl;
    
    ave_0_0_conflict.push_back(a);
    //cout << "average 0_0 conflict" << "\t" << ave_0_0_conflict.at(ave_0_0_conflict.size()-1) << endl;
    ave_0_1_conflict.push_back(b);
    //cout << "average 0_1 conflict" << "\t" << ave_0_1_conflict.at(ave_0_1_conflict.size()-1) << endl;
    ave_1_0_conflict.push_back(c);
    //cout << "average 1_0 conflict" << "\t" << ave_1_0_conflict.at(ave_1_0_conflict.size()-1) << endl;
    ave_1_1_conflict.push_back(d);
    //cout << "average 1_1 conflict" << "\t" << ave_1_1_conflict.at(ave_1_1_conflict.size()-1) << endl;
    
    ave_0_conflict.push_back(e);
    ave_1_conflict.push_back(f);
}


/////////////////////////////////////////////////////////////////
//gets the statistical data for the trail
void CCEA::get_statistics()
{
    double min_0_sum = 0;
    for (int indv=0; indv<pP->team_sizes.at(0); indv++)
    {
        min_0_sum += corp.at(0).agents.at(indv).policies.at(0).policy_fitness;
    }
    min_team_0_fitness.push_back(min_0_sum/pP->team_sizes.at(0));
    
    double ave_0_sum = 0;
    for (int indv=0; indv<pP->team_sizes.at(0); indv++)
    {
        for (int p=0; p<pP->num_policies; p++)
        {
            ave_0_sum += corp.at(0).agents.at(indv).policies.at(p).policy_fitness;
        }
    }
    ave_team_0_fitness.push_back(ave_0_sum/(pP->team_sizes.at(0)*pP->num_policies));
    
    double max_0_sum = 0;
    for (int indv=0; indv<pP->team_sizes.at(0); indv++)
    {
        max_0_sum += corp.at(0).agents.at(indv).policies.at(pP->num_policies-1).policy_fitness;
    }
    max_team_0_fitness.push_back(max_0_sum/pP->team_sizes.at(0));
    
    
    if (pP->num_teams == 2)
    {
        double min_1_sum = 0;
        for (int indv=0; indv<pP->team_sizes.at(1); indv++)
        {
            min_1_sum += corp.at(1).agents.at(indv).policies.at(0).policy_fitness;
        }
        min_team_1_fitness.push_back(min_1_sum/pP->team_sizes.at(1));
        
        double ave_1_sum = 0;
        for (int indv=0; indv<pP->team_sizes.at(1); indv++)
        {
            for (int p=0; p<pP->num_policies; p++)
            {
                ave_1_sum += corp.at(1).agents.at(indv).policies.at(p).policy_fitness;
            }
        }
        ave_team_1_fitness.push_back(ave_1_sum/(pP->team_sizes.at(1)*pP->num_policies));
        
        double max_1_sum = 0;
        for (int indv=0; indv<pP->team_sizes.at(1); indv++)
        {
            max_1_sum += corp.at(1).agents.at(indv).policies.at(pP->num_policies-1).policy_fitness;
        }
        max_team_1_fitness.push_back(max_1_sum/pP->team_sizes.at(1));
    }
}


/////////////////////////////////////////////////////////////////
//writes the statistical data for the trail to a txt file
void CCEA::write_statistics_to_file(int sr)
{
    ofstream File1;
    File1.open("Team 0 Min Fitness For All Gens.txt", ios_base::app);
    ofstream File2;
    File2.open("Team 0 Ave Fitness For All Gens.txt", ios_base::app);
    ofstream File3;
    File3.open("Team 0 Max Fitness For All Gens.txt", ios_base::app);
    //File1 << "Stat Run" << "\t" << sr << "\t";
    //File2 << "Stat Run" << "\t" << sr << "\t";
    //File3 << "Stat Run" << "\t" << sr << "\t";
    for (int i=0; i<pP->gen_max; i++)
    {
        File1 << min_team_0_fitness.at(i) << "\t";
        File2 << ave_team_0_fitness.at(i) << "\t";
        File3 << max_team_0_fitness.at(i) << "\t";
    }
    File1 << endl;
    File2 << endl;
    File3 << endl;
    
    ofstream File4;
    File4.open("Team 1 Min Fitness For All Gens.txt", ios_base::app);
    ofstream File5;
    File5.open("Team 1 Ave Fitness For All Gens.txt", ios_base::app);
    ofstream File6;
    File6.open("Team 1 Max Fitness For All Gens.txt", ios_base::app);
    if (pP->num_teams == 2)
    {
        //File4 << "Stat Run" << "\t" << sr << "\t";
        //File5 << "Stat Run" << "\t" << sr << "\t";
        //File6 << "Stat Run" << "\t" << sr << "\t";
        for (int i=0; i<pP->gen_max; i++)
        {
            File4 << min_team_1_fitness.at(i) << "\t";
            File5 << ave_team_1_fitness.at(i) << "\t";
            File6 << max_team_1_fitness.at(i) << "\t";
        }
        File4 << endl;
        File5 << endl;
        File6 << endl;
    }
    File1.close();
    File2.close();
    File3.close();
}


/////////////////////////////////////////////////////////////////
//writes the conflict data for the trail to a txt file
void CCEA::write_conflict_data_to_file(int sr)
{
    ofstream File7;
    File7.open("0_0 Conflict Data.txt", ios_base::app);
    ofstream File8;
    File8.open("0_1 Conflict Data.txt", ios_base::app);
    ofstream File9;
    File9.open("1_0 Conflict Data.txt", ios_base::app);
    ofstream File10;
    File10.open("1_1 Conflict Data.txt", ios_base::app);
    ofstream File12;
    File12.open("Team 0 Conflict Data.txt", ios_base::app);
    ofstream File13;
    File13.open("Team 1 Conflict Data.txt", ios_base::app);
    //File7 << "Stat Run" << "\t" << sr << "\t";
    for (int i=0; i<ave_0_0_conflict.size(); i++)
    {
        File7 << ave_0_0_conflict.at(i) << "\t";
    }
    File7 << endl;
    //File8 << "Stat Run" << "\t" << sr << "\t";
    for (int i=0; i<ave_0_1_conflict.size(); i++)
    {
        File8 << ave_0_1_conflict.at(i) << "\t";
    }
    File8 << endl;
    //File9 << "Stat Run" << "\t" << sr << "\t";
    for (int i=0; i<ave_1_0_conflict.size(); i++)
    {
        File9 << ave_1_0_conflict.at(i) << "\t";
    }
    File9 << endl;
    //File10 << "Stat Run" << "\t" << sr << "\t";
    for (int i=0; i<ave_1_1_conflict.size(); i++)
    {
        File10 << ave_1_1_conflict.at(i) << "\t";
    }
    File10 << endl;
    //File12 << "Stat Run" << "\t" << sr << "\t";
    for (int i=0; i<ave_0_conflict.size(); i++)
    {
        File12 << ave_0_conflict.at(i) << "\t";
    }
    File12 << endl;
    //File13 << "Stat Run" << "\t" << sr << "\t";
    for (int i=0; i<ave_1_conflict.size(); i++)
    {
        File13 << ave_1_conflict.at(i) << "\t";
    }
    File13 << endl;
    
    File7.close();
    File8.close();
    File9.close();
    File10.close();
    File12.close();
    File13.close();
}



/////////////////////////////////////////////////////////////////
//writes the parameters for the trail to a txt file
void CCEA::write_parameters_to_file(float seconds)
{
    ofstream File11;
    File11.open("Parameters.txt");
    File11 << "Simulation Parameters" << endl;
    File11 << "Map parameters x y z" << endl;
    File11 << pP->min_x_dim << "\t" << pP->max_x_dim << "\t" << pP->min_y_dim << "\t" << pP->max_y_dim << "\t" << pP->min_z_dim << "\t" <<pP->max_z_dim << endl;
    File11 << " " << endl;
    File11 << "Time parameters" << endl;
    File11 << "time max" << "\t" << pP->time_max << endl;
    File11 << "sampling time" << "\t" << pP->delta_t << endl;
    File11 << " " << endl;
    File11 << "velocity parameters" << endl;
    File11 << "max velocity" << "\t" << pP->max_flight_velocity << endl;
    File11 << "ca velocity" << "\t" << pP->ca_flight_speed << endl;
    File11 << " " << endl;
    File11 << "CA parameters" << endl;
    File11 << "ca radius" << "\t" << pP->ca_radius << endl;
    File11 << "ca incrementation" << "\t" << pP->ca_inc << endl;
    File11 << " " << endl;
    File11 << "CCEA Parameters" << endl;
    File11 << "Team parameters" << endl;
    File11 << "number of teams" << "\t" << "\t" << pP->num_teams << endl;
    for (int team=0; team<pP->num_teams; team++)
    {
        File11 << "team" << "\t" << team << "\t" << "number of agents" << "\t" << pP->team_sizes.at(team) << endl;
    }
    File11 << "number of policies" << "\t" << pP->num_policies << endl;
    File11 << "number of waypoints" << "\t" << pP->num_waypoints << endl;
    File11 << " " << endl;
    File11 << "EA parameters" << endl;
    File11 << "gerneration max" << "\t" << "\t" << pP->gen_max << endl;
    File11 << "mutation percentage" << "\t" << pP->mutate_percentage << endl;
    File11 << "mutation range" << "\t" << pP->mutation_range << endl;
    File11 << " " << endl;
    File11 << "Experiment Parameters" << endl;
    if (pP->coop_no_len == 0)
    {
        File11 << "coop no leniency off" << endl;
    }
    else
    {
        File11 << "coop no leniency on" << endl;
    }
    if (pP->coop_with_len == 0)
    {
        File11 << "coop with leniency off" << endl;
    }
    else
    {
        File11 << "coop with leniency on" << endl;
    }
    if (pP->stat_coop_with_loaded_wp_with_len == 0)
    {
        File11 << "static coop with loaded wp with leniency off" << endl;
    }
    else
    {
        File11 << "static coop with loaded wp with leniency on" << endl;
    }
    if (pP->coop_fair == 0)
    {
        File11 << "coop fair trial off" << endl;
    }
    else
    {
        File11 << "coop fair trial on" << endl;
    }
    if (pP->uncoop_no_len == 0)
    {
        File11 << "uncoop no leniency off" << endl;
    }
    else
    {
        File11 << "ucoop no leniency on" << endl;
    }
    if (pP->uncoop_with_len == 0)
    {
        File11 << "uncoop with leniency off" << endl;
    }
    else
    {
        File11 << "ucoop with leniency on" << endl;
    }
    if (pP->stat_uncoop_with_loaded_wp_with_len == 0)
    {
        File11 << "static uncoop with loaded wp with leniency off" << endl;
    }
    else
    {
        File11 << "static uncoop with loaded wp with leniency on" << endl;
    }
    if (pP->uncoop_behavioral_switch_with_len == 0)
    {
        File11 << "uncoop behavioral switch with leniency off" << endl;
    }
    else
    {
        File11 << "uncoop behavioral switch with leniency on" << endl;
    }
    if (pP->domino_with_len == 0)
    {
        File11 << "domino with leniency off" << endl;
    }
    else
    {
        File11 << "domino with leniency on" << endl;
    }
    if (pP->stat_domino_with_loaded_wp_with_len == 0)
    {
        File11 << "static domino with loaded waypoints with leniency off" << endl;
    }
    else
    {
        File11 << "static domino with loaded waypoints with leniency on" << endl;
    }
    if (pP->domino_behavioral_switch_with_len == 0)
    {
        File11 << "domino behavioral switch with leniency off" << endl;
    }
    else
    {
        File11 << "domino behavioral switch with leniency on" << endl;
    }
    if (pP->malicious_with_len == 0)
    {
        File11 << "malicious with leniency off" << endl;
    }
    else
    {
        File11 << "malicious with leniency on" << endl;
    }
    if (pP->stat_full_malicious_with_len == 0)
    {
        File11 << "static full malicious with leniency off" << endl;
    }
    else
    {
        File11 << "static full malicious with leniency on" << endl;
    }
    if (pP->stat_full_malicious_with_loaded_wp_with_len == 0)
    {
        File11 << "static full malicious with loaded wp with leniency off" << endl;
    }
    else
    {
        File11 << "static full malicious with loaded wp with leniency on" << endl;
    }
    if (pP->leniency == 0)
    {
        File11 << "leniency off" << endl;
    }
    else
    {
        File11 << "leniency on" << endl;
    }
    if (pP->fair_trial == 0)
    {
        File11 << "fair trial off" << endl;
    }
    else
    {
        File11 << "fair trial on" << endl;
    }
    if (pP->uncoop == 0)
    {
        File11 << "uncoop off" << endl;
    }
    else
    {
        File11 << "uncoop on" << endl;
    }
    
    File11 << " " << endl;
    File11 << "run time" << "\t" << seconds << "\t" << "seconds" << endl;
    File11.close();
}

void CCEA::write_policies_to_file()
{
    if (pP->num_teams == 1)
    {
        ofstream File12;
        File12.open("Best_Policies");
        for (int inv=0; inv<pP->team_sizes.at(0); inv++)
        {
            for (int w=0; w<pP->num_waypoints+2; w++)
            {
                for (int i=0; i<3; i++)
                {
                 File12 << corp.at(0).agents.at(inv).policies.at(0).check_points.at(w).waypoint_telem.at(i) << "\t";
                }
            }
            File12 << " " << endl;
        }
        File12.close();
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Runs the entire CCEA process
void CCEA::run_CCEA(int sr, int stat_run)
{
    clock_t t1, t2;
    t1 = clock();
    build_world();
    loadwaypoints();
    Simulator_test_functions();     //go to function to run tests
    set_up_experiment_parameters();
    for (int gen=0; gen<pP->gen_max; gen++)
    {
        set_conflict_counter();     //resets the conflict counters
        if (gen < pP->gen_max-1)
        {
            cout << "sr::gen" << "\t" << sr << "::" << gen << endl;
            cout << endl;
            build_team(gen);            //runs the entire build and simulation for each sim_team
            //cout << "---------------------------------------------" << endl;
            sort_agent_policies();
            if (pP->fair_trial == 0)
            {
                get_statistics();
            }
            if (gen == 0)
            {
                cout << "First Generation" << endl;
                for (int team=0; team<pP->num_teams; team++)
                {
                    cout << "team" << "\t" << team << endl;
                    for (int indv=0; indv<pP->team_sizes.at(team); indv++)
                    {
                        cout << "agent" << "\t" << indv << endl;
                        for (int p=0; p<pP->num_policies; p++)
                        {
                            cout << "policy" << "\t" << p << endl;
                            cout << "\t" << "fitness" << "\t" <<  corp.at(team).agents.at(indv).policies.at(p).policy_fitness << endl;
                        }
                        cout << endl;
                    }
                }
            }
            
            natural_selection();
            sort_agent_policies();
            
            //cout << "check nautral seclection" << endl;
            /*
             if (gen>0)
             {
             if (gen<pP->gen_max-1)
             {
             for (int team=0; team<pP->num_teams; team++)
             {
             for (int indv=0; indv<pP->team_sizes.at(team); indv++)
             {
             cout << "agent" << "\t" << indv << endl;
             for (int p=0; p<pP->num_policies; p++)
             {
             cout << "policy" << "\t" << p << endl;
             cout << "\t" << "fitness" << "\t" <<  corp.at(team).agents.at(indv).policies.at(p).policy_fitness << endl;
             assert(corp.at(team).agents.at(indv).policies.at(p).selection_counter == pP->num_policies/2);
             }
             cout << endl;
             }
             }
             }
             }
             */
            
        }
        if (gen == pP->gen_max-1)
        {
            cout << "-----------------------------------------------------------------------------------" << endl;
            cout << "-----------------------------------------------------------------------------------" << endl;
            cout << "Final Generation" << endl;
            build_team(gen);       //runs the entire build and simulation for each sim_team
            cout << "---------------------------------------------" << endl;
            sort_agent_policies();
            if (pP->fair_trial == 0)
            {
                get_statistics();
            }
            for (int team=0; team<pP->num_teams; team++)
            {
                cout << "team" << "\t" << team << endl;
                for (int indv=0; indv<pP->team_sizes.at(team); indv++)
                {
                    cout << "agent" << "\t" << indv << endl;
                    for (int p=0; p<pP->num_policies; p++)
                    {
                        cout << "policy" << "\t" << p << endl;
                        cout << "\t" << "fitness" << "\t" <<  corp.at(team).agents.at(indv).policies.at(p).policy_fitness << endl;
                    }
                    cout << endl;
                }
            }
        }
        //store conflict data here
        store_ave_conflict_data();
    }
    cout << endl;
    t2 = clock();
    float diff ((float)t2-(float)t1);
    //cout<<diff<<endl;
    //system ("pause");
    float seconds = diff / CLOCKS_PER_SEC;
    cout << "run time" << "\t" << seconds << endl;
    write_statistics_to_file(sr);
    write_conflict_data_to_file(sr);
    write_policies_to_file();
    if (sr == stat_run-1)
    {
     write_parameters_to_file(seconds);   
    }
}









/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Test Functions


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//One Policy Same Team

/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Same Team On A Collision Course
void CCEA::two_agents_one_policy_same_team_collide()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Same Team Near Miss
void CCEA::two_agents_one_policy_same_team_near_miss()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Same Team Exact Miss
void CCEA::two_agents_one_policy_same_team_exact_miss()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Same Team Close Parallel
void CCEA::two_agents_one_policy_same_team_close_parallel()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Same Team Exact Parallel
void CCEA::two_agents_one_policy_same_team_exact_parallel()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Same Team Far Parallel
void CCEA::two_agents_one_policy_same_team_far_parallel()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 6;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Two Policies Same Team

/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Two Policy Same Team On A Collision Course
void CCEA::two_agents_two_policy_same_team_collide()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 5;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 0;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Two Policy Same Team Near Miss
void CCEA::two_agents_two_policy_same_team_near_miss()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
    
    
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 1;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Two Policy Same Team Exact Miss
void CCEA::two_agents_two_policy_same_team_exact_miss()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
    
    
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 1;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Two Policy Same Team Close Parallel
void CCEA::two_agents_two_policy_same_team_close_parallel()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
    
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 1;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Two Policy Same Team Exact Parallel
void CCEA::two_agents_two_policy_same_team_exact_parallel()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
    
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 1;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Two Policy Same Team Far Parallel
void CCEA::two_agents_two_policy_same_team_far_parallel()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 6;
    
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(0).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 1;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(1).waypoint_telem.at(2) = 7;
    
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(1).check_points.at(2).waypoint_telem.at(2) = 6;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//One Policies Differnt Team

/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Differnt Team On A Collision Course
void CCEA::two_agents_one_policy_different_team_collide()
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Differnt Team Near Miss
void CCEA::two_agents_one_policy_different_team_near_miss()
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Differnt Team Exact Miss
void CCEA::two_agents_one_policy_different_team_exact_miss()
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Differnt Team Close Parallel
void CCEA::two_agents_one_policy_different_team_close_parallel()
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Differnt Team Exact Parallel
void CCEA::two_agents_one_policy_different_team_exact_parallel()
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Differnt Team Far Parallel
void CCEA::two_agents_one_policy_different_team_far_parallel()
{
    //set num_teams=2, team_0=2, team_1=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 6;
}


/////////////////////////////////////////////////////////////////
//One Simulation Four Agents One Policy Differnt Team On A Collision Course For Each Team
void CCEA::four_agents_one_policy_different_team_collide_same_team()
{
    //set num_teams=2, team_0=2, team_1=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 10;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 10;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
}


/////////////////////////////////////////////////////////////////
//One Simulation Four Agents One Policy Differnt Team Near Miss For Each Team
void CCEA::four_agents_one_policy_different_team_near_miss_same_team()
{
    //set num_teams=2, team_0=2, team_1=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 10;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 10;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 20;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 20;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Four Agents One Policy Differnt Team Exact Miss For Each Team
void CCEA::four_agents_one_policy_different_team_exact_miss_same_team()
{
    //set num_teams=2, team_0=2, team_1=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 10;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 10;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 20;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 20;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 40;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Four Agents One Policy Differnt Team Close Parallel For Each Team
void CCEA::four_agents_one_policy_different_team_close_parallel_same_team()
{
    //set num_teams=2, team_0=2, team_1=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 4;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Four Agents One Policy Differnt Team Exact Parallel For Each Team
void CCEA::four_agents_one_policy_different_team_exact_parallel_same_team()
{
    //set num_teams=2, team_0=2, team_1=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 5;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Four Agents One Policy Differnt Team Far Parallel For Each Team
void CCEA::four_agents_one_policy_different_team_far_parallel_same_team()
{
    //set num_teams=2, team_0=2, team_1=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 6;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 6;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 6;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 6;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 6;
}


/////////////////////////////////////////////////////////////////
//One Simulation Four Agents One Policy Differnt Team Far Parallel For Each Team
void CCEA::four_agents_one_policy_different_team_all_collide()
{
    //set num_teams=2, team_0=2, team_1=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1, gen_max=1
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(1) = 50;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(0).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(0) = 25;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(1) = 25;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(1).waypoint_telem.at(2) = 0;
    
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 50;
    corp.at(0).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 50;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(1).agents.at(0).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(0) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(1) = 0;
    corp.at(1).agents.at(1).policies.at(0).check_points.at(2).waypoint_telem.at(2) = 0;
}



void CCEA::Simulator_test_functions()
{
    //uncomment a fucntion to run its test
    //read test function parameters before running test
    
    //One Policy Same Team
    //two_agents_one_policy_same_team_collide();
    //two_agents_one_policy_same_team_near_miss();
    //two_agents_one_policy_same_team_exact_miss();
    //two_agents_one_policy_same_team_close_parallel();
    //two_agents_one_policy_same_team_exact_parallel();
    //two_agents_one_policy_same_team_far_parallel();
    
    //Two Policies Same Team
    //two_agents_two_policy_same_team_collide();
    //two_agents_two_policy_same_team_near_miss();
    //two_agents_two_policy_same_team_exact_miss();
    //two_agents_two_policy_same_team_close_parallel();
    //two_agents_two_policy_same_team_exact_parallel();
    //two_agents_two_policy_same_team_far_parallel();
    
    //One Policy Different Teams
    //two_agents_one_policy_different_team_collide();
    //two_agents_one_policy_different_team_near_miss();
    //two_agents_one_policy_different_team_exact_miss();
    //two_agents_one_policy_different_team_close_parallel();
    //two_agents_one_policy_different_team_exact_parallel();
    //two_agents_one_policy_different_team_far_parallel();
    //four_agents_one_policy_different_team_collide_same_team();
    //four_agents_one_policy_different_team_near_miss_same_team();
    //four_agents_one_policy_different_team_exact_miss_same_team();
    //four_agents_one_policy_different_team_close_parallel_same_team();
    //four_agents_one_policy_different_team_exact_parallel_same_team();
    //four_agents_one_policy_different_team_far_parallel_same_team();
    //four_agents_one_policy_different_team_all_collide();
}


#endif /* CCEA_hpp */
