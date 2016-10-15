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

#include "Sim_team.hpp"


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
    //CCEA Setup
    vector<Team> corp;
    Parameters* pP;
    vector<Policy> sim_team;
    
    
    vector<Sim_team> s_team;
    
    void create_population();
    void create_starting_telem();
    void create_checkpoints();
    void create_target_telem();
    void build_world();
    
    
    
    void build_teams_for_simulation();
    void selection_id_reset();
    void create_Sim_teams();
    void simulate_team(vector<Sim_team>* ps_team);
    
    void run_CCEA();
    
    //Fitness Functions
    void get_agent_fitness(int pop_size, int num_teams, vector<int> team_sizes, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints);
    
    
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
    
    //void run_inner_CCEA(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, double max_flight_velocity, double delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, int target_waypoint, double dist_to_target_waypoint, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double agent_fitness);
    
private:
    
    
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// Current Issues
//Clean up build_team
//////// Resloved Issues
////Contructing a temporary vector containing a team for simulation
//Setting up pointers for the simulator
//Getting the fitness for each policy of a team for simulation
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
//Build World for CCEA
//creates the instaces for the enitre population with memory alocation and creates the waypoint paths for each policy of each agent of each team
void CCEA::build_world()
{
    create_population();
    create_starting_telem();
    create_checkpoints();
    create_target_telem();
    
    for (int ii=0; ii<pP->num_teams; ii++)
    {
        cout << "team" << "\t" << ii << endl;
        for (int jj=0; jj<pP->team_sizes.at(ii); jj++)
        {
            cout << "agent" << "\t" << jj << endl;
            for (int po=0;  po<pP->num_policies; po++)
            {
                cout << "policy" << "\t" << po << endl;
                for (int ww=0; ww<pP->num_waypoints+2; ww++)
                {
                    cout << "waypoint" << "\t" << ww << "\t" << "telem" << endl;
                    for (int te=0; te<3; te++)
                    {
                        cout << corp.at(ii).agents.at(jj).policies.at(po).check_points.at(ww).waypoint_telem.at(te) << "\t";
                    }
                    cout << endl;
                }
            }
            cout << endl;
        }
    }
}


/////////////////////////////////////////////////////////////////
//Resets the selection identifier for each indvidual for each team
void CCEA::selection_id_reset()
{
    for (int team=0; team<pP->num_teams; team++)
    {
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            for (int p=0; p<pP->num_policies; p++)
            {
                corp.at(team).agents.at(indv).policies.at(p).selected = -1;      //resets the selection identifier
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create the instances for the Sim_team vector
void CCEA::create_Sim_teams()
{
    for (int team=0; team<pP->num_teams; team++)
    {
        Sim_team ST;
        s_team.push_back(ST);
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            Policy P;
            s_team.at(team).s_pol.push_back(P);
        }
    }
}


/////////////////////////////////////////////////////////////////
//Runs the simulation for each indivdual policy selected
void CCEA::simulate_team(vector<Sim_team>* ps_team)
{
    Simulator S;
    Parameters P;
    S.pP = &P;
    S.run_simulation(ps_team);
}


/////////////////////////////////////////////////////////////////
//Build Teams for Simulation
//randomly selects a policy of each agent in the corp
void CCEA::build_teams_for_simulation()
{
    selection_id_reset();
    create_Sim_teams();
    //runs the a simulation for num_policies
    for (int p=0; p<pP->num_policies; p++)
    {
        //will build Sim_team for num_teams
        for (int team=0; team<pP->num_teams; team++)
        {
            //will build individual for team_sizes.at(team)
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
                corp.at(team).agents.at(indv).policies.at(rand_select).simulation_id = p;    //sets team selection identifier
                s_team.at(team).s_pol.at(indv) = corp.at(team).agents.at(indv).policies.at(rand_select);
                
                corp.at(team).agents.at(indv).policies.at(rand_select).selected = 1;    //changes selection identifier to selected
            }
        }
        
        vector<Sim_team>* ps_team = &s_team;
        simulate_team(ps_team);
        
        for (int team=0; team<pP->num_teams; team++)
        {
            s_team.at(team).team_fitness = 0;
            for (int indv=0; indv<pP->team_sizes.at(team); indv++)
            {
                s_team.at(team).team_fitness += s_team.at(team).s_pol.at(indv).policy_fitness;
            }
        }
        
        for (int team=0; team<pP->num_teams; team++)
        {
            for (int indv=0; indv<pP->team_sizes.at(team); indv++)
            {
                for (int pp=0; pp<pP->num_policies; pp++)
                {
                    if (corp.at(team).agents.at(indv).policies.at(pp).simulation_id == p)
                    {
                        corp.at(team).agents.at(indv).policies.at(pp).policy_fitness = s_team.at(team).team_fitness;
                    }
                }
            }
        }
        
        //run simulation here
        //run resets here
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    for (int team=0; team<pP->num_teams; team++)
    {
        selection_id_reset();
        //builds teams for simulation of randomly selected policies from each agent
        for (int po=0; po<pP->num_policies; po++)
        {
            cout << "-----------------------------------------------------------------------------------" << endl;
            cout << "-----------------------------------------------------------------------------------" << endl;
            for (int indv=0; indv<pP->team_sizes.at(team); indv++)
            {
                int rand_select = 0;
                rand_select = rand () % pP->num_policies;       //radomly picks a policy
                for (int p=0; p<pP->num_policies; p++)
                {
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
                    corp.at(team).agents.at(indv).policies.at(rand_select).team_selected = po;    //sets team selection identifier
                    
                    sim_team.push_back(corp.at(team).agents.at(indv).policies.at(rand_select));
                    
                    corp.at(team).agents.at(indv).policies.at(rand_select).selected = 1;    //changes selection identifier to selected
                    break;
                    
                    //select that policy for team
                    //build team vector for simulation
                }
            }
            
            vector<Policy>* psim_team = &sim_team;
            simulate_team(psim_team);
            
            double sum = 0;
            for (int indv=0; indv<pP->team_sizes.at(team); indv++)
            {
                sum += sim_team.at(indv).policy_fitness;
            }
            
            for (int team=0; team<pP->num_teams; team++)
            {
                for (int indv=0; indv<pP->team_sizes.at(team); indv++)
                {
                    for (int p=0; p<pP->num_policies; p++)
                    {
                      if (corp.at(team).agents.at(indv).policies.at(p).team_selected == po)
                      {
                          corp.at(team).agents.at(indv).policies.at(p).policy_fitness = sum;
                      }
                    }
                }
            }
            sim_team.erase(sim_team.begin(), sim_team.end());
            
        }
    }
    
    for (int team=0; team<pP->num_teams; team++)
    {
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            //cout << "agent" << "\t" << indv << endl;
            for (int p=0; p<pP->num_policies; p++)
            {
                //cout << "policy" << "\t" << p << "\t" << "selected team" << "\t" << corp.at(team).agents.at(indv).policies.at(p).team_selected << endl;
                //cout << "\t" << "fitness" << "\t" <<  corp.at(team).agents.at(indv).policies.at(p).policy_fitness << endl;
            }
            //cout << endl;
        }
    }
}




/////////////////////////////////////////////////////////////////
//Runs the entire CCEA process
//
void CCEA::run_CCEA()
{
    build_world();
    Simulator_test_functions();     //go to function to run tests
    
    for (int gen=0; gen<pP->gen_max; gen++)
    {
        build_teams_for_simulation();
    }
}


/////////////////////////////////////////////////////////////////
//Fitness functions
//gets the fitness for each agent in each team
/*
void CCEA::get_agent_fitness(int pop_size, int num_teams, vector<int> team_sizes, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints)
{
    for (int ss=0; ss < pop_size; ss++)
    {
        for (int ii=0; ii < num_teams; ii++)
        {
            for (int jj=0; jj < team_sizes.at(ii); jj++)
            {
                cout << endl;
                cout << endl;
                if (sim.at(ss).system.at(ii).agents.at(jj).current_telem.at(0) == sim.at(ss).system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telem.at(0))
                {
                    if (sim.at(ss).system.at(ii).agents.at(jj).current_telem.at(1) == sim.at(ss).system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telem.at(1))
                    {
                        if (sim.at(ss).system.at(ii).agents.at(jj).current_telem.at(2) == sim.at(ss).system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telem.at(2))
                        {
                            cout << "simulation" << "\t" << ss << "\t" << "team" << "\t" << ii << "\t" << "agent" << "\t" << jj << "\t" << "has reached its final destination" << endl;
                            cout << "fitness" << "\t" << sim.at(ss).system.at(ii).agents.at(jj).agent_fitness;
                            continue;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                    cout << "simulation" << "\t" << ss << "\t" << "team" << "\t" << ii << "\t" << "agent" << "\t" << jj << "\t" << "has not reached its final destination" << endl;
                    cout << "fitness" << "\t" << sim.at(ss).system.at(ii).agents.at(jj).agent_fitness;
                }
            }
        }
    }
    cout << endl;
}
*/




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Test Functions


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//One Policy Same Team

/////////////////////////////////////////////////////////////////
//One Simulation Two Agents One Policy Same Team On A Collision Course
void CCEA::two_agents_one_policy_same_team_collide()
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=1;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2;
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
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40, num_policies=2;
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


void CCEA::Simulator_test_functions()
{
    //uncomment a fucntion to run its test
    //read test function parameters before running test
    
    //One Policy Same Team
    two_agents_one_policy_same_team_collide();
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
}


#endif /* CCEA_hpp */
