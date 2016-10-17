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
    void build_world();
    
    //Functions for Simulation
    void build_team();
    void reset_selection_identifiers(int team);
    void build_sim_team(int team, int po);
    void get_policy_fitness(int team, int po);
    void simulate_team(vector<Policy>* sim_team);
    
    //Binary Selection Funcitons
    struct less_than_policy_fitness;
    void sort_agent_policies();
    
    //CCEA main
    void run_CCEA();
    
    
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
    
private:
    
    
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// Current Issues
//
//////// Resloved Issues
////Contructing a temporary vector containing a team for simulation
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
                corp.at(ii).agents.at(jj).policies.at(pp).policy_id = pp;
                
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
    
    /*
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
    */
}


/////////////////////////////////////////////////////////////////
//simulates the radomly generated sim_team
void CCEA::simulate_team(vector<Policy>* psim_team)
{
    Simulator S;
    Parameters P;
    S.pP = &P;
    S.run_simulation(psim_team);
}


/////////////////////////////////////////////////////////////////
//resets the selection identifier
void CCEA::reset_selection_identifiers(int team)
{
    //resets each agents policy selection identifier
    for (int indv=0; indv<pP->team_sizes.at(team); indv++)
    {
        for (int p=0; p<pP->num_policies; p++)
        {
            corp.at(team).agents.at(indv).policies.at(p).selected = -1;
        }
    }
}


/////////////////////////////////////////////////////////////////
//build the randomly generated sim_team
void CCEA::build_sim_team(int team, int po)
{
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
            corp.at(team).agents.at(indv).policies.at(rand_select).simulation_id = po;    //sets team selection identifier
            
            sim_team.at(indv) = (corp.at(team).agents.at(indv).policies.at(rand_select));
            
            corp.at(team).agents.at(indv).policies.at(rand_select).selected = 1;    //changes selection identifier to selected
            break;
            
            //select that policy for team
            //build team vector for simulation
        }
    }
}


/////////////////////////////////////////////////////////////////
//build the randomly generated sim_team
void CCEA::get_policy_fitness(int team, int po)
{
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
                if (corp.at(team).agents.at(indv).policies.at(p).simulation_id == po)
                {
                    corp.at(team).agents.at(indv).policies.at(p).policy_fitness = sum;
                }
            }
        }
    }
}



/////////////////////////////////////////////////////////////////
//runs the entire build and simulation for each sim_team
void CCEA::build_team()
{
    for (int team=0; team<pP->num_teams; team++)
    {
        reset_selection_identifiers(team);
        
        //builds teams for simulation of randomly selected policies from each agent
        for (int po=0; po<pP->num_policies; po++)
        {
            sim_team.resize(pP->team_sizes.at(team));
            //cout << "-----------------------------------------------------------------------------------" << endl;
            //cout << "-----------------------------------------------------------------------------------" << endl;
            
            build_sim_team(team, po);
            
            vector<Policy>* psim_team = &sim_team;
            simulate_team(psim_team);
            
            get_policy_fitness(team, po);
            
            sim_team.erase(sim_team.begin(), sim_team.end());
        }
    }
    
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
}


//-----------------------------------------------------------
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
        cout << "-----------------------------------------------------------------------------------" << endl;
        cout << "generation" << "\t" << gen << endl;
        cout << endl;
        build_team();       //runs the entire build and simulation for each sim_team
        cout << "-----------------------------------------------------------------------------------" << endl;
        sort_agent_policies();
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
}


#endif /* CCEA_hpp */
