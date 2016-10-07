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
    //CCEA Setup
    vector<Team> corp;
    Parameters* pP;
    
    void create_population();
    void create_sarting_telem();
    void create_checkpoints();
    void create_target_telem();
    void build_world();
    
    
    
    void build_team();
    void simulate_team(vector<Policy>* sim_team);
    
    void run_CCEA();
    
    //Fitness Functions
    void get_agent_fitness(int pop_size, int num_teams, vector<int> team_sizes, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints);
    
    
    //void run_inner_CCEA(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, double max_flight_velocity, double delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, int target_waypoint, double dist_to_target_waypoint, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double agent_fitness);
    
private:
    
    
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// Current Issues
//Contructing a temporary vector containing a team for simulation
//Setting up pointers for the simulator
//Getting the fitness for each policy of a team for simulation
//////// Resloved Issues
//
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
                for (int ww=0; ww < pP->num_waypoints; ww++)
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
void CCEA::create_sarting_telem()
{
    for (int ii=0; ii < pP->num_teams; ii++)
    {
        for (int jj=0; jj<pP->team_sizes.at(ii); ii++)
        {
            double x_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_x_dim);
            double y_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_y_dim);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 2;
            //int y_waypoint = 4;
            double z_waypoint = 0;
            
            //comment out for test fucntions
            //{
            corp.at(ii).agents.at(jj).policies.at(0).check_points.at(0).waypoint_telem.at(0)=x_waypoint;
            corp.at(ii).agents.at(jj).policies.at(0).check_points.at(0).waypoint_telem.at(1)=y_waypoint;
            corp.at(ii).agents.at(jj).policies.at(0).check_points.at(0).waypoint_telem.at(2)=z_waypoint;
        }
    }
    
    //makes each starting telemetry for each policy for each agent the same
    for (int ii=0; ii < pP->num_teams; ii++)
    {
        for (int jj=0; jj < pP->team_sizes.at(ii); jj++)
        {
            for (int pp=1;  pp < pP->num_policies; pp++)
            {
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(0) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(0);
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(1) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(1);
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(2) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(2);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Intermediate Waypoints
//sets the telemetry for each intermediate waypoint for each policy for each agent in each team
void CCEA::create_checkpoints()
{
    for (int ii=0; ii < pP->num_teams; ii++)
    {
        for (int jj=0; jj < pP->team_sizes.at(ii); jj++)
        {
            for (int pp=1;  pp < pP->num_policies; pp++)
            {
                for (int ww=1; ww <pP->num_waypoints+1; ww++)
                {
                    double x_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_x_dim);
                    double y_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_y_dim);
                    double z_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_z_dim);
                    //int x_waypoint = 5;
                    //int y_waypoint = 6;
                    //int z_waypoint = 8;
                    
                    //comment out for test fucntions
                    //{
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(ww).waypoint_telem.at(0) = x_waypoint;
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(ww).waypoint_telem.at(1) = y_waypoint;
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(ww).waypoint_telem.at(2) = z_waypoint;
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
    for (int ii=0; ii < pP->num_teams; ii++)
    {
        for (int jj=0; jj<pP->team_sizes.at(ii); ii++)
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
    for (int ii=0; ii < pP->num_teams; ii++)
    {
        for (int jj=0; jj < pP->team_sizes.at(ii); jj++)
        {
            for (int pp=1;  pp < pP->num_policies; pp++)
            {
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(pP->num_waypoints+1).waypoint_telem.at(0) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(pP->num_waypoints+1).waypoint_telem.at(0);
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(pP->num_waypoints+1).waypoint_telem.at(1) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(pP->num_waypoints+1).waypoint_telem.at(1);
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(pP->num_waypoints+1).waypoint_telem.at(2) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(pP->num_waypoints+1).waypoint_telem.at(2);
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
    create_sarting_telem();
    create_checkpoints();
    create_target_telem();
}



void CCEA::simulate_team(vector<Policy>* sim_team)
{
   Simulator S;
    S.run_simulation(sim_team);
}






/////////////////////////////////////////////////////////////////
//Build Teams for Simulation
//randomly selects a policy of each agent in the corp
void CCEA::build_team()
{
    for (int team=0; team<pP->num_teams; team++)
    {
        //resets each agents policy selection identifier
        for (int indv=0; indv<pP->team_sizes.at(team); indv++)
        {
            for (int p=0; p<pP->num_policies; p++)
            {
                corp.at(team).agents.at(indv).policies.at(p).selected = -1;      //resets the selection identifier
            }
        }
        
        
        //builds teams for simulation of randomly selected policies from each agent
        for (int po=0; po<pP->num_policies; po++)
        {
            vector<Policy> sim_team;
            vector<Policy>* psim_team = &sim_team;
            for (int indv=0; indv<pP->team_sizes.at(team); indv++)
            {
                int rand_select = 0;
                rand_select = rand () % pP->num_policies;       //radomly picks a policy
                for (int p=0; p<pP->num_policies; p++)
                {
                    cout << corp.at(team).agents.at(indv).policies.at(rand_select).selected << endl;
                    while (corp.at(team).agents.at(indv).policies.at(rand_select).selected != -1)
                    {
                        rand_select = rand () % pP->num_policies;       //randomly selects another policy if previous selected policy has already been selected
                        if (corp.at(team).agents.at(indv).policies.at(rand_select).selected == 1)
                        {
                            cout << corp.at(team).agents.at(indv).policies.at(rand_select).selected << endl;
                        }
                    }
                    cout << po << endl;
                    corp.at(team).agents.at(indv).policies.at(rand_select).team_selected = po;    //sets team selection identifier
                    
                    sim_team.push_back(corp.at(team).agents.at(indv).policies.at(rand_select));
                    
                    corp.at(team).agents.at(indv).policies.at(rand_select).selected = 1;    //changes selection identifier to selected
                    break;
                    
                    //select that policy for team
                    //build team vector for simulation
                }
            }
            simulate_team(psim_team);
        }
    }
}




/////////////////////////////////////////////////////////////////
//Runs the entire CCEA process
//
void CCEA::run_CCEA()
{
    create_population();
    for (int gen=0; gen<pP->gen_max; gen++)
    {
        build_team();
    }
    
    
    
    
    
    
    
    
    
    
    //uncomment for loop for test fucntions
    /*
    for (int ss=0; ss < pop_size; ss++)
    {
        /////////////////////////////
        //test functions
    
        //note that when running test with two different teams that agents will be on differnt simulations as to meet the CCEA criteria
    
        //read fucntion parameters before running
        //sim.at(ss).two_agents_same_team_collide(waypoint_telem);
        //sim.at(ss).two_agents_same_team_near_miss(waypoint_telem);
        //sim.at(ss).two_agents_same_team_exact_miss(waypoint_telem);
        //sim.at(ss).two_agents_same_team_close_parallel(waypoint_telem);
        //sim.at(ss).two_agents_same_team_exact_parallel(waypoint_telem);
        //sim.at(ss).two_agents_same_team_far_parallel(waypoint_telem);
        //sim.at(ss).two_agents_diff_team_collide(waypoint_telem);
        //sim.at(ss).two_agents_diff_team_near_miss(waypoint_telem);
        //sim.at(ss).two_agents_diff_team_exact_miss(waypoint_telem);
        //sim.at(ss).two_agents_diff_team_close_parallel(waypoint_telem);
        //sim.at(ss).two_agents_diff_team_exact_parallel(waypoint_telem);
        //sim.at(ss).two_agents_diff_team_far_parallel(waypoint_telem);
        //
        //sim.at(ss).two_sims_two_agents_same_team_collide(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_same_team_near_miss(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_same_team_exact_miss(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_same_team_close_parallel(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_same_team_exact_parallel(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_same_team_far_parallel(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_diff_team_collide(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_diff_team_near_miss(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_diff_team_exact_miss(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_diff_team_close_parallel(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_diff_team_exact_parallel(waypoint_telem);
        //sim.at(ss).two_sims_two_agents_diff_team_far_parallel(waypoint_telem);
        /////////////////////////////
    }
    */
    
    //create_sarting_telem(pop_size, num_teams, team_sizes, waypoint_telem, max_x_dim, max_y_dim, max_z_dim);
    //create_checkpoints(pop_size, num_teams, team_sizes, waypoint_telem, num_waypoints, max_x_dim, max_y_dim, max_z_dim);
    //create_target_telem(pop_size, num_teams, team_sizes, waypoint_telem, num_waypoints, max_x_dim, max_y_dim, max_z_dim);
    /*
    for (int ii=0; ii < pop_size; ii++)
    {
       sim.at(ii).create_starting_flight_velocity(num_teams, team_sizes, max_flight_velocity);
    }
     */
    
    /*
    cout << "number of simulations" << "\t" << sim.size() << endl;
    for (int ii=0; ii < pop_size; ii++)
    {
        cout << "simulation" << "\t" << ii << endl;
        cout << "total number of teams" << "\t" << sim.at(ii).system.size() << endl;
        for(int i=0; i < num_teams; i++)
        {
            cout << "-------------------------------------------------------------------------" << endl;
            cout << endl;
            cout << "team" << "\t" << i << "\t" << "has" << "\t" << sim.at(ii).system.at(i).agents.size() << "\t" << "agents" << endl;
            cout << endl;
            for (int j=0; j < sim.at(ii).system.at(i).agents.size(); j++)
            {
                cout << "team" << "\t" << i << "\t" << "agent" << "\t" << j << endl;
                for (int k=0; k < num_waypoints+2; k++)
                {
                    if (k == 0)
                    {
                        cout << "Starting Telemetry" << "\t";
                    }
                    if (k > 0)
                    {
                        if (k < num_waypoints+1)
                        {
                            cout << "Waypoint" << "\t" << k << "\t" << "Telemetry" << "\t";
                        }
                    }
                    if (k == num_waypoints+1)
                    {
                        cout << "Final Destination Telemetry" << "\t";
                    }
                    for (int h=0; h < 3; h++)
                    {
                        cout << sim.at(ii).system.at(i).agents.at(j).check_points.at(k).waypoint_telem.at(h) << "\t";
                    }
                    cout << endl;
                }
                cout << "Starting Flight Velocity" << "\t" << sim.at(ii).system.at(i).agents.at(j).current_travel_speed << endl;
                cout << endl;
            }
        }
        cout << "-------------------------------------------------------------------------" << endl;
        cout << endl;
        cout << "-------------------------------------------------------------------------" << endl;
        cout << "-------------------------------------------------------------------------" << endl;
        cout << endl;
    }
    cout << "-------------------------------------------------------------------------" << endl;
    cout << endl;
    cout << "-------------------------------------------------------------------------" << endl;
    cout << "-------------------------------------------------------------------------" << endl;
    cout << "-------------------------------------------------------------------------" << endl;
    cout << endl;
     */
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


/*
/////////////////////////////////////////////////////////////////
//Run Inner CCEA
//executes the inner CCEA
void CCEA::run_inner_CCEA(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, double max_flight_velocity, double delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, int target_waypoint, double dist_to_target_waypoint, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double agent_fitness)
{
    for (int ss=0; ss < pop_size; ss++)
    {
        */
        /*
        cout << "simulation" << "\t" << ss << endl;
        cout << "-------------------------------------------------------------------------" << endl;
        cout << "-------------------------------------------------------------------------" << endl;
        cout << "-------------------------------------------------------------------------" << endl;
         */
/*
        sim.at(ss).run_simulation(num_teams, team_sizes, waypoint_telem, max_flight_velocity, delta_t, max_travel_dist, current_telem, time_max, num_waypoints, max_x_dim, max_y_dim, max_z_dim, target_waypoint, dist_to_target_waypoint, current_travel_speed, ca_max_travel_dist, projected_telem, inc_projected_telem, ca_inc, ca_radius, ca_flight_speed, agent_fitness);
    }
    get_agent_fitness(pop_size, num_teams, team_sizes, current_telem, waypoint_telem, num_waypoints);
}
*/


#endif /* CCEA_hpp */