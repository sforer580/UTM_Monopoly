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
    vector<Simulator> sim;
    void create_population(int pop_size, int num_teams, vector<int> team_sizes, int num_waypoints, vector<double> waypoint_telem);
    void create_sarting_telem(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int max_x_dim, int max_y_dim, int max_z_dim);
    void create_checkpoints(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    void create_target_telem(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    void build_simulator(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, double max_flight_velocity);
    
    
    void run_inner_CCEA(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, double max_flight_velocity, double delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, int target_waypoint, double dist_to_target_waypoint, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed);
    
private:
    
    
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// Current Issues
//stuck in simulation time loop
//////// Resloved Issues
//terminating with uncaught exception in build simulator
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
//Creates Population for each simulation
//creates an instacne for each agent in each team in each simulation
void CCEA::create_population(int pop_size, int num_teams, vector<int> team_sizes, int num_waypoints, vector<double> waypoint_telem)
{
    for (int ii=0; ii < pop_size; ii++)
    {
        Simulator S;
        sim.push_back(S);
        for (int jj=0; jj < num_teams; jj++)
        {
            Team T;
            sim.at(ii).system.push_back(T);
            for (int kk=0; kk < team_sizes.at(jj); kk++)
            {
                Individual I;
                sim.at(ii).system.at(jj).agents.push_back(I);
                for (int ww=0; ww < num_waypoints+2; ww++)
                {
                    Waypoint W;
                    sim.at(ii).system.at(jj).agents.at(kk).check_points.push_back(W);
                    sim.at(ii).system.at(jj).agents.at(kk).check_points.at(ww).waypoint_telem.resize(3);
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Starting Coordinates
//sets the starting telemetry for each agent in each team
void CCEA::create_sarting_telem(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int max_x_dim, int max_y_dim, int max_z_dim)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            double x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
            double y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 2;
            //int y_waypoint = 4;
            double z_waypoint = 0;
            
            //comment out for test fucntions
            //{
            sim.at(0).system.at(ii).agents.at(jj).check_points.at(0).waypoint_telem.at(0)=x_waypoint;
            sim.at(0).system.at(ii).agents.at(jj).check_points.at(0).waypoint_telem.at(1)=y_waypoint;
            sim.at(0).system.at(ii).agents.at(jj).check_points.at(0).waypoint_telem.at(2)=z_waypoint;
            //}
        }
    }
    //makes each starting telemetry for each corresponding agent in each simulation the same
    for (int ii=1; ii < pop_size; ii++)
    {
        for (int jj=0; jj < num_teams; jj++)
        {
            for (int kk=0; kk < team_sizes.at(jj); kk++)
            {
                sim.at(ii).system.at(jj).agents.at(kk).check_points.at(0).waypoint_telem.at(0)=sim.at(0).system.at(jj).agents.at(kk).check_points.at(0).waypoint_telem.at(0);
                sim.at(ii).system.at(jj).agents.at(kk).check_points.at(0).waypoint_telem.at(1)=sim.at(0).system.at(jj).agents.at(kk).check_points.at(0).waypoint_telem.at(1);
                sim.at(ii).system.at(jj).agents.at(kk).check_points.at(0).waypoint_telem.at(2)=sim.at(0).system.at(jj).agents.at(kk).check_points.at(0).waypoint_telem.at(2);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Intermediate Waypoints
//sets the telemetry for each intermediate waypoint for each agent in each team
void CCEA::create_checkpoints(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    for (int ii=0; ii < pop_size; ii++)
    {
        for (int jj=0; jj < num_teams; jj++)
        {
            for (int kk=0; kk < team_sizes.at(jj); kk++)
            {
                for (int ww=1; ww <num_waypoints+1; ww++)
                {
                    double x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
                    double y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
                    double z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
                    //int x_waypoint = 5;
                    //int y_waypoint = 6;
                    //int z_waypoint = 8;
                    
                    //comment out for test fucntions
                    //{
                    sim.at(ii).system.at(jj).agents.at(kk).check_points.at(ww).waypoint_telem.at(0)=x_waypoint;
                    sim.at(ii).system.at(jj).agents.at(kk).check_points.at(ww).waypoint_telem.at(1)=y_waypoint;
                    sim.at(ii).system.at(jj).agents.at(kk).check_points.at(ww).waypoint_telem.at(2)=z_waypoint;
                    //}
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Target Coordinates
//sets the final telemetry for each agent in each team
void CCEA::create_target_telem(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            double x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
            double y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 10;
            //int y_waypoint = 15;
            double z_waypoint = 0;
            
            //comment out for test fucntions
            //{
            sim.at(0).system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telem.at(0)=x_waypoint;
            sim.at(0).system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telem.at(1)=y_waypoint;
            sim.at(0).system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telem.at(2)=z_waypoint;
            //}
        }
    }
    //makes each ending telemetry for each corresponding agent in each simulation the same
    for (int ii=1; ii < pop_size; ii++)
    {
        for (int jj=0; jj < num_teams; jj++)
        {
            for (int kk=0; kk < team_sizes.at(jj); kk++)
            {
                sim.at(ii).system.at(jj).agents.at(kk).check_points.at(num_waypoints+1).waypoint_telem.at(0)=sim.at(0).system.at(jj).agents.at(kk).check_points.at(num_waypoints+1).waypoint_telem.at(0);
                sim.at(ii).system.at(jj).agents.at(kk).check_points.at(num_waypoints+1).waypoint_telem.at(1)=sim.at(0).system.at(jj).agents.at(kk).check_points.at(num_waypoints+1).waypoint_telem.at(1);
                sim.at(ii).system.at(jj).agents.at(kk).check_points.at(num_waypoints+1).waypoint_telem.at(2)=sim.at(0).system.at(jj).agents.at(kk).check_points.at(num_waypoints+1).waypoint_telem.at(2);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Build Simulator
//bulids the simulator and couts agent flight plans
void CCEA::build_simulator(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, double max_flight_velocity)
{
    Simulator S;
    create_population(pop_size, num_teams, team_sizes, num_waypoints, waypoint_telem);
    
    //uncomment for loop for test fucntions
    //{
    //for (int ss=0; ss < pop_size; ss++)
    //{
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
    //}
    //}
    
    create_sarting_telem(pop_size, num_teams, team_sizes, waypoint_telem, max_x_dim, max_y_dim, max_z_dim);
    create_checkpoints(pop_size, num_teams, team_sizes, waypoint_telem, num_waypoints, max_x_dim, max_y_dim, max_z_dim);
    create_target_telem(pop_size, num_teams, team_sizes, waypoint_telem, num_waypoints, max_x_dim, max_y_dim, max_z_dim);
    for (int ii=0; ii < pop_size; ii++)
    {
       sim.at(ii).create_starting_flight_velocity(num_teams, team_sizes, max_flight_velocity);
    }
    
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
}



/////////////////////////////////////////////////////////////////
//Run inner CCEA
//executes the inner CCEA
void CCEA::run_inner_CCEA(int pop_size, int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, double max_flight_velocity, double delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, int target_waypoint, double dist_to_target_waypoint, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed)
{
    for (int ss=0; ss < pop_size; ss++)
    {
        cout << "simulation" << "\t" << ss << endl;
        cout << "-------------------------------------------------------------------------" << endl;
        cout << "-------------------------------------------------------------------------" << endl;
        cout << "-------------------------------------------------------------------------" << endl;
        sim.at(ss).run_simulation(num_teams, team_sizes, waypoint_telem, max_flight_velocity, delta_t, max_travel_dist, current_telem, time_max, num_waypoints, max_x_dim, max_y_dim, max_z_dim, target_waypoint, dist_to_target_waypoint, current_travel_speed, ca_max_travel_dist, projected_telem, inc_projected_telem, ca_inc, ca_radius, ca_flight_speed);
    }
}


#endif /* CCEA_hpp */
