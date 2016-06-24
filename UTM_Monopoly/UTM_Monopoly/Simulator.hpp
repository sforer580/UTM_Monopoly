//
//  Simulator.hpp
//  UTM_Monopoly
//
//  Created by Scott S Forer on 5/31/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef Simulator_hpp
#define Simulator_hpp

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>

//#include "Parameters.hpp"
#include "Team.hpp"
#include "Individual.hpp"
#include "Waypoint.hpp"

using namespace std;


class Simulator
{
    friend class Parameters;
    friend class Team;
    friend class Individual;
    friend class Waypoint;
    
protected:
    
    
public:
    vector<Team> system;
    
    void create_teams(int);
    void create_individuals(int num_teams, vector<int> team_sizes);
    void create_waypoints(int num_teams, vector<int> team_sizes, int num_waypoints);
    void create_sarting_telm(int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int max_x_dim, int max_y_dim, int max_z_dim);
    void create_checkpoints(int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    void create_target_telm(int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    void create_starting_flight_velocity(int num_teams, vector<int> team_sizes, double max_flight_velocity);
    
    void build_simulator(int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, double max_flight_velocity);
    
    ///////
    void set_initial_telem(int num_teams, vector<int> team_sizes);
    void get_dist_to_target_waypoint(vector<double> waypoint_telem, vector<double> current_telem, int ii, int jj, int target_waypoint, double dist_to_target_waypoint);
    //
    void get_projected_telem(int ii, int jj, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> waypoint_telem, vector<double> current_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, double delta_t);
    void calc_projected_telem(vector<double> waypoint_telem, double current_travel_speed, double delta_t, vector<double> projected_telem, vector<double> current_telem, int ii, int jj, int target_waypoint);
    void get_inc_projected_telem(int ii, int jj, int ca_inc);
    void check_for_collisions(int num_teams, vector<int> team_sizes, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double max_flight_speed);
    void compare_agents_projected_telem(int ii, int jj, int kk, int iii, int jjj, int ca_radius, double current_travel_speed, double ca_flight_speed, double max_flight_velocity);
    void crash_avoidance(int num_teams, vector<int> team_sizes, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double max_flight_speed, double delta_t);
    //
    void get_new_telem(int ii, int jj, double current_travel_speed, double max_flight_velocity, double ca_flight_speed, vector<double> current_telem, vector<double> projected_telem, vector<double> waypoint_telem, double delta_t, int target_waypoint, double dist_to_target_waypoint, double max_travel_dist, int num_waypoints, double ca_max_travel_dist);
    void get_new_telem_option_1(int ii, int jj, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, int num_waypoints, double current_max_travel_dist);
    //void get_new_telem_option_2(int ii, int jj, double dist_to_target_waypoint, double max_travel_dist, vector<double> waypoint_telem, vector<double>current_telem, int target_waypoint);
    void check_if_at_waypoint(int ii, int jj, double dist_to_target_waypoint, vector<double> waypoint_telem, double max_travel_dist, vector<double> current_telem, int target_waypoint, int num_waypoints);
    void check_if_at_final_destination(int ii, int jj, int target_waypoint, int num_waypoints);
    ///////
    
    void run_simulation(int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, double max_flight_velocity, double delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, int target_waypoint, double dist_to_target_waypoint, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed);
    
private:
    
    
};


//THERE BE DRAGONS AHEAD


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Current Issues
//collisions being detected then passing --> resolved
//agents do not progress passed the collision telemetry --> resolved
//agents get hung up at waypoint
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
//Create teams
void Simulator::create_teams(int num_teams)
{
    for (int i=0; i < num_teams; i++)
    {
        Team T;
        system.push_back(T);
    }
}


/////////////////////////////////////////////////////////////////
//Create Individuals
void Simulator::create_individuals(int num_teams, vector<int> team_sizes)
{
    for (int i=0; i < num_teams; i++)
    {
        for (int j=0; j < team_sizes.at(i); j++)
        {
            Individual I;
            system.at(i).agents.push_back(I);
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Waypoint Instances
void Simulator::create_waypoints(int num_teams, vector<int> team_sizes, int num_waypoints)
{
    for (int i=0; i < num_teams; i++)
    {
        for (int j=0; j < team_sizes.at(i); j++)
        {
            for (int k=0; k < num_waypoints+2; k++)
            {
                Waypoint W;
                system.at(i).agents.at(j).check_points.push_back(W);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Starting Coordinates
void Simulator::create_sarting_telm(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int max_x_dim, int max_y_dim, int max_z_dim)
{
    //for (int ii=0; ii < num_teams; ii++)
    //{
        //for (int jj=0; jj < team_sizes.at(ii); jj++)
        //{
            //int x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
            //int y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 2;
            //int y_waypoint = 4;
            //int z_waypoint = 0;
            
            //system.at(ii).agents.at(jj).check_points.at(0).waypoint_telem.push_back(x_waypoint);
            //system.at(ii).agents.at(jj).check_points.at(0).waypoint_telem.push_back(y_waypoint);
            //system.at(ii).agents.at(jj).check_points.at(0).waypoint_telem.push_back(z_waypoint);
            
            //CA check one team two agents on a collision course
            system.at(0).agents.at(0).check_points.at(0).waypoint_telem.push_back(0);
            system.at(0).agents.at(0).check_points.at(0).waypoint_telem.push_back(0);
            system.at(0).agents.at(0).check_points.at(0).waypoint_telem.push_back(0);
            system.at(0).agents.at(1).check_points.at(0).waypoint_telem.push_back(50);
            system.at(0).agents.at(1).check_points.at(0).waypoint_telem.push_back(0);
            system.at(0).agents.at(1).check_points.at(0).waypoint_telem.push_back(0);
        //}
    //}
}


/////////////////////////////////////////////////////////////////
//Create Intermediate Waypoints
void Simulator::create_checkpoints(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    //for (int ii=0; ii < num_teams; ii++)
    //{
        //for (int jj=0; jj < team_sizes.at(ii); jj++)
        //{
            //for (int kk=1; kk < num_waypoints+1; kk++)
            //{
                //double x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
                //double y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
                //double z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
                //int x_waypoint = 5;
                //int y_waypoint = 6;
                //int z_waypoint = 8;
                
                //system.at(ii).agents.at(jj).check_points.at(kk).waypoint_telem.push_back(x_waypoint);
                //system.at(ii).agents.at(jj).check_points.at(kk).waypoint_telem.push_back(y_waypoint);
                //system.at(ii).agents.at(jj).check_points.at(kk).waypoint_telem.push_back(z_waypoint);
                
                //CA check one team two agents on a collision course
                system.at(0).agents.at(0).check_points.at(1).waypoint_telem.push_back(25);
                system.at(0).agents.at(0).check_points.at(1).waypoint_telem.push_back(25);
                system.at(0).agents.at(0).check_points.at(1).waypoint_telem.push_back(0);
                system.at(0).agents.at(1).check_points.at(1).waypoint_telem.push_back(25);
                system.at(0).agents.at(1).check_points.at(1).waypoint_telem.push_back(25);
                system.at(0).agents.at(1).check_points.at(1).waypoint_telem.push_back(0);
            //}
        //}
    //}
}


/////////////////////////////////////////////////////////////////
//Create Target Coordinates
void Simulator::create_target_telm(int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    //for (int ii=0; ii < num_teams; ii++)
    //{
        //for (int jj=0; jj < team_sizes.at(ii); jj++)
        //{
            //int x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
            //int y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 10;
            //int y_waypoint = 15;
            //int z_waypoint = 0;
            
            //system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telem.push_back(x_waypoint);
            //system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telem.push_back(y_waypoint);
            //system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telem.push_back(z_waypoint);
            
            //CA check one team two agents on a collision course
            system.at(0).agents.at(0).check_points.at(2).waypoint_telem.push_back(50);
            system.at(0).agents.at(0).check_points.at(2).waypoint_telem.push_back(50);
            system.at(0).agents.at(0).check_points.at(2).waypoint_telem.push_back(0);
            system.at(0).agents.at(1).check_points.at(2).waypoint_telem.push_back(0);
            system.at(0).agents.at(1).check_points.at(2).waypoint_telem.push_back(50);
            system.at(0).agents.at(1).check_points.at(2).waypoint_telem.push_back(0);
        //}
    //}
}


/////////////////////////////////////////////////////////////////
//Create Starting Flight Velocity
void Simulator::create_starting_flight_velocity(int num_teams, vector<int> team_sizes, double max_flight_velocity)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            system.at(ii).agents.at(jj).current_travel_speed = max_flight_velocity;
        }
    }
}


/////////////////////////////////////////////////////////////////
//Build Simulator
void Simulator::build_simulator(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, double max_flight_velocity)
{
    create_teams(num_teams);
    create_individuals(num_teams, team_sizes);
    create_waypoints(num_teams, team_sizes, num_waypoints);
    create_sarting_telm(num_teams, team_sizes, waypoint_telm, max_x_dim, max_y_dim, max_z_dim);
    create_checkpoints(num_teams, team_sizes, waypoint_telm, num_waypoints, max_x_dim, max_y_dim, max_z_dim);
    create_target_telm(num_teams, team_sizes, waypoint_telm, num_waypoints, max_x_dim, max_y_dim, max_z_dim);
    create_starting_flight_velocity(num_teams, team_sizes, max_flight_velocity);
    
    cout << "total number of teams" << "\t" << system.size() << endl;
    for(int i=0; i < num_teams; i++)
    {
        cout << "-------------------------------------------------------------------------" << endl;
        cout << endl;
        cout << "team" << "\t" << i << "\t" << "has" << "\t" << system.at(i).agents.size() << "\t" << "agents" << endl;
        cout << endl;
        for (int j=0; j < system.at(i).agents.size(); j++)
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
                    cout << system.at(i).agents.at(j).check_points.at(k).waypoint_telem.at(h) << "\t";
                }
                cout << endl;
            }
            cout << "Starting Flight Velocity" << "\t" << system.at(i).agents.at(j).current_travel_speed << endl;
            cout << endl;
        }
    }
    cout << "-------------------------------------------------------------------------" << endl;
    cout << endl;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//BEGIN TELEMETRY CALCULATIONS


/////////////////////////////////////////////////////////////////
//Set The Starting Telemetry
void Simulator::set_initial_telem(int num_teams, vector<int> team_sizes)
{
    cout << "current time" << "\t" << 0.0 << endl;
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            double current_x;
            double current_y;
            double current_z;
            current_x = system.at(ii).agents.at(jj).check_points.at(0).waypoint_telem.at(0);
            current_y = system.at(ii).agents.at(jj).check_points.at(0).waypoint_telem.at(1);
            current_z = system.at(ii).agents.at(jj).check_points.at(0).waypoint_telem.at(2);
            system.at(ii).agents.at(jj).current_telem.push_back(current_x);
            system.at(ii).agents.at(jj).current_telem.push_back(current_y);
            system.at(ii).agents.at(jj).current_telem.push_back(current_z);
            cout << "team" << "\t" << ii << "\t" << "agent" << "\t" << jj << endl;
            cout << "current telem" << endl;
            for (int ll=0; ll < 3; ll++)
            {
                cout << system.at(ii).agents.at(jj).current_telem.at(ll) << "\t";
            }
            cout << endl;
            cout << "current travel speed" << "\t" << system.at(ii).agents.at(jj).current_travel_speed << endl;
        }
    }
}


/////////////////////////////////////////////////////////////////
//Calculate Distance to Target Waypoint
void Simulator::get_dist_to_target_waypoint(vector<double> waypoint_telm, vector<double> current_telem, int ii, int jj, int target_waypoint, double dist_to_target_waypoint)
{
    system.at(ii).agents.at(jj).dist_to_target_waypoint = 0;
    vector <double> V;
    vector <double> D;
    double V_mag_1;
    double V_mag_2;
    double V_mag_3;
    int P1 = system.at(ii).agents.at(jj).target_waypoint;
    V.push_back(system.at(ii).agents.at(jj).check_points.at(P1).waypoint_telem.at(0) - system.at(ii).agents.at(jj).current_telem.at(0));
    V.push_back(system.at(ii).agents.at(jj).check_points.at(P1).waypoint_telem.at(1) - system.at(ii).agents.at(jj).current_telem.at(1));
    V.push_back(system.at(ii).agents.at(jj).check_points.at(P1).waypoint_telem.at(2) - system.at(ii).agents.at(jj).current_telem.at(2));
    V_mag_1 = V.at(0)*V.at(0);
    V_mag_2 = V.at(1)*V.at(1);
    V_mag_3 = V.at(2)*V.at(2);
    system.at(ii).agents.at(jj).dist_to_target_waypoint = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
}


/////////////////////////////////////////////////////////////////
//Calculates The Projected Telemetry
void Simulator::get_projected_telem(int ii, int jj, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> waypoint_telem, vector<double> current_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, double delta_t)
{
    //cout << "cp5" << endl;
    //check_if_at_waypoint(ii, jj, dist_to_target_waypoint, waypoint_telem, max_travel_dist, current_telem, target_waypoint, num_waypoints);
    
    if (system.at(ii).agents.at(jj).dist_to_target_waypoint > 0)
    {
        if (system.at(ii).agents.at(jj).dist_to_target_waypoint < max_travel_dist)
        {
            //cout << "cp3" << endl;
            //checks to see if the distance from the current telemetry to the target waypoint is within the max travel distance
            get_new_telem_option_1(ii, jj, dist_to_target_waypoint, max_travel_dist, target_waypoint, num_waypoints, ca_max_travel_dist);
        }
    }
    if (system.at(ii).agents.at(jj).dist_to_target_waypoint > 0)
    {
        if (system.at(ii).agents.at(jj).dist_to_target_waypoint > max_travel_dist)
            {
                //cout << "cp4" << endl;
                //calculates the new projected telemetry
                calc_projected_telem(waypoint_telem, current_travel_speed, delta_t, projected_telem, current_telem, ii, jj, target_waypoint);
            }
    }
    //cout << "projected telem" << endl;
    //for (int ll=0; ll < 3; ll++)
    //{
        //cout << system.at(ii).agents.at(jj).projected_telem.at(ll) << "\t";
    //}
    cout << endl;
}


/////////////////////////////////////////////////////////////////
//Calculates New Projected Telemetry
//sets new projected telemetry to target waypoint telmetry if the distance to the target waypoint is less than the max travel distance
void Simulator::get_new_telem_option_1(int ii, int jj, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, int num_waypoints, double current_max_travel_dist)
{
    //cout << "reached target waypoint" << endl;
    system.at(ii).agents.at(jj).projected_telem.push_back(system.at(ii).agents.at(jj).check_points.at(system.at(ii).agents.at(jj).target_waypoint).waypoint_telem.at(0));
    system.at(ii).agents.at(jj).projected_telem.push_back(system.at(ii).agents.at(jj).check_points.at(system.at(ii).agents.at(jj).target_waypoint).waypoint_telem.at(1));
    system.at(ii).agents.at(jj).projected_telem.push_back(system.at(ii).agents.at(jj).check_points.at(system.at(ii).agents.at(jj).target_waypoint).waypoint_telem.at(2));
    //cout << "op1" << endl;
    //system.at(ii).agents.at(jj).target_waypoint = system.at(ii).agents.at(jj).target_waypoint + 1;
    //checks to see if agent has reached their final destination
    //check_if_at_final_destination(ii, jj, target_waypoint, num_waypoints);
}


/////////////////////////////////////////////////////////////////
//Calculates New Projected Telemetry
void Simulator::calc_projected_telem(vector<double> waypoint_telem, double current_travel_speed, double delta_t, vector<double> projected_telem, vector<double> current_telem, int ii, int jj, int target_waypoint)
{
    vector <double> V;
    vector <double> D;
    double V_mag_1;
    double V_mag_2;
    double V_mag_3;
    double V_mag;
    double current_x;
    double current_y;
    double current_z;
    double travel_dist;
    int P1 = system.at(ii).agents.at(jj).target_waypoint;
    int P2 = system.at(ii).agents.at(jj).target_waypoint -1;
    
    static int counter;
    counter++;
    
    //Calculates the travel distance
    travel_dist = system.at(ii).agents.at(jj).current_travel_speed*delta_t;
    
    //Creates Vector Between Waypoints
    V.push_back(system.at(ii).agents.at(jj).check_points.at(P1).waypoint_telem.at(0) - system.at(ii).agents.at(jj).check_points.at(P2).waypoint_telem.at(0));
    V.push_back(system.at(ii).agents.at(jj).check_points.at(P1).waypoint_telem.at(1) - system.at(ii).agents.at(jj).check_points.at(P2).waypoint_telem.at(1));
    V.push_back(system.at(ii).agents.at(jj).check_points.at(P1).waypoint_telem.at(2) - system.at(ii).agents.at(jj).check_points.at(P2).waypoint_telem.at(2));
    V_mag_1 = V.at(0)*V.at(0);
    V_mag_2 = V.at(1)*V.at(1);
    V_mag_3 = V.at(2)*V.at(2);
    V_mag = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
    
    //Creates Unit Vector Between Waypoints
    D.push_back(V.at(0)/V_mag);
    D.push_back(V.at(1)/V_mag);
    D.push_back(V.at(2)/V_mag);
    
    //Calculates current telemetry
    current_x = system.at(ii).agents.at(jj).current_telem.at(0) + travel_dist*D.at(0);
    current_y = system.at(ii).agents.at(jj).current_telem.at(1) + travel_dist*D.at(1);
    current_z = system.at(ii).agents.at(jj).current_telem.at(2) + travel_dist*D.at(2);
    system.at(ii).agents.at(jj).projected_telem.clear();
    system.at(ii).agents.at(jj).projected_telem.push_back(current_x);
    system.at(ii).agents.at(jj).projected_telem.push_back(current_y);
    system.at(ii).agents.at(jj).projected_telem.push_back(current_z);
    //cout << "cp2" << endl;
}


/////////////////////////////////////////////////////////////////
//Calculates The Incremented Projected Telemetry
void Simulator::get_inc_projected_telem(int ii, int jj, int ca_inc)
{
    system.at(ii).agents.at(jj).inc_projected_telem.clear();        //clears previous incremented projected telemetry
    double x_inc_movement;
    double y_inc_movement;
    double z_inc_movement;
    //calculates the incremented movement for each dimension
    x_inc_movement = (system.at(ii).agents.at(jj).projected_telem.at(0) - system.at(ii).agents.at(jj).current_telem.at(0))/ca_inc;
    y_inc_movement = (system.at(ii).agents.at(jj).projected_telem.at(1) - system.at(ii).agents.at(jj).current_telem.at(1))/ca_inc;
    z_inc_movement = (system.at(ii).agents.at(jj).projected_telem.at(2) - system.at(ii).agents.at(jj).current_telem.at(2))/ca_inc;
    //pushes the incremented projected telemetry into a vector
    for (int iii=0; iii < ca_inc+2; iii++)
    {
        system.at(ii).agents.at(jj).inc_projected_telem.push_back(system.at(ii).agents.at(jj).current_telem.at(0) + iii*x_inc_movement);
        system.at(ii).agents.at(jj).inc_projected_telem.push_back(system.at(ii).agents.at(jj).current_telem.at(1) + iii*y_inc_movement);
        system.at(ii).agents.at(jj).inc_projected_telem.push_back(system.at(ii).agents.at(jj).current_telem.at(2) + iii*z_inc_movement);
    }
}


/////////////////////////////////////////////////////////////////
//Checks For Possible Collisions
void Simulator::check_for_collisions(int num_teams, vector<int> team_sizes, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double max_flight_velocity)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            //only considers agent who have not reached their final destination
            if (system.at(ii).agents.at(jj).target_waypoint < num_waypoints + 2)
            {
                for (int kk=0; kk < ca_inc+2; kk++)
                {
                    for (int iii=0; iii < num_teams; iii++)
                    {
                        for (int jjj=0; jjj < team_sizes.at(iii); jjj++)
                        {
                            //only considers agent who have not reached their final destination
                            if (system.at(iii).agents.at(jjj).target_waypoint < num_waypoints + 2)
                            {
                                //will not compare an individual agent to itself
                                if (ii==iii)
                                {
                                    if (jj!=jjj)
                                    {
                                        compare_agents_projected_telem(ii, jj, kk, iii, jjj, ca_radius, current_travel_speed, ca_flight_speed, max_flight_velocity);
                                    }
                                }
                                if (ii!=iii)
                                {
                                    if (jj==jjj)
                                    {
                                       compare_agents_projected_telem(ii, jj, kk, iii, jjj, ca_radius, current_travel_speed, ca_flight_speed, max_flight_velocity); 
                                    }
                                }
                            }
                        }
                    }
                }
            }
            //will not comare agents who have reached their final destination
            else
            {
                continue;
            }
        }
    }
}

/////////////////////////////////////////////////////////////////
//Comapres Agents Incremented Projected Telemetry
void Simulator::compare_agents_projected_telem(int ii, int jj, int kk, int iii, int jjj, int ca_radius, double current_travel_speed, double ca_flight_speed, double max_flight_velocity)
{
    double x;
    double x_mag;
    double y;
    double y_mag;
    double z;
    double z_mag;
    double r;
    x = system.at(ii).agents.at(jj).inc_projected_telem.at(kk+(kk*2))-system.at(iii).agents.at(jjj).inc_projected_telem.at(kk+(kk*2));
    y = system.at(ii).agents.at(jj).inc_projected_telem.at(kk+(kk*2)+1)-system.at(iii).agents.at(jjj).inc_projected_telem.at(kk+(kk*2)+1);
    z = system.at(ii).agents.at(jj).inc_projected_telem.at(kk+(kk*2)+2)-system.at(iii).agents.at(jjj).inc_projected_telem.at(kk+(kk*2)+2);
    x_mag = x*x;
    y_mag = y*y;
    z_mag = z*z;
    r = x_mag+y_mag+z_mag;
    //cout << r << endl;
    if (r <= (ca_radius*ca_radius))
    {
        //cout << "collision decteded" << endl;
        system.at(ii).agents.at(jj).current_travel_speed = ca_flight_speed;
        //cout << system.at(ii).agents.at(jj).current_travel_speed << endl;
    }
    else
    {
        //cout << "pass" << endl;
        system.at(ii).agents.at(jj).current_travel_speed = max_flight_velocity;
    }
}

/////////////////////////////////////////////////////////////////
//Runs The Crash Avoidance Check
void Simulator::crash_avoidance(int num_teams, vector<int> team_sizes, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double max_flight_velocity, double delta_t)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            //only considers agent who have not reached their final destination
            if (system.at(ii).agents.at(jj).target_waypoint < num_waypoints + 2)
            {
                system.at(ii).agents.at(jj).projected_telem.clear();
                //gets the distance from the current telemetry to the target waypoint
                get_dist_to_target_waypoint(waypoint_telem, current_telem, ii, jj, target_waypoint, dist_to_target_waypoint);
                //gets the projected telemetry
                get_projected_telem(ii, jj, dist_to_target_waypoint, max_travel_dist, target_waypoint, waypoint_telem, current_telem, num_waypoints, current_travel_speed, ca_max_travel_dist, projected_telem, inc_projected_telem, delta_t);
                //cout << "cp1" << endl;
                //gets the incremented projected telemetry
                get_inc_projected_telem(ii, jj, ca_inc);
            }
        }
    }
    check_for_collisions(num_teams, team_sizes, dist_to_target_waypoint, max_travel_dist, target_waypoint, current_telem, waypoint_telem, num_waypoints, current_travel_speed, ca_max_travel_dist, projected_telem, inc_projected_telem, ca_inc, ca_radius, ca_flight_speed, max_flight_velocity);
}


/////////////////////////////////////////////////////////////////
//Checks If Agent Is at Their Final Destination
void Simulator::check_if_at_final_destination(int ii, int jj, int target_waypoint, int num_waypoints)
{
    if (system.at(ii).agents.at(jj).current_telem.at(0) == system.at(ii).agents.at(jj).check_points.at(num_waypoints + 1).waypoint_telem.at(0))
    {
        if (system.at(ii).agents.at(jj).current_telem.at(1) == system.at(ii).agents.at(jj).check_points.at(num_waypoints + 1).waypoint_telem.at(1))
        {
            if (system.at(ii).agents.at(jj).current_telem.at(2) == system.at(ii).agents.at(jj).check_points.at(num_waypoints + 1).waypoint_telem.at(2))
            {
                cout << "reached final destination" << endl;
                system.at(ii).agents.at(jj).current_telem = system.at(ii).agents.at(jj).current_telem;
                //add in an if statement so that the time to final destination is not overwritten
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Gets New Target Waypoint
//checks to see if the current telemetry matches the target waypoint
void Simulator::check_if_at_waypoint(int ii, int jj, double dist_to_target_waypoint, vector<double> waypoint_telem, double max_travel_dist, vector<double> current_telem, int target_waypoint, int num_waypoints)
{
    //cout <<system.at(ii).agents.at(jj).dist_to_target_waypoint << endl;
    get_dist_to_target_waypoint(waypoint_telem, current_telem, ii, jj, target_waypoint, dist_to_target_waypoint);
    //cout <<system.at(ii).agents.at(jj).dist_to_target_waypoint << endl;
    if (system.at(ii).agents.at(jj).dist_to_target_waypoint == 0)
    {
        cout << "reached target waypoint" << endl;
        //calc_new_telem(waypoint_telem, max_travel_dist, current_telem, ii, jj, target_waypoint);
        cout << system.at(ii).agents.at(jj).target_waypoint << endl;
        system.at(ii).agents.at(jj).target_waypoint++;
        cout << system.at(ii).agents.at(jj).target_waypoint << endl;
        //checks to see if agent has reached their final destination
        check_if_at_final_destination(ii, jj, target_waypoint, num_waypoints);
    }
}


/////////////////////////////////////////////////////////////////
//Calculates New Telemetry
void Simulator::get_new_telem(int ii, int jj, double current_travel_speed, double max_flight_velocity, double ca_flight_speed, vector<double> current_telem, vector<double> projected_telem, vector<double> waypoint_telem, double delta_t, int target_waypoint, double dist_to_target_waypoint, double max_travel_dist, int num_waypoints, double ca_max_travel_dist)
{
    //system.at(ii).agents.at(jj).current_telem.clear();
    //cout << system.at(ii).agents.at(jj).dist_to_target_waypoint << endl;
    
    if (system.at(ii).agents.at(jj).dist_to_target_waypoint > 0)
    {
        if (system.at(ii).agents.at(jj).dist_to_target_waypoint < system.at(ii).agents.at(jj).current_travel_speed*delta_t)
        {
                system.at(ii).agents.at(jj).current_telem.at(0) = (system.at(ii).agents.at(jj).check_points.at(system.at(ii).agents.at(jj).target_waypoint).waypoint_telem.at(0));
                system.at(ii).agents.at(jj).current_telem.at(1) = (system.at(ii).agents.at(jj).check_points.at(system.at(ii).agents.at(jj).target_waypoint).waypoint_telem.at(1));
                system.at(ii).agents.at(jj).current_telem.at(2) = (system.at(ii).agents.at(jj).check_points.at(system.at(ii).agents.at(jj).target_waypoint).waypoint_telem.at(2));
        }
        //cout << "current telem" << endl;
        //for (int ll=0; ll < 3; ll++)
        //{
            //cout << system.at(ii).agents.at(jj).current_telem.at(ll) << "\t";
        //}
        //cout << endl;
    }
    if (system.at(ii).agents.at(jj).dist_to_target_waypoint > 0)
    {
        if (system.at(ii).agents.at(jj).dist_to_target_waypoint > system.at(ii).agents.at(jj).current_travel_speed*delta_t)
        {
            if (system.at(ii).agents.at(jj).current_travel_speed==max_flight_velocity)
            {
                system.at(ii).agents.at(jj).current_telem.at(0) = (system.at(ii).agents.at(jj).projected_telem.at(0));
                system.at(ii).agents.at(jj).current_telem.at(1) = (system.at(ii).agents.at(jj).projected_telem.at(1));
                system.at(ii).agents.at(jj).current_telem.at(2) = (system.at(ii).agents.at(jj).projected_telem.at(2));
            }
            if (system.at(ii).agents.at(jj).current_travel_speed==ca_flight_speed)
            {
                calc_projected_telem(waypoint_telem, current_travel_speed, delta_t, projected_telem, current_telem, ii, jj, target_waypoint);
                system.at(ii).agents.at(jj).current_telem.at(0) = (system.at(ii).agents.at(jj).projected_telem.at(0));
                system.at(ii).agents.at(jj).current_telem.at(1) = (system.at(ii).agents.at(jj).projected_telem.at(1));
                system.at(ii).agents.at(jj).current_telem.at(2) = (system.at(ii).agents.at(jj).projected_telem.at(2));
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//END TELEMETRY CALCULATIONS


/////////////////////////////////////////////////////////////////
//Runs Entire Simulation
//need to add the collision avoidence statements and functions
//need to add final destination statement
void Simulator::run_simulation(int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, double max_flight_velocity, double delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, int target_waypoint, double dist_to_target_waypoint, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed)
{
    build_simulator(num_teams, team_sizes, waypoint_telem, num_waypoints, max_x_dim, max_y_dim, max_z_dim, max_flight_velocity);
    cout << "-------------------------------------------------------------------------" << endl;
    cout << "-------------------------------------------------------------------------" << endl;
    cout << endl;
    set_initial_telem(num_teams, team_sizes);
    cout << "-------------------------------------------------------------------------" << endl;
    cout << endl;
    double current_time = delta_t;
    while (current_time < time_max)
    {
        //checks for crash avoidance
        crash_avoidance(num_teams, team_sizes, dist_to_target_waypoint, max_travel_dist, target_waypoint, current_telem, waypoint_telem, num_waypoints, current_travel_speed, ca_max_travel_dist, projected_telem, inc_projected_telem, ca_inc, ca_radius, ca_flight_speed, max_flight_velocity, delta_t);
        
        cout << "current time" << "\t" << current_time << endl;
        for (int ii=0; ii < num_teams; ii++)
        {
            for (int jj=0; jj < team_sizes.at(ii); jj++)
            {
                //only considers agent who have not reached their final destination
                if (system.at(ii).agents.at(jj).target_waypoint < num_waypoints + 2)
                {
                    //cout << "cp1" << endl;
                    cout << "team" << "\t" << ii << "\t" << "agent" << "\t" << jj << endl;
                    
                    cout << "target waypoint" << endl;
                    cout << system.at(ii).agents.at(jj).target_waypoint << endl;
                    
                    cout << "distance to target waypoint" << endl;
                    cout << system.at(ii).agents.at(jj).dist_to_target_waypoint << endl;
                    
                    cout << "projected telem" << endl;
                    for (int ll=0; ll < 3; ll++)
                    {
                        cout << system.at(ii).agents.at(jj).projected_telem.at(ll) << "\t";
                    }
                    cout << endl;
                    
                    //gets the new telemetry based on the flight speed from the CA
                    get_new_telem(ii, jj, current_travel_speed, max_flight_velocity, ca_flight_speed, current_telem, projected_telem, waypoint_telem, delta_t, target_waypoint, dist_to_target_waypoint, max_travel_dist, num_waypoints, ca_max_travel_dist);
                    //cout << system.at(ii).agents.at(jj).target_waypoint << endl;
                    
                    //runs a check to see if the agent has reached their target waypoint
                    check_if_at_waypoint(ii, jj, dist_to_target_waypoint, waypoint_telem, max_travel_dist, current_telem, target_waypoint, num_waypoints);
                    
                    
                    //cout << system.at(ii).agents.at(jj).target_waypoint << endl;
                    //checks to see if the distance from the current telemetry to the target waypoint is within the max travel distance
                    //get_new_telem_option_1(ii, jj, dist_to_target_waypoint, max_travel_dist, target_waypoint, num_waypoints, ca_max_travel_dist);
                    
                    //checks to see if the current telemetry is greater then the max travel distance
                    //get_new_telem_option_2(ii, jj, dist_to_target_waypoint, max_travel_dist, waypoint_telem, current_telem, target_waypoint);
                    
                    //checks to see if the current telemetry matches the target waypoint
                    //check_if_at_waypoint(ii, jj, dist_to_target_waypoint, waypoint_telem, max_travel_dist, current_telem, target_waypoint, num_waypoints);
                    
                    cout << "current telem" << endl;
                    for (int ll=0; ll < 3; ll++)
                    {
                        cout << system.at(ii).agents.at(jj).current_telem.at(ll) << "\t";
                    }
                    cout << endl;
                    cout << "current travel speed" << "\t" << system.at(ii).agents.at(jj).current_travel_speed << endl;
                    
                    //resets the travel speed to the max travel speed for the following time step
                    system.at(ii).agents.at(jj).current_travel_speed=max_flight_velocity;
                }
                cout << endl;
            }
        }
        current_time = current_time + delta_t;
        cout << "-------------------------------------------------------------------------" << endl;
        cout << endl;
    }
}



#endif /* Simulator_hpp */