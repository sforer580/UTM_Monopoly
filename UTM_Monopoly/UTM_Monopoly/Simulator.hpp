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
//#include "Physics.hpp"
#include "Team.hpp"
#include "Individual.hpp"
#include "Waypoint.hpp"

using namespace std;


class Simulator
{
    friend class Parameters;
    friend class Team;
    friend class Individual;
    friend class Physics;
    friend class Waypoint;
    
protected:
    
    
public:
    vector<Team> system;
    
    void create_teams(int);
    void create_individuals(int num_teams, vector<int> team_sizes);
    void create_waypoints(int num_teams, vector<int> team_sizes, int num_waypoints);
    void create_sarting_telm(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int max_x_dim, int max_y_dim, int max_z_dim);
    void create_checkpoints(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    void create_target_telm(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    void agent_movement(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int flight_velocity, int delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints);
    
private:
    
    
};


//THERE BE DRAGONS AHEAD


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
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            int x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
            int y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 2;
            //int y_waypoint = 4;
            int z_waypoint = 0;
            
            system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.push_back(x_waypoint);
            system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.push_back(y_waypoint);
            system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.push_back(z_waypoint);
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Intermediate Waypoints
void Simulator::create_checkpoints(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            for (int kk=1; kk < num_waypoints+1; kk++)
            {
                double x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
                double y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
                double z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
                //int x_waypoint = 5;
                //int y_waypoint = 6;
                //int z_waypoint = 8;
                
                system.at(ii).agents.at(jj).check_points.at(kk).waypoint_telm.push_back(x_waypoint);
                system.at(ii).agents.at(jj).check_points.at(kk).waypoint_telm.push_back(y_waypoint);
                system.at(ii).agents.at(jj).check_points.at(kk).waypoint_telm.push_back(z_waypoint);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Target Coordinates
void Simulator::create_target_telm(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            int x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
            int y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 10;
            //int y_waypoint = 15;
            int z_waypoint = 0;
            
            system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telm.push_back(x_waypoint);
            system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telm.push_back(y_waypoint);
            system.at(ii).agents.at(jj).check_points.at(num_waypoints+1).waypoint_telm.push_back(z_waypoint);
        }
    }
}


/////////////////////////////////////////////////////////////////
//Track Agent movements
void Simulator::agent_movement(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int flight_velocity, int delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints)
{
    for (int current_time=0; current_time < time_max; current_time+delta_t)
    {
        for (int hh=0; hh < num_waypoints+2; hh++)
        {
            //if ()  //runs for loops untill current position = waypoint_telm.at(hh)
            //needs a lot of work
            //{
                for (int ii=0; ii < num_teams; ii++)
                {
                    for (int jj=0; jj < team_sizes.at(ii); jj++)
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
                        
                        //Creates Vector Between Waypoints
                        V.push_back(system.at(ii).agents.at(jj).check_points.at(1).waypoint_telm.at(0) - system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.at(0));
                        V.push_back(system.at(ii).agents.at(jj).check_points.at(1).waypoint_telm.at(1) - system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.at(1));
                        V.push_back(system.at(ii).agents.at(jj).check_points.at(1).waypoint_telm.at(2) - system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.at(2));
                        V_mag_1 = V.at(0)*V.at(0);
                        V_mag_2 = V.at(1)*V.at(1);
                        V_mag_3 = V.at(2)*V.at(2);
                        V_mag = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
                        
                        //Creates Unit Vector Between Waypoints
                        D.push_back(V.at(0)/V_mag);
                        D.push_back(V.at(1)/V_mag);
                        D.push_back(V.at(2)/V_mag);
                        
                        //Calculates current telemetry
                        current_x = system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.at(0) + max_travel_dist*D.at(0);
                        current_y = system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.at(1) + max_travel_dist*D.at(1);
                        current_z = system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.at(2) + max_travel_dist*D.at(2);
                        system.at(ii).agents.at(jj).current_telem.push_back(current_x);
                        system.at(ii).agents.at(jj).current_telem.push_back(current_y);
                        system.at(ii).agents.at(jj).current_telem.push_back(current_z);
                    }
                }
            //}
        }
    }
}




#endif /* Simulator_hpp */