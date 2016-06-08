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
    
    void build_simulator(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    
    void set_initial_telem(int num_teams, vector<int> team_sizes);
    void calc_new_telem(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int flight_velocity, int delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints);
    
    
    void run_simulation(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int flight_velocity, int delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    
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
//Build Simulator
void Simulator::build_simulator(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    create_teams(num_teams);
    create_individuals(num_teams, team_sizes);
    create_waypoints(num_teams, team_sizes, num_waypoints);
    create_sarting_telm(num_teams, team_sizes, waypoint_telm, max_x_dim, max_y_dim, max_z_dim);
    create_checkpoints(num_teams, team_sizes, waypoint_telm, num_waypoints, max_x_dim, max_y_dim, max_z_dim);
    create_target_telm(num_teams, team_sizes, waypoint_telm, num_waypoints, max_x_dim, max_y_dim, max_z_dim);
    
    cout << "total number of teams" << "\t" << system.size() << endl;
    cout << endl;
    for(int i=0; i < num_teams; i++)
    {
        cout << "-------------------------------------------------------------------------" << endl;
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
                    cout << system.at(i).agents.at(j).check_points.at(k).waypoint_telm.at(h) << "\t";
                }
                cout << endl;
            }
            cout << endl;
        }
    }
    cout << "-------------------------------------------------------------------------" << endl;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//BEGIN TELEMETRY CALCULATIONS


/////////////////////////////////////////////////////////////////
//Set The Starting Telemetry
void Simulator::set_initial_telem(int num_teams, vector<int> team_sizes)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            double current_x;
            double current_y;
            double current_z;
            current_x = system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.at(0);
            current_y = system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.at(1);
            current_z = system.at(ii).agents.at(jj).check_points.at(0).waypoint_telm.at(2);
            system.at(ii).agents.at(jj).current_telem.push_back(current_x);
            system.at(ii).agents.at(jj).current_telem.push_back(current_y);
            system.at(ii).agents.at(jj).current_telem.push_back(current_z);
        }
    }
}




/////////////////////////////////////////////////////////////////
//Track Agent movements
void Simulator::calc_new_telem(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int flight_velocity, int delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints)
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//END TELEMETRY CALCULATIONS


/////////////////////////////////////////////////////////////////
//Runs Entire Simulation
void Simulator::run_simulation(int num_teams, vector<int> team_sizes, vector<double> waypoint_telm, int flight_velocity, int delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    build_simulator(num_teams, team_sizes, waypoint_telm, num_waypoints, max_x_dim, max_y_dim, max_z_dim);
    set_initial_telem(num_teams, team_sizes);
    
}



#endif /* Simulator_hpp */