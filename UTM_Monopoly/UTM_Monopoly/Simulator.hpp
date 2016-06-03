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
    void create_sarting_telm(int num_teams, vector<int> team_sizes, vector<int> waypoint_telm);
    void create_checkpoints(int num_teams, vector<int> team_sizes);
    void create_target_telm(int num_teams, vector<int> team_sizes);
    
private:
    
    
};


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
            for (int k=0; k < num_waypoints; k++)
            {
                Waypoint W;
                system.at(i).agents.at(j).check_points.push_back(W);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create starting coordinates
void Simulator::create_sarting_telm(int num_teams, vector<int> team_sizes, vector<int> waypoint_telm)
{
    for (int i=0; i < num_teams; i++)
    {
        for (int j=0; j < team_sizes.at(i); j++)
        {
            int x_waypoint = 2;
            int y_waypoint = 4;
            int z_waypoint = 1;
            
            system.at(i).agents.at(j).check_points.at(0).waypoint_telm.at(0).push_back(x_waypoint);
            system.at(i).agents.at(j).check_points.at(0).waypoint_telm.at(1).push_back(y_waypoint);
            system.at(i).agents.at(j).check_points.at(0).waypoint_telm.at(2).push_back(z_waypoint);
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create intermediate waypoints
void Simulator::create_checkpoints(int num_teams, vector<int> team_sizes)
{
    
}


/////////////////////////////////////////////////////////////////
//Create target coordinates
void Simulator::create_target_telm(int num_teams, vector<int> team_sizes)
{
    
}


#endif /* Simulator_hpp */