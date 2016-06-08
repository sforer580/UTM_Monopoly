//
//  Paramters.hpp
//  UTM_Monopoly
//
//  Created by Scott S Forer on 5/31/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef Paramters_hpp
#define Paramters_hpp

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>

//#include "Physics.hpp"
//#include "Team.hpp"
//#include "Individual.hpp"
//#include "Simulator.hpp"


using namespace std;


class Parameters
{
    friend class Team;
    friend class Individual;
    friend class Physics;
    friend class Simulator;
    
protected:
    
    
public:
    //Simulator Settings
    int min_x_dim = 0;
    int max_x_dim = 50;
    int min_y_dim = 0;
    int max_y_dim = 50;
    int min_z_dim = 0;
    int max_z_dim = 30;
    int time_max = 120;
    double delta_t = 0.1;           //simulator time step
    int flight_velocity = 5;
    double max_travel_dist = flight_velocity*delta_t;       //max distance a agetn can travel in a time step
    int ca_radius = 5;
    int time_delay = 5;
    
    vector<int> sim_dim;
    
    void set_sim_dim();
    
    //Team Settings
    int num_teams = 2;          //must be an interger 0-10
    int team_0 = 2;
    int team_1 = 3;
    int team_2 = 2;
    int team_3 = 8;
    int team_4 = 7;
    int team_5 = 4;
    int team_6 = 5;
    int team_7 = 8;
    int team_8 = 10;
    int team_9 = 2;
    int num_waypoints = 2;      //number of intermediate waypoints
    
    vector<int> team_sizes;
    
    void set_team_sizes();
    
    
private:
    
    
};


/////////////////////////////////////////////////////////////////
//Map Dimension Settings
//(min_x_dim, max_x_dim, min_y_dim, max_y_dim, min_z_dim, max_z_dim)
void Parameters::set_sim_dim()
{
    sim_dim.push_back(min_x_dim);
    sim_dim.push_back(max_x_dim);
    sim_dim.push_back(min_y_dim);
    sim_dim.push_back(max_y_dim);
    sim_dim.push_back(min_z_dim);
    sim_dim.push_back(max_z_dim);
}

/////////////////////////////////////////////////////////////////
//Team Size Settings
void Parameters::set_team_sizes()
{
    team_sizes.push_back(team_0);
    team_sizes.push_back(team_1);
    team_sizes.push_back(team_2);
    team_sizes.push_back(team_3);
    team_sizes.push_back(team_4);
    team_sizes.push_back(team_5);
    team_sizes.push_back(team_6);
    team_sizes.push_back(team_7);
    team_sizes.push_back(team_8);
    team_sizes.push_back(team_9);
}

#endif /* Paramters_hpp */
