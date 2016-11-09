//
//  Parameters.hpp
//  UTM_Monoploy_V2
//
//  Created by Scott S Forer on 8/8/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef Parameters_hpp
#define Parameters_hpp

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


class Parameters
{
    friend class Team;
    friend class Individual;
    friend class Simulator;
    friend class CCEA;
    
protected:
    
    
public:
    //Simulator Settings
    int min_x_dim = 0;
    int max_x_dim = 100;
    int min_y_dim = 0;
    int max_y_dim = 100;
    int min_z_dim = 0;
    int max_z_dim = 100;
    int time_max = 200;                                         //max time simulator will run
    double delta_t = 0.1;                                         //simulator time step
    double max_flight_velocity = 5.0;                           //max velocity at which any given agetn can travel
    double max_travel_dist = max_flight_velocity*delta_t;       //max distance a agetn can travel in a time step
    int ca_radius = 5;                                         //collision avoidance radius
    int ca_inc = 100;                                           //amount of increments between the current telem and projected telem
    double ca_flight_speed = max_flight_velocity/2;             //collision avoidance speed
    double ca_max_travel_dist = ca_flight_speed*delta_t;        //max distance any agent can travel when collision avoidance is acitvated
    
    vector<int> sim_dim;
    
    void set_sim_dim();
    
    //Team Settings
    int num_teams = 2;          //must be an interger 0-10
    int team_0 = 2;
    int team_1 = 2;
    int team_2 = 0;
    int team_3 = 0;
    int team_4 = 0;
    int team_5 = 0;
    int team_6 = 0;
    int team_7 = 0;
    int team_8 = 0;
    int team_9 = 0;
    int num_policies = 1;
    int num_waypoints = 1;      //number of intermediate waypoints, total number of waypoints is num_waypoints + 2
    
    vector<int> team_sizes;
    
    void set_team_sizes();
    
    //CCEA Settings
    int gen_max = 1;
    int to_kill = num_policies/2;
    int to_replicate = to_kill;
    double mutate_percentage = 50;
    double mutation_range = 1;
    int leniency = 0;           //0=off, 1=on
    int fair_trial = 0;         //0=off, 1=on
    //Experiments
    int uncoop = 0;             //0=off, 1=on
    int behavior_change = 0;    //0=off, 1=on       //if on team 0 will switch to uncooperative at gen_max/2    uncoop must = 0
    int domino = 1;             //0=off, 1=on       //if on uncoop must = 0
    int c_count = 0;
    
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

#endif /* Parameters_hpp */
