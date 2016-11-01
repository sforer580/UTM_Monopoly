//
//  Policy.hpp
//  UTM_Monopoly_V_3
//
//  Created by Scott S Forer on 9/22/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef Policy_hpp
#define Policy_hpp

#include <stdio.h>
#include "Waypoint.hpp"

class Policy
{
    friend class Parameters;
    friend class Simulator;
    friend class Team;
    friend class Individual;
    friend class Waypoint;

protected:
    
    
public:
    vector<Waypoint> check_points;
    
    double dist_to_target_waypoint;
    double policy_fitness = 10000000000;
    double current_travel_speed;
    int simulation_id;
    int selected;       //0 if not yet seleceted, 1 for selected
    int target_waypoint = 1;                    //agents proceeds to target waypoint and starts at waypoint 0
    int policy_id;
    int selection_counter;
    int corp_id;
    int at_final_destination;
    
    vector<double> current_telem;
    vector<double> projected_telem;
    vector<double> inc_projected_telem;
private:
    
    
};


#endif /* Policy_hpp */
