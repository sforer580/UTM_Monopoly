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
    double policy_fitness;
    double current_travel_speed;
    int team_selected;
    int selected;       //0 if not yet seleceted, 1 for selected
    
    vector<double> current_telem;
    vector<double> projected_telem;
    vector<double> inc_projected_telem;
private:
    
    
};


#endif /* Policy_hpp */