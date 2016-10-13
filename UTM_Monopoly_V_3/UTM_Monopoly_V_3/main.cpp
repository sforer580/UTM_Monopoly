//
//  main.cpp
//  UTM_Monopoly_V_3
//
//  Created by Scott S Forer on 9/22/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>

#include "Parameters.hpp"
#include "Team.hpp"
#include "Individual.hpp"
#include "Policy.hpp"
#include "Waypoint.hpp"
#include "Simulator.hpp"
#include "CCEA.hpp"

using namespace std;

int main()
{
    srand(time(NULL));
    Simulator S;
    Parameters P;
    Waypoint W;
    Individual I;
    Policy Po;
    CCEA EA;
    
    P.set_team_sizes();
    EA.pP =  &P;
    S.pP = &P;
    
    EA.run_CCEA();
    
    
    
    
    
    
    //EA.build_simulator(P.pop_size, P.num_teams, P.team_sizes, W.waypoint_telem, P.num_waypoints, P.max_x_dim, P.max_y_dim, P.max_z_dim, P.max_flight_velocity);
    //EA.run_inner_CCEA(P.pop_size, P.num_teams, P.team_sizes, W.waypoint_telem, P.max_flight_velocity, P.delta_t, P.max_travel_dist, I.current_telem, P.time_max, P.num_waypoints, P.max_x_dim, P.max_y_dim, P.max_z_dim, I.target_waypoint, I.dist_to_target_waypoint, I.current_travel_speed, P.ca_max_travel_dist, I.projected_telem, I.inc_projected_telem, P.ca_inc, P.ca_radius, P.ca_flight_speed, I.agent_fitness);
}
