//
//  main.cpp
//  UTM_Monopoly
//
//  Created by Scott S Forer on 5/31/16.
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
#include "Physics.hpp"
#include "Team.hpp"
#include "Individual.hpp"
#include "Simulator.hpp"

using namespace std;

int main()
{
    srand(time(NULL));
    Simulator S;
    Parameters P;
    Waypoint W;
    Individual I;
    P.set_team_sizes();
    S.run_simulation(P.num_teams, P.team_sizes, W.waypoint_telm, P.flight_velocity, P.delta_t, P.max_travel_dist, I.current_telem, P.time_max, P.num_waypoints, P.max_x_dim, P.max_y_dim, P.max_z_dim);
    
    
}
