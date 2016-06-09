//
//  Individual.hpp
//  UTM_Monopoly
//
//  Created by Scott S Forer on 5/31/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef Individual_hpp
#define Individual_hpp

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
//#include "Team.hpp"
//#include "Simulator.hpp"
#include "Waypoint.hpp"

using namespace std;


class Individual
{
    friend class Parameters;
    friend class Team;
    friend class Physics;
    friend class Simulator;
    friend class Waypoint;
    
protected:
    
    
public:
    int target_waypoint = 1;
    double dist_to_target_waypoint;
    double agent_fitness;
    
    vector<Waypoint> check_points;
    vector<double> current_telem;
    
private:
    
    
};

#endif /* Individual_hpp */
