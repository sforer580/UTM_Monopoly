//
//  Individual.hpp
//  UTM_Monoploy_V2
//
//  Created by Scott S Forer on 8/8/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef Individual_hpp
#define Individual_hpp

//#include "Parameters.hpp"
//#include "Team.hpp"
//#include "Simulator.hpp"
#include "Policy.hpp"

using namespace std;


class Individual
{
    friend class Parameters;
    friend class Team;
    friend class Simulator;
    friend class Waypoint;
    friend class CCEA;
    friend class Policy;
protected:
    
    
public:
    int target_waypoint = 1;                    //agents proceeds to target waypoint and starts at waypoint 0
    double dist_to_target_waypoint;
    double agent_fitness = 0;
    double current_travel_speed;
    
    vector<Policy> policies;
    vector<Waypoint> check_points;
    vector<double> current_telem;
    vector<double> projected_telem;
    vector<double> inc_projected_telem;
    
private:
    
    
};


#endif /* Individual_hpp */
