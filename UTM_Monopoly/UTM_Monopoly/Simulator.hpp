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

#include "Parameters.hpp"
#include "Physics.hpp"
#include "Team.hpp"
#include "Individual.hpp"

using namespace std;


class Simulator
{
    friend class Parameters;
    friend class Team;
    friend class Individual;
    friend class Physics;
    
protected:
    
    
public:
    vector<Team> system;
    void set_team_sizes(int);
    
private:
    
    
};


/////////////////////////////////////////////////////////////////
//Team settings
void set_team_sizes(int num_teams)
{
    for (int i=0; i <num_teams; i++)
    {
        Team T;
        Teams.push_back(T);
    }
}

#endif /* Simulator_hpp */
