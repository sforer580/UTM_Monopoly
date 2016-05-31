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

#include "Physics.hpp"
#include "Team.hpp"
#include "Individual.hpp"
#include "Simulator.hpp"


using namespace std;


class Parameters
{
    friend class Team;
    friend class Individual;
    friend class Physics;
    friend class Simulator;
    
protected:
    
    
public:
    int x_dim = 50;
    int y_dim = 50;
    int z_dim = 30;
    vector<int> sim_dim;
    void set_sim_dim();
    int num_teams = 1;
    int ca_radius = 5;
    
    
private:
    
    
};


/////////////////////////////////////////////////////////////////
//Map Dimension Settings
void Parameters::set_sim_dim()
{
    sim_dim.push_back(x_dim);
    sim_dim.push_back(y_dim);
    sim_dim.push_back(z_dim);
}

#endif /* Paramters_hpp */
