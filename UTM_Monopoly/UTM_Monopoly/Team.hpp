//
//  Team.hpp
//  UTM_Monopoly
//
//  Created by Scott S Forer on 5/31/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef Team_hpp
#define Team_hpp

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
#include "Individual.hpp"
#include "Simulator.hpp"

using namespace std;


class Team
{
    friend class Parameters;
    friend class Individual;
    friend class Physics;
    friend class Simulator;
    
protected:
    
    
public:
    vector<Individual> agent;
    void set_team_sizes();
    double team_fitness;
    
private:
    
    
};


#endif /* Team_hpp */
