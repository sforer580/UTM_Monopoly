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

//#include "Parameters.hpp"
//#include "Physics.hpp"
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
    void create_teams(int);
    void create_individuals(int num_teams, vector<int> team_sizes);
    
private:
    
    
};


/////////////////////////////////////////////////////////////////
//Create teams
void Simulator::create_teams(int num_teams)
{
    for (int i=0; i < num_teams; i++)
    {
        Team T;
        system.push_back(T);
    }
}


/////////////////////////////////////////////////////////////////
//Create Individuals
void Simulator::create_individuals(int num_teams, vector<int> team_sizes)
{
    for (int i=0; i < num_teams; i++)
    {
        for (int j=0; j < team_sizes.at(i); j++)
        {
            Individual I;
            system.at(i).agents.push_back(I);
        }
    }
}

#endif /* Simulator_hpp */
