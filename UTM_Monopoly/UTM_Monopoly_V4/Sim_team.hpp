//
//  Sim_team.hpp
//  UTM_Monopoly_V_3
//
//  Created by Scott S Forer on 10/15/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef Sim_team_hpp
#define Sim_team_hpp

#include <stdio.h>


class Sim_team
{
    friend class Parameters;
    friend class Individual;
    friend class Simulator;
    friend class World;
    friend class CCEA;
    friend class Sim_policy;
    
protected:
    
    
public:
    vector<Policy> s_pol;
    void set_team_sizes();
    double team_fitness;
    
private:
    
    
};

#endif /* Sim_team_hpp */
