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
    Simulator S;
    Parameters P;
    S.create_teams(P.num_teams);
    P.set_team_sizes();
    S.create_individuals(P.num_teams, P.team_sizes);
    cout << "total number of teams" << "\t" << S.system.size() << endl;
    for(int i=0; i < P.num_teams; i++)
    {
        cout << "team" << "\t" << i << "\t" << "has" << "\t" << S.system.at(i).agents.size() << "\t" << "agents" << endl;
    }
}
