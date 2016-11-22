//
//  main.cpp
//  UTM_Monopoly_V4
//
//  Created by Scott S Forer on 10/31/16.
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
#include <cassert>
#include <ctime>

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
    int stat_run = 3;
    srand(time(NULL));
    for (int sr=0; sr<stat_run; sr++)
    {
        cout << "-----------------------------------------------------------------------------------" << endl;
        cout << "New Stat Run" << endl;
        Simulator S;
        Parameters P;
        CCEA EA;
        EA.pP = &P;
        S.pP = &P;
        
        P.set_team_sizes();
        EA.run_CCEA(sr, stat_run);
    }
}

