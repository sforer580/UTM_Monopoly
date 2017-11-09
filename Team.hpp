//
//  Team.hpp
//  UTM_Monoploy_V2
//
//  Created by Scott S Forer on 8/8/16.
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

//#include "Parameters.hpp"
#include "Individual.hpp"
//#include "Simulator.hpp"

using namespace std;


class Team
{
    friend class Parameters;
    friend class Individual;
    friend class Simulator;
    friend class CCEA;
    
protected:
    
    
public:
    vector<Individual> agents;
    void set_team_sizes();
    
private:
    
    
};


#endif /* Team_hpp */
