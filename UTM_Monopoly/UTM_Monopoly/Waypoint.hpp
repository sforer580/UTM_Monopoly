//
//  Waypoint.hpp
//  UTM_Monopoly
//
//  Created by Scott S Forer on 6/2/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef Waypoint_hpp
#define Waypoint_hpp

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>


using namespace std;


class Waypoint
{
    friend class Team;
    friend class Individual;
    friend class Simulator;
    friend class Parameters;
    
protected:
    
    
public:
    
    vector<double> waypoint_telem;      //contains the x, y, and z coordinates for the waypoint
    
    
private:
    
    
};


#endif /* Waypoint_hpp */
