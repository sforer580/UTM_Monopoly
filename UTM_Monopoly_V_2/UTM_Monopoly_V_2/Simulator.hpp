//
//  Simulator.hpp
//  UTM_Monoploy_V2
//
//  Created by Scott S Forer on 8/8/16.
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
#include "Team.hpp"
#include "Individual.hpp"
#include "Waypoint.hpp"

using namespace std;


class Simulator
{
    friend class Parameters;
    friend class Team;
    friend class Individual;
    friend class Waypoint;
    
protected:
    
    
public:
    vector<Team> system;
    
    //Simulator Setup
    void create_starting_flight_velocity(int num_teams, vector<int> team_sizes, double max_flight_velocity);
    
    //Intialize Simulation
    void set_initial_telem(int pp, vector<int> team_sizes);
    
    //Gather Information
    void get_dist_to_target_waypoint(vector<double> waypoint_telem, vector<double> current_telem, int pp, int jj, int target_waypoint, double dist_to_target_waypoint);
    void get_projected_telem(int pp, int jj, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> waypoint_telem, vector<double> current_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, double delta_t);
    void calc_projected_telem(vector<double> waypoint_telem, double current_travel_speed, double delta_t, vector<double> projected_telem, vector<double> current_telem, int pp, int jj, int target_waypoint);
    void get_inc_projected_telem(int pp, int jj, int ca_inc);
    
    //Crash Avoidance
    void check_for_collisions(int pp, vector<int> team_sizes, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double max_flight_speed);
    void compare_agents_projected_telem(int pp, int jj, int kk, int jjj, int ca_radius, double current_travel_speed, double ca_flight_speed, double max_flight_velocity);
    void crash_avoidance(int pp, vector<int> team_sizes, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double max_flight_speed, double delta_t);
    
    //New Telemetry Calculations
    void get_new_telem(int pp, int jj, double current_travel_speed, double max_flight_velocity, double ca_flight_speed, vector<double> current_telem, vector<double> projected_telem, vector<double> waypoint_telem, double delta_t, int target_waypoint, double dist_to_target_waypoint, double max_travel_dist, int num_waypoints, double ca_max_travel_dist);
    void get_new_telem_option_1(int pp, int jj, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, int num_waypoints, double current_max_travel_dist);
    //void get_new_telem_option_2(int pp, int jj, double dist_to_target_waypoint, double max_travel_dist, vector<double> waypoint_telem, vector<double>current_telem, int target_waypoint);
    void check_if_at_waypoint(int pp, int jj, double dist_to_target_waypoint, vector<double> waypoint_telem, double max_travel_dist, vector<double> current_telem, int target_waypoint, int num_waypoints);
    void check_if_at_final_destination(int pp, int jj, int target_waypoint, int num_waypoints);
    
    //Simulation Main
    void run_simulation(int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, double max_flight_velocity, double delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, int target_waypoint, double dist_to_target_waypoint, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed);
    
    //Test Fucntions
    //One Simulator
    void two_agents_same_team_collide(vector<double> waypoint_telm);
    void two_agents_same_team_near_miss(vector<double> waypoint_telm);
    void two_agents_same_team_exact_miss(vector<double> waypoint_telm);
    void two_agents_same_team_close_parallel(vector<double> waypoint_telm);
    void two_agents_same_team_exact_parallel(vector<double> waypoint_telm);
    void two_agents_same_team_far_parallel(vector<double> waypoint_telm);
    void two_agents_diff_team_collide(vector<double> waypoint_telm);
    void two_agents_diff_team_near_miss(vector<double> waypoint_telm);
    void two_agents_diff_team_exact_miss(vector<double> waypoint_telm);
    void two_agents_diff_team_close_parallel(vector<double> waypoint_telm);
    void two_agents_diff_team_exact_parallel(vector<double> waypoint_telm);
    void two_agents_diff_team_far_parallel(vector<double> waypoint_telm);
    //Two Simulators
    void two_sims_two_agents_same_team_collide(vector<double> waypoint_telm);
    void two_sims_two_agents_same_team_near_miss(vector<double> waypoint_telm);
    void two_sims_two_agents_same_team_exact_miss(vector<double> waypoint_telm);
    void two_sims_two_agents_same_team_close_parallel(vector<double> waypoint_telm);
    void two_sims_two_agents_same_team_exact_parallel(vector<double> waypoint_telm);
    void two_sims_two_agents_same_team_far_parallel(vector<double> waypoint_telm);
    void two_sims_two_agents_diff_team_collide(vector<double> waypoint_telm);
    void two_sims_two_agents_diff_team_near_miss(vector<double> waypoint_telm);
    void two_sims_two_agents_diff_team_exact_miss(vector<double> waypoint_telm);
    void two_sims_two_agents_diff_team_close_parallel(vector<double> waypoint_telm);
    void two_sims_two_agents_diff_team_exact_parallel(vector<double> waypoint_telm);
    void two_sims_two_agents_diff_team_far_parallel(vector<double> waypoint_telm);
    
    
private:
    
    
};


//THERE BE DRAGONS AHEAD


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// Current Issues
//
//////// Resloved Issues
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
//Create Starting Flight Velocity
//sets the current travel speed for each agent in each team
void Simulator::create_starting_flight_velocity(int num_teams, vector<int> team_sizes, double max_flight_velocity)
{
    for (int jj=0; jj < num_teams; jj++)
    {
        for (int kk=0; kk < team_sizes.at(jj); kk++)
        {
            //intital travel speed set to max flight vleocity
            system.at(jj).agents.at(kk).current_travel_speed = max_flight_velocity;
        }
    }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//BEGIN TELEMETRY CALCULATIONS


/////////////////////////////////////////////////////////////////
//Set The Starting Telemetry
//sets the current telemetry to the starting telemetry
void Simulator::set_initial_telem(int pp, vector<int> team_sizes)
{
    cout << "current time" << "\t" << 0.0 << endl;
        for (int jj=0; jj < team_sizes.at(pp); jj++)
        {
            double current_x;
            double current_y;
            double current_z;
            current_x = system.at(pp).agents.at(jj).check_points.at(0).waypoint_telem.at(0);
            current_y = system.at(pp).agents.at(jj).check_points.at(0).waypoint_telem.at(1);
            current_z = system.at(pp).agents.at(jj).check_points.at(0).waypoint_telem.at(2);
            system.at(pp).agents.at(jj).current_telem.push_back(current_x);
            system.at(pp).agents.at(jj).current_telem.push_back(current_y);
            system.at(pp).agents.at(jj).current_telem.push_back(current_z);
            cout << "team" << "\t" << pp << "\t" << "agent" << "\t" << jj << endl;
            cout << "current telem" << endl;
            for (int ll=0; ll < 3; ll++)
            {
                cout << system.at(pp).agents.at(jj).current_telem.at(ll) << "\t";
            }
            cout << endl;
            cout << "current travel speed" << "\t" << system.at(pp).agents.at(jj).current_travel_speed << endl;
            cout << endl;
        }
        cout << endl;
    cout << "-------------------------------------------------------------------------" << endl;
    cout << endl;
}


/////////////////////////////////////////////////////////////////
//Calculate Distance to Target Waypoint
//gets the distance from an agents current telemetry to the telemetry of its target waypoint
void Simulator::get_dist_to_target_waypoint(vector<double> waypoint_telm, vector<double> current_telem, int pp, int jj, int target_waypoint, double dist_to_target_waypoint)
{
    system.at(pp).agents.at(jj).dist_to_target_waypoint = 0;
    vector <double> V;
    vector <double> D;
    double V_mag_1;
    double V_mag_2;
    double V_mag_3;
    int P1 = system.at(pp).agents.at(jj).target_waypoint;
    V.push_back(system.at(pp).agents.at(jj).check_points.at(P1).waypoint_telem.at(0) - system.at(pp).agents.at(jj).current_telem.at(0));
    V.push_back(system.at(pp).agents.at(jj).check_points.at(P1).waypoint_telem.at(1) - system.at(pp).agents.at(jj).current_telem.at(1));
    V.push_back(system.at(pp).agents.at(jj).check_points.at(P1).waypoint_telem.at(2) - system.at(pp).agents.at(jj).current_telem.at(2));
    V_mag_1 = V.at(0)*V.at(0);
    V_mag_2 = V.at(1)*V.at(1);
    V_mag_3 = V.at(2)*V.at(2);
    system.at(pp).agents.at(jj).dist_to_target_waypoint = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
}


/////////////////////////////////////////////////////////////////
//Calculates The Projected Telemetry
//gets the projected telemetry of an agent if it were to travel at its max velocity
void Simulator::get_projected_telem(int pp, int jj, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> waypoint_telem, vector<double> current_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, double delta_t)
{
    //cout << "cp5" << endl;
    //check_if_at_waypoint(pp, jj, dist_to_target_waypoint, waypoint_telem, max_travel_dist, current_telem, target_waypoint, num_waypoints);
    
    if (system.at(pp).agents.at(jj).dist_to_target_waypoint > 0)
    {
        if (system.at(pp).agents.at(jj).dist_to_target_waypoint < max_travel_dist)
        {
            //cout << "cp3" << endl;
            //checks to see if the distance from the current telemetry to the target waypoint is within the max travel distance
            get_new_telem_option_1(pp, jj, dist_to_target_waypoint, max_travel_dist, target_waypoint, num_waypoints, ca_max_travel_dist);
        }
    }
    if (system.at(pp).agents.at(jj).dist_to_target_waypoint > 0)
    {
        if (system.at(pp).agents.at(jj).dist_to_target_waypoint > max_travel_dist)
        {
            //cout << "cp4" << endl;
            //calculates the new projected telemetry
            calc_projected_telem(waypoint_telem, current_travel_speed, delta_t, projected_telem, current_telem, pp, jj, target_waypoint);
        }
    }
    //cout << "projected telem" << endl;
    //for (int ll=0; ll < 3; ll++)
    //{
    //cout << system.at(pp).agents.at(jj).projected_telem.at(ll) << "\t";
    //}
    //cout << endl;
}


/////////////////////////////////////////////////////////////////
//Calculates New Projected Telemetry
//sets new projected telemetry to target waypoint telmetry if the distance to the target waypoint is less than the max travel distance
void Simulator::get_new_telem_option_1(int ii, int jj, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, int num_waypoints, double current_max_travel_dist)
{
    //cout << "reached target waypoint" << endl;
    system.at(ii).agents.at(jj).projected_telem.push_back(system.at(ii).agents.at(jj).check_points.at(system.at(ii).agents.at(jj).target_waypoint).waypoint_telem.at(0));
    system.at(ii).agents.at(jj).projected_telem.push_back(system.at(ii).agents.at(jj).check_points.at(system.at(ii).agents.at(jj).target_waypoint).waypoint_telem.at(1));
    system.at(ii).agents.at(jj).projected_telem.push_back(system.at(ii).agents.at(jj).check_points.at(system.at(ii).agents.at(jj).target_waypoint).waypoint_telem.at(2));
    //cout << "op1" << endl;
    //system.at(ii).agents.at(jj).target_waypoint = system.at(ii).agents.at(jj).target_waypoint + 1;
    //checks to see if agent has reached their final destination
    //check_if_at_final_destination(ii, jj, target_waypoint, num_waypoints);
}


/////////////////////////////////////////////////////////////////
//Calculates New Projected Telemetry
//gets the projected telemetry using an agents current travel speed
void Simulator::calc_projected_telem(vector<double> waypoint_telem, double current_travel_speed, double delta_t, vector<double> projected_telem, vector<double> current_telem, int pp, int jj, int target_waypoint)
{
    vector <double> V;
    vector <double> D;
    double V_mag_1;
    double V_mag_2;
    double V_mag_3;
    double V_mag;
    double current_x;
    double current_y;
    double current_z;
    double travel_dist;
    int P1 = system.at(pp).agents.at(jj).target_waypoint;
    int P2 = system.at(pp).agents.at(jj).target_waypoint -1;
    
    static int counter;
    counter++;
    
    //Calculates the travel distance
    travel_dist = system.at(pp).agents.at(jj).current_travel_speed*delta_t;
    
    //Creates Vector Between Waypoints
    V.push_back(system.at(pp).agents.at(jj).check_points.at(P1).waypoint_telem.at(0) - system.at(pp).agents.at(jj).check_points.at(P2).waypoint_telem.at(0));
    V.push_back(system.at(pp).agents.at(jj).check_points.at(P1).waypoint_telem.at(1) - system.at(pp).agents.at(jj).check_points.at(P2).waypoint_telem.at(1));
    V.push_back(system.at(pp).agents.at(jj).check_points.at(P1).waypoint_telem.at(2) - system.at(pp).agents.at(jj).check_points.at(P2).waypoint_telem.at(2));
    V_mag_1 = V.at(0)*V.at(0);
    V_mag_2 = V.at(1)*V.at(1);
    V_mag_3 = V.at(2)*V.at(2);
    V_mag = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
    
    //Creates Unit Vector Between Waypoints
    D.push_back(V.at(0)/V_mag);
    D.push_back(V.at(1)/V_mag);
    D.push_back(V.at(2)/V_mag);
    
    //Calculates current telemetry
    current_x = system.at(pp).agents.at(jj).current_telem.at(0) + travel_dist*D.at(0);
    current_y = system.at(pp).agents.at(jj).current_telem.at(1) + travel_dist*D.at(1);
    current_z = system.at(pp).agents.at(jj).current_telem.at(2) + travel_dist*D.at(2);
    system.at(pp).agents.at(jj).projected_telem.clear();
    system.at(pp).agents.at(jj).projected_telem.push_back(current_x);
    system.at(pp).agents.at(jj).projected_telem.push_back(current_y);
    system.at(pp).agents.at(jj).projected_telem.push_back(current_z);
    //cout << "cp2" << endl;
}


/////////////////////////////////////////////////////////////////
//Calculates The Incremented Projected Telemetry
//gets the incremented projected telemetry for each agent using a preset parameter
void Simulator::get_inc_projected_telem(int pp, int jj, int ca_inc)
{
    system.at(pp).agents.at(jj).inc_projected_telem.clear();        //clears previous incremented projected telemetry
    double x_inc_movement;
    double y_inc_movement;
    double z_inc_movement;
    
    //calculates the incremented movement for each dimension
    x_inc_movement = (system.at(pp).agents.at(jj).projected_telem.at(0) - system.at(pp).agents.at(jj).current_telem.at(0))/ca_inc;
    y_inc_movement = (system.at(pp).agents.at(jj).projected_telem.at(1) - system.at(pp).agents.at(jj).current_telem.at(1))/ca_inc;
    z_inc_movement = (system.at(pp).agents.at(jj).projected_telem.at(2) - system.at(pp).agents.at(jj).current_telem.at(2))/ca_inc;
    
    //pushes the incremented projected telemetry into a vector
    for (int iii=0; iii < ca_inc+2; iii++)
    {
        system.at(pp).agents.at(jj).inc_projected_telem.push_back(system.at(pp).agents.at(jj).current_telem.at(0) + iii*x_inc_movement);
        system.at(pp).agents.at(jj).inc_projected_telem.push_back(system.at(pp).agents.at(jj).current_telem.at(1) + iii*y_inc_movement);
        system.at(pp).agents.at(jj).inc_projected_telem.push_back(system.at(pp).agents.at(jj).current_telem.at(2) + iii*z_inc_movement);
    }
}


/////////////////////////////////////////////////////////////////
//Checks For Possible Collisions
//checks the distance of each agents projected telemetry against one another
void Simulator::check_for_collisions(int pp, vector<int> team_sizes, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double max_flight_velocity)
{
    for (int jj=0; jj < team_sizes.at(pp); jj++)
    {
        //only considers agent who have not reached their final destination
        if (system.at(pp).agents.at(jj).target_waypoint < num_waypoints + 2)
        {
            for (int kk=0; kk < ca_inc+2; kk++)
            {
                for (int jjj=0; jjj < team_sizes.at(pp); jjj++)
                {
                    //only considers agent who have not reached their final destination
                    if (system.at(pp).agents.at(jjj).target_waypoint < num_waypoints + 2)
                    {
                        //will not compare agents with itself
                        if (jj!=jjj)
                        {
                            compare_agents_projected_telem(pp, jj, kk, jjj, ca_radius, current_travel_speed, ca_flight_speed, max_flight_velocity);
                        }
                        else
                        {
                            continue;
                        }
                    }
                }
            }
        }
        //will not compare agents who have reached their final destination
        else
        {
            continue;
        }
    }
}


/////////////////////////////////////////////////////////////////
//Comapres Agents Incremented Projected Telemetry
//gets distnace from one agents projected telemetry to another
void Simulator::compare_agents_projected_telem(int pp, int jj, int kk, int jjj, int ca_radius, double current_travel_speed, double ca_flight_speed, double max_flight_velocity)
{
    double x;
    double x_mag;
    double y;
    double y_mag;
    double z;
    double z_mag;
    double r;
    x = system.at(pp).agents.at(jj).inc_projected_telem.at(kk+(kk*2))-system.at(pp).agents.at(jjj).inc_projected_telem.at(kk+(kk*2));
    y = system.at(pp).agents.at(jj).inc_projected_telem.at(kk+(kk*2)+1)-system.at(pp).agents.at(jjj).inc_projected_telem.at(kk+(kk*2)+1);
    z = system.at(pp).agents.at(jj).inc_projected_telem.at(kk+(kk*2)+2)-system.at(pp).agents.at(jjj).inc_projected_telem.at(kk+(kk*2)+2);
    x_mag = x*x;
    y_mag = y*y;
    z_mag = z*z;
    r = x_mag+y_mag+z_mag;
    //cout << r << endl;
    if (r <= (ca_radius*ca_radius))
    {
        //cout << "collision decteded" << endl;
        system.at(pp).agents.at(jj).current_travel_speed = ca_flight_speed;
        //cout << system.at(ii).agents.at(jj).current_travel_speed << endl;
    }
    else
    {
        //cout << "pass" << endl;
        system.at(pp).agents.at(jj).current_travel_speed = max_flight_velocity;
    }
}

/////////////////////////////////////////////////////////////////
//Runs The Crash Avoidance Check
//crash avoidance main function
void Simulator::crash_avoidance(int pp, vector<int> team_sizes, double dist_to_target_waypoint, double max_travel_dist, int target_waypoint, vector<double> current_telem, vector<double> waypoint_telem, int num_waypoints, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed, double max_flight_velocity, double delta_t)
{
        for (int jj=0; jj < team_sizes.at(pp); jj++)
        {
            //only considers agent who have not reached their final destination
            if (system.at(pp).agents.at(jj).target_waypoint < num_waypoints + 2)
            {
                system.at(pp).agents.at(jj).projected_telem.clear();
                //gets the distance from the current telemetry to the target waypoint
                get_dist_to_target_waypoint(waypoint_telem, current_telem, pp, jj, target_waypoint, dist_to_target_waypoint);
                //gets the projected telemetry
                get_projected_telem(pp, jj, dist_to_target_waypoint, max_travel_dist, target_waypoint, waypoint_telem, current_telem, num_waypoints, current_travel_speed, ca_max_travel_dist, projected_telem, inc_projected_telem, delta_t);
                //cout << "cp1" << endl;
                //gets the incremented projected telemetry
                get_inc_projected_telem(pp, jj, ca_inc);
            }
        }
    check_for_collisions(pp, team_sizes, dist_to_target_waypoint, max_travel_dist, target_waypoint, current_telem, waypoint_telem, num_waypoints, current_travel_speed, ca_max_travel_dist, projected_telem, inc_projected_telem, ca_inc, ca_radius, ca_flight_speed, max_flight_velocity);
}


/////////////////////////////////////////////////////////////////
//Gets New Target Waypoint
//checks to see if the current telemetry matches the target waypoint
void Simulator::check_if_at_waypoint(int pp, int jj, double dist_to_target_waypoint, vector<double> waypoint_telem, double max_travel_dist, vector<double> current_telem, int target_waypoint, int num_waypoints)
{
    //cout <<system.at(pp).agents.at(jj).dist_to_target_waypoint << endl;
    get_dist_to_target_waypoint(waypoint_telem, current_telem, pp, jj, target_waypoint, dist_to_target_waypoint);
    //cout <<system.at(pp).agents.at(jj).dist_to_target_waypoint << endl;
    if (system.at(pp).agents.at(jj).dist_to_target_waypoint == 0)
    {
        cout << "reached waypoint" << "\t" << system.at(pp).agents.at(jj).target_waypoint << endl;
        //calc_new_telem(waypoint_telem, max_travel_dist, current_telem, pp, jj, target_waypoint);
        //cout << system.at(pp).agents.at(jj).target_waypoint << endl;
        system.at(pp).agents.at(jj).target_waypoint++;
        if (system.at(pp).agents.at(jj).target_waypoint != num_waypoints+2)
        {
            cout << "new target waypoint" << "\t" << system.at(pp).agents.at(jj).target_waypoint << endl;
        }
        //cout << system.at(pp).agents.at(jj).target_waypoint << endl;
        //checks to see if agent has reached their final destination
        check_if_at_final_destination(pp, jj, target_waypoint, num_waypoints);
    }
}


/////////////////////////////////////////////////////////////////
//Checks If Agent Is at Their Final Destination
void Simulator::check_if_at_final_destination(int ii, int jj, int target_waypoint, int num_waypoints)
{
    if (system.at(ii).agents.at(jj).current_telem.at(0) == system.at(ii).agents.at(jj).check_points.at(num_waypoints + 1).waypoint_telem.at(0))
    {
        if (system.at(ii).agents.at(jj).current_telem.at(1) == system.at(ii).agents.at(jj).check_points.at(num_waypoints + 1).waypoint_telem.at(1))
        {
            if (system.at(ii).agents.at(jj).current_telem.at(2) == system.at(ii).agents.at(jj).check_points.at(num_waypoints + 1).waypoint_telem.at(2))
            {
                cout << "reached final destination" << endl;
                system.at(ii).agents.at(jj).current_telem = system.at(ii).agents.at(jj).current_telem;
                //add in an if statement so that the time to final destination is not overwritten
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Calculates New Telemetry
//gets the new telemetry based on current travel speed determuned by the crash avoidance
void Simulator::get_new_telem(int pp, int jj, double current_travel_speed, double max_flight_velocity, double ca_flight_speed, vector<double> current_telem, vector<double> projected_telem, vector<double> waypoint_telem, double delta_t, int target_waypoint, double dist_to_target_waypoint, double max_travel_dist, int num_waypoints, double ca_max_travel_dist)
{
    //system.at(ii).agents.at(jj).current_telem.clear();
    //cout << system.at(ii).agents.at(jj).dist_to_target_waypoint << endl;
    
    //if the agent is within the distacne it can travel given its current travel speed to its target waypoint
    if (system.at(pp).agents.at(jj).dist_to_target_waypoint >= 0)
    {
        if (system.at(pp).agents.at(jj).dist_to_target_waypoint <= system.at(pp).agents.at(jj).current_travel_speed*delta_t)
        {
            system.at(pp).agents.at(jj).current_telem.at(0) = (system.at(pp).agents.at(jj).check_points.at(system.at(pp).agents.at(jj).target_waypoint).waypoint_telem.at(0));
            system.at(pp).agents.at(jj).current_telem.at(1) = (system.at(pp).agents.at(jj).check_points.at(system.at(pp).agents.at(jj).target_waypoint).waypoint_telem.at(1));
            system.at(pp).agents.at(jj).current_telem.at(2) = (system.at(pp).agents.at(jj).check_points.at(system.at(pp).agents.at(jj).target_waypoint).waypoint_telem.at(2));
        }
        //cout << "current telem" << endl;
        //for (int ll=0; ll < 3; ll++)
        //{
        //cout << system.at(ii).agents.at(jj).current_telem.at(ll) << "\t";
        //}
        //cout << endl;
    }
    
    //if the agent is not within the distacne it can travel given its current travel speed to its target waypoint
    if (system.at(pp).agents.at(jj).dist_to_target_waypoint > 0)
    {
        if (system.at(pp).agents.at(jj).dist_to_target_waypoint > system.at(pp).agents.at(jj).current_travel_speed*delta_t)
        {
            if (system.at(pp).agents.at(jj).current_travel_speed==max_flight_velocity)
            {
                system.at(pp).agents.at(jj).current_telem.at(0) = (system.at(pp).agents.at(jj).projected_telem.at(0));
                system.at(pp).agents.at(jj).current_telem.at(1) = (system.at(pp).agents.at(jj).projected_telem.at(1));
                system.at(pp).agents.at(jj).current_telem.at(2) = (system.at(pp).agents.at(jj).projected_telem.at(2));
            }
            if (system.at(pp).agents.at(jj).current_travel_speed==ca_flight_speed)
            {
                calc_projected_telem(waypoint_telem, current_travel_speed, delta_t, projected_telem, current_telem, pp, jj, target_waypoint);
                system.at(pp).agents.at(jj).current_telem.at(0) = (system.at(pp).agents.at(jj).projected_telem.at(0));
                system.at(pp).agents.at(jj).current_telem.at(1) = (system.at(pp).agents.at(jj).projected_telem.at(1));
                system.at(pp).agents.at(jj).current_telem.at(2) = (system.at(pp).agents.at(jj).projected_telem.at(2));
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//END TELEMETRY CALCULATIONS


/////////////////////////////////////////////////////////////////
//Runs Entire Simulation
void Simulator::run_simulation(int num_teams, vector<int> team_sizes, vector<double> waypoint_telem, double max_flight_velocity, double delta_t, double max_travel_dist, vector<double> current_telem, int time_max, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim, int target_waypoint, double dist_to_target_waypoint, double current_travel_speed, double ca_max_travel_dist, vector<double> projected_telem, vector<double> inc_projected_telem, int ca_inc, int ca_radius, double ca_flight_speed)
{
    for (int pp=0; pp < num_teams; pp++)
    {
        set_initial_telem(pp, team_sizes);
        double current_time = delta_t;
        while (current_time < time_max)
        {
            //checks for crash avoidance
            crash_avoidance(pp, team_sizes, dist_to_target_waypoint, max_travel_dist, target_waypoint, current_telem, waypoint_telem, num_waypoints, current_travel_speed, ca_max_travel_dist, projected_telem, inc_projected_telem, ca_inc, ca_radius, ca_flight_speed, max_flight_velocity, delta_t);
            cout << "current time" << "\t" << current_time << endl;
            cout << endl;
            for (int jj=0; jj < team_sizes.at(pp); jj++)
            {
                //only considers agent who have not reached their final destination
                if (system.at(pp).agents.at(jj).target_waypoint < num_waypoints + 2)
                {
                    //cout << "cp1" << endl;
                    cout << "team" << "\t" << pp << "\t" << "agent" << "\t" << jj << endl;
                    
                    cout << "current telem" << endl;
                    for (int ll=0; ll < 3; ll++)
                    {
                        cout << system.at(pp).agents.at(jj).current_telem.at(ll) << "\t";
                    }
                    cout << endl;
                        
                    cout << "target waypoint" << "\t" << system.at(pp).agents.at(jj).target_waypoint << endl;
                        
                    cout << "distance to target waypoint" << "\t" << system.at(pp).agents.at(jj).dist_to_target_waypoint << endl;
                
                    cout << "projected telem" << endl;
                    for (int ll=0; ll < 3; ll++)
                    {
                        cout << system.at(pp).agents.at(jj).projected_telem.at(ll) << "\t";
                    }
                    cout << endl;
                        
                    //gets the new telemetry based on the flight speed from the CA
                    get_new_telem(pp, jj, current_travel_speed, max_flight_velocity, ca_flight_speed, current_telem, projected_telem, waypoint_telem, delta_t, target_waypoint, dist_to_target_waypoint, max_travel_dist, num_waypoints, ca_max_travel_dist);
                    //cout << system.at(ii).agents.at(jj).target_waypoint << endl;
                        
                    //runs a check to see if the agent has reached their target waypoint
                    check_if_at_waypoint(pp, jj, dist_to_target_waypoint, waypoint_telem, max_travel_dist, current_telem, target_waypoint, num_waypoints);
                        
                    cout << "new telem" << endl;
                    for (int ll=0; ll < 3; ll++)
                    {
                        cout << system.at(pp).agents.at(jj).current_telem.at(ll) << "\t";
                    }
                    cout << endl;
                    cout << "current travel speed" << "\t" << system.at(pp).agents.at(jj).current_travel_speed << endl;
                    cout << endl;
                    
                    //resets the travel speed to the max travel speed for the following time step
                    system.at(pp).agents.at(jj).current_travel_speed=max_flight_velocity;
                }
            }
            current_time = current_time + delta_t;
            cout << endl;
            cout << "-------------------------------------------------------------------------" << endl;
        }
    }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Test Functions


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Same Team

/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team On A Collision Course
void Simulator::two_agents_same_team_collide(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=0;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=0;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=0;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Near Miss
void Simulator::two_agents_same_team_near_miss(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Exact Miss
//waypoint distance exactly equal to the CA radius
void Simulator::two_agents_same_team_exact_miss(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Close Range Parallel
//flight path is parallel within the CA Radius
void Simulator::two_agents_same_team_close_parallel(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Exact Parallel
//flight path is parallel with a distance exactly equal to the CA radius
void Simulator::two_agents_same_team_exact_parallel(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Far Range Parallel
//flight path is parallel with a distance exactly equal to the CA radius
void Simulator::two_agents_same_team_far_parallel(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=6;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=6;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=6;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Different Teams

/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Differnt Teams On A Collision Course
void Simulator::two_agents_diff_team_collide(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Near Miss
void Simulator::two_agents_diff_team_near_miss(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Exact Miss
//waypoint distance exactly equal to the CA radius
void Simulator::two_agents_diff_team_exact_miss(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Close Range Parallel
//flight path is parallel within the CA Radius
void Simulator::two_agents_diff_team_close_parallel(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Exact Parallel
//flight path is parallel with a distance exactly equal to the CA radius
void Simulator::two_agents_diff_team_exact_parallel(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Far Range Parallel
//flight path is parallel with a distance exactly equal to the CA radius
void Simulator::two_agents_diff_team_far_parallel(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=6;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=6;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=6;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Same Team Two Simulations

/////////////////////////////////////////////////////////////////
//Two Simulations Two Agents Same Team On A Collision Course
void Simulator::two_sims_two_agents_same_team_collide(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=0;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=0;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=0;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Near Miss
void Simulator::two_sims_two_agents_same_team_near_miss(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Exact Miss
//waypoint distance exactly equal to the CA radius
void Simulator::two_sims_two_agents_same_team_exact_miss(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Close Range Parallel
//flight path is parallel within the CA Radius
void Simulator::two_sims_two_agents_same_team_close_parallel(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Exact Parallel
//flight path is parallel with a distance exactly equal to the CA radius
void Simulator::two_sims_two_agents_same_team_exact_parallel(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Same Team Far Range Parallel
//flight path is parallel with a distance exactly equal to the CA radius
void Simulator::two_sims_two_agents_same_team_far_parallel(vector<double> waypoint_telm)
{
    //set num_teams=1, team_0=2, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(1).check_points.at(0).waypoint_telem.at(2)=6;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(1).check_points.at(1).waypoint_telem.at(2)=6;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(1).check_points.at(2).waypoint_telem.at(2)=6;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Different Teams

/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Differnt Teams On A Collision Course
void Simulator::two_sims_two_agents_diff_team_collide(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Near Miss
void Simulator::two_sims_two_agents_diff_team_near_miss(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Exact Miss
//waypoint distance exactly equal to the CA radius
void Simulator::two_sims_two_agents_diff_team_exact_miss(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Close Range Parallel
//flight path is parallel within the CA Radius
void Simulator::two_sims_two_agents_diff_team_close_parallel(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=4;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=4;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Exact Parallel
//flight path is parallel with a distance exactly equal to the CA radius
void Simulator::two_sims_two_agents_diff_team_exact_parallel(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=5;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=5;
}


/////////////////////////////////////////////////////////////////
//One Simulation Two Agents Different Team Far Range Parallel
//flight path is parallel with a distance exactly equal to the CA radius
void Simulator::two_sims_two_agents_diff_team_far_parallel(vector<double> waypoint_telm)
{
    //set num_teams=2, team_0=1, team_1=1, max_x_dim=50, max_y_dim=50, max_z_dim=10, CA_radius=5, delta_t = 0.1, max_flight_velocity = 5.0, time_max = 40
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(0).agents.at(0).check_points.at(0).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(0)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(1)=0;
    system.at(1).agents.at(0).check_points.at(0).waypoint_telem.at(2)=6;
    
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(0).agents.at(0).check_points.at(1).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(0)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(1)=25;
    system.at(1).agents.at(0).check_points.at(1).waypoint_telem.at(2)=6;
    
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(0).agents.at(0).check_points.at(2).waypoint_telem.at(2)=0;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(0)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(1)=50;
    system.at(1).agents.at(0).check_points.at(2).waypoint_telem.at(2)=6;
}


#endif /* Simulator_hpp */
