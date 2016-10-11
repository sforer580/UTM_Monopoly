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
    friend class Policy;
    friend class Waypoint;
    
protected:
    
    
public:
    vector<Team> system;
    
    Parameters* pP;
    
    //Simulator Setup
    void create_starting_flight_velocity(vector<Policy>* sim_team);
    
    //Intialize Simulation
    void set_initial_telem(vector<Policy>* sim_team);
    
    //Gather Information
    void get_dist_to_target_waypoint(vector<Policy>* sim_team, int sim_p);
    void get_projected_telem(vector<Policy>* sim_team, int sim_p);
    void calc_projected_telem(vector<Policy>* sim_team, int sim_p);
    void get_inc_projected_telem(vector<Policy>* sim_team, int sim_p);
    
    //Crash Avoidance
    void check_for_collisions(vector<Policy>* sim_team);
    void compare_agents_projected_telem(vector<Policy>* sim_team, int sim_p, int sim_pp, int kk);
    void crash_avoidance(vector<Policy>* sim_team);
    
    //New Telemetry Calculations
    void get_new_telem(vector<Policy>* sim_team, int sim_p);
    void get_new_telem_option_1(vector<Policy>* sim_team, int sim_p);
    //void get_new_telem_option_2(int pp, int jj, double dist_to_target_waypoint, double max_travel_dist, vector<double> waypoint_telem, vector<double>current_telem, int target_waypoint);
    void check_if_at_waypoint(vector<Policy>* sim_team, int sim_p);
    void check_if_at_final_destination(vector<Policy>* sim_team, int sim_p);
    
    //Fitness Calculations
    void get_agent_CA_fitness(int num_teams, vector<int> team_sizes, int pp, double current_travel_speed, double ca_flight_speed, double agent_fitness, int num_waypoints);
    void get_agent_destination_fitness(int pp, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints);
    
    
    //Simulation Main
    void run_simulation(vector<Policy>* sim_team);
    
    
    
    
    
    
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
//Calling the right team for simulation
//Getting fitness values for ach policy for a team for simulation
//////// Resloved Issues
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
//Create Starting Flight Velocity
//sets the current travel speed for each agent in each team
void Simulator::create_starting_flight_velocity(vector<Policy>* sim_team)
{
    for (int sim_p=0; sim_p<sim_team->size(); sim_p++)
    {
        //sim_team->at(sim_p).current_travel_speed = pP->max_flight_velocity;
    }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//BEGIN TELEMETRY CALCULATIONS


/////////////////////////////////////////////////////////////////
//Set The Starting Telemetry
//sets the current telemetry to the starting telemetry

void Simulator::set_initial_telem(vector<Policy>* sim_team)
{
    cout << "current time" << "\t" << 0.0 << endl;
        for (int sim_p=0; sim_p<sim_team->size(); sim_p++)
        {
            double current_x;
            double current_y;
            double current_z;
            current_x = sim_team->at(sim_p).check_points.at(0).waypoint_telem.at(0);
            current_y = sim_team->at(sim_p).check_points.at(0).waypoint_telem.at(1);
            current_z = sim_team->at(sim_p).check_points.at(0).waypoint_telem.at(2);
            sim_team->at(sim_p).current_telem.push_back(current_x);
            sim_team->at(sim_p).current_telem.push_back(current_y);
            sim_team->at(sim_p).current_telem.push_back(current_z);
 
            cout << "agent" << "\t" << sim_p << endl;
            cout << "current telem" << endl;
            for (int ll=0; ll < 3; ll++)
            {
                cout << sim_team->at(sim_p).current_telem.at(ll) << "\t";
            }
            cout << endl;
            cout << "current travel speed" << "\t" << sim_team->at(sim_p).current_travel_speed << endl;
            cout << endl;
 
        }

        cout << endl;
    cout << "-------------------------------------------------------------------------" << endl;
    cout << endl;
     
}



/////////////////////////////////////////////////////////////////
//Calculate Distance to Target Waypoint
//gets the distance from an agents current telemetry to the telemetry of its target waypoint
void Simulator::get_dist_to_target_waypoint(vector<Policy>* sim_team, int sim_p)
{
        sim_team->at(sim_p).dist_to_target_waypoint = 0;
        vector <double> V;
        vector <double> D;
        double V_mag_1;
        double V_mag_2;
        double V_mag_3;
        int P1 = sim_team->at(sim_p).target_waypoint;
        V.push_back(sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(0) - sim_team->at(sim_p).current_telem.at(0));
        V.push_back(sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(1) - sim_team->at(sim_p).current_telem.at(1));
        V.push_back(sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(2) - sim_team->at(sim_p).current_telem.at(2));
        V_mag_1 = V.at(0)*V.at(0);
        V_mag_2 = V.at(1)*V.at(1);
        V_mag_3 = V.at(2)*V.at(2);
        sim_team->at(sim_p).dist_to_target_waypoint = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
}


/////////////////////////////////////////////////////////////////
//Calculates The Projected Telemetry
//gets the projected telemetry of an agent if it were to travel at its max velocity
void Simulator::get_projected_telem(vector<Policy>* sim_team, int sim_p)
{
    //cout << "cp5" << endl;
    //check_if_at_waypoint(pp, jj, dist_to_target_waypoint, waypoint_telem, max_travel_dist, current_telem, target_waypoint, num_waypoints);
    
    if (sim_team->at(sim_p).dist_to_target_waypoint > 0)
    {
        if (sim_team->at(sim_p).dist_to_target_waypoint < pP->max_travel_dist)
        {
            //cout << "cp3" << endl;
            //checks to see if the distance from the current telemetry to the target waypoint is within the max travel distance
            get_new_telem_option_1(sim_team, sim_p);
        }
    }
    if (sim_team->at(sim_p).dist_to_target_waypoint > 0)
    {
        if (sim_team->at(sim_p).dist_to_target_waypoint > pP->max_travel_dist)
        {
            //cout << "cp4" << endl;
            //calculates the new projected telemetry
            calc_projected_telem(sim_team, sim_p);
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
void Simulator::get_new_telem_option_1(vector<Policy>* sim_team, int sim_p)
{
    //cout << "reached target waypoint" << endl;
    sim_team->at(sim_p).projected_telem.push_back(sim_team->at(sim_p).check_points.at(sim_team->at(sim_p).target_waypoint).waypoint_telem.at(0));
    sim_team->at(sim_p).projected_telem.push_back(sim_team->at(sim_p).check_points.at(sim_team->at(sim_p).target_waypoint).waypoint_telem.at(1));
    sim_team->at(sim_p).projected_telem.push_back(sim_team->at(sim_p).check_points.at(sim_team->at(sim_p).target_waypoint).waypoint_telem.at(2));
    //cout << "op1" << endl;
    //system.at(ii).agents.at(jj).target_waypoint = system.at(ii).agents.at(jj).target_waypoint + 1;
    //checks to see if agent has reached their final destination
    //check_if_at_final_destination(ii, jj, target_waypoint, num_waypoints);
}


/////////////////////////////////////////////////////////////////
//Calculates New Projected Telemetry
//gets the projected telemetry using an agents current travel speed
void Simulator::calc_projected_telem(vector<Policy>* sim_team, int sim_p)
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
    int P1 = sim_team->at(sim_p).target_waypoint;
    int P2 = sim_team->at(sim_p).target_waypoint -1;
    
    static int counter;
    counter++;
    
    //Calculates the travel distance
    travel_dist = pP->max_travel_dist;
    
    //Creates Vector Between Waypoints
    V.push_back(sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(0) - sim_team->at(sim_p).check_points.at(P2).waypoint_telem.at(0));
    V.push_back(sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(1) - sim_team->at(sim_p).check_points.at(P2).waypoint_telem.at(1));
    V.push_back(sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(2) - sim_team->at(sim_p).check_points.at(P2).waypoint_telem.at(2));
    V_mag_1 = V.at(0)*V.at(0);
    V_mag_2 = V.at(1)*V.at(1);
    V_mag_3 = V.at(2)*V.at(2);
    V_mag = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
    
    //Creates Unit Vector Between Waypoints
    D.push_back(V.at(0)/V_mag);
    D.push_back(V.at(1)/V_mag);
    D.push_back(V.at(2)/V_mag);
    
    //Calculates current telemetry
    current_x = sim_team->at(sim_p).current_telem.at(0) + travel_dist*D.at(0);
    current_y = sim_team->at(sim_p).current_telem.at(1) + travel_dist*D.at(1);
    current_z = sim_team->at(sim_p).current_telem.at(2) + travel_dist*D.at(2);
    sim_team->at(sim_p).projected_telem.clear();
    sim_team->at(sim_p).projected_telem.push_back(current_x);
    sim_team->at(sim_p).projected_telem.push_back(current_y);
    sim_team->at(sim_p).projected_telem.push_back(current_z);
    //cout << "cp2" << endl;
}


/////////////////////////////////////////////////////////////////
//Calculates The Incremented Projected Telemetry
//gets the incremented projected telemetry for each agent using a preset parameter
void Simulator::get_inc_projected_telem(vector<Policy>* sim_team, int sim_p)
{
    sim_team->at(sim_p).inc_projected_telem.clear();        //clears previous incremented projected telemetry
    double x_inc_movement;
    double y_inc_movement;
    double z_inc_movement;
    
    //calculates the incremented movement for each dimension
    x_inc_movement = (sim_team->at(sim_p).projected_telem.at(0) - sim_team->at(sim_p).current_telem.at(0))/pP->ca_inc;
    y_inc_movement = (sim_team->at(sim_p).projected_telem.at(1) - sim_team->at(sim_p).current_telem.at(1))/pP->ca_inc;
    z_inc_movement = (sim_team->at(sim_p).projected_telem.at(2) - sim_team->at(sim_p).current_telem.at(2))/pP->ca_inc;
    
    //pushes the incremented projected telemetry into a vector
    for (int iii=0; iii < pP->ca_inc+2; iii++)
    {
        sim_team->at(sim_p).inc_projected_telem.push_back(sim_team->at(sim_p).current_telem.at(0) + iii*x_inc_movement);
        sim_team->at(sim_p).inc_projected_telem.push_back(sim_team->at(sim_p).current_telem.at(1) + iii*y_inc_movement);
        sim_team->at(sim_p).inc_projected_telem.push_back(sim_team->at(sim_p).current_telem.at(2) + iii*z_inc_movement);
    }
}


/////////////////////////////////////////////////////////////////
//Checks For Possible Collisions
//checks the distance of each agents projected telemetry against one another
void Simulator::check_for_collisions(vector<Policy>* sim_team)
{
    for (int sim_p=0; sim_p< sim_team->size(); sim_p++)
    {
        //only considers agent who have not reached their final destination
        if (sim_team->at(sim_p).target_waypoint < pP->num_waypoints + 2)
        {
            for (int kk=0; kk < pP->ca_inc+2; kk++)
            {
                for (int sim_pp=0; sim_pp < sim_team->size(); sim_pp++)
                {
                    //only considers agent who have not reached their final destination
                    if (sim_team->at(sim_pp).target_waypoint < pP->num_waypoints + 2)
                    {
                        //will not compare agents with itself
                        if (sim_p!=sim_pp)
                        {
                            if (sim_team->at(sim_p).current_travel_speed == pP->ca_flight_speed)
                            {
                                continue;
                            }
                            else
                            {
                                compare_agents_projected_telem(sim_team, sim_p, sim_pp, kk);
                            }
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
void Simulator::compare_agents_projected_telem(vector<Policy>* sim_team, int sim_p, int sim_pp, int kk)
{
    double x;
    double x_mag;
    double y;
    double y_mag;
    double z;
    double z_mag;
    double r;
    x = sim_team->at(sim_p).inc_projected_telem.at(kk+(kk*2))-sim_team->at(sim_pp).inc_projected_telem.at(kk+(kk*2));
    y = sim_team->at(sim_p).inc_projected_telem.at(kk+(kk*2)+1)-sim_team->at(sim_pp).inc_projected_telem.at(kk+(kk*2)+1);
    z = sim_team->at(sim_p).inc_projected_telem.at(kk+(kk*2)+2)-sim_team->at(sim_pp).inc_projected_telem.at(kk+(kk*2)+2);
    x_mag = x*x;
    y_mag = y*y;
    z_mag = z*z;
    r = x_mag+y_mag+z_mag;
    //cout << r << endl;
    if (r <= (pP->ca_radius*pP->ca_radius))
    {
        //cout << "collision decteded" << endl;
        sim_team->at(sim_p).current_travel_speed = pP->ca_flight_speed;
        //cout << system.at(ii).agents.at(jj).current_travel_speed << endl;
    }
    else
    {
        //cout << "pass" << endl;
        sim_team->at(sim_p).current_travel_speed = pP->max_flight_velocity;
    }
}

/////////////////////////////////////////////////////////////////
//Runs The Crash Avoidance Check
//crash avoidance main function
void Simulator::crash_avoidance(vector<Policy>* sim_team)
{
        for (int sim_p=0; sim_p< sim_team->size(); sim_p++)
        {
            //only considers agent who have not reached their final destination
            if (sim_team->at(sim_p).target_waypoint < pP->num_waypoints + 2)
            {
                sim_team->at(sim_p).current_travel_speed = pP->max_flight_velocity;
                sim_team->at(sim_p).projected_telem.clear();
                //gets the distance from the current telemetry to the target waypoint
                get_dist_to_target_waypoint(sim_team, sim_p);
                //gets the projected telemetry
                get_projected_telem(sim_team, sim_p);
                //cout << "cp1" << endl;
                //gets the incremented projected telemetry
                get_inc_projected_telem(sim_team, sim_p);
            }
        }
    check_for_collisions(sim_team);
}


/////////////////////////////////////////////////////////////////
//Gets New Target Waypoint
//checks to see if the current telemetry matches the target waypoint
void Simulator::check_if_at_waypoint(vector<Policy>* sim_team, int sim_p)
{
    //cout <<system.at(pp).agents.at(jj).dist_to_target_waypoint << endl;
    get_dist_to_target_waypoint(sim_team, sim_p);
    //cout <<system.at(pp).agents.at(jj).dist_to_target_waypoint << endl;
    if (sim_team->at(sim_p).dist_to_target_waypoint == 0)
    {
        /*
        cout << "reached waypoint" << "\t" << system.at(pp).agents.at(jj).target_waypoint << endl;
         */
        //calc_new_telem(waypoint_telem, max_travel_dist, current_telem, pp, jj, target_waypoint);
        //cout << system.at(pp).agents.at(jj).target_waypoint << endl;
        sim_team->at(sim_p).target_waypoint++;
        /*
        if (system.at(pp).agents.at(jj).target_waypoint != num_waypoints+2)
        {
            cout << "new target waypoint" << "\t" << system.at(pp).agents.at(jj).target_waypoint << endl;
        }
         */
        //cout << system.at(pp).agents.at(jj).target_waypoint << endl;
        //checks to see if agent has reached their final destination
        check_if_at_final_destination(sim_team, sim_p);
    }
}


/////////////////////////////////////////////////////////////////
//Checks If Agent Is at Their Final Destination
void Simulator::check_if_at_final_destination(vector<Policy>* sim_team, int sim_p)
{
    if (sim_team->at(sim_p).current_telem.at(0) == sim_team->at(sim_p).check_points.at(pP->num_waypoints + 1).waypoint_telem.at(0))
    {
        if (sim_team->at(sim_p).current_telem.at(1) == sim_team->at(sim_p).check_points.at(pP->num_waypoints + 1).waypoint_telem.at(1))
        {
            if (sim_team->at(sim_p).current_telem.at(2) == sim_team->at(sim_p).check_points.at(pP->num_waypoints + 1).waypoint_telem.at(2))
            {
                /*
                cout << "reached final destination" << endl;
                */
                sim_team->at(sim_p).current_telem = sim_team->at(sim_p).current_telem;
                //add in an if statement so that the time to final destination is not overwritten
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Calculates New Telemetry
//gets the new telemetry based on current travel speed determuned by the crash avoidance
void Simulator::get_new_telem(vector<Policy>* sim_team, int sim_p)
{
    //system.at(ii).agents.at(jj).current_telem.clear();
    //cout << system.at(ii).agents.at(jj).dist_to_target_waypoint << endl;
    
    //if the agent is within the distacne it can travel given its current travel speed to its target waypoint
    if (sim_team->at(sim_p).dist_to_target_waypoint >= 0)
    {
        if (sim_team->at(sim_p).dist_to_target_waypoint <= sim_team->at(sim_p).current_travel_speed*pP->delta_t)
        {
            sim_team->at(sim_p).current_telem.at(0) = (sim_team->at(sim_p).check_points.at(sim_team->at(sim_p).target_waypoint).waypoint_telem.at(0));
            sim_team->at(sim_p).current_telem.at(1) = (sim_team->at(sim_p).check_points.at(sim_team->at(sim_p).target_waypoint).waypoint_telem.at(1));
            sim_team->at(sim_p).current_telem.at(2) = (sim_team->at(sim_p).check_points.at(sim_team->at(sim_p).target_waypoint).waypoint_telem.at(2));
        }
        //cout << "current telem" << endl;
        //for (int ll=0; ll < 3; ll++)
        //{
        //cout << system.at(ii).agents.at(jj).current_telem.at(ll) << "\t";
        //}
        //cout << endl;
    }
    
    //if the agent is not within the distacne it can travel given its current travel speed to its target waypoint
    if (sim_team->at(sim_p).dist_to_target_waypoint > 0)
    {
        if (sim_team->at(sim_p).dist_to_target_waypoint > sim_team->at(sim_p).current_travel_speed*pP->delta_t)
        {
            if (sim_team->at(sim_p).current_travel_speed==pP->max_flight_velocity)
            {
                sim_team->at(sim_p).current_telem.at(0) = (sim_team->at(sim_p).projected_telem.at(0));
                sim_team->at(sim_p).current_telem.at(1) = (sim_team->at(sim_p).projected_telem.at(1));
                sim_team->at(sim_p).current_telem.at(2) = (sim_team->at(sim_p).projected_telem.at(2));
            }
            if (sim_team->at(sim_p).current_travel_speed==pP->ca_flight_speed)
            {
                calc_projected_telem(sim_team, sim_p);
                sim_team->at(sim_p).current_telem.at(0) = (sim_team->at(sim_p).projected_telem.at(0));
                sim_team->at(sim_p).current_telem.at(1) = (sim_team->at(sim_p).projected_telem.at(1));
                sim_team->at(sim_p).current_telem.at(2) = (sim_team->at(sim_p).projected_telem.at(2));
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//END TELEMETRY CALCULATIONS


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//Fittness Functions
//Calculates an agents fitness
void Simulator::get_agent_CA_fitness(int num_teams, vector<int> team_sizes, int pp, double current_travel_speed, double ca_flight_speed, double agent_fitness, int num_waypoints)
{
    for (int iii=0; iii < team_sizes.at(pp); iii++)
    {
        if (system.at(pp).agents.at(iii).target_waypoint < num_waypoints + 2)
        {
            if (system.at(pp).agents.at(iii).current_travel_speed == ca_flight_speed)
            {
                system.at(pp).agents.at(iii).agent_fitness =+ system.at(pp).agents.at(iii).agent_fitness + 1;
            }
            else
            {
                continue;
            }
        }
    }
}

/////////////////////////////////////////////////////////////////
//Calculates an agents fitness
void Simulator::get_agent_destination_fitness(int pp, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints)
{
    for (int iii=0; iii < team_sizes.at(pp); iii++)
    {
        if (system.at(pp).agents.at(iii).current_telem.at(0) == system.at(pp).agents.at(iii).check_points.at(num_waypoints+1).waypoint_telem.at(0))
        {
            if (system.at(pp).agents.at(iii).current_telem.at(1) == system.at(pp).agents.at(iii).check_points.at(num_waypoints+1).waypoint_telem.at(1))
            {
                if (system.at(pp).agents.at(iii).current_telem.at(2) == system.at(pp).agents.at(iii).check_points.at(num_waypoints+1).waypoint_telem.at(2))
                {
                    //cout << "team" << "\t" << pp << "\t" << "agent" << "\t" << iii << "\t" << "has reached its final destination" << endl;
                    continue;
                }
                else
                {
                    //cout << "team" << "\t" << pp << "\t" << "agent" << "\t" << iii << "\t" << "has not reached its final destination" << endl;
                    system.at(pp).agents.at(iii).agent_fitness = system.at(pp).agents.at(iii).agent_fitness + 10;
                }
            }
            else
            {
                //cout << "team" << "\t" << pp << "\t" << "agent" << "\t" << iii << "\t" << "has not reached its final destination" << endl;
                system.at(pp).agents.at(iii).agent_fitness = system.at(pp).agents.at(iii).agent_fitness + 10;
            }
        }
        else
        {
            //cout << "team" << "\t" << pp << "\t" << "agent" << "\t" << iii << "\t" << "has not reached its final destination" << endl;
            system.at(pp).agents.at(iii).agent_fitness = system.at(pp).agents.at(iii).agent_fitness + 10;
        }
    }
}



/////////////////////////////////////////////////////////////////
//Runs Entire Simulation
void Simulator::run_simulation(vector<Policy>* sim_team)
{
    cout << sim_team->size() << endl;
    for (int sim_p=0; sim_p<sim_team->size(); sim_p++)
    {
        sim_team->at(sim_p).current_travel_speed = pP->max_flight_velocity;
        //cout << sim_team->at(sim_p).current_travel_speed << endl;
    }
    double d = rand() % 10;
    for (int p=0; p<sim_team->size(); p++)
    {
        //cout << "in" << endl;
        sim_team->at(p).policy_fitness = d;
    }
    
    
    //runs the acutal simulation
    create_starting_flight_velocity(sim_team);
    set_initial_telem(sim_team);
    double current_time = pP->delta_t;
    while (current_time < pP->time_max)
    {
        //checks for crash avoidance
        crash_avoidance(sim_team);
        cout << "current time" << "\t" << current_time << endl;
        for (int sim_p=0; sim_p<sim_team->size(); sim_p++)
        {
            //only considers agent who have not reached their final destination
            if (sim_team->at(sim_p).target_waypoint < pP->num_waypoints + 2)
            {
                cout << "agent" << "\t" << sim_p << endl;
                cout << "current telem" << endl;
                for (int ll=0; ll < 3; ll++)
                {
                    cout << sim_team->at(sim_p).current_telem.at(ll) << "\t";
                    cout << endl;
                    
                    cout << "target waypoint" << "\t" << sim_team->at(sim_p).target_waypoint << endl;
                    
                    cout << "distance to target waypoint" << "\t" << sim_team->at(sim_p).dist_to_target_waypoint << endl;
                    
                    cout << "projected telem" << endl;
                    for (int ll=0; ll < 3; ll++)
                    {
                        cout << sim_team->at(sim_p).projected_telem.at(ll) << "\t";
                    }
                    cout << endl;
                    
                    
                    
                    //gets the new telemetry based on the flight speed from the CA
                    get_new_telem(sim_team, sim_p);
                    //cout << system.at(ii).agents.at(jj).target_waypoint << endl;
                    
                    //runs a check to see if the agent has reached their target waypoint
                    check_if_at_waypoint(sim_team, sim_p);
                    /*
                     cout << "new telem" << endl;
                     for (int ll=0; ll < 3; ll++)
                     {
                     cout << system.at(pp).agents.at(jj).current_telem.at(ll) << "\t";
                     }
                     cout << endl;
                     cout << "current travel speed" << "\t" << system.at(pp).agents.at(jj).current_travel_speed << endl;
                     cout << endl;
                     */
                }
            }
        }
    }
    
    
    
    
    
    
    
    /*
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
                    /*
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
                    /*
                    cout << "new telem" << endl;
                    for (int ll=0; ll < 3; ll++)
                    {
                        cout << system.at(pp).agents.at(jj).current_telem.at(ll) << "\t";
                    }
                    cout << endl;
                    cout << "current travel speed" << "\t" << system.at(pp).agents.at(jj).current_travel_speed << endl;
                    cout << endl;
     
                }
            }
            current_time = current_time + delta_t;
            get_agent_CA_fitness(num_teams, team_sizes, pp, current_travel_speed, ca_flight_speed, agent_fitness, num_waypoints);
            /*
            cout << endl;
            cout << "-------------------------------------------------------------------------" << endl;
     
        }
        get_agent_destination_fitness(pp, team_sizes, waypoint_telem, num_waypoints);
    }
     */
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
