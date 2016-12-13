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
    friend class CCEA;
    
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
    void check_for_collisions(vector<Policy>* sim_team, int gen, vector<double>* pconflict_counter);
    double get_distance_to_other_agent(vector<Policy>* sim_team, int sim_p, int sim_pp, double distance);
    void compare_agents_projected_telem(vector<Policy>* sim_team, int sim_p, int sim_pp, int kk);
    void crash_avoidance(vector<Policy>* sim_team, int gen, vector<double>* pconflict_counter);
    
    //Experiment Fucntions
    void run_conflict_counter(vector<Policy>* sim_team, int sim_p, int sim_pp, vector<double>* pconflict_counter);
    void run_cooperative_case(vector<Policy>* sim_team, int sim_p, int sim_pp);
    void run_domino_case(vector<Policy>* sim_team, int sim_p, int sim_pp);
    void run_uncoop_case(vector<Policy>* sim_team, int sim_p, int sim_pp);
    void run_behavorial_change_case(vector<Policy>* sim_team, int gen, int sim_p, int sim_pp);
    void run_behaviroal_domino_case(vector<Policy>* sim_team, int sim_p, int sim_pp, int gen);
    void run_malicious_case(vector<Policy>* sim_team, int sim_p, int sim_pp);
    void run_static_full_malicious_case(vector<Policy>* sim_team, int sim_p, int sim_pp);
    
    //New Telemetry Calculations
    void get_new_telem(vector<Policy>* sim_team, int sim_p);
    void get_new_telem_option_1(vector<Policy>* sim_team, int sim_p);
    void check_if_at_waypoint(vector<Policy>* sim_team, int sim_p);
    void check_if_at_final_destination(vector<Policy>* sim_team, int sim_p);
    
    //Fitness Calculations
    void get_agent_CA_fitness(vector<Policy>* sim_team);
    void get_agent_destination_fitness(vector<Policy>* sim_team);
    
    
    //Simulation Main
    void run_simulation(vector<Policy>* sim_team, int gen, vector<double>* pconflict_counter);
    
private:
    
    
};


//THERE BE DRAGONS AHEAD


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// Current Issues
//
//////// Resloved Issues
//Calling the right team for simulation
//Getting fitness values for each policy for a team for simulation
//Verify simulation calculations
//Implement and verify fitness calculations
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
//Create Starting Flight Velocity
//sets the current travel speed for each agent in each team
void Simulator::create_starting_flight_velocity(vector<Policy>* sim_team)
{
    for (int sim_p=0; sim_p<sim_team->size(); sim_p++)
    {
        sim_team->at(sim_p).current_travel_speed = pP->max_flight_velocity;
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
    /*
     cout << "current time" << "\t" << 0.0 << endl;
     cout << "check simulation policies" << endl;
     for (int sim_p=0; sim_p<sim_team->size(); sim_p++)
     {
     cout << "agent" << "\t" << sim_p << endl;
     for (int ww=0; ww<pP->num_waypoints+2; ww++)
     {
     cout << "waypoint" << "\t" << ww << "\t" << "telem" << endl;
     for (int te=0; te<3; te++)
     {
     cout << sim_team->at(sim_p).check_points.at(ww).waypoint_telem.at(te) << "\t";
     }
     cout << endl;
     }
     cout << endl;
     }
     cout << endl;
     cout << endl;
     */
    
    
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
        
        /*
         cout << "agent" << "\t" << sim_p << endl;
         cout << "current telem" << endl;
         for (int ll=0; ll < 3; ll++)
         {
         cout << sim_team->at(sim_p).current_telem.at(ll) << "\t";
         }
         cout << endl;
         cout << "current travel speed" << "\t" << sim_team->at(sim_p).current_travel_speed << endl;
         cout << endl;
         */
        
    }
    /*
     cout << endl;
     cout << "-------------------------------------------------------------------------" << endl;
     cout << endl;
     */
    
}



/////////////////////////////////////////////////////////////////
//Calculate Distance to Target Waypoint
//gets the distance from an agents current telemetry to the telemetry of its target waypoint
void Simulator::get_dist_to_target_waypoint(vector<Policy>* sim_team, int sim_p)
{
    sim_team->at(sim_p).dist_to_target_waypoint = 0;
    double V_1;
    double V_2;
    double V_3;
    double V_mag_1;
    double V_mag_2;
    double V_mag_3;
    int P1 = sim_team->at(sim_p).target_waypoint;
    V_1 = (sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(0) - sim_team->at(sim_p).current_telem.at(0));
    V_2 = (sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(1) - sim_team->at(sim_p).current_telem.at(1));
    V_3 = (sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(2) - sim_team->at(sim_p).current_telem.at(2));
    V_mag_1 = V_1*V_1;
    V_mag_2 = V_2*V_2;
    V_mag_3 = V_3*V_3;
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
    sim_team->at(sim_p).projected_telem.at(0) = (sim_team->at(sim_p).check_points.at(sim_team->at(sim_p).target_waypoint).waypoint_telem.at(0));
    sim_team->at(sim_p).projected_telem.at(1) = (sim_team->at(sim_p).check_points.at(sim_team->at(sim_p).target_waypoint).waypoint_telem.at(1));
    sim_team->at(sim_p).projected_telem.at(2) = (sim_team->at(sim_p).check_points.at(sim_team->at(sim_p).target_waypoint).waypoint_telem.at(2));
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
    double V_1;
    double V_2;
    double V_3;
    double D_1;
    double D_2;
    double D_3;
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
    
    //Calculates the travel distance
    travel_dist = pP->max_travel_dist;
    
    //Creates Vector Between Waypoints
    V_1 = (sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(0) - sim_team->at(sim_p).check_points.at(P2).waypoint_telem.at(0));
    V_2 = (sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(1) - sim_team->at(sim_p).check_points.at(P2).waypoint_telem.at(1));
    V_3 = (sim_team->at(sim_p).check_points.at(P1).waypoint_telem.at(2) - sim_team->at(sim_p).check_points.at(P2).waypoint_telem.at(2));
    V_mag_1 = V_1*V_1;
    V_mag_2 = V_2*V_2;
    V_mag_3 = V_3*V_3;
    V_mag = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
    
    //Creates Unit Vector Between Waypoints
    D_1 = (V_1/V_mag);
    D_2 = (V_2/V_mag);
    D_3 = (V_3/V_mag);
    
    //Calculates current telemetry
    current_x = sim_team->at(sim_p).current_telem.at(0) + travel_dist*D_1;
    current_y = sim_team->at(sim_p).current_telem.at(1) + travel_dist*D_2;
    current_z = sim_team->at(sim_p).current_telem.at(2) + travel_dist*D_3;
    sim_team->at(sim_p).projected_telem.empty();
    sim_team->at(sim_p).projected_telem.at(0) = current_x;
    sim_team->at(sim_p).projected_telem.at(1) = current_y;
    sim_team->at(sim_p).projected_telem.at(2) = current_z;
    //cout << "cp2" << endl;
}


/////////////////////////////////////////////////////////////////
//Calculates The Incremented Projected Telemetry
//gets the incremented projected telemetry for each agent using a preset parameter
void Simulator::get_inc_projected_telem(vector<Policy>* sim_team, int sim_p)
{
    sim_team->at(sim_p).inc_projected_telem.clear();        //clears previous incremented projected telemetry
    sim_team->at(sim_p).inc_projected_telem.resize(3*(pP->ca_inc+2));
    double x_inc_movement;
    double y_inc_movement;
    double z_inc_movement;
    
    //calculates the incremented movement for each dimension
    x_inc_movement = (sim_team->at(sim_p).projected_telem.at(0) - sim_team->at(sim_p).current_telem.at(0))/pP->ca_inc;
    y_inc_movement = (sim_team->at(sim_p).projected_telem.at(1) - sim_team->at(sim_p).current_telem.at(1))/pP->ca_inc;
    z_inc_movement = (sim_team->at(sim_p).projected_telem.at(2) - sim_team->at(sim_p).current_telem.at(2))/pP->ca_inc;
    
    //puts the incremented projected telemetry into a vector
    for (int iii=0; iii<pP->ca_inc+2; iii++)
    {
        sim_team->at(sim_p).inc_projected_telem.at(iii*3) = (sim_team->at(sim_p).current_telem.at(0) + iii*x_inc_movement);
        sim_team->at(sim_p).inc_projected_telem.at(iii*3+1) = (sim_team->at(sim_p).current_telem.at(1) + iii*y_inc_movement);
        sim_team->at(sim_p).inc_projected_telem.at(iii*3+2) = (sim_team->at(sim_p).current_telem.at(2) + iii*z_inc_movement);
    }
}


/////////////////////////////////////////////////////////////////
//Finds the distance of the current telems of the two agents in question
double Simulator::get_distance_to_other_agent(vector<Policy>* sim_team, int sim_p, int sim_pp, double distance)
{
    double x;
    double x_mag;
    double y;
    double y_mag;
    double z;
    double z_mag;
    double r;
    
    /*
    cout << "agent" << "\t" << sim_p << endl;
    cout << "current telem" << endl;
    for (int i=0; i<3; i++)
    {
        cout << sim_team->at(sim_p).current_telem.at(i) << "\t";
    }
    cout << endl;
    cout << "agent" << "\t" << sim_pp << endl;
    cout << "current telem" << endl;
    for (int i=0; i<3; i++)
    {
        cout << sim_team->at(sim_pp).current_telem.at(i) << "\t";
    }
    cout << endl;
    */
    
    x = sim_team->at(sim_p).current_telem.at(0)-sim_team->at(sim_pp).current_telem.at(0);
    y = sim_team->at(sim_p).current_telem.at(1)-sim_team->at(sim_pp).current_telem.at(1);
    z = sim_team->at(sim_p).current_telem.at(2)-sim_team->at(sim_pp).current_telem.at(2);
    x_mag = x*x;
    y_mag = y*y;
    z_mag = z*z;
    r = x_mag+y_mag+z_mag;
    return r;
}


/////////////////////////////////////////////////////////////////
//runs the domino effect case
void Simulator::run_conflict_counter(vector<Policy>* sim_team, int sim_p, int sim_pp, vector<double>* pconflict_counter)
{
    //conflict case 0_0
    if (sim_team->at(sim_p).corp_id == 0)
    {
        if (sim_team->at(sim_pp).corp_id == 0)
        {
            pconflict_counter->at(0) += 1;
            //cout << "conflict counter 0_0" << "\t" << pconflict_counter->at(0) << endl;
        }
    }
    //conflict case 0_1
    if (sim_team->at(sim_p).corp_id == 0)
    {
        if (sim_team->at(sim_pp).corp_id == 1)
        {
            pconflict_counter->at(1) += 1;
            //cout << "conflict counter 0_1" << "\t" << pconflict_counter->at(1) << endl;
        }
    }
    //conflict case 1_0
    if (sim_team->at(sim_p).corp_id == 1)
    {
        if (sim_team->at(sim_pp).corp_id == 0)
        {
            pconflict_counter->at(2) += 1;
            //cout << "conflict counter 1_0" << "\t" << pconflict_counter->at(2) << endl;
        }
    }
    //conflict case 1_1
    if (sim_team->at(sim_p).corp_id == 1)
    {
        if (sim_team->at(sim_pp).corp_id == 1)
        {
            pconflict_counter->at(3) += 1;
            //cout << "conflict counter 1_1" << "\t" << pconflict_counter->at(3) << endl;
        }
    }
}


/////////////////////////////////////////////////////////////////
//runs a strictly cooperative case
void Simulator::run_cooperative_case(vector<Policy>* sim_team, int sim_p, int sim_pp)
{
    sim_team->at(sim_p).policy_fitness += 1;
}


/////////////////////////////////////////////////////////////////
//runs the domino effect case
void Simulator::run_domino_case(vector<Policy>* sim_team, int sim_p, int sim_pp)
{
    if (sim_team->at(sim_p).corp_id == 0)
    {
        sim_team->at(sim_p).policy_fitness += 1;
    }
    if (sim_team->at(sim_p).corp_id == 1)
    {
        sim_team->at(sim_p).policy_fitness += 1;
        //team 1 will recieve a penalty for all conflicts that are with their own team and the other team
        if (sim_team->at(sim_p).corp_id == sim_team->at(sim_pp).corp_id)
        {
            sim_team->at(0).policy_fitness = sim_team->at(0).policy_fitness - 1;
            /*
            for (int tt=0; tt<sim_team->size(); tt++)
            {
                //rewards all policies on team 0 if team 1 comflicts with team 1
                if (sim_team->at(tt).corp_id == 0)
                {
                    sim_team->at(tt).policy_fitness = sim_team->at(tt).policy_fitness - 1/pP->team_sizes.at(0);
                }
            }
            */
            //cout << "team 1 inner team conflict" << endl;
            //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
        }
        if (sim_team->at(sim_p).corp_id != sim_team->at(sim_pp).corp_id)
        {
            //cout << "team 1 conflict" << endl;
            //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
        }
    }
}


/////////////////////////////////////////////////////////////////
//runs the uncooperative case
void Simulator::run_uncoop_case(vector<Policy>* sim_team, int sim_p, int sim_pp)
{
    if (sim_team->at(sim_p).corp_id == 0)
    {
        if (sim_team->at(sim_p).corp_id == sim_team->at(sim_pp).corp_id)
        {
            sim_team->at(sim_p).policy_fitness += 1;
            //cout << "team 0 inner team conflict" << endl;
            //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
            //team 0 will only recieve a penalty if they conflict with in their own team
        }
        else
        {
            //cout << "team 0 conflict" << endl;
            //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
        }
    }
    if (sim_team->at(sim_p).corp_id == 1)
    {
        sim_team->at(sim_p).policy_fitness += 1;
        //team 1 will recieve a penalty for all conflicts that are with their own team and the other team
        if (sim_team->at(sim_p).corp_id == sim_team->at(sim_pp).corp_id)
        {
            //cout << "team 1 inner team conflict" << endl;
            //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
        }
        if (sim_team->at(sim_p).corp_id != sim_team->at(sim_pp).corp_id)
        {
            //cout << "team 1 conflict" << endl;
            //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
        }
    }
}


/////////////////////////////////////////////////////////////////
//runs the behavorial change case
void Simulator::run_behavorial_change_case(vector<Policy>* sim_team, int gen, int sim_p, int sim_pp)
{
    if (gen < pP->gen_max/2)
    {
        sim_team->at(sim_p).policy_fitness += 1;
    }
    if (gen >= pP->gen_max/2)
    {
        if (sim_team->at(sim_p).corp_id == 0)
        {
            if (sim_team->at(sim_p).corp_id == sim_team->at(sim_pp).corp_id)
            {
                sim_team->at(sim_p).policy_fitness += 1;
                //cout << "team 0 inner team conflict" << endl;
                //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
                //team 0 will only recieve a penalty if they conflict with in their own team
            }
            else
            {
                //cout << "team 0 conflict" << endl;
                //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
            }
        }
        if (sim_team->at(sim_p).corp_id == 1)
        {
            sim_team->at(sim_p).policy_fitness += 1;
            //team 1 will recieve a penalty for all conflicts that are with their own team and the other team
            if (sim_team->at(sim_p).corp_id == sim_team->at(sim_pp).corp_id)
            {
                //cout << "team 1 inner team conflict" << endl;
                //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
            }
            if (sim_team->at(sim_p).corp_id != sim_team->at(sim_pp).corp_id)
            {
                //cout << "team 1 conflict" << endl;
                //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//runs the behavorial domino change case
void Simulator::run_behaviroal_domino_case(vector<Policy>* sim_team, int sim_p, int sim_pp, int gen)
{
    if (gen < pP->gen_max/2)
    {
        sim_team->at(sim_p).policy_fitness += 1;
    }
    if (gen >= pP->gen_max/2)
    {
        if (sim_team->at(sim_p).corp_id == 0)
        {
            sim_team->at(sim_p).policy_fitness += 1;
        }
        if (sim_team->at(sim_p).corp_id == 1)
        {
            sim_team->at(sim_p).policy_fitness += 1;
            //team 1 will recieve a penalty for all conflicts that are with their own team and the other team
            if (sim_team->at(sim_p).corp_id == sim_team->at(sim_pp).corp_id)
            {
                sim_team->at(0).policy_fitness = sim_team->at(0).policy_fitness - 1;
                /*
                 for (int tt=0; tt<sim_team->size(); tt++)
                 {
                 //rewards all policies on team 0 if team 1 comflicts with team 1
                 if (sim_team->at(tt).corp_id == 0)
                 {
                 sim_team->at(tt).policy_fitness = sim_team->at(tt).policy_fitness - 1/pP->team_sizes.at(0);
                 }
                 }
                 */
                //cout << "team 1 inner team conflict" << endl;
                //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
            }
            if (sim_team->at(sim_p).corp_id != sim_team->at(sim_pp).corp_id)
            {
                //cout << "team 1 conflict" << endl;
                //cout << "agent" << "\t" << sim_p << "\t" << "CA acitvated by" << "\t" << "agent" << "\t" << sim_pp << endl;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//runs the malicious case
void Simulator::run_malicious_case(vector<Policy>* sim_team, int sim_p, int sim_pp)
{
    if (sim_team->at(sim_p).corp_id == 0)
    {
       //Do nothing
    }
    if (sim_team->at(sim_p).corp_id == 1)
    {
        sim_team->at(sim_p).policy_fitness += 1;
        sim_team->at(0).policy_fitness = sim_team->at(0).policy_fitness - 1;
    }
}


/////////////////////////////////////////////////////////////////
//runs the static full malicious case
void Simulator::run_static_full_malicious_case(vector<Policy>* sim_team, int sim_p, int sim_pp)
{
    if (sim_team->at(sim_p).corp_id == 0)
    {
        //Do nothing
    }
    if (sim_team->at(sim_p).corp_id == 1)
    {
        sim_team->at(sim_p).policy_fitness += 1;
        sim_team->at(0).policy_fitness = sim_team->at(0).policy_fitness - 1;
    }
}


/////////////////////////////////////////////////////////////////
//Checks For Possible Collisions
//checks the distance of each agents projected telemetry against one another
void Simulator::check_for_collisions(vector<Policy>* sim_team, int gen, vector<double>* pconflict_counter)
{
    //cout << sim_team->size() << endl;
    for (int sim_p=0; sim_p< sim_team->size(); sim_p++)
    {
        //only considers agent who have not reached their final destination
        if (sim_team->at(sim_p).target_waypoint < pP->num_waypoints + 2)
        {
            for (int sim_pp=0; sim_pp < sim_team->size(); sim_pp++)
            {
                //only considers agent who have not reached their final destination
                if (sim_team->at(sim_pp).target_waypoint < pP->num_waypoints + 2)
                {
                    if (sim_p!=sim_pp)
                    {
                        double distance = 0;
                        //get distance to other agent
                        distance = get_distance_to_other_agent(sim_team, sim_p, sim_pp, distance);
                        //cout << distance << endl;
                        //4*(2*pP->max_travel_dist+2*pP->ca_radius)
                        if (distance<=4*(2*pP->max_travel_dist+2*pP->ca_radius))
                        {
                            //cout << sim_p << "\t" << sim_pp << endl;
                            for (int kk=0; kk < pP->ca_inc+2; kk++)
                            {
                                compare_agents_projected_telem(sim_team, sim_p, sim_pp, kk);
                            }
                            if (sim_team->at(sim_p).current_travel_speed == pP->ca_flight_speed)
                            {
                                ////////////////////////////////////////////////////////////////////////
                                //coop no leniency
                                if (pP->coop_no_len == 1)
                                {
                                    run_cooperative_case(sim_team, sim_p, sim_pp);
                                }
                                //coop with leniency
                                if (pP->coop_with_len == 1)
                                {
                                    run_cooperative_case(sim_team, sim_p, sim_pp);
                                }
                                if (pP->stat_coop_with_loaded_wp_with_len == 1)
                                {
                                    run_cooperative_case(sim_team, sim_p, sim_pp);
                                }
                                //coop fair trial
                                if (pP->coop_fair == 1)
                                {
                                    run_cooperative_case(sim_team, sim_p, sim_pp);
                                }
                                //uncoop no leniency
                                if (pP->uncoop_no_len == 1)
                                {
                                    run_uncoop_case(sim_team, sim_p, sim_pp);
                                }
                                //uncoop with leniency
                                if (pP->uncoop_with_len == 1)
                                {
                                    run_uncoop_case(sim_team, sim_p, sim_pp);
                                }
                                if (pP->stat_uncoop_with_loaded_wp_with_len == 1)
                                {
                                    run_uncoop_case(sim_team, sim_p, sim_pp);
                                }
                                //uncoop behavioral switch with leniency
                                if (pP->uncoop_behavioral_switch_with_len == 1)
                                {
                                    run_behavorial_change_case(sim_team, gen, sim_p, sim_pp);
                                }
                                //domino effect with leniency
                                if (pP->domino_with_len == 1)
                                {
                                    run_domino_case(sim_team, sim_p, sim_pp);
                                }
                                if (pP->stat_domino_with_loaded_wp_with_len == 1)
                                {
                                    run_domino_case(sim_team, sim_p, sim_pp);
                                }
                                //domino behavioral switch with leniency
                                if (pP->domino_behavioral_switch_with_len == 1)
                                {
                                    run_behaviroal_domino_case(sim_team, sim_p, sim_pp, gen);
                                }
                                //malicious with leniency
                                if (pP->malicious_with_len == 1)
                                {
                                    run_malicious_case(sim_team, sim_p, sim_pp);
                                }
                                //static full malicious with leniency
                                if (pP->stat_full_malicious_with_len == 1)
                                {
                                    run_static_full_malicious_case(sim_team, sim_p, sim_pp);
                                }
                                if (pP->stat_full_malicious_with_loaded_wp_with_len == 1)
                                {
                                    run_static_full_malicious_case(sim_team, sim_p, sim_pp);
                                }
                                run_conflict_counter(sim_team, sim_p, sim_pp, pconflict_counter);
                            }
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                    continue;
                }
            }
        }
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
void Simulator::crash_avoidance(vector<Policy>* sim_team, int gen, vector<double>* pconflict_counter)
{
    for (int sim_p=0; sim_p< sim_team->size(); sim_p++)
    {
        //only considers agent who have not reached their final destination
        if (sim_team->at(sim_p).target_waypoint < pP->num_waypoints + 2)
        {
            sim_team->at(sim_p).current_travel_speed = pP->max_flight_velocity;
            sim_team->at(sim_p).projected_telem.empty();
            //gets the distance from the current telemetry to the target waypoint
            get_dist_to_target_waypoint(sim_team, sim_p);
            //gets the projected telemetry
            get_projected_telem(sim_team, sim_p);
            //cout << "cp1" << endl;
            //gets the incremented projected telemetry
            get_inc_projected_telem(sim_team, sim_p);
        }
    }
    check_for_collisions(sim_team, gen, pconflict_counter);
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
        
        //cout << "reached waypoint" << "\t" << sim_team->at(sim_p).target_waypoint << endl;
        
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
void Simulator::get_agent_CA_fitness(vector<Policy>* sim_team)
{
    for (int p=0; p<sim_team->size(); p++)
    {
        if (sim_team->at(p).target_waypoint < pP->num_waypoints + 2)
        {
            if (sim_team->at(p).current_travel_speed == pP->ca_flight_speed)
            {
                sim_team->at(p).policy_fitness += 1;
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
void Simulator::get_agent_destination_fitness(vector<Policy>* sim_team)
{
    for (int p=0; p<sim_team->size(); p++)
    {
        if (sim_team->at(p).current_telem.at(0) == sim_team->at(p).check_points.at(pP->num_waypoints+1).waypoint_telem.at(0))
        {
            if (sim_team->at(p).current_telem.at(1) == sim_team->at(p).check_points.at(pP->num_waypoints+1).waypoint_telem.at(1))
            {
                if (sim_team->at(p).current_telem.at(2) == sim_team->at(p).check_points.at(pP->num_waypoints+1).waypoint_telem.at(2))
                {
                    //cout << "sim agent" << "\t" << p << "\t" << "has reached its final destination" << endl;
                    sim_team->at(p).at_final_destination = 1;
                    continue;
                }
                else
                {
                    //cout << "sim agent" << "\t" << p << "\t" << "has not reached its final destination" << endl;
                    sim_team->at(p).policy_fitness = sim_team->at(p).policy_fitness + 10;
                    sim_team->at(p).at_final_destination = 0;
                }
            }
            else
            {
                //cout << "sim agent" << "\t" << p << "\t" << "has not reached its final destination" << endl;
                sim_team->at(p).policy_fitness = sim_team->at(p).policy_fitness + 10;
                sim_team->at(p).at_final_destination = 0;
            }
        }
        else
        {
            //cout << "sim agent" << "\t" << p << "\t" << "has not reached its final destination" << endl;
            sim_team->at(p).policy_fitness = sim_team->at(p).policy_fitness + 10;
            sim_team->at(p).at_final_destination = 0;
        }
    }
}



/////////////////////////////////////////////////////////////////
//Runs Entire Simulation
void Simulator::run_simulation(vector<Policy>* psim_team, int gen, vector<double>* pconflict_counter)
{
    //cout << gen << endl;
    for (int p=0; p<psim_team->size(); p++)
    {
        psim_team->at(p).policy_fitness = 0;
        psim_team->at(p).projected_telem.resize(3);
    }
    
    
    create_starting_flight_velocity(psim_team);
    //cout << sim_team->size() << endl;
    
    //double d = rand() % 10;
    
    //for (int p=0; p<psim_team->size(); p++)
    //{
    //cout << "in" << endl;
    //psim_team->at(p).policy_fitness = d;
    //}
    
    
    //runs the acutal simulation
    create_starting_flight_velocity(psim_team);
    set_initial_telem(psim_team);
    double current_time = 0;
    while (current_time < pP->time_max)
    {
        //checks for crash avoidance
        crash_avoidance(psim_team, gen, pconflict_counter);
        //cout << "current time" << "\t" << current_time << endl;
        for (int sim_p=0; sim_p<psim_team->size(); sim_p++)
        {
            //only considers agent who have not reached their final destination
            if (psim_team->at(sim_p).target_waypoint < pP->num_waypoints + 2)
            {
                /*
                 cout << "agent" << "\t" << sim_p << endl;
                 cout << "target waypoint" << "\t" << psim_team->at(sim_p).target_waypoint << endl;
                 
                 cout << "distance to target waypoint" << "\t" << psim_team->at(sim_p).dist_to_target_waypoint << endl;
                 cout << "current travel speed" << "\t" << psim_team->at(sim_p).current_travel_speed << endl;
                 cout << "current telem" << endl;
                 for (int ll=0; ll < 3; ll++)
                 {
                 cout << psim_team->at(sim_p).current_telem.at(ll) << "\t";
                 }
                 cout << endl;
                 cout << "projected telem" << endl;
                 for (int ll=0; ll < 3; ll++)
                 {
                 cout << psim_team->at(sim_p).projected_telem.at(ll) << "\t";
                 }
                 cout << endl;
                 */
                
                
                //gets the new telemetry based on the flight speed from the CA
                get_new_telem(psim_team, sim_p);
                //cout << system.at(ii).agents.at(jj).target_waypoint << endl;
                
                //runs a check to see if the agent has reached their target waypoint
                check_if_at_waypoint(psim_team, sim_p);
                
                /*
                 cout << "new telem" << endl;
                 for (int ll=0; ll < 3; ll++)
                 {
                 cout << psim_team->at(sim_p).current_telem.at(ll) << "\t";
                 }
                 cout << endl;
                 cout << "current travel speed" << "\t" << psim_team->at(sim_p).current_travel_speed << endl;
                 cout << endl;
                */
            }
            //cout << endl;
        }
        //cout << endl;
        //cout << "--------" << endl;
        current_time += pP->delta_t;
        //cout << "current time" << "\t" << current_time << endl;
        //cout << endl;
        //get_agent_CA_fitness(psim_team);
    }
    get_agent_destination_fitness(psim_team);
    /*
     for (int p=0; p<psim_team->size(); p++)
     {
     cout << "sim agent" << p << "\t" << "fitness" << "\t" << psim_team->at(p).policy_fitness << endl;
     }
     cout << endl;
     cout << endl;
     */
}

#endif /* Simulator_hpp */
