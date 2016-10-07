//
//  World.hpp
//  UTM_Monopoly_V_3
//
//  Created by Scott S Forer on 9/22/16.
//  Copyright © 2016 Scott S Forer. All rights reserved.
//

#ifndef World_hpp
#define World_hpp

#include <stdio.h>
#include "Team.hpp"
#include "Individual.hpp"
#include "Policy.hpp"
#include "Waypoint.hpp"


class World
{
    friend class Parameters;
    friend class Team;
    friend class Individual;
    friend class Waypoint;
    
protected:
    
    
public:
    vector<Team> corp;
    
    void create_population(int pop_size, int num_teams, vector<int> team_sizes, int num_polices, int num_waypoints, vector<double> waypoint_telem);
    void create_sarting_telem(int pop_size, int num_teams, int num_policies, vector<int> team_sizes, vector<double> waypoint_telm, int max_x_dim, int max_y_dim, int max_z_dim);
    void create_checkpoints(int pop_size, int num_teams, int num_policies, vector<int> team_sizes, vector<double> waypoint_tel, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    void create_target_telem(int pop_size, int num_teams, int num_policies, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
    void build_world(int pop_size, int num_teams, int num_policies, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim);
private:
    
    
};



/////////////////////////////////////////////////////////////////
//Creates Population for each simulation
//creates an instacne for each agent in each team in each simulation
void World::create_population(int pop_size, int num_teams, vector<int> team_sizes, int num_policies, int num_waypoints, vector<double> waypoint_telem)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        Team T;
        corp.push_back(T);
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            Individual I;
            corp.at(ii).agents.push_back(I);
            for (int pp=0;  pp < num_policies; pp++)
            {
                Policy P;
                corp.at(ii).agents.at(jj).policies.push_back(P);
                for (int ww=0; ww < num_waypoints; ww++)
                {
                    Waypoint W;
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.push_back(W);
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(ww).waypoint_telem.resize(3);
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Starting Coordinates
//sets the starting telemetry for each agent in each team
void World::create_sarting_telem(int pop_size, int num_teams, int num_policies, vector<int> team_sizes, vector<double> waypoint_telm, int max_x_dim, int max_y_dim, int max_z_dim)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj<team_sizes.at(ii); ii++)
        {
                double x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
                double y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
                //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
                //int x_waypoint = 2;
                //int y_waypoint = 4;
                double z_waypoint = 0;
                
                //comment out for test fucntions
                //{
                corp.at(ii).agents.at(jj).policies.at(0).check_points.at(0).waypoint_telem.at(0)=x_waypoint;
                corp.at(ii).agents.at(jj).policies.at(0).check_points.at(0).waypoint_telem.at(1)=y_waypoint;
                corp.at(ii).agents.at(jj).policies.at(0).check_points.at(0).waypoint_telem.at(2)=z_waypoint;
        }
    }
    
    //makes each starting telemetry for each policy for each agent the same
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            for (int pp=1;  pp < num_policies; pp++)
            {
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(0) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(0);
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(1) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(1);
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(2) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(0).waypoint_telem.at(2);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Intermediate Waypoints
//sets the telemetry for each intermediate waypoint for each agent in each team
void World::create_checkpoints(int pop_size, int num_teams, int num_policies, vector<int> team_sizes, vector<double> waypoint_tel, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            for (int pp=1;  pp < num_policies; pp++)
            {
                for (int ww=1; ww <num_waypoints+1; ww++)
                {
                    double x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
                    double y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
                    double z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
                    //int x_waypoint = 5;
                    //int y_waypoint = 6;
                    //int z_waypoint = 8;
                    
                    //comment out for test fucntions
                    //{
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(ww).waypoint_telem.at(0) = x_waypoint;
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(ww).waypoint_telem.at(1) = y_waypoint;
                    corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(ww).waypoint_telem.at(2) = z_waypoint;
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Create Target Coordinates
//sets the final telemetry for each agent in each team
void World::create_target_telem(int pop_size, int num_teams, int num_policies, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj<team_sizes.at(ii); ii++)
        {
            double x_waypoint = abs(((double)rand()/RAND_MAX)*max_x_dim);
            double y_waypoint = abs(((double)rand()/RAND_MAX)*max_y_dim);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 2;
            //int y_waypoint = 4;
            double z_waypoint = 0;
            
            //comment out for test fucntions
            //{
            corp.at(ii).agents.at(jj).policies.at(0).check_points.at(num_waypoints+1).waypoint_telem.at(0)=x_waypoint;
            corp.at(ii).agents.at(jj).policies.at(0).check_points.at(num_waypoints+1).waypoint_telem.at(1)=y_waypoint;
            corp.at(ii).agents.at(jj).policies.at(0).check_points.at(num_waypoints+1).waypoint_telem.at(2)=z_waypoint;
        }
    }
    
    //makes each ending telemetry for each policy for each agent the same
    for (int ii=0; ii < num_teams; ii++)
    {
        for (int jj=0; jj < team_sizes.at(ii); jj++)
        {
            for (int pp=1;  pp < num_policies; pp++)
            {
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(num_waypoints+1).waypoint_telem.at(0) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(num_waypoints+1).waypoint_telem.at(0);
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(num_waypoints+1).waypoint_telem.at(1) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(num_waypoints+1).waypoint_telem.at(1);
                corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(num_waypoints+1).waypoint_telem.at(2) = corp.at(ii).agents.at(jj).policies.at(pp).check_points.at(num_waypoints+1).waypoint_telem.at(2);
            }
        }
    }
}


void World::build_world(int pop_size, int num_teams, int num_policies, vector<int> team_sizes, vector<double> waypoint_telem, int num_waypoints, int max_x_dim, int max_y_dim, int max_z_dim)
{
    create_population(pop_size, num_teams, team_sizes, num_policies, num_waypoints, waypoint_telem);
    
}

#endif /* World_hpp */
