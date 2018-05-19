//
// Created by robyncastro on 28/10/17.
//

#ifndef HOLE_TRACKER_LIDARDECISION_H
#define HOLE_TRACKER_LIDARDECISION_H

// Messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// Obstacle Manager
#include <LidarObstacleManager.h>

// Utilities
#include <LinearAlgebra.h>

using namespace std;

class HoleTrackerDecision {
public:
    HoleTrackerDecision(double angular_vel_cap, double linear_vel_cap, double angular_vel_multiplier,
                        double linear_vel_multiplier, double theta_scaling_multiplier, double max_distance_from_goal);

    // Required empty constructor
    HoleTrackerDecision();

    /**
     *  Determines the motion in order to get to the hole.
     *
     *  @param merged_points points grouped based on proximity
     *  @param hole where the hole in the wall is
     *
     *  @return the twist motion
     */
    geometry_msgs::Twist determineDesiredMotion(vector<vector<geometry_msgs::Point>> merged_points,
                                                geometry_msgs::Point hole);

    /**
     *  Determines the turning velocity
     *
     *  @param theta the angle to turn
     *  @param distance the distance to the goal
     *  @return the turning velocity
     */
    double determineTurningVel(double theta, double distance);

    /**
     *  Determines the moving velocity
     *
     *  @param theta the angle to turn
     *  @param distance the distance to the goal
     *
     *  @return the moving velocity
     */
    double determineMovingVel(double theta, double distance);

    /**
     *  Set the components of the twist that we don't care about to 0.
     *
     *  @param twist the twist to be modified
     */
    void initTwist(geometry_msgs::Twist &twist);

private:
    geometry_msgs::Point origin;

    double angular_vel_cap;
    double linear_vel_cap;
    double linear_vel_multiplier;
    double angular_vel_multiplier;
    double theta_scaling_multiplier;
    double max_distance_from_goal;
};

#endif //HOLE_TRACKER_LIDARDECISION_H
