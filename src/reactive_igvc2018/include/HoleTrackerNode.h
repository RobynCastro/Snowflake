//
// Created by robyncastro on 10/11/17.
//

#ifndef HOLE_TRACKER_HOLETRACKERNODE_H
#define HOLE_TRACKER_HOLETRACKERNODE_H

// Messages
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

// Obstacle
#include <ObstacleManager.h>

// Motion
#include <HoleTrackerDecision.h>

// ROS
#include <ros/ros.h>

// Utilities
#include <sb_utils.h>
#include "mapping_igvc/LineObstacle.h"
#include "mapping_igvc/ConeObstacle.h"

// Avoiders
#include "ConeAvoider.h"
#include "LineAvoider.h"
#include "ConeLineAvoider.h"

using namespace std;
class HoleTrackerNode {

public:
    HoleTrackerNode(int argc, char **argv, string node_name);

private:
    /**
     *  Callback function for when a new array of cones is received
     *
     *  @param cones the array of cones received in the callback
     */
    void coneMessageCallBack(mapping_igvc::ConeObstacle coneObstacle);

    /**
     *  Callback function for when a new polynomial representing line
     *  boundaries is received.
     *
     *  @param line the line boundary received in the callback
     */
    void lineMessageCallBack(mapping_igvc::LineObstacle lineObstacle);

    /**
     *
     */
    void updateTargetDestination(std::vector<mapping_igvc::ConeObstacle> new_cones, mapping_igvc::LineObstacle new_line, bool updateCones, bool updateLines);

    /**
     *  Publishes the twist.
     *
     *  @param msg_to_publish the string to publish
     */
    void publishTwist(geometry_msgs::Twist twist_msg);

    /**
     *  Setup the decision parameters.
     *
     *  @param private_nh the private node handle
     */
    void initDecisionParams(ros::NodeHandle private_nh);

    /**
     *  Setup the obstacle manager parameters.
     *
     *  @param private_nh the private node handle
     */
    void initObstacleManagerParams(ros::NodeHandle private_nh);

    /**
     *  Setup the subscribers.
     *
     *  @param nh the node handle
     */
    void initSubscribers(ros::NodeHandle nh);

    /**
     *  Setup the publishers.
     *
     *  @param nh the node handle
     */
    void initPublishers(ros::NodeHandle nh);

    // Decision parameters
    double angular_vel_cap;
    double linear_vel_cap;
    double linear_vel_multiplier;
    double angular_vel_multiplier;
    double theta_scaling_multiplier;
    double max_distance_from_goal;

    // Obstacle Manager parameters
    /*TODO: */

    // Obstacles
    std::vector<mapping_igvc::ConeObstacle> cones;
    mapping_igvc::LineObstacle line;

    // Obstacle Subscribers
    ros::Subscriber cone_message_subscriber;
    ros::Subscriber line_message_subscriber;

    // Motion Publisher
    ros::Publisher twist_publisher;

    // Debug Publishers
    /*TODO:*/

    // Controller
    HoleTrackerDecision hole_tracker_decision;

    // Avoiders

    ConeAvoider cone_avoider;
    LineAvoider line_avoider;
    ConeLineAvoider cone_line_avoider;

    // Obstacle Manager
    ObstacleManager obstacle_manager;

};


#endif //HOLE_TRACKER_HOLETRACKERNODE_H
