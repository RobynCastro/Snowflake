//
// Created by robyncastro on 10/11/17.
//

#ifndef REACTIVE_IGVC_REACTIVEDECISIONNODE_H
#define REACTIVE_IGVC_REACTIVEDECISIONNODE_H

// Messages
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include "mapping_igvc/LineObstacle.h"
#include "mapping_igvc/ConeObstacle.h"

// Motion
#include <MotionController.h>

// ROS
#include <ros/ros.h>

// Utilities
#include <sb_utils.h>
#include <RvizUtils.h>

// Avoiders
#include "ConeAvoider.h"
#include "LineAvoider.h"
#include "ConeLineAvoider.h"

using namespace std;
class ReactiveDecisionNode {

public:
    ReactiveDecisionNode(int argc, char **argv, string node_name);

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

    void gpsCallBack(const geometry_msgs::Twist::ConstPtr& gps_decision);

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

    // Robot Parameters
    double robot_width;

    // Destination Parameters
    double distance_away_from_robot;
    double distance_away_from_line;

    // Obstacles
    std::vector<mapping_igvc::ConeObstacle> cones;
    mapping_igvc::LineObstacle line;

    // Obstacle Subscribers
    ros::Subscriber cone_message_subscriber;
    ros::Subscriber line_message_subscriber;

    // GPS subscriber
    ros::Subscriber gps_subscriber;
    geometry_msgs::Twist recent_gps;

    // Motion Publisher
    ros::Publisher twist_publisher;

    // Debug Publishers
    ros::Publisher destination_debug_publisher;

    // Controller
    MotionController hole_tracker_decision;

    // Avoiders
    ConeAvoider cone_avoider;
    LineAvoider line_avoider;
    ConeLineAvoider cone_line_avoider;

};


#endif // REACTIVE_IGVC_REACTIVEDECISIONNODE_H
