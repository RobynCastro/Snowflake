/*
 * Created By: Robyn Castro
 * Created On: October 28, 2017
 * Description: Decides how the robot should move
 *              to get through a hole in a line of
 *              cones
 */

#include <ReactiveDecisionNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "reactive_decision_node";
    // Create an instance of your class
    ReactiveDecisionNode hole_tracker_node(argc, argv, node_name);
    // Start up ROS, this will continue to run until the node is killed
    ros::spin();
    // Once the node stops, return 0
    return 0;
}