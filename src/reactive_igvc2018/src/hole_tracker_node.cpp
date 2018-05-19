/*
 * Created By: Robyn Castro
 * Created On: October 28, 2017
 * Description: Decides how the robot should move
 *              to get through a hole in a line of
 *              cones
 */

#include <HoleTrackerNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "hole_tracker_node";
    // Create an instance of your class
    HoleTrackerNode hole_tracker_node(argc, argv, node_name);
    // Start up ROS, this will continue to run until the node is killed
    ros::spin();
    // Once the node stops, return 0
    return 0;
}