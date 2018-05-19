/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Takes in a laser scan and publishes a twist, and visualisation messages.
 *
 */
#include <HoleTrackerNode.h>

HoleTrackerNode::HoleTrackerNode(int argc, char **argv, std::string node_name) {

    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    initDecisionParams(private_nh);
    initObstacleManagerParams(private_nh);
    initSubscribers(nh);
    initPublishers(nh);

    hole_tracker_decision = HoleTrackerDecision(angular_vel_cap, linear_vel_cap, angular_vel_multiplier,
                                                linear_vel_multiplier,
                                                theta_scaling_multiplier, max_distance_from_goal);

}

void HoleTrackerNode::coneMessageCallBack(/*TODO*/) {
    std::vector<int> cones;
    int line;
    updateTargetDestination(cones, line, true, false);
}

void HoleTrackerNode::lineMessageCallBack(/*TODO*/) {
    std::vector<int> cones;
    int line;
    updateTargetDestination(cones, line, false, true);
}

void HoleTrackerNode::updateTargetDestination(std::vector<int> new_cones, int new_line, bool updateCones, bool updateLines) {
    geometry_msgs::Point destination;

    // Update any obstacles
    if(updateCones)
        cones = new_cones;
    if(updateLines)
        line = new_line;

    // Determine Destination
    if (cones.size() == 0 && !new_line) {
        /*TODO: Aim for gps waypoint*/
    } else if (cones.size() > 0 && !new_line) {
        destination = cone_avoider.getTargetDestination(cones);
    } else if (cones.size() == 0 && new_line) {
        destination = line_avoider.getTargetDestination(line);
    } else {
        destination = cone_line_avoider.getTargetDestination(cones, line);
    }

    // Get desired motion
    geometry_msgs::Twist desired_motion = hole_tracker_decision.determineDesiredMotion(destination);

    // Move towards desired destination
    twist_publisher.publish(desired_motion);

}
void HoleTrackerNode::publishTwist(geometry_msgs::Twist twist_msg) {
    twist_publisher.publish(twist_msg);
}

void HoleTrackerNode::initDecisionParams(ros::NodeHandle private_nh) {
    SB_getParam(private_nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(private_nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(private_nh, "angular_vel_multiplier", angular_vel_multiplier, 1.0);
    SB_getParam(private_nh, "linear_vel_multiplier", linear_vel_multiplier, 1.0);
    SB_getParam(private_nh, "theta_scaling_multiplier", theta_scaling_multiplier, 1.0);
    SB_getParam(private_nh, "max_distance_from_goal", max_distance_from_goal, 1.0);
}

void HoleTrackerNode::initObstacleManagerParams(ros::NodeHandle private_nh) {
}

void HoleTrackerNode::initSubscribers(ros::NodeHandle nh) {
//    std::string topic_to_subscribe_to = "/scan";
//    uint32_t refresh_rate = 10;
//    cone_message_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &HoleTrackerNode::coneMessageCallBack,
//                                         this);
}

void HoleTrackerNode::initPublishers(ros::NodeHandle private_nh) {
    uint32_t queue_size = 1;
    std::string topic = private_nh.resolveName("cmd_vel");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}