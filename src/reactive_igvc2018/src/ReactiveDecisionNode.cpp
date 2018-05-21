/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Takes in a laser scan and publishes a twist, and visualisation messages.
 *
 */
#include <ReactiveDecisionNode.h>

using namespace std;
using namespace mapping_igvc;

ReactiveDecisionNode::ReactiveDecisionNode(int argc, char **argv, string node_name) {

    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    initDecisionParams(private_nh);

    hole_tracker_decision = MotionController(angular_vel_cap, linear_vel_cap, angular_vel_multiplier,
                                                linear_vel_multiplier,
                                                theta_scaling_multiplier, max_distance_from_goal);
    cone_avoider = ConeAvoider(robot_width);

    line_avoider = LineAvoider(distance_away_from_robot, distance_away_from_line);

    initSubscribers(nh);
    initPublishers(nh);

}

void ReactiveDecisionNode::gpsCallBack(const geometry_msgs::Twist::ConstPtr &gps_decision) {
    recent_gps = *gps_decision;
}

void ReactiveDecisionNode::coneMessageCallBack(ConeObstacle coneObstacle) {
    vector<ConeObstacle> cones;
    cones.push_back(coneObstacle);
    updateTargetDestination(cones, line, true, false);
}

void ReactiveDecisionNode::lineMessageCallBack(LineObstacle lineObstacle) {
    updateTargetDestination(cones, lineObstacle, false, true);
}

void ReactiveDecisionNode::updateTargetDestination(vector<ConeObstacle> new_cones, LineObstacle new_line, bool updateCones, bool updateLines) {
    geometry_msgs::Point destination;

    // Update any obstacles
    if(updateCones)
        cones = new_cones;
    if(updateLines)
        line = new_line;

    geometry_msgs::Twist desired_motion;

    // Determine destination and accompanying twist message.
    if (cones.size() > 0) {
        destination = cone_avoider.getTargetDestination(cones);
        desired_motion = hole_tracker_decision.determineDesiredMotion(destination);
    } else if (new_line.header.stamp.isValid()) {
        destination = line_avoider.getTargetDestination(line);
        desired_motion = hole_tracker_decision.determineDesiredMotion(destination);
    } else {
        desired_motion = recent_gps;
    }

    // Move towards desired destination
    publishTwist(desired_motion);

}
void ReactiveDecisionNode::publishTwist(geometry_msgs::Twist twist_msg) {
    twist_publisher.publish(twist_msg);
}

void ReactiveDecisionNode::initDecisionParams(ros::NodeHandle private_nh) {
    // ConeAvoider Parameters
    SB_getParam(private_nh, "robot_width", robot_width, 1.0);

    // LineAvoider Parameters
    SB_getParam(private_nh, "distance_away_from_robot", distance_away_from_robot, 3.0);
    SB_getParam(private_nh, "distance_away_from_line", distance_away_from_line, 0.5);

    // Velocity Parameters
    SB_getParam(private_nh, "angular_vel_cap", angular_vel_cap, 1.0);
    SB_getParam(private_nh, "linear_vel_cap", linear_vel_cap, 1.0);
    SB_getParam(private_nh, "angular_vel_multiplier", angular_vel_multiplier, 1.0);
    SB_getParam(private_nh, "linear_vel_multiplier", linear_vel_multiplier, 1.0);
    SB_getParam(private_nh, "theta_scaling_multiplier", theta_scaling_multiplier, 1.0);
    SB_getParam(private_nh, "max_distance_from_goal", max_distance_from_goal, 1.0);
}

void ReactiveDecisionNode::initSubscribers(ros::NodeHandle nh) {
    string topic_to_subscribe_to = "/output_cone_obstacle";
    uint32_t refresh_rate = 10;
    cone_message_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &ReactiveDecisionNode::coneMessageCallBack,
                                         this);

    string line_extractor_topic = "/output_line_obstacle";
    line_message_subscriber = nh.subscribe(line_extractor_topic, refresh_rate, &ReactiveDecisionNode::lineMessageCallBack,
                                            this);

    string gps_decision_topic_name    = "/gps_decision/twist";
    gps_subscriber = nh.subscribe(
            gps_decision_topic_name, refresh_rate, &ReactiveDecisionNode::gpsCallBack, this);
}

void ReactiveDecisionNode::initPublishers(ros::NodeHandle private_nh) {
    uint32_t queue_size = 1;
    string topic = private_nh.resolveName("cmd_vel");
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size);
}