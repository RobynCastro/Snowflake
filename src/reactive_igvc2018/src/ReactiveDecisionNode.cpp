/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Takes in a laser scan and publishes a twist, and visualisation messages.
 *
 */
#include <ReactiveDecisionNode.h>

using namespace std;
using namespace mapping_igvc;
using namespace snowbots;

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
    vector<ConeObstacle> cones;
    LineObstacle line;
    updateTargetDestination(cones, line, false, false);
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
    destination.x = 0;
    destination.y = 0;
    destination.z = 0;
    
    
    // Update any obstacles
    if(updateCones)
        cones = new_cones;
    if(updateLines)
        line = new_line;

    geometry_msgs::Twist desired_motion;

    std::vector<std_msgs::ColorRGBA> colors;
    std_msgs::ColorRGBA color;
    color.a = 1.0f;

    string frame_id = "base_link";
    string ns = "debug";

    // Determine destination and accompanying twist message.
    if (cones.size() > 0) {
        destination = cone_avoider.getTargetDestination(cones);
        desired_motion = hole_tracker_decision.determineDesiredMotion(destination);

        // Red Marker
        color.r = 1.0f;
        color.g = 0;
        color.b = 0;
    } else if (new_line.coefficients.size() > 0) {
        destination = line_avoider.getTargetDestination(line);
        desired_motion = hole_tracker_decision.determineDesiredMotion(destination);

        // Blue Marker
        color.r = 0;
        color.g = 0;
        color.b = 1.0f;
    } else {
        desired_motion = recent_gps;

        // Green Marker
        color.r = 0;
        color.g = 1.0f;
        color.b = 0;
    }

    colors.push_back(color);

    visualization_msgs::Marker::_scale_type scale = RvizUtils::createrMarkerScale(0.1, 0.1, 0.1);
    destination_debug_publisher.publish(RvizUtils::createMarker(destination, colors, scale, frame_id, ns));

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
    string topic_to_subscribe_to = "/cone_extractor_node/output_cone_obstacle";
    uint32_t refresh_rate = 10;
    cone_message_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &ReactiveDecisionNode::coneMessageCallBack,
                                         this);

    string line_extractor_topic = "/line_extractor_node/output_line_obstacle";
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

    std::string hole_debug_topic = private_nh.resolveName("debug/destination");
    destination_debug_publisher = private_nh.advertise<visualization_msgs::Marker>(hole_debug_topic, queue_size);
}
