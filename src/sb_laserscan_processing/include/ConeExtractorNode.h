/**
 * Created by William Gu on 24/03/18.
 * Class declaration for a Cone Extractor Node that identifies cones from a laser msg
 */

#ifndef LASERSCAN_CONE_MANAGER_H
#define LASERSCAN_CONE_MANAGER_H

#include <ConeIdentification.h>
#include <iostream>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/LaserScan.h>

class ConeExtractorNode {
    public:
        ConeExtractorNode(int argc, char** argv, std::string node_name);

    private:
        ros::Subscriber laser_subscriber;
        ros::Publisher cone_publisher;

        /**
         * Callback function for receiving laser scan msgs
         * @param ptr
         */
        void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& ptr);

        float cone_dist_tol; //Distance tolerance between cones in cluster
};

#endif //LASERSCAN_CONE_MANAGER_H
