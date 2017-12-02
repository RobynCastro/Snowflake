/*
 * Created by: Robyn Castro
 * Created On: November 25, 2017
 * Description: Takes in a point cloud and transforms it.
 *
 */

#ifndef VISION_PCL_FILTER_H
#define VISION_PCL_FILTER_H

// ROS
#include <ros/ros.h>

// PCL
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// Snowbots
#include <sb_utils.h>


class HSVFilterNode {
public:
    /**
     * Constructor
     */
    HSVFilterNode(int argc, char** argv, std::string node_name);

private:
    /**
     * Callback for whenever a point cloud is received
     *
     * @param address of filtered image matrix
     */
    void pclCallBack(const sensor_msgs::PointCloud2);

};

#endif
