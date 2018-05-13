/*
 * Created By: Min Gyo Kim
 * Created On: May 12th 2018
 * Description: Class that provides helper functions for path-finder-test
 */

#ifndef PATHFINDING_IGVC_PATHFINDERTESTUTILS_H
#define PATHFINDING_IGVC_PATHFINDERTESTUTILS_H

#include <geometry_msgs/Pose.h>

class PathFinderTestUtils {
public:
    static geometry_msgs::Pose constructPose(double x, double y, double angle) {
        geometry_msgs::Pose origin;

        // set position of the origin
        geometry_msgs::Point position;
        position.x = x;
        position.y = y;
        position.z = 0.0;
        origin.position = position;

        // set orientation of the origin
        tf::Quaternion q;
        tf::Matrix3x3 rotationMatrix = tf::Matrix3x3();
        rotationMatrix.setEulerYPR(angle, 0.0, 0.0); // only set Z rotation since it's 2D
        rotationMatrix.getRotation(q);
        tf::quaternionTFToMsg(q, origin.orientation);

        return origin;
    }
};

#endif //PATHFINDING_IGVC_PATHFINDERTESTUTILS_H
