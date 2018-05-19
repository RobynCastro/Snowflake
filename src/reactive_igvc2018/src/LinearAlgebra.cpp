/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Utilities to compute vector and point math
 *
 */
#include <LinearAlgebra.h>

LinearAlgebra::LinearAlgebra() {
    // Empty Constructor
}

double LinearAlgebra::distanceBetweenPoints(geometry_msgs::Point point1, geometry_msgs::Point point2) {
    // Euclidean distance
    return sqrt(pow(point1.y - point2.y, 2) + pow(point1.x - point2.x, 2));
}

geometry_msgs::Point LinearAlgebra::getMiddlePoint(geometry_msgs::Point point1, geometry_msgs::Point point2) {
    geometry_msgs::Point hole;
    hole.x = (point1.x + point2.x) / 2.0;
    hole.y = (point1.y + point2.y) / 2.0;

    return hole;
}

pair<geometry_msgs::Point, geometry_msgs::Point> LinearAlgebra::getClosestPair(
        vector<geometry_msgs::Point> points1, vector<geometry_msgs::Point> points2) {
    pair<geometry_msgs::Point, geometry_msgs::Point> closest_pair;

    double closest_distance = distanceBetweenPoints(points1[0], points2[0]);
    closest_pair.first = points1[0];
    closest_pair.second = points2[0];

    // Cross compare all points
    for (int i = 0; i < points1.size(); i++) {
        for (int j = 0; j < points2.size(); j++) {
            double cur_distance = distanceBetweenPoints(points1[i], points2[j]);
            // If distance between the current points is less than the current closest, update the closest pair
            if (cur_distance < closest_distance) {
                closest_distance = cur_distance;
                closest_pair.first = points1[i];
                closest_pair.second = points2[j];
            }
        }
    }

    return closest_pair;
}

double LinearAlgebra::getAngleToPoint(geometry_msgs::Point point) {
    if (point.x == 0)
        return 0;

    double angle = atan(point.y / point.x);

    if (point.x < 0)
        angle = -angle + (M_PI / 2.0) * (-fabs(angle) / angle);

    return angle;
}