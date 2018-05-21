//
// Created by robyn on 19/05/18.
//

#include "LineAvoider.h"

using namespace std;
using namespace mapping_igvc;

LineAvoider::LineAvoider(double distance_away_from_robot, double distance_away_from_line):
                        distance_away_from_robot(distance_away_from_robot),
                        distance_away_from_line(distance_away_from_line){}

LineAvoider::LineAvoider() {}

geometry_msgs::Point LineAvoider::getTargetDestination(LineObstacle line) {
    geometry_msgs::Point destination;

    LineObstacle linear_polynomial = reduceToFirstDegreePolynomial(line);

    // Get parallel line closer to the middle of the course.
    LineObstacle target_parallel_line;
    double parallel_y_intercept = linear_polynomial.coefficients[0] - distance_away_from_line
                                                                      * linear_polynomial.coefficients[0]
                                                                      / fabs(linear_polynomial.coefficients[0]);

    target_parallel_line.coefficients.push_back(parallel_y_intercept); // Set the y-intercept
    target_parallel_line.coefficients.push_back(linear_polynomial.coefficients[1]); // Set the slope

    geometry_msgs::Point perpendicular_intersection = getPerpendicularIntersection(target_parallel_line);
    double distance_to_perpendicular_intersection = sqrt(
            pow(perpendicular_intersection.x, 2) + pow(perpendicular_intersection.y, 2));

    // cos(theta) = A/H
    double scale_projection_angle = acos(distance_to_perpendicular_intersection / distance_away_from_robot);

    // tan(theta) = O/A
    double angle_perpendicular_intersection = atan(perpendicular_intersection.x / perpendicular_intersection.y);

    // angle_perpendicular_intersection + angle_heading = scalar_projection_angle
    double angle_heading = scale_projection_angle - angle_perpendicular_intersection;

    // Polar coordinates
    destination.x = distance_away_from_robot * cos(angle_heading);
    destination.y = distance_away_from_robot * sin(angle_heading);

    return destination;
}

geometry_msgs::Point LineAvoider::getPerpendicularIntersection(mapping_igvc::LineObstacle line) {

    geometry_msgs::Point perpendicular_intersection;

    double neg_reciprocal = -1.0 / line.coefficients[1];

    // Set the two sides equal then isolate x to one side.
    double isolated_x_slope = neg_reciprocal - line.coefficients[1];

    // Divide both sides by the isolated slope to get the x point intersection.
    perpendicular_intersection.x = line.coefficients[0] / isolated_x_slope;

    // Plug in the xIntersection to get the y point intersection.
    perpendicular_intersection.y = neg_reciprocal * perpendicular_intersection.x;

    return perpendicular_intersection;
}

LineObstacle LineAvoider::reduceToFirstDegreePolynomial(mapping_igvc::LineObstacle line) {
    geometry_msgs::Point point_min;
    geometry_msgs::Point point_max;

    point_min.x = line.x_min;
    point_max.x = line.x_max;

    point_min.x = 0;
    point_max.x = 0;

    for (int i = 0; i < line.coefficients.size(); i++) {
        point_min.y += line.coefficients[i]*pow(point_min.x, i);
        point_max.y += line.coefficients[i]*pow(point_max.x, i);
    }

    double slope = (point_max.y - point_min.y) / (point_max.x - point_min.x);
    double y_intercept = point_max.y - slope*point_max.x;

    LineObstacle first_degree_polynomial;
    first_degree_polynomial.x_min = line.x_min;
    first_degree_polynomial.x_max = line.x_max;

    first_degree_polynomial.coefficients.push_back(y_intercept);
    first_degree_polynomial.coefficients.push_back(slope);

    return first_degree_polynomial;

}