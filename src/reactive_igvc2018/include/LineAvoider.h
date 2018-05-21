//
// Created by robyn on 19/05/18.
//

#ifndef PROJECT_LINEAVOIDER_H
#define PROJECT_LINEAVOIDER_H


#include <geometry_msgs/Point.h>

#include "mapping_igvc/LineObstacle.h"

class LineAvoider{
public:
    LineAvoider(double distance_away_from_robot, double distance_away_from_line);

    // Required empty constructor
    LineAvoider();

    geometry_msgs::Point getTargetDestination(mapping_igvc::LineObstacle line);
private:

    mapping_igvc::LineObstacle reduceToFirstDegreePolynomial(mapping_igvc::LineObstacle line);
    geometry_msgs::Point getPerpendicularIntersection(mapping_igvc::LineObstacle line);

    double distance_away_from_robot;
    double distance_away_from_line;

};


#endif //PROJECT_LINEAVOIDER_H
