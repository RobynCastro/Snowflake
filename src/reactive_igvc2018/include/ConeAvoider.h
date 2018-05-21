//
// Created by robyn on 19/05/18.
//

#ifndef PROJECT_CONEAVOIDER_H
#define PROJECT_CONEAVOIDER_H

#include <geometry_msgs/Point.h>
#include "mapping_igvc/ConeObstacle.h"

#include "LinearAlgebra.h"

class ConeAvoider{
public:
    ConeAvoider(float robot_width);

    // Required empty constructor
    ConeAvoider();

    geometry_msgs::Point getTargetDestination(std::vector<mapping_igvc::ConeObstacle> cones);
private:

    std::vector<std::vector<mapping_igvc::ConeObstacle> > mergeCones(std::vector<mapping_igvc::ConeObstacle> cones);

    bool findMatch(std::vector<std::vector<mapping_igvc::ConeObstacle> > &merged_cones,
                   mapping_igvc::ConeObstacle cone);

    float robot_width;
};
#endif //PROJECT_CONEAVOIDER_H
