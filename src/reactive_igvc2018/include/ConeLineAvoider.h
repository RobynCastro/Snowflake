//
// Created by robyn on 19/05/18.
//

#ifndef PROJECT_CONELINEAVOIDER_H
#define PROJECT_CONELINEAVOIDER_H

#include <geometry_msgs/Point.h>
#include "mapping_igvc/ConeObstacle.h"
#include "mapping_igvc/LineObstacle.h"

class ConeLineAvoider{
public:
    ConeLineAvoider(int todo/*TODO: Determine parameters*/);

    // Required empty constructor
    ConeLineAvoider();

    geometry_msgs::Point getTargetDestination(std::vector<mapping_igvc::ConeObstacle> cones,
                                              mapping_igvc::LineObstacle line);
private:



};
#endif //PROJECT_CONELINEAVOIDER_H
