//
// Created by robyn on 19/05/18.
//

#ifndef PROJECT_CONELINEAVOIDER_H
#define PROJECT_CONELINEAVOIDER_H

#include <geometry_msgs/Point.h>

class ConeLineAvoider{
public:
    ConeLineAvoider(int todo/*TODO: Determine parameters*/);

    // Required empty constructor
    ConeLineAvoider();

    geometry_msgs::Point getTargetDestination(std::vector<int> cones, int line);
private:



};
#endif //PROJECT_CONELINEAVOIDER_H
