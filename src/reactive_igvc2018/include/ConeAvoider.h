//
// Created by robyn on 19/05/18.
//

#ifndef PROJECT_CONEAVOIDER_H
#define PROJECT_CONEAVOIDER_H

#include <geometry_msgs/Point.h>

class ConeAvoider{
public:
    ConeAvoider(int todo/*TODO: Determine parameters*/);

    // Required empty constructor
    ConeAvoider();

    geometry_msgs::Point getTargetDestination(std::vector<int> cones);
private:



};
#endif //PROJECT_CONEAVOIDER_H
