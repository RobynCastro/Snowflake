//
// Created by robyn on 19/05/18.
//

#ifndef PROJECT_LINEAVOIDER_H
#define PROJECT_LINEAVOIDER_H


#include <geometry_msgs/Point.h>

class LineAvoider{
public:
    LineAvoider(int todo/*TODO: Determine parameters*/);

    // Required empty constructor
    LineAvoider();

    geometry_msgs::Point getTargetDestination(int line);
private:



};


#endif //PROJECT_LINEAVOIDER_H
