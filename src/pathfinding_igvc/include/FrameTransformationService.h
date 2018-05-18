/*
 * Created By: Min Gyo Kim
 * Created On: May 13th 2018
 * Description: A service that transforms points between two frames,
 * specifically map and grid frame.
 */

#ifndef PATHFINDING_IGVC_FRAMETRANSFORMATIONSERVICE_H
#define PATHFINDING_IGVC_FRAMETRANSFORMATIONSERVICE_H

#include <PathFinderUtils.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

class FrameTransformationService {
    tf::Transform _transformation_to_grid;
    tf::Transform _transformation_to_map;

  public:
    /**
     * Returns a FrameTransformationService that transforms points
     * between grid frame and map frame
     *
     * Takes in the rotation/orientation and position of the grid frame relative
     * map frame
     *
     * @param rotation rotation/orientation of the grid frame relative expressed
     * map frame
     * @param position position of the grid frame expressed in map frame
     * @return FrameTransformationService
     */
    static FrameTransformationService buildService(tf::Quaternion rotation,
                                                   tf::Vector3 position);

    /**
     * Takes a point represented in map frame and returns its position in grid
     * frame
     *
     * @param point a point represented in map frame
     * @return point represented in grid frame
     */
    geometry_msgs::Point transformToGridFrame(geometry_msgs::Point point);

    /**
     * Takes a point represented in grid frame and returns its position in map
     * frame
     *
     * @param point a point represented in grid frame
     * @return point represented in map frame
     */
    geometry_msgs::Point transformToMapFrame(geometry_msgs::Point point);
};

#endif // PATHFINDING_IGVC_FRAMETRANSFORMATIONSERVICE_H
