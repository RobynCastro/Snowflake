//
// Created by robyn on 19/05/18.
//

#include <ConeAvoider.h>

using namespace std;
using namespace mapping_igvc;

ConeAvoider::ConeAvoider (float robot_width):
                robot_width(robot_width) {}

ConeAvoider::ConeAvoider() {}

// Sort the vectors based on size
bool sortMerges(vector<ConeObstacle> a, vector<ConeObstacle> b) {
    return a.size() > b.size();
}

// Sort the cones based on the y-axis
bool sortConesY(ConeObstacle a, ConeObstacle b) {
    return a.center.y < b.center.y;
}

geometry_msgs::Point ConeAvoider::getTargetDestination(vector<ConeObstacle> cones) {
    geometry_msgs::Point destination;

    vector<vector<ConeObstacle> > merged_cones = mergeCones(cones);
    sort(merged_cones.begin(), merged_cones.end(), sortMerges);

    float distance_from_cone = 5;

    for (int j = 0; j < merged_cones.size(); j++) {
        if (merged_cones[j][0].center.y < robot_width / 2) {
            sort(merged_cones[j].begin(), merged_cones[j].end(), sortConesY);
            ConeObstacle cone_to_avoid;
            if(abs(merged_cones[j][0].center.y) < abs(merged_cones[j][0].center.y)) {
                cone_to_avoid = merged_cones[j][0];
            } else {
                cone_to_avoid = merged_cones[j][merged_cones.size() - 1];
            }
            destination.x = cone_to_avoid.center.x;
            destination.y = cone_to_avoid.center.y + (distance_from_cone + cone_to_avoid.radius)
                                                          * cone_to_avoid.center.y
                                                          / abs(cone_to_avoid.center.y);
            return destination;
        }
    }

    // Go straight if no blockade found
    destination.x = 3;
    destination.y = 0;

    return destination;
}

// Sort the cones based on distance to y-axis
bool sortConesYAbsolute(ConeObstacle a, ConeObstacle b) {
    return abs(a.center.y) < abs(b.center.y);
}

vector<vector<ConeObstacle> > ConeAvoider::mergeCones(vector<ConeObstacle> cones) {

    vector<vector<ConeObstacle> > merged_cones;
    for (int i = 0; i < cones.size(); i++) {
        bool found_match = findMatch(merged_cones, cones[i]);

        // If didn't find a match start a new merge list.
        if (!found_match) {
            vector<ConeObstacle> new_merge;
            new_merge.push_back(cones[i]);
            merged_cones.push_back(new_merge);
        }
    }

    for (int j = 0; j < merged_cones.size(); j++) {
        // Sort cones based on distance from y-axis
        sort(merged_cones[j].begin(), merged_cones[j].end(), sortConesYAbsolute);
    }

    return merged_cones;
}

bool ConeAvoider::findMatch(vector<vector<ConeObstacle> > &merged_cones,
                                     ConeObstacle cone) {
    geometry_msgs::Point cone_center;
    cone_center.x = cone.center.x;
    cone_center.y = cone_center.y;

    for (int j = 0; j < merged_cones.size(); j++) {
        for (int k = 0; k < merged_cones[j].size(); k++) {
            geometry_msgs::Point cone_to_compare_center;
            cone_to_compare_center.x = merged_cones[j][k].center.x;
            cone_to_compare_center.y = merged_cones[j][k].center.y;

            double cone_grouping_tolerance = cone.radius + merged_cones[j][k].radius + robot_width;
            if (LinearAlgebra().distanceBetweenPoints(cone_center, cone_to_compare_center) < cone_grouping_tolerance) {
                merged_cones[j].push_back(cone);
                return true;
            }
        }
    }
    return false;
}



