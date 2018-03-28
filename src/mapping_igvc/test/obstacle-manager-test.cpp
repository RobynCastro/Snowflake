/*
 * Created By: Gareth Ellis
 * Created On: January 9, 2018
 * Description: TODO
 */

// Snowbots Includes
#include "ObstacleManager.h"
#include "TestUtils.h"

// GTest Includes
#include <gtest/gtest.h>

using namespace sb_geom;

class ObstacleManagerTest : public testing::Test {
protected:
    ObstacleManagerTest():
            obstacle_manager_with_one_cone(0.5, 0)
            {}

    virtual void SetUp() {
        obstacle_manager_with_one_cone.addObstacle(Cone(0,0,0.2));
    }

    ObstacleManager obstacle_manager_with_one_cone;
};

TEST_F(ObstacleManagerTest, add_single_cone){
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();
    EXPECT_EQ(1, actual.size());
}

TEST_F(ObstacleManagerTest, add_cone_outside_of_merging_tolerance_positive_coordinates){
    // Expected obstacles are the current obstacles with our new one appended
    Cone cone2(0.4,0.31,0.4);
    std::vector<Cone> expected = obstacle_manager_with_one_cone.getConeObstacles();
    expected.push_back(cone2);

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(2, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_cone_outside_of_merging_tolerance_negative_coordinates){
    // Expected obstacles are the current obstacles with our new one appended
    Cone cone2(0.4,-0.31,0.4);
    std::vector<Cone> expected = obstacle_manager_with_one_cone.getConeObstacles();
    expected.push_back(cone2);

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(2, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_cone_within_of_merging_tolerance_positive_coordinates){
    // Since this cone is within merging tolerance, we expect that it will be
    // merged (which will overwrite the coordinates of the already present
    // cone with cone2)
    Cone cone2(0.4,0.29,0.4);
    std::vector<Cone> expected = {cone2};

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(1, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_cone_within_of_merging_tolerance_negative_coordinates){
    // Since this cone is within merging tolerance, we expect that it will be
    // merged (which will overwrite the coordinates of the already present
    // cone with cone2)
    Cone cone2(-0.4,-0.29,0.4);
    std::vector<Cone> expected = {cone2};

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(1, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_several_cones){
    ObstacleManager obstacle_manager(1,0);

    // A list of a cones, all within merging tolerance of each other
    std::vector<Cone> cones_within_merging_tolerance = {
            Cone(5,5,0.2),
            Cone(5.5,5,0.2),
            Cone(5.2,5.1,0.2),
            Cone(4.9,5.3,0.2)
    };

    // A list of cones, all far enough from each other to not be merged
    std::vector<Cone> cones_outside_merging_tolerance = {
            Cone(0,0,0.3),
            Cone(-10,-5,0.4),
            Cone(-200,400000,0.4),
            Cone(-10,100,0.4)
    };

    // Add all the cones, alternating between the two lists
    for (int i = 0; i < 4; i++){
        obstacle_manager.addObstacle(cones_within_merging_tolerance[i]);
        obstacle_manager.addObstacle(cones_outside_merging_tolerance[i]);
    }

    std::vector<Cone> expected;

    // We expect that all the cones within merging tolerance were merged,
    // with the final cone that was added taking priority over all prior
    expected.push_back(cones_within_merging_tolerance[3]);

    // We expect that all the cones that were greater then the merging tolerance
    // were not merged, and hence should all be present
    expected.insert(expected.end(),
                    cones_outside_merging_tolerance.begin(),
                    cones_outside_merging_tolerance.end());

    std::vector<Cone> actual = obstacle_manager.getConeObstacles();
    // TODO: Use googleMock to compare lists without ordering (gMock supported by catkin in 0.7.9 but not in ubuntu PPA yet)
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_single_line){
    ObstacleManager obstacle_manager(10, 10);

    Spline spline({{29,20}, {50,20}, {55, 20}});

    obstacle_manager.addObstacle(spline);

    std::vector<Spline> expected = {spline};
    EXPECT_EQ(expected, obstacle_manager.getLineObstacles());
}

// Test adding two lines to an ObstacleManager just outside of the tolerance
// needed to merge the two lines
TEST_F(ObstacleManagerTest, add_two_lines_just_outside_merging_tolerance){
    ObstacleManager obstacle_manager(1, 4.9);

    // These splines are basically two straight lines,
    // one at y=0, and one at y=5
    Spline spline1({{0,0}, {10,0}});
    Spline spline2({{2,5}, {7,5}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    std::vector<Spline> known_lines = obstacle_manager.getLineObstacles();
    std::vector<Spline> expected_lines = {spline1, spline2};
    EXPECT_EQ(expected_lines.size(), known_lines.size());
    EXPECT_EQ(expected_lines, known_lines);
}

// Test adding two lines to an ObstacleManager just within of the tolerance
// needed to merge the two lines
TEST_F(ObstacleManagerTest, add_two_lines_just_within_merging_tolerance){
    ObstacleManager obstacle_manager(1, 5.1);

    Spline spline1({{0,0}, {10,0}});
    Spline spline2({{2,5}, {7,5}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    // We expect that the second spline will be merged into the first. Since the closest
    // points to the endpoints of the second Spline on the first are in the middle section
    // of the first spline, we expect no interpolation points to be "cut" from the first
    // spline, and so the resulting spline should interpolate through all interpolation
    // points on both splines
    std::vector<Spline> expected_lines = { Spline({{0,0}, {2,5}, {7,5}, {10,0}}) };
    std::vector<Spline> known_lines = obstacle_manager.getLineObstacles();

    EXPECT_EQ(expected_lines.size(), known_lines.size());
    EXPECT_EQ(expected_lines, known_lines);
}

// Test merging two lines where the new line overlaps last point of the known line
TEST_F(ObstacleManagerTest, add_two_lines_second_line_overlapping_last_point_of_first_line){
    ObstacleManager obstacle_manager(1, 5);

    Spline spline1({{0,0}, {5,0}, {10,0}});
    Spline spline2({{7, 3}, {15, 4}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    // We expect a single spline which is basically `spline1` with it's last interpolation
    // point replaced by the interpolation points of `spline2`
    std::vector<Spline> expected_splines = {
            Spline({{0,0}, {5,0}, {7,3}, {15,4}}),
    };

    EXPECT_EQ(expected_splines, obstacle_manager.getLineObstacles());
}

// Test merging two lines where the new line overlaps first point of the known line
TEST_F(ObstacleManagerTest, add_two_lines_second_line_overlapping_first_point_of_first_line){
    ObstacleManager obstacle_manager(1, 5);

    Spline spline1({{0,0}, {5,0}, {10,0}});
    Spline spline2({{-5, 4}, {3, 4}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    // We expect a single spline which is basically `spline1` with it's last interpolation
    // point replaced by the interpolation points of `spline2`
    std::vector<Spline> expected_splines = {
            Spline({{-5,4}, {3,4}, {5,0}, {10,0}}),
    };

    EXPECT_EQ(expected_splines, obstacle_manager.getLineObstacles());
}

// Test merging two lines where the second totally overlaps the first
TEST_F(ObstacleManagerTest, add_two_lines_second_totally_overlaps_first){
    ObstacleManager obstacle_manager(1,4);

    Spline spline1({{0,0}, {5,3}, {10,0}});
    Spline spline2({{-1,1}, {5,3}, {11,1}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    std::vector<Spline> expected = {spline2};

    EXPECT_EQ(expected, obstacle_manager.getLineObstacles());
}

// Test merging a small line into a much longer and more complex one
TEST_F(ObstacleManagerTest, merge_simple_and_small_line_into_large_and_complex_one){
    ObstacleManager obstacle_manager(1,4);

    Spline spline1({{0,0}, {2,2}, {8,8}, {6,0}, {10,-6}});
    Spline spline2({{7, 9}, {10, 10}, {8,6.5}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    // We expect the 2nd spline to overwrite the point {8,8} on the first spline
    std::vector<Spline> expected = {
            Spline({{0,0}, {2,2}, {7, 9}, {10, 10}, {8,6.5}, {6,0}, {10,-6}})
    };

    EXPECT_EQ(expected, obstacle_manager.getLineObstacles());
}

// Add several overlapping lines, and a few ones that don't overlap
TEST_F(ObstacleManagerTest, add_variety_of_overlapping_and_not_overlapping_lines){
    ObstacleManager obstacle_manager(1,2);

    // A simple straight line from (0,0) to (10,0)
    Spline spline1({{0,0}, {5,0}, {10,0}});
    // An arc around (5,0) that should merge with `spline1`
    Spline spline2({{4, 1}, {5, 10}, {6,1}});
    // A horizontal line that should merge into the right side of the arc
    // from `spline2`
    Spline spline3({{7, 3}, {7, 8}});
    // A line outside of the merging tolerance of all known lines
    Spline spline4({{10,10}, {11,11}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);
    obstacle_manager.addObstacle(spline3);
    obstacle_manager.addObstacle(spline4);

    std::vector<Spline> expected = {
            Spline({{0,0}, {4,1}, {5,10}, {7,8}, {7,3}, {6,1}, {10,0}}),
            spline4,
    };

    EXPECT_EQ(expected, obstacle_manager.getLineObstacles());
}


// TODO: Delete me, not a real test
//TEST_F(ObstacleManagerTest, messing_about){
//    ObstacleManager obstacle_manager(10, 30);
//
//    //Spline spline1({{0,1}, {20,1}, {30,1}, {40,1}, {50,1}, {75,1}, {100,1}});
//    //Spline spline2({{29,20}, {50,20}, {55, 20}});
//    Spline spline1({{0,0}, {100,0}});
//    Spline spline2({{30,20}, {70,20}});
//
//    // TODO: YOU ARE HERE - seems like checking the distance between two splines is fast,
//    // TODO: but when we go to actually merge the splines and check point to spline distance, it's slow
//    //Spline spline1({{0,0}, {10, -10}, {10, 20}, {100,-100}});
//    //Spline spline2({{0,10}, {10, 20}, {10, 30}, {100,100}});
//
//    obstacle_manager.addObstacle(spline1);
//    obstacle_manager.addObstacle(spline2);
//
//    std::vector<sb_geom::Spline> lines = obstacle_manager.getLineObstacles();
//
//    for (sb_geom::Spline& spline : lines){
//        std::cout << "~~~~~~~~~~~~~" << std::endl;
//        int num_points = 100;
//        for (int i = 0; i < num_points; i++){
//            double u = i * 1.0/(double)num_points;
//            sb_geom::Point2D p = spline(u);
//            std::cout << u << ", " << p.x() << ", " << p.y() << std::endl;
//        }
//    }
//}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}