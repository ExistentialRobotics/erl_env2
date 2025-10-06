#include "erl_common/test_helper.hpp"
#include "erl_env/scene_graph.hpp"

#include <filesystem>

TEST(ERL_ENV, SceneGraph) {
    GTEST_PREPARE_OUTPUT_DIR();
    std::filesystem::path path = gtest_src_dir / "building.yaml";

    erl::env::scene_graph::Building building;
    ASSERT_TRUE(building.FromYamlFile(path.string()));

    EXPECT_EQ(building.uuid, 1);
    EXPECT_EQ(building.id, 4);
    EXPECT_EQ(building.parent_id, -1);
    EXPECT_EQ(building.parent_uuid, -1);
    EXPECT_EQ(building.type, erl::env::scene_graph::Node::Type::kBuilding);
    EXPECT_EQ(building.name, "Benevolence");
    EXPECT_EQ(building.floors.size(), 3);
    EXPECT_EQ(building.num_floors, 3);
    ASSERT_EIGEN_VECTOR_EQUAL(
        "building.reference_point",
        building.reference_point,
        Eigen::Vector3f(0, 0, 0));
    ASSERT_EIGEN_VECTOR_NEAR(
        "building.size",
        building.size,
        Eigen::Vector3f(5.580856967034837, 10.439912021678497, 8.416298157597547),
        1e-6);

    auto floor = building.floors[0];
    EXPECT_EQ(floor->uuid, 3);
    EXPECT_EQ(floor->id, 0);
    EXPECT_EQ(floor->parent_id, 4);
    EXPECT_EQ(floor->parent_uuid, 1);
    EXPECT_EQ(floor->type, erl::env::scene_graph::Node::Type::kFloor);
    EXPECT_EQ(floor->name, "A");
    EXPECT_EQ(floor->rooms.size(), 5);
    EXPECT_EQ(floor->num_rooms, 5);
    EXPECT_EQ(floor->down_stairs_id, -1);
    EXPECT_EQ(floor->up_stairs_id, 14);
    EXPECT_EQ(floor->down_stairs_uuid, -1);
    EXPECT_EQ(floor->up_stairs_uuid, 18);
    EXPECT_EQ(floor->down_stairs_cost, std::numeric_limits<double>::infinity());
    EXPECT_EQ(floor->up_stairs_cost, 4.48469257f);
    EXPECT_EQ(floor->down_stairs_portal, std::nullopt);
    EXPECT_EQ(floor->up_stairs_portal.value()[0], 73);
    EXPECT_EQ(floor->up_stairs_portal.value()[1], 745);
    EXPECT_EQ(floor->room_map, "room_maps/0.png");
    EXPECT_EQ(floor->cat_map, "cat_maps/0.png");
    EXPECT_NEAR(floor->ground_z, -2.6354567878618376, 1e-6);
    ASSERT_EIGEN_VECTOR_NEAR(
        "grid_map_origin",
        floor->grid_map_origin,
        Eigen::Vector2f(-3.9349700000000003, -8.69993),
        1e-6);
    ASSERT_EIGEN_VECTOR_NEAR(
        "grid_map_resolution",
        floor->grid_map_resolution,
        Eigen::Vector2f(0.009985776293823039, 0.009991059907834101),
        1e-6);
    ASSERT_EIGEN_VECTOR_EQUAL("grid_map_size", floor->grid_map_size, Eigen::Vector2i(599, 1085));

    auto room = floor->rooms[1];
    EXPECT_EQ(room->id, 1);
    EXPECT_EQ(room->uuid, 2);
    EXPECT_EQ(room->parent_id, 0);    // floor->id
    EXPECT_EQ(room->parent_uuid, 3);  // floor->uuid
    EXPECT_EQ(room->type, erl::env::scene_graph::Node::Type::kRoom);
    EXPECT_EQ(room->name, "bathroom");
    EXPECT_EQ(room->objects.size(), 3);
    EXPECT_EQ(room->num_objects, 3);
    EXPECT_EQ(room->connected_room_ids[0], 10);
    EXPECT_EQ(room->connected_room_uuids[0], 14);
    EXPECT_TRUE(room->door_grids.find(10) != room->door_grids.end());
    EXPECT_EQ(room->door_grids[10].cols(), 73);
    EXPECT_EQ(room->door_grids[10](0, 0), 375);
    EXPECT_EQ(room->door_grids[10](1, 0), 175);
    ASSERT_EIGEN_VECTOR_EQUAL("grid_map_min", room->grid_map_min, Eigen::Vector2i(367, 98));
    ASSERT_EIGEN_VECTOR_EQUAL("grid_map_max", room->grid_map_max, Eigen::Vector2i(536, 327));
    ASSERT_EIGEN_VECTOR_NEAR(
        "location",
        room->location,
        Eigen::Vector3f(0.5904309999999999, -6.549355, -1.4833679999999998),
        1e-6);
    ASSERT_EIGEN_VECTOR_NEAR(
        "size",
        room->size,
        Eigen::Vector3f(1.620558, 2.2485100000000005, 2.342164),
        1e-6);

    auto object = room->objects[7];
    EXPECT_EQ(object->id, 7);
    EXPECT_EQ(object->uuid, 27);
    EXPECT_EQ(object->parent_id, 1);    // room->id
    EXPECT_EQ(object->parent_uuid, 2);  // room->uuid
    EXPECT_EQ(object->type, erl::env::scene_graph::Node::Type::kObject);
    EXPECT_EQ(object->name, "sink");
    ASSERT_EIGEN_VECTOR_EQUAL("grid_map_min", object->grid_map_min, Eigen::Vector2i(473, 102));
    ASSERT_EIGEN_VECTOR_EQUAL("grid_map_max", object->grid_map_max, Eigen::Vector2i(529, 162));
    ASSERT_EIGEN_VECTOR_NEAR(
        "location",
        object->location,
        Eigen::Vector3f(1.0723643216018577, -7.369863979104055, -1.8417292314783524),
        1e-6);
    ASSERT_EIGEN_VECTOR_NEAR(
        "size",
        object->size,
        Eigen::Vector3f(0.5124483939678722, 0.5692844456293251, 0.17976860229091773),
        1e-6);

    cv::Mat room_map = building.LoadRoomMap(gtest_src_dir, 0);
    cv::Mat cat_map = building.LoadCatMap(gtest_src_dir, 0);
    double min, max;
    cv::minMaxLoc(room_map, &min, &max);
    EXPECT_EQ(min, -5);
    cv::minMaxLoc(cat_map, &min, &max);
    EXPECT_EQ(min, -5);

    // cv::Mat mask(cat_map.rows, cat_map.cols, CV_32SC1,
    // int(erl::env::scene_graph::Object::SOC::kGround));
    cv::Mat mask = cat_map != int(erl::env::scene_graph::Object::SOC::kGround);
    cv::Mat sub_mask = mask(cv::Rect(10, 10, 100, 100));  // x, y, w, h
    std::cout << "mask size: " << mask.rows << " rows x " << mask.cols << " cols" << std::endl;
    std::cout << "sub mask size: " << sub_mask.rows << " rows x " << sub_mask.cols << " cols"
              << std::endl;
    std::cout << "sub mask is subMatrix: " << sub_mask.isSubmatrix()
              << std::endl;  // "sub mask is subMatrix: 1
    std::cout << "sub mask is continuous: " << sub_mask.isContinuous() << std::endl;
    std::cout << "mask(10, 10): " << int(mask.at<uint8_t>(10, 10)) << std::endl;
    std::cout << "sub mask(0, 0): " << int(sub_mask.at<uint8_t>(0, 0)) << std::endl;
    mask.at<uint8_t>(10, 10) = 100;
    std::cout << "mask(10, 10): " << int(mask.at<uint8_t>(10, 10)) << std::endl;
    std::cout << "sub mask(0, 0): " << int(sub_mask.at<uint8_t>(0, 0)) << std::endl;
    cv::Mat mask2 = mask;  // not copy data
    std::cout << "mask data addr: " << (void *) (mask.data) << std::endl;
    std::cout << "mask2 data addr: " << (void *) (mask2.data) << std::endl;
    std::cout << "sub mask data addr: " << (void *) (sub_mask.data) << std::endl;
    cv::imshow("ground mask", mask);
    cv::waitKey(10000);

    // uncomment the following lines to show the maps
    // cv::Mat color_room_map;
    // erl::common::ColorGrayToJet(room_map, color_room_map);
    // cv::Mat color_cat_map;
    // erl::common::ColorGrayToJet(cat_map, color_cat_map);
    // cv::imshow("room_map", color_room_map);
    // cv::imshow("cat_map", color_cat_map);
    // cv::waitKey(10000);
}
