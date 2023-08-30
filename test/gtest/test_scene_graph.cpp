#include <filesystem>
#include "erl_common/test_helper.hpp"
#include "erl_env/scene_graph.hpp"

TEST(ERL_ENV, SceneGraph) {
    std::filesystem::path path = __FILE__;
    path = path.parent_path();
    path = path / "building.yaml";

    erl::env::scene_graph::Building building;
    building.FromYamlFile(path.string());

    EXPECT_EQ(building.uuid, 1);
    EXPECT_EQ(building.id, 4);
    EXPECT_EQ(building.parent_id, -1);
    EXPECT_EQ(building.parent_uuid, -1);
    EXPECT_EQ(building.type, erl::env::scene_graph::Node::Type::kBuilding);
    EXPECT_EQ(building.name, "Benevolence");
    EXPECT_EQ(building.floors.size(), 3);
    EXPECT_EQ(building.num_floors, 3);
    ASSERT_EIGEN_VECTOR_EQUAL("building.reference_point", building.reference_point, Eigen::Vector3d(0, 0, 0));
    ASSERT_EIGEN_VECTOR_NEAR("building.size", building.size, Eigen::Vector3d(5.580856967034837, 10.439912021678497, 8.416298157597547), 1e-15);

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
    EXPECT_EQ(floor->room_map, "room_maps/0.png");
    EXPECT_EQ(floor->cat_map, "cat_maps/0.png");
    EXPECT_NEAR(floor->ground_z, -2.6354567878618376, 1e-15);
    ASSERT_EIGEN_VECTOR_NEAR("grid_map_origin", floor->grid_map_origin, Eigen::Vector2d(-3.9349700000000003, 2.14037), 1e-15);
    ASSERT_EIGEN_VECTOR_NEAR("grid_map_resolution", floor->grid_map_resolution, Eigen::Vector2d(0.009985776293823039, 0.009991059907834101), 1e-15);
    ASSERT_EIGEN_VECTOR_EQUAL("grid_map_size", floor->grid_map_size, Eigen::Vector2i(1085, 599));

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
    ASSERT_EIGEN_VECTOR_EQUAL("grid_map_min", room->grid_map_min, Eigen::Vector2i(759, 371));
    ASSERT_EIGEN_VECTOR_EQUAL("grid_map_max", room->grid_map_max, Eigen::Vector2i(984, 536));
    ASSERT_EIGEN_VECTOR_NEAR("location", room->location, Eigen::Vector3d(0.5904309999999999, -6.549355, -1.4833679999999998), 1e-15);
    ASSERT_EIGEN_VECTOR_NEAR("size", room->size, Eigen::Vector3d(1.620558, 2.2485100000000005, 2.342164), 1e-15);

    auto object = room->objects[7];
    EXPECT_EQ(object->id, 7);
    EXPECT_EQ(object->uuid, 27);
    EXPECT_EQ(object->parent_id, 1);    // room->id
    EXPECT_EQ(object->parent_uuid, 2);  // room->uuid
    EXPECT_EQ(object->type, erl::env::scene_graph::Node::Type::kObject);
    EXPECT_EQ(object->name, "sink");
    ASSERT_EIGEN_VECTOR_NEAR("location", object->location, Eigen::Vector3d(1.0723643216018577, -7.369863979104055, -1.8417292314783524), 1e-15);
    ASSERT_EIGEN_VECTOR_NEAR("size", object->size, Eigen::Vector3d(0.5124483939678722, 0.5692844456293251, 0.17976860229091773), 1e-15);
}
