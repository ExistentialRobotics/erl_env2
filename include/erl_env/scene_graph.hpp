#pragma once

// #include "atomic_proposition.hpp"

#include "erl_common/opencv.hpp"
#include "erl_common/yaml.hpp"

namespace erl::env::scene_graph {
    struct Node : public common::Yamlable<Node> {
        inline static int uuid_counter = 0;

        enum class Type {
            kOcc = 0,
            kObject = 1,
            kRoom = 2,
            kFloor = 3,
            kBuilding = 4,
        };

        int uuid = -1;           // unique scene graph element id
        int id = -1;             // unique id of the same type
        int parent_id = -1;      // parent node id (not uuid)
        int parent_uuid = -1;    // parent node uuid
        Type type = Type::kOcc;  // type of the node
        std::string name = {};   // node name

        ERL_REFLECT_SCHEMA(
            Node,
            ERL_REFLECT_MEMBER(Node, uuid),
            ERL_REFLECT_MEMBER(Node, id),
            ERL_REFLECT_MEMBER(Node, parent_id),
            ERL_REFLECT_MEMBER(Node, parent_uuid),
            ERL_REFLECT_MEMBER(Node, type),
            ERL_REFLECT_MEMBER(Node, name));

        Node()
            : uuid(uuid_counter++) {}
    };

    struct Object : public common::Yamlable<Object, Node> {

        enum class SOC {  // special object category
            kGround = 0,
            kStairsUp = -1,
            kStairsDown = -2,
            kWall = -3,
            kCeiling = -4,
            kNA = -5,
        };

        std::vector<std::string> action_affordance = {};  // action affordances
        Eigen::Vector2i grid_map_min = {};                // grid_map min
        Eigen::Vector2i grid_map_max = {};                // grid_map max
        Eigen::Vector3f location = {};                    // object center location
        Eigen::Vector3f size = {};                        // object size

        ERL_REFLECT_SCHEMA(
            Object,
            ERL_REFLECT_MEMBER(Object, action_affordance),
            ERL_REFLECT_MEMBER(Object, grid_map_min),
            ERL_REFLECT_MEMBER(Object, grid_map_max),
            ERL_REFLECT_MEMBER(Object, location),
            ERL_REFLECT_MEMBER(Object, size));

        bool
        PostDeserialization() override {
            if (type != Node::Type::kObject) {
                ERL_WARN("Node type is not kObject");
                return false;
            }
            return true;
        }
    };

    struct Room : public common::Yamlable<Room, Node> {
        std::unordered_map<int, std::shared_ptr<Object>> objects = {};  // objects
        uint32_t num_objects = 0;                                       // number of objects
        std::vector<int> connected_room_ids = {};                       // neighbor room ids
        std::vector<int> connected_room_uuids = {};                     // neighbor room uuids
        std::unordered_map<int, Eigen::Matrix2Xi> door_grids = {};      // door grids
        Eigen::Vector2i grid_map_min = {};                              // grid_map min
        Eigen::Vector2i grid_map_max = {};                              // grid_map max
        Eigen::Vector3f location = {};                                  // room center location
        Eigen::Vector3f size = {};                                      // room size

        ERL_REFLECT_SCHEMA(
            Room,
            ERL_REFLECT_MEMBER(Room, objects),
            ERL_REFLECT_MEMBER(Room, num_objects),
            ERL_REFLECT_MEMBER(Room, connected_room_ids),
            ERL_REFLECT_MEMBER(Room, connected_room_uuids),
            ERL_REFLECT_MEMBER(Room, door_grids),
            ERL_REFLECT_MEMBER(Room, grid_map_min),
            ERL_REFLECT_MEMBER(Room, grid_map_max),
            ERL_REFLECT_MEMBER(Room, location),
            ERL_REFLECT_MEMBER(Room, size));

        bool
        PostDeserialization() override {
            if (type != Node::Type::kRoom) {
                ERL_WARN("Node type is not kRoom");
                return false;
            }
            ERL_ASSERTM(objects.size() == num_objects, "Number of objects does not match");
            for (const auto &object_itr: objects) {
                ERL_ASSERTM(
                    object_itr.second->id == object_itr.first,
                    "Object %d has wrong id: %d",
                    object_itr.first,
                    object_itr.second->id);
            }
            return true;
        }
    };

    struct Floor : public common::Yamlable<Floor, Node> {
        int down_stairs_id = -1;                                          // down stairs id
        int up_stairs_id = -1;                                            // up stairs id
        int down_stairs_uuid = -1;                                        // down stairs uuid
        int up_stairs_uuid = -1;                                          // up stairs uuid
        float down_stairs_cost = std::numeric_limits<float>::infinity();  // downstairs cost
        float up_stairs_cost = std::numeric_limits<float>::infinity();    // upstairs cost
        std::optional<Eigen::Vector2i> up_stairs_portal = {};             // upstairs portal
        std::optional<Eigen::Vector2i> down_stairs_portal = {};           // downstairs portal
        float ground_z = -1;                                              // ground z coordinate
        std::string room_map = {};  // relative path of room segmentation map
        std::string cat_map = {};   // relative path of object segmentation map
        std::unordered_map<int, std::shared_ptr<Room>> rooms;  // rooms
        int num_rooms = 0;                                     // number of rooms
        Eigen::Vector2f grid_map_origin = {};                  // grid_map origin
        Eigen::Vector2f grid_map_resolution = {};              // grid_map resolution
        Eigen::Vector2i grid_map_size = {};                    // grid_map size

        ERL_REFLECT_SCHEMA(
            Floor,
            ERL_REFLECT_MEMBER(Floor, down_stairs_id),
            ERL_REFLECT_MEMBER(Floor, up_stairs_id),
            ERL_REFLECT_MEMBER(Floor, down_stairs_uuid),
            ERL_REFLECT_MEMBER(Floor, up_stairs_uuid),
            ERL_REFLECT_MEMBER(Floor, down_stairs_cost),
            ERL_REFLECT_MEMBER(Floor, up_stairs_cost),
            ERL_REFLECT_MEMBER(Floor, up_stairs_portal),
            ERL_REFLECT_MEMBER(Floor, down_stairs_portal),
            ERL_REFLECT_MEMBER(Floor, ground_z),
            ERL_REFLECT_MEMBER(Floor, room_map),
            ERL_REFLECT_MEMBER(Floor, cat_map),
            ERL_REFLECT_MEMBER(Floor, rooms),
            ERL_REFLECT_MEMBER(Floor, num_rooms),
            ERL_REFLECT_MEMBER(Floor, grid_map_origin),
            ERL_REFLECT_MEMBER(Floor, grid_map_resolution),
            ERL_REFLECT_MEMBER(Floor, grid_map_size));

        bool
        PostDeserialization() override {
            if (type != Node::Type::kFloor) {
                ERL_WARN("Node type is not kFloor");
                return false;
            }
            ERL_ASSERTM(
                rooms.size() == static_cast<std::size_t>(num_rooms),
                "Number of rooms does not match");
            for (const auto &room_itr: rooms) {
                ERL_ASSERTM(
                    room_itr.second->id == room_itr.first,
                    "Room %d has wrong id: %d",
                    room_itr.first,
                    room_itr.second->id);
            }
            return true;
        }
    };

    struct Building : public common::Yamlable<Building, Node> {
        std::unordered_map<int, std::shared_ptr<Floor>> floors = {};  // floors
        int num_floors = 0;                                           // number of floors
        Eigen::Vector3f reference_point = {};                         // reference 3d coordinate
        Eigen::Vector3f size = {};                                    // building size

        ERL_REFLECT_SCHEMA(
            Building,
            ERL_REFLECT_MEMBER(Building, floors),
            ERL_REFLECT_MEMBER(Building, num_floors),
            ERL_REFLECT_MEMBER(Building, reference_point),
            ERL_REFLECT_MEMBER(Building, size));

        std::vector<int> room_ids;
        std::vector<int> room_uuids;
        std::vector<int> object_ids;
        std::vector<int> object_uuids;
        std::unordered_map<int, std::shared_ptr<Object>> id_to_object = {};
        std::unordered_map<int, std::shared_ptr<Room>> id_to_room = {};
        std::unordered_map<uint32_t, std::shared_ptr<Node>> uuid_to_node = {};

        bool
        PostDeserialization() override {
            if (type != Node::Type::kBuilding) {
                ERL_WARN("Node type is not kBuilding");
                return false;
            }
            ERL_ASSERTM(
                floors.size() == static_cast<std::size_t>(num_floors),
                "Number of floors does not match");
            for (int i = 0; i < num_floors; ++i) {
                ERL_ASSERTM(floors.find(i) != floors.end(), "Floor %d is missing", i);
                ERL_ASSERTM(floors[i]->id == i, "Floor %d has wrong id: %d", i, floors[i]->id);
            }
            UpdateIdMapping();
            return true;
        }

        void
        UpdateIdMapping() {
            room_ids.clear();
            room_uuids.clear();
            object_ids.clear();
            object_uuids.clear();
            id_to_object.clear();
            id_to_room.clear();
            uuid_to_node.clear();
            for (auto &floor_itr: floors) {
                auto &floor = floor_itr.second;
                ERL_ASSERTM(
                    uuid_to_node.try_emplace(floor->uuid, floor).second,
                    "Duplicate floor uuid: %d",
                    floor->uuid);
                for (auto &room_itr: floor->rooms) {
                    auto &room = room_itr.second;
                    room_ids.push_back(room->id);
                    room_uuids.push_back(room->uuid);
                    ERL_ASSERTM(
                        id_to_room.try_emplace(room->id, room).second,
                        "Duplicate room id: %d",
                        room->id);
                    ERL_ASSERTM(
                        uuid_to_node.try_emplace(room->uuid, room).second,
                        "Duplicate room uuid: %d",
                        room->uuid);
                    for (auto &object_itr: room->objects) {
                        auto &object = object_itr.second;
                        object_ids.push_back(object->id);
                        object_uuids.push_back(object->uuid);
                        ERL_ASSERTM(
                            id_to_object.try_emplace(object->id, object).second,
                            "Duplicate object id: %d",
                            object->id);
                        ERL_ASSERTM(
                            uuid_to_node.try_emplace(object->uuid, object).second,
                            "Duplicate object uuid: %d",
                            object->uuid);
                    }
                }
            }
            std::sort(room_ids.begin(), room_ids.end());
            std::sort(room_uuids.begin(), room_uuids.end());
            std::sort(object_ids.begin(), object_ids.end());
            std::sort(object_uuids.begin(), object_uuids.end());
        }

        template<class T>
        std::shared_ptr<T>
        GetNode(uint32_t uuid) {
            auto itr = uuid_to_node.find(uuid);
            if (itr == uuid_to_node.end()) { return nullptr; }
            return std::dynamic_pointer_cast<T>(itr->second);
        }

        [[nodiscard]] cv::Mat
        LoadRoomMap(const std::filesystem::path &data_dir, int floor_id) const {
            auto file_path = data_dir / floors.at(floor_id)->room_map;
            cv::Mat room_map = cv::imread(file_path.string(), cv::IMREAD_GRAYSCALE);
            room_map.convertTo(room_map, CV_32SC1);
            room_map += static_cast<int>(Object::SOC::kNA);
            return room_map;
        }

        [[nodiscard]] cv::Mat
        LoadCatMap(const std::filesystem::path &data_dir, int floor_id) const {
            auto file_path = data_dir / floors.at(floor_id)->cat_map;
            cv::Mat cat_map = cv::imread(file_path.string(), cv::IMREAD_GRAYSCALE);
            cat_map.convertTo(cat_map, CV_32SC1);
            cat_map += static_cast<int>(Object::SOC::kNA);
            return cat_map;
        }
    };
}  // namespace erl::env::scene_graph

ERL_REFLECT_ENUM_SCHEMA(
    erl::env::scene_graph::Node::Type,
    5,
    ERL_REFLECT_ENUM_MEMBER("kOcc", erl::env::scene_graph::Node::Type::kOcc),
    ERL_REFLECT_ENUM_MEMBER("kObject", erl::env::scene_graph::Node::Type::kObject),
    ERL_REFLECT_ENUM_MEMBER("kRoom", erl::env::scene_graph::Node::Type::kRoom),
    ERL_REFLECT_ENUM_MEMBER("kFloor", erl::env::scene_graph::Node::Type::kFloor),
    ERL_REFLECT_ENUM_MEMBER("kBuilding", erl::env::scene_graph::Node::Type::kBuilding));

ERL_ENUM_YAML_CONVERT(erl::env::scene_graph::Node::Type, 5);

