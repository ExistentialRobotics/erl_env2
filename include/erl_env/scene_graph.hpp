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
    };

    struct Building : public common::Yamlable<Building, Node> {
        std::unordered_map<int, std::shared_ptr<Floor>> floors = {};  // floors
        int num_floors = 0;                                           // number of floors
        Eigen::Vector3f reference_point = {};                         // reference 3d coordinate
        Eigen::Vector3f size = {};                                    // building size
        std::vector<int> room_ids;
        std::vector<int> room_uuids;
        std::vector<int> object_ids;
        std::vector<int> object_uuids;
        std::unordered_map<int, std::shared_ptr<Object>> id_to_object = {};
        std::unordered_map<int, std::shared_ptr<Room>> id_to_room = {};
        std::unordered_map<uint32_t, std::shared_ptr<Node>> uuid_to_node = {};

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

namespace YAML {

    template<>
    struct convert<erl::env::scene_graph::Node::Type> {
        static Node
        encode(const erl::env::scene_graph::Node::Type &node_type) {
            Node node;
            switch (node_type) {
                case erl::env::scene_graph::Node::Type::kOcc:
                    node = "kOcc";
                    break;
                case erl::env::scene_graph::Node::Type::kObject:
                    node = "kObject";
                    break;
                case erl::env::scene_graph::Node::Type::kRoom:
                    node = "kRoom";
                    break;
                case erl::env::scene_graph::Node::Type::kFloor:
                    node = "kFloor";
                    break;
                case erl::env::scene_graph::Node::Type::kBuilding:
                    node = "kBuilding";
                    break;
            }
            return node;
        }

        static bool
        decode(const Node &node, erl::env::scene_graph::Node::Type &node_type) {
            if (!node.IsScalar()) { return false; }
            auto type = node.as<std::string>();
            if (type == "kOcc") {
                node_type = erl::env::scene_graph::Node::Type::kOcc;
            } else if (type == "kObject") {
                node_type = erl::env::scene_graph::Node::Type::kObject;
            } else if (type == "kRoom") {
                node_type = erl::env::scene_graph::Node::Type::kRoom;
            } else if (type == "kFloor") {
                node_type = erl::env::scene_graph::Node::Type::kFloor;
            } else if (type == "kBuilding") {
                node_type = erl::env::scene_graph::Node::Type::kBuilding;
            } else {
                throw std::runtime_error("Unknown scene_graph::Node::Type: " + type);
            }
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Node> {
        static Node
        encode(const erl::env::scene_graph::Node &sg_node) {
            Node node;
            ERL_YAML_SAVE_ATTR(node, sg_node, uuid);
            ERL_YAML_SAVE_ATTR(node, sg_node, id);
            ERL_YAML_SAVE_ATTR(node, sg_node, parent_id);
            ERL_YAML_SAVE_ATTR(node, sg_node, parent_uuid);
            ERL_YAML_SAVE_ATTR(node, sg_node, type);
            ERL_YAML_SAVE_ATTR(node, sg_node, name);
            return node;
        }

        static bool
        decode(const Node &node, erl::env::scene_graph::Node &sg_node) {
            if (!node.IsMap()) { return false; }
            ERL_YAML_LOAD_ATTR(node, sg_node, uuid);
            ERL_YAML_LOAD_ATTR(node, sg_node, id);
            ERL_YAML_LOAD_ATTR(node, sg_node, parent_id);
            ERL_YAML_LOAD_ATTR(node, sg_node, parent_uuid);
            ERL_YAML_LOAD_ATTR(node, sg_node, type);
            ERL_YAML_LOAD_ATTR(node, sg_node, name);
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Object> {
        static Node
        encode(const erl::env::scene_graph::Object &object) {
            Node node = convert<erl::env::scene_graph::Node>::encode(object);
            ERL_YAML_SAVE_ATTR(node, object, action_affordance);
            ERL_YAML_SAVE_ATTR(node, object, grid_map_min);
            ERL_YAML_SAVE_ATTR(node, object, grid_map_max);
            ERL_YAML_SAVE_ATTR(node, object, location);
            ERL_YAML_SAVE_ATTR(node, object, size);
            return node;
        }

        static bool
        decode(const Node &node, erl::env::scene_graph::Object &object) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::env::scene_graph::Node>::decode(node, object)) { return false; }
            if (object.type != erl::env::scene_graph::Node::Type::kObject) {
                ERL_WARN("Node type is not kObject");
                return false;
            }
            ERL_YAML_LOAD_ATTR(node, object, action_affordance);
            ERL_YAML_LOAD_ATTR(node, object, grid_map_min);
            ERL_YAML_LOAD_ATTR(node, object, grid_map_max);
            ERL_YAML_LOAD_ATTR(node, object, location);
            ERL_YAML_LOAD_ATTR(node, object, size);
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Room> {
        static Node
        encode(const erl::env::scene_graph::Room &room) {
            Node node = convert<erl::env::scene_graph::Node>::encode(room);
            ERL_YAML_SAVE_ATTR(node, room, objects);
            ERL_YAML_SAVE_ATTR(node, room, num_objects);
            ERL_YAML_SAVE_ATTR(node, room, connected_room_ids);
            ERL_YAML_SAVE_ATTR(node, room, connected_room_uuids);
            ERL_YAML_SAVE_ATTR(node, room, door_grids);
            ERL_YAML_SAVE_ATTR(node, room, grid_map_min);
            ERL_YAML_SAVE_ATTR(node, room, grid_map_max);
            ERL_YAML_SAVE_ATTR(node, room, location);
            ERL_YAML_SAVE_ATTR(node, room, size);
            return node;
        }

        static bool
        decode(const Node &node, erl::env::scene_graph::Room &room) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::env::scene_graph::Node>::decode(node, room)) { return false; }
            if (room.type != erl::env::scene_graph::Node::Type::kRoom) {
                ERL_WARN("Node type is not kRoom");
                return false;
            }
            ERL_YAML_LOAD_ATTR(node, room, objects);
            ERL_YAML_LOAD_ATTR(node, room, num_objects);
            ERL_ASSERTM(
                room.objects.size() == room.num_objects,
                "Number of objects does not match");
            for (const auto &object_itr: room.objects) {
                ERL_ASSERTM(
                    object_itr.second->id == object_itr.first,
                    "Object %d has wrong id: %d",
                    object_itr.first,
                    object_itr.second->id);
            }
            ERL_YAML_LOAD_ATTR(node, room, connected_room_ids);
            ERL_YAML_LOAD_ATTR(node, room, connected_room_uuids);
            ERL_YAML_LOAD_ATTR(node, room, door_grids);
            ERL_YAML_LOAD_ATTR(node, room, grid_map_min);
            ERL_YAML_LOAD_ATTR(node, room, grid_map_max);
            ERL_YAML_LOAD_ATTR(node, room, location);
            ERL_YAML_LOAD_ATTR(node, room, size);
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Floor> {
        static Node
        encode(const erl::env::scene_graph::Floor &floor) {
            Node node = convert<erl::env::scene_graph::Node>::encode(floor);
            ERL_YAML_SAVE_ATTR(node, floor, down_stairs_id);
            ERL_YAML_SAVE_ATTR(node, floor, up_stairs_id);
            ERL_YAML_SAVE_ATTR(node, floor, down_stairs_uuid);
            ERL_YAML_SAVE_ATTR(node, floor, up_stairs_uuid);
            ERL_YAML_SAVE_ATTR(node, floor, down_stairs_cost);
            ERL_YAML_SAVE_ATTR(node, floor, up_stairs_cost);
            ERL_YAML_SAVE_ATTR(node, floor, down_stairs_portal);
            ERL_YAML_SAVE_ATTR(node, floor, up_stairs_portal);
            ERL_YAML_SAVE_ATTR(node, floor, ground_z);
            ERL_YAML_SAVE_ATTR(node, floor, room_map);
            ERL_YAML_SAVE_ATTR(node, floor, cat_map);
            ERL_YAML_SAVE_ATTR(node, floor, rooms);
            ERL_YAML_SAVE_ATTR(node, floor, num_rooms);
            ERL_YAML_SAVE_ATTR(node, floor, grid_map_origin);
            ERL_YAML_SAVE_ATTR(node, floor, grid_map_resolution);
            ERL_YAML_SAVE_ATTR(node, floor, grid_map_size);
            return node;
        }

        static bool
        decode(const Node &node, erl::env::scene_graph::Floor &floor) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::env::scene_graph::Node>::decode(node, floor)) { return false; }
            if (floor.type != erl::env::scene_graph::Node::Type::kFloor) {
                ERL_WARN("Node type is not kFloor");
                return false;
            }
            ERL_YAML_LOAD_ATTR(node, floor, down_stairs_id);
            ERL_YAML_LOAD_ATTR(node, floor, up_stairs_id);
            ERL_YAML_LOAD_ATTR(node, floor, down_stairs_uuid);
            ERL_YAML_LOAD_ATTR(node, floor, up_stairs_uuid);
            ERL_YAML_LOAD_ATTR(node, floor, down_stairs_cost);
            ERL_YAML_LOAD_ATTR(node, floor, up_stairs_cost);
            ERL_YAML_LOAD_ATTR(node, floor, down_stairs_portal);
            ERL_YAML_LOAD_ATTR(node, floor, up_stairs_portal);
            ERL_YAML_LOAD_ATTR(node, floor, ground_z);
            ERL_YAML_LOAD_ATTR(node, floor, room_map);
            ERL_YAML_LOAD_ATTR(node, floor, cat_map);
            ERL_YAML_LOAD_ATTR(node, floor, rooms);
            ERL_YAML_LOAD_ATTR(node, floor, num_rooms);
            ERL_ASSERTM(
                floor.rooms.size() == static_cast<std::size_t>(floor.num_rooms),
                "Number of rooms does not match");
            for (const auto &room_itr: floor.rooms) {
                ERL_ASSERTM(
                    room_itr.second->id == room_itr.first,
                    "Room %d has wrong id: %d",
                    room_itr.first,
                    room_itr.second->id);
            }
            ERL_YAML_LOAD_ATTR(node, floor, grid_map_origin);
            ERL_YAML_LOAD_ATTR(node, floor, grid_map_resolution);
            ERL_YAML_LOAD_ATTR(node, floor, grid_map_size);
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Building> {
        static Node
        encode(const erl::env::scene_graph::Building &building) {
            Node node = convert<erl::env::scene_graph::Node>::encode(building);
            ERL_YAML_SAVE_ATTR(node, building, floors);
            ERL_YAML_SAVE_ATTR(node, building, num_floors);
            ERL_YAML_SAVE_ATTR(node, building, reference_point);
            ERL_YAML_SAVE_ATTR(node, building, size);
            return node;
        }

        static bool
        decode(const Node &node, erl::env::scene_graph::Building &building) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::env::scene_graph::Node>::decode(node, building)) { return false; }
            if (building.type != erl::env::scene_graph::Node::Type::kBuilding) {
                ERL_WARN("Node type is not kBuilding");
                return false;
            }
            ERL_YAML_LOAD_ATTR(node, building, floors);
            ERL_YAML_LOAD_ATTR(node, building, num_floors);
            ERL_ASSERTM(
                building.floors.size() == static_cast<std::size_t>(building.num_floors),
                "Number of floors does not match");
            for (int i = 0; i < building.num_floors; ++i) {
                ERL_ASSERTM(
                    building.floors.find(i) != building.floors.end(),
                    "Floor %d is missing",
                    i);
                ERL_ASSERTM(
                    building.floors[i]->id == i,
                    "Floor %d has wrong id: %d",
                    i,
                    building.floors[i]->id);
            }
            ERL_YAML_LOAD_ATTR(node, building, reference_point);
            ERL_YAML_LOAD_ATTR(node, building, size);
            building.UpdateIdMapping();
            return true;
        }
    };
}  // namespace YAML
