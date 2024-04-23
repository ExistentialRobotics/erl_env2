#pragma once

#include "erl_common/yaml.hpp"
#include "erl_common/opencv.hpp"
#include "atomic_proposition.hpp"

namespace erl::env::scene_graph {
    struct Node : public common::Yamlable<Node> {
        inline static int uuid_counter = 0;

    public:
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

    struct Object : public common::OverrideYamlable<Node, Object> {

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
        Eigen::Vector3d location = {};                    // object center location
        Eigen::Vector3d size = {};                        // object size
    };

    struct Room : public common::OverrideYamlable<Node, Room> {
        std::unordered_map<int, std::shared_ptr<Object>> objects = {};  // objects
        uint32_t num_objects = 0;                                       // number of objects
        std::vector<int> connected_room_ids = {};                       // neighbor room ids
        std::vector<int> connected_room_uuids = {};                     // neighbor room uuids
        std::unordered_map<int, Eigen::Matrix2Xi> door_grids = {};      // door grids
        Eigen::Vector2i grid_map_min = {};                              // grid_map min
        Eigen::Vector2i grid_map_max = {};                              // grid_map max
        Eigen::Vector3d location = {};                                  // room center location
        Eigen::Vector3d size = {};                                      // room size
    };

    struct Floor : public common::OverrideYamlable<Node, Floor> {
        int down_stairs_id = -1;                                            // down stairs id
        int up_stairs_id = -1;                                              // up stairs id
        int down_stairs_uuid = -1;                                          // down stairs uuid
        int up_stairs_uuid = -1;                                            // up stairs uuid
        double down_stairs_cost = std::numeric_limits<double>::infinity();  // down stairs cost
        double up_stairs_cost = std::numeric_limits<double>::infinity();    // up stairs cost
        std::optional<std::array<int, 2>> up_stairs_portal = {};            // up stairs portal
        std::optional<std::array<int, 2>> down_stairs_portal = {};          // down stairs portal
        double ground_z = -1;                                               // ground z coordinate
        std::string room_map = {};                                          // relative path of room segmentation map
        std::string cat_map = {};                                           // relative path of object segmentation map
        std::unordered_map<int, std::shared_ptr<Room>> rooms;               // rooms
        int num_rooms = 0;                                                  // number of rooms
        Eigen::Vector2d grid_map_origin = {};                               // grid_map origin
        Eigen::Vector2d grid_map_resolution = {};                           // grid_map resolution
        Eigen::Vector2i grid_map_size = {};                                 // grid_map size
    };

    struct Building : public common::OverrideYamlable<Node, Building> {
        std::unordered_map<int, std::shared_ptr<Floor>> floors = {};  // floors
        int num_floors = 0;                                           // number of floors
        Eigen::Vector3d reference_point = {};                         // reference 3d coordinate
        Eigen::Vector3d size = {};                                    // building size
        std::vector<int> room_ids;
        std::vector<int> room_uuids;
        std::vector<int> object_ids;
        std::vector<int> object_uuids;
        std::unordered_map<int, std::shared_ptr<Object>> id_to_object = {};
        std::unordered_map<int, std::shared_ptr<Room>> id_to_room = {};
        std::unordered_map<uint32_t, std::shared_ptr<Node>> uuid_to_node = {};

    public:
        void
        UpdateIdMapping() {
            room_ids.clear();
            room_uuids.clear();
            object_ids.clear();
            object_uuids.clear();
            id_to_object.clear();
            id_to_room.clear();
            uuid_to_node.clear();
            for (auto& floor_itr: floors) {
                auto& floor = floor_itr.second;
                ERL_ASSERTM(uuid_to_node.try_emplace(floor->uuid, floor).second, "Duplicate floor uuid: %d", floor->uuid);
                for (auto& room_itr: floor->rooms) {
                    auto& room = room_itr.second;
                    room_ids.push_back(room->id);
                    room_uuids.push_back(room->uuid);
                    ERL_ASSERTM(id_to_room.try_emplace(room->id, room).second, "Duplicate room id: %d", room->id);
                    ERL_ASSERTM(uuid_to_node.try_emplace(room->uuid, room).second, "Duplicate room uuid: %d", room->uuid);
                    for (auto& object_itr: room->objects) {
                        auto& object = object_itr.second;
                        object_ids.push_back(object->id);
                        object_uuids.push_back(object->uuid);
                        ERL_ASSERTM(id_to_object.try_emplace(object->id, object).second, "Duplicate object id: %d", object->id);
                        ERL_ASSERTM(uuid_to_node.try_emplace(object->uuid, object).second, "Duplicate object uuid: %d", object->uuid);
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
        LoadRoomMap(const std::filesystem::path& data_dir, int floor_id) const {
            auto file_path = data_dir / floors.at(floor_id)->room_map;
            cv::Mat room_map = cv::imread(file_path.string(), cv::IMREAD_GRAYSCALE);
            room_map.convertTo(room_map, CV_32SC1);
            room_map += int(Object::SOC::kNA);
            return room_map;
        }

        [[nodiscard]] cv::Mat
        LoadCatMap(const std::filesystem::path& data_dir, int floor_id) const {
            auto file_path = data_dir / floors.at(floor_id)->cat_map;
            cv::Mat cat_map = cv::imread(file_path.string(), cv::IMREAD_GRAYSCALE);
            cat_map.convertTo(cat_map, CV_32SC1);
            cat_map += int(Object::SOC::kNA);
            return cat_map;
        }
    };
}  // namespace erl::env::scene_graph

namespace YAML {

    template<>
    struct convert<erl::env::scene_graph::Node::Type> {
        inline static Node
        encode(const erl::env::scene_graph::Node::Type& rhs) {
            Node node;
            switch (rhs) {
                case erl::env::scene_graph::Node::Type::kOcc:
                    node = "kNA";
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

        inline static bool
        decode(const Node& node, erl::env::scene_graph::Node::Type& rhs) {
            if (!node.IsScalar()) { return false; }
            auto type = node.as<std::string>();
            if (type == "kNA") {
                rhs = erl::env::scene_graph::Node::Type::kOcc;
            } else if (type == "kObject") {
                rhs = erl::env::scene_graph::Node::Type::kObject;
            } else if (type == "kRoom") {
                rhs = erl::env::scene_graph::Node::Type::kRoom;
            } else if (type == "kFloor") {
                rhs = erl::env::scene_graph::Node::Type::kFloor;
            } else if (type == "kBuilding") {
                rhs = erl::env::scene_graph::Node::Type::kBuilding;
            } else {
                throw std::runtime_error("Unknown scene_graph::Node::Type: " + type);
            }
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Node> {
        inline static Node
        encode(const erl::env::scene_graph::Node& rhs) {
            Node node;
            node["uuid"] = rhs.uuid;
            node["id"] = rhs.id;
            node["parent_id"] = rhs.parent_id;
            node["parent_uuid"] = rhs.parent_uuid;
            node["type"] = rhs.type;
            node["name"] = rhs.name;
            return node;
        }

        inline static bool
        decode(const Node& node, erl::env::scene_graph::Node& rhs) {
            if (!node.IsMap()) { return false; }
            rhs.uuid = node["uuid"].as<int>();
            rhs.id = node["id"].as<int>();
            rhs.parent_id = node["parent_id"].as<int>();
            rhs.parent_uuid = node["parent_uuid"].as<int>();
            rhs.type = node["type"].as<erl::env::scene_graph::Node::Type>();
            rhs.name = node["name"].as<std::string>();
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Object> {
        inline static Node
        encode(const erl::env::scene_graph::Object& rhs) {
            Node node = convert<erl::env::scene_graph::Node>::encode(rhs);
            node["action_affordance"] = rhs.action_affordance;
            node["grid_map_min"] = rhs.grid_map_min;
            node["grid_map_max"] = rhs.grid_map_max;
            node["location"] = rhs.location;
            node["size"] = rhs.size;
            return node;
        }

        inline static bool
        decode(const Node& node, erl::env::scene_graph::Object& rhs) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::env::scene_graph::Node>::decode(node, rhs)) { return false; }
            if (rhs.type != erl::env::scene_graph::Node::Type::kObject) {
                ERL_WARN("Node type is not kObject");
                return false;
            }
            rhs.action_affordance = node["action_affordance"].as<std::vector<std::string>>();
            rhs.grid_map_min = node["grid_map_min"].as<Eigen::Vector2i>();
            rhs.grid_map_max = node["grid_map_max"].as<Eigen::Vector2i>();
            rhs.location = node["location"].as<Eigen::Vector3d>();
            rhs.size = node["size"].as<Eigen::Vector3d>();
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Room> {
        inline static Node
        encode(const erl::env::scene_graph::Room& rhs) {
            Node node = convert<erl::env::scene_graph::Node>::encode(rhs);
            node["objects"] = rhs.objects;
            node["num_objects"] = rhs.num_objects;
            node["connected_room_ids"] = rhs.connected_room_ids;
            node["connected_room_uuids"] = rhs.connected_room_uuids;
            node["door_grids"] = rhs.door_grids;
            node["grid_map_min"] = rhs.grid_map_min;
            node["grid_map_max"] = rhs.grid_map_max;
            node["location"] = rhs.location;
            node["size"] = rhs.size;
            return node;
        }

        inline static bool
        decode(const Node& node, erl::env::scene_graph::Room& rhs) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::env::scene_graph::Node>::decode(node, rhs)) { return false; }
            if (rhs.type != erl::env::scene_graph::Node::Type::kRoom) {
                ERL_WARN("Node type is not kRoom");
                return false;
            }
            rhs.objects = node["objects"].as<std::unordered_map<int, std::shared_ptr<erl::env::scene_graph::Object>>>();
            rhs.num_objects = node["num_objects"].as<uint32_t>();
            ERL_ASSERTM(rhs.objects.size() == rhs.num_objects, "Number of objects does not match");
            for (auto object_itr: rhs.objects) {
                ERL_ASSERTM(object_itr.second->id == object_itr.first, "Object %d has wrong id: %d", object_itr.first, object_itr.second->id);
            }
            rhs.connected_room_ids = node["connected_room_ids"].as<std::vector<int>>();
            rhs.connected_room_uuids = node["connected_room_uuids"].as<std::vector<int>>();
            rhs.door_grids = node["door_grids"].as<std::unordered_map<int, Eigen::Matrix2Xi>>();
            rhs.grid_map_min = node["grid_map_min"].as<Eigen::Vector2i>();
            rhs.grid_map_max = node["grid_map_max"].as<Eigen::Vector2i>();
            rhs.location = node["location"].as<Eigen::Vector3d>();
            rhs.size = node["size"].as<Eigen::Vector3d>();
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Floor> {
        inline static Node
        encode(const erl::env::scene_graph::Floor& rhs) {
            Node node = convert<erl::env::scene_graph::Node>::encode(rhs);
            node["down_stairs_id"] = rhs.down_stairs_id;
            node["up_stairs_id"] = rhs.up_stairs_id;
            node["down_stairs_uuid"] = rhs.down_stairs_uuid;
            node["up_stairs_uuid"] = rhs.up_stairs_uuid;
            node["down_stairs_cost"] = rhs.down_stairs_cost;
            node["up_stairs_cost"] = rhs.up_stairs_cost;
            node["up_stairs_portal"] = rhs.up_stairs_portal;
            node["down_stairs_portal"] = rhs.down_stairs_portal;
            node["ground_z"] = rhs.ground_z;
            node["room_map"] = rhs.room_map;
            node["cat_map"] = rhs.cat_map;
            node["rooms"] = rhs.rooms;
            node["num_rooms"] = rhs.num_rooms;
            node["grid_map_origin"] = rhs.grid_map_origin;
            node["grid_map_resolution"] = rhs.grid_map_resolution;
            node["grid_map_size"] = rhs.grid_map_size;
            return node;
        }

        inline static bool
        decode(const Node& node, erl::env::scene_graph::Floor& rhs) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::env::scene_graph::Node>::decode(node, rhs)) { return false; }
            if (rhs.type != erl::env::scene_graph::Node::Type::kFloor) {
                ERL_WARN("Node type is not kFloor");
                return false;
            }
            rhs.down_stairs_id = node["down_stairs_id"].as<int>();
            rhs.up_stairs_id = node["up_stairs_id"].as<int>();
            rhs.down_stairs_uuid = node["down_stairs_uuid"].as<int>();
            rhs.up_stairs_uuid = node["up_stairs_uuid"].as<int>();
            rhs.down_stairs_cost = node["down_stairs_cost"].as<double>();
            rhs.up_stairs_cost = node["up_stairs_cost"].as<double>();
            rhs.up_stairs_portal = node["up_stairs_portal"].as<std::optional<std::array<int, 2>>>();
            rhs.down_stairs_portal = node["down_stairs_portal"].as<std::optional<std::array<int, 2>>>();
            rhs.ground_z = node["ground_z"].as<double>();
            rhs.room_map = node["room_map"].as<std::string>();
            rhs.cat_map = node["cat_map"].as<std::string>();
            rhs.rooms = node["rooms"].as<std::unordered_map<int, std::shared_ptr<erl::env::scene_graph::Room>>>();
            rhs.num_rooms = node["num_rooms"].as<int>();
            ERL_ASSERTM(rhs.rooms.size() == std::size_t(rhs.num_rooms), "Number of rooms does not match");
            for (auto room_itr: rhs.rooms) {
                ERL_ASSERTM(room_itr.second->id == room_itr.first, "Room %d has wrong id: %d", room_itr.first, room_itr.second->id);
            }
            rhs.grid_map_origin = node["grid_map_origin"].as<Eigen::Vector2d>();
            rhs.grid_map_resolution = node["grid_map_resolution"].as<Eigen::Vector2d>();
            rhs.grid_map_size = node["grid_map_size"].as<Eigen::Vector2i>();
            return true;
        }
    };

    template<>
    struct convert<erl::env::scene_graph::Building> {
        inline static Node
        encode(const erl::env::scene_graph::Building& rhs) {
            Node node = convert<erl::env::scene_graph::Node>::encode(rhs);
            node["floors"] = rhs.floors;
            node["num_floors"] = rhs.num_floors;
            node["reference_point"] = rhs.reference_point;
            node["size"] = rhs.size;
            return node;
        }

        inline static bool
        decode(const Node& node, erl::env::scene_graph::Building& rhs) {
            if (!node.IsMap()) { return false; }
            if (!convert<erl::env::scene_graph::Node>::decode(node, rhs)) { return false; }
            if (rhs.type != erl::env::scene_graph::Node::Type::kBuilding) {
                ERL_WARN("Node type is not kBuilding");
                return false;
            }
            rhs.floors = node["floors"].as<std::unordered_map<int, std::shared_ptr<erl::env::scene_graph::Floor>>>();
            rhs.num_floors = node["num_floors"].as<int>();
            ERL_ASSERTM(rhs.floors.size() == std::size_t(rhs.num_floors), "Number of floors does not match");
            for (int i = 0; i < int(rhs.num_floors); ++i) {
                ERL_ASSERTM(rhs.floors.find(i) != rhs.floors.end(), "Floor %d is missing", i);
                ERL_ASSERTM(rhs.floors[i]->id == i, "Floor %d has wrong id: %d", i, rhs.floors[i]->id);
            }
            rhs.reference_point = node["reference_point"].as<Eigen::Vector3d>();
            rhs.size = node["size"].as<Eigen::Vector3d>();
            rhs.UpdateIdMapping();
            return true;
        }
    };
}  // namespace YAML
