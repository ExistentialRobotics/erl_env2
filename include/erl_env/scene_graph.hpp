#pragma once

#include "erl_common/yaml.hpp"

namespace erl::env::scene_graph {
    struct Node {
        inline static uint32_t uuid_counter = 0;

    public:
        enum class Type {
            kNA = 0,
            kObject = 1,
            kRoom = 2,
            kFloor = 3,
            kBuilding = 4,
        };

        uint32_t uuid = -1;     // unique scene graph element id
        int id = -1;            // unique id of the same type
        int parent_id = -1;     // parent node id (not uuid)
        int parent_uuid = -1;   // parent node uuid
        Type type = Type::kNA;  // type of the node
        std::string name = {};  // node name

        Node()
            : uuid(uuid_counter++) {}
    };

    struct Object : public Node, public common::Yamlable<Object> {
        std::vector<std::string> action_affordance = {};  // action affordances
        Eigen::Vector3d location = {};                    // object center location
        Eigen::Vector3d size = {};                        // object size
    };

    struct Room : public Node, public common::Yamlable<Room> {
        std::unordered_map<int, std::shared_ptr<Object>> objects = {};  // objects
        uint32_t num_objects = 0;                                       // number of objects
        std::vector<int> connected_room_ids = {};                       // neighbor room ids
        Eigen::Vector2i grid_map_min = {};                              // grid_map min
        Eigen::Vector2i grid_map_max = {};                              // grid_map max
        Eigen::Vector3d location = {};                                  // room center location
        Eigen::Vector3d size = {};                                      // room size
    };

    struct Floor : public Node, public common::Yamlable<Floor> {
        int down_stairs_id = -1;                               // down stairs id
        int up_stairs_id = -1;                                 // up stairs id
        double ground_z = -1;                                  // ground z coordinate
        std::string room_map = {};                             // relative path of room segmentation map
        std::string cat_map = {};                              // relative path of object segmentation map
        std::unordered_map<int, std::shared_ptr<Room>> rooms;  // rooms
        uint32_t num_rooms = 0;                                // number of rooms
        Eigen::Vector2d grid_map_origin = {};                  // grid_map origin
        Eigen::Vector2d grid_map_resolution = {};              // grid_map resolution
        Eigen::Vector2i grid_map_size = {};                    // grid_map size
    };

    struct Building : public Node, public common::Yamlable<Building> {
        std::unordered_map<int, std::shared_ptr<Floor>> floors = {};  // floors
        uint32_t num_floors = 0;                                      // number of floors
        Eigen::Vector3d reference_point = {};                         // reference 3d coordinate
        Eigen::Vector3d size = {};                                    // building size

    private:
        std::unordered_map<int, std::shared_ptr<Object>> id_to_object = {};
        std::unordered_map<int, std::shared_ptr<Room>> id_to_room = {};
        std::unordered_map<uint32_t, std::shared_ptr<Node>> uuid_to_node = {};

    public:
        void
        UpdateIdMapping() {
            id_to_object.clear();
            id_to_room.clear();
            uuid_to_node.clear();
            for (auto& floor_itr: floors) {
                auto& floor = floor_itr.second;
                ERL_ASSERTM(uuid_to_node.try_emplace(floor->uuid, floor).second, "Duplicate floor uuid: %d", floor->uuid);
                for (auto& room_itr: floor->rooms) {
                    auto& room = room_itr.second;
                    ERL_ASSERTM(id_to_room.try_emplace(room->id, room).second, "Duplicate room id: %d", room->id);
                    ERL_ASSERTM(uuid_to_node.try_emplace(room->uuid, room).second, "Duplicate room uuid: %d", room->uuid);
                    for (auto& object_itr: room->objects) {
                        auto& object = object_itr.second;
                        ERL_ASSERTM(id_to_object.try_emplace(object->id, object).second, "Duplicate object id: %d", object->id);
                        ERL_ASSERTM(uuid_to_node.try_emplace(object->uuid, object).second, "Duplicate object uuid: %d", object->uuid);
                    }
                }
            }
        }

        std::shared_ptr<Object>
        GetObject(int object_id) {
            if (id_to_object.find(object_id) == id_to_object.end()) { return nullptr; }
            return id_to_object[object_id];
        }

        std::shared_ptr<Room>
        GetRoom(int room_id) {
            if (id_to_room.find(room_id) == id_to_room.end()) { return nullptr; }
            return id_to_room[room_id];
        }

        std::shared_ptr<Floor>
        GetFloor(int floor_id) {
            if (floors.find(floor_id) == floors.end()) { return nullptr; }
            return floors[floor_id];
        }

        template<class T>
        std::shared_ptr<T>
        GetNode(uint32_t uuid) {
            if (uuid_to_node.find(uuid) == uuid_to_node.end()) { return nullptr; }
            return std::dynamic_pointer_cast<T>(uuid_to_node[uuid]);
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
                case erl::env::scene_graph::Node::Type::kNA:
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
            auto type = node.as<std::string>();
            if (type == "kNA") {
                rhs = erl::env::scene_graph::Node::Type::kNA;
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

    inline Emitter&
    operator<<(Emitter& out, const erl::env::scene_graph::Node::Type& rhs) {
        switch (rhs) {
            case erl::env::scene_graph::Node::Type::kNA:
                out << "kNA";
                break;
            case erl::env::scene_graph::Node::Type::kObject:
                out << "kObject";
                break;
            case erl::env::scene_graph::Node::Type::kRoom:
                out << "kRoom";
                break;
            case erl::env::scene_graph::Node::Type::kFloor:
                out << "kFloor";
                break;
            case erl::env::scene_graph::Node::Type::kBuilding:
                out << "kBuilding";
                break;
        }
        return out;
    }

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
            rhs.uuid = node["uuid"].as<uint32_t>();
            rhs.id = node["id"].as<int>();
            rhs.parent_id = node["parent_id"].as<int>();
            rhs.parent_uuid = node["parent_uuid"].as<int>();
            rhs.type = node["type"].as<erl::env::scene_graph::Node::Type>();
            rhs.name = node["name"].as<std::string>();
            return true;
        }
    };

    inline Emitter&
    operator<<(Emitter& out, const erl::env::scene_graph::Node& rhs) {
        out << BeginMap;
        out << Key << "uuid" << Value << rhs.uuid;
        out << Key << "id" << Value << rhs.id;
        out << Key << "parent_id" << Value << rhs.parent_id;
        out << Key << "parent_uuid" << Value << rhs.parent_uuid;
        out << Key << "type" << Value << rhs.type;
        out << Key << "name" << Value << rhs.name;
        out << EndMap;
        return out;
    }

    template<>
    struct convert<erl::env::scene_graph::Object> {
        inline static Node
        encode(const erl::env::scene_graph::Object& rhs) {
            Node node = convert<erl::env::scene_graph::Node>::encode(rhs);
            node["location"] = rhs.location;
            node["size"] = rhs.size;
            node["action_affordance"] = rhs.action_affordance;
            return node;
        }

        inline static bool
        decode(const Node& node, erl::env::scene_graph::Object& rhs) {
            if (!convert<erl::env::scene_graph::Node>::decode(node, rhs)) { return false; }
            if (rhs.type != erl::env::scene_graph::Node::Type::kObject) {
                ERL_WARN("Node type is not kObject");
                return false;
            }
            rhs.location = node["location"].as<Eigen::Vector3d>();
            rhs.size = node["size"].as<Eigen::Vector3d>();
            rhs.action_affordance = node["action_affordance"].as<std::vector<std::string>>();
            return true;
        }
    };

    inline Emitter&
    operator<<(Emitter& out, const erl::env::scene_graph::Object& rhs) {
        out << static_cast<const erl::env::scene_graph::Node&>(rhs);
        out << BeginMap;
        out << Key << "location" << Value << rhs.location;
        out << Key << "size" << Value << rhs.size;
        out << Key << "action_affordance" << Value << rhs.action_affordance;
        out << EndMap;
        return out;
    }

    template<>
    struct convert<erl::env::scene_graph::Room> {
        inline static Node
        encode(const erl::env::scene_graph::Room& rhs) {
            Node node = convert<erl::env::scene_graph::Node>::encode(rhs);
            node["objects"] = rhs.objects;
            node["num_objects"] = rhs.num_objects;
            node["connected_room_ids"] = rhs.connected_room_ids;
            node["grid_map_min"] = rhs.grid_map_min;
            node["grid_map_max"] = rhs.grid_map_max;
            node["location"] = rhs.location;
            node["size"] = rhs.size;
            return node;
        }

        inline static bool
        decode(const Node& node, erl::env::scene_graph::Room& rhs) {
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
            rhs.grid_map_min = node["grid_map_min"].as<Eigen::Vector2i>();
            rhs.grid_map_max = node["grid_map_max"].as<Eigen::Vector2i>();
            rhs.location = node["location"].as<Eigen::Vector3d>();
            rhs.size = node["size"].as<Eigen::Vector3d>();
            return true;
        }
    };

    inline Emitter&
    operator<<(Emitter& out, const erl::env::scene_graph::Room& rhs) {
        out << static_cast<const erl::env::scene_graph::Node&>(rhs);
        out << BeginMap;
        out << Key << "objects" << Value << rhs.objects;
        out << Key << "num_objects" << Value << rhs.num_objects;
        out << Key << "location" << Value << rhs.location;
        out << Key << "size" << Value << rhs.size;
        out << EndMap;
        return out;
    }

    template<>
    struct convert<erl::env::scene_graph::Floor> {
        inline static Node
        encode(const erl::env::scene_graph::Floor& rhs) {
            Node node = convert<erl::env::scene_graph::Node>::encode(rhs);
            node["down_stairs_id"] = rhs.down_stairs_id;
            node["up_stairs_id"] = rhs.up_stairs_id;
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
            if (!convert<erl::env::scene_graph::Node>::decode(node, rhs)) { return false; }
            if (rhs.type != erl::env::scene_graph::Node::Type::kFloor) {
                ERL_WARN("Node type is not kFloor");
                return false;
            }
            rhs.down_stairs_id = node["down_stairs_id"].as<int>();
            rhs.up_stairs_id = node["up_stairs_id"].as<int>();
            rhs.ground_z = node["ground_z"].as<double>();
            rhs.room_map = node["room_map"].as<std::string>();
            rhs.cat_map = node["cat_map"].as<std::string>();
            rhs.rooms = node["rooms"].as<std::unordered_map<int, std::shared_ptr<erl::env::scene_graph::Room>>>();
            rhs.num_rooms = node["num_rooms"].as<uint32_t>();
            ERL_ASSERTM(rhs.rooms.size() == rhs.num_rooms, "Number of rooms does not match");
            for (auto room_itr: rhs.rooms) {
                ERL_ASSERTM(room_itr.second->id == room_itr.first, "Room %d has wrong id: %d", room_itr.first, room_itr.second->id);
            }
            rhs.grid_map_origin = node["grid_map_origin"].as<Eigen::Vector2d>();
            rhs.grid_map_resolution = node["grid_map_resolution"].as<Eigen::Vector2d>();
            rhs.grid_map_size = node["grid_map_size"].as<Eigen::Vector2i>();
            return true;
        }
    };

    inline Emitter&
    operator<<(Emitter& out, const erl::env::scene_graph::Floor& rhs) {
        out << static_cast<const erl::env::scene_graph::Node&>(rhs);
        out << BeginMap;
        out << Key << "down_stairs_id" << Value << rhs.down_stairs_id;
        out << Key << "up_stairs_id" << Value << rhs.up_stairs_id;
        out << Key << "ground_z" << Value << rhs.ground_z;
        out << Key << "room_map" << Value << rhs.room_map;
        out << Key << "cat_map" << Value << rhs.cat_map;
        out << Key << "rooms" << Value << rhs.rooms;
        out << Key << "num_rooms" << Value << rhs.num_rooms;
        out << Key << "grid_map_origin" << Value << rhs.grid_map_origin;
        out << Key << "grid_map_resolution" << Value << rhs.grid_map_resolution;
        out << Key << "grid_map_size" << Value << rhs.grid_map_size;
        out << EndMap;
        return out;
    }

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
            if (!convert<erl::env::scene_graph::Node>::decode(node, rhs)) { return false; }
            if (rhs.type != erl::env::scene_graph::Node::Type::kBuilding) {
                ERL_WARN("Node type is not kBuilding");
                return false;
            }
            rhs.floors = node["floors"].as<std::unordered_map<int, std::shared_ptr<erl::env::scene_graph::Floor>>>();
            rhs.num_floors = node["num_floors"].as<uint32_t>();
            ERL_ASSERTM(rhs.floors.size() == rhs.num_floors, "Number of floors does not match");
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
