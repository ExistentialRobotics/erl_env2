#pragma once

#include "differential_drive_model.hpp"
#include "motion_primitive.hpp"

namespace erl::env {
    typedef MotionPrimitive<DifferentialDriveControl> DdcMotionPrimitive;
}  // namespace erl::env

namespace YAML {

    template<>
    struct convert<erl::env::DdcMotionPrimitive> {
        static Node
        encode(const erl::env::DdcMotionPrimitive &primitive) {
            Node node;
            node["controls"] = primitive.controls;
            node["durations"] = primitive.durations;
            node["costs"] = primitive.costs;
            return node;
        }

        static bool
        decode(const Node &node, erl::env::DdcMotionPrimitive &primitive) {
            if (!node.IsMap()) { return false; }
            if (!node["controls"]) { return false; }
            if (!node["controls"].IsSequence()) { return false; }
            if (!node["durations"]) { return false; }
            if (!node["durations"].IsSequence()) { return false; }
            if (!node["costs"]) { return false; }
            if (!node["costs"].IsSequence()) { return false; }
            primitive.controls = node["controls"].as<std::vector<erl::env::DifferentialDriveControl>>();
            primitive.durations = node["durations"].as<std::vector<double>>();
            primitive.costs = node["costs"].as<std::vector<double>>();
            return true;
        }
    };

    // This is a faster way to dump DdcMotionPrimitive as YAML format to a stream.
    // An alternative way is to use YAML::convert<erl::env::DdcMotionPrimitive>::encode(erl::env::DdcMotionPrimitive) -> YAML::Node and YAML::Dump(YAML::Node).
    inline Emitter &
    operator<<(Emitter &out, const erl::env::DdcMotionPrimitive &mp) {
        out << BeginMap;
        out << Key << "control" << Value << mp.controls;
        out << Key << "duration" << Value << mp.durations;
        out << Key << "cost" << Value << mp.costs;
        out << EndMap;
        return out;
    }
}  // namespace YAML

namespace erl::env {
    inline std::vector<DdcMotionPrimitive>
    LoadDdcMotionPrimitivesFromYaml(const std::string &filename) {
        std::vector<YAML::Node> nodes = YAML::LoadAllFromFile(filename);

        if (nodes.empty()) { throw std::runtime_error("No motion primitives found in file: " + filename); }

        if (nodes.size() == 1) {
            auto &node = nodes[0];
            if (node.IsSequence()) {  // this is a sequence node of a bunch of motion primitives
                return node.as<std::vector<DdcMotionPrimitive>>();
            } else {  // this is a single motion primitive
                return {node.as<DdcMotionPrimitive>()};
            }
        } else {  // this file contains multiple nodes, each of which is a motion primitive
            std::vector<DdcMotionPrimitive> primitives(nodes.size());
            std::transform(nodes.begin(), nodes.end(), primitives.begin(), [](const YAML::Node &node) { return node.as<DdcMotionPrimitive>(); });
            return primitives;
        }
    }
}  // namespace erl::env
