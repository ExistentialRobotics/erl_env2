#pragma once

#include "erl_common/yaml.hpp"

// namespace erl::env {

// struct AtomicProposition : public common::Yamlable<AtomicProposition> {
//     enum class Type {
//         kNA = 0,
//         kEnterRoom = 1,
//         kReachObject = 2,
//     };

// Type type = Type::kNA;
// int uuid = -1;
// double reach_distance = -1.0;

// AtomicProposition() = default;

// AtomicProposition(Type type, int uuid, double reach_distance)
//     : type(type), uuid(uuid), reach_distance(reach_distance) {}
// };
// }  // namespace erl::env

// template<>
// struct YAML::convert<erl::env::AtomicProposition> {
//     static Node
//     encode(const erl::env::AtomicProposition& rhs) {
//         Node node;
//         ERL_YAML_SAVE_ATTR(node, rhs, expr);
//         return node;
//     }

// static bool
// decode(const Node& node, erl::env::AtomicProposition& rhs) {
//     if (!node.IsMap()) { return false; }
//     ERL_YAML_LOAD_ATTR(node, rhs, expr);
//     return true;
// }
// };  // namespace YAML
