// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <yaml-cpp/yaml.h>
#include <fnmatch.h>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_sensors_description
{
// Native GPU LiDAR intensity stores a 16-bit index into the external class table.
class SemanticLabels
{
public:
  explicit SemanticLabels(const std::string & path)
  {
    const auto config = YAML::LoadFile(path);
    if (!config["classes"].IsMap() || !config["rules"].IsSequence()) {
      throw std::runtime_error("Semantic config requires classes and rules");
    }
    std::map<int, std::string> classes;
    for (const auto & entry : config["classes"]) {
      const int id = entry.first.as<int>();
      const auto name = entry.second.as<std::string>();
      if (id < 0 || id > 65534 || name.empty()) {
        throw std::runtime_error("Semantic class IDs must be 0..65534 with names");
      }
      classes.emplace(id, name);
    }
    if (classes.count(0) == 0 || classes.size() > 65535) {
      throw std::runtime_error("Define class 0 and at most 65535 semantic classes");
    }
    for (const auto & entry : classes) {
      indices_[entry.first] = decode_.size();
      decode_.push_back(entry.first);
    }
    for (const auto & entry : config["rules"]) {
      const auto pattern = entry["visual"].as<std::string>();
      const int id = entry["class_id"].as<int>();
      if (pattern.empty() || !indices_.count(id)) {
        throw std::runtime_error("Semantic rule references an undefined class");
      }
      rules_.push_back({pattern, indices_.at(id)});
    }
  }

  int IndexForVisual(std::string name, const std::string & world) const
  {
    const auto prefix = world + "::";
    if (name.compare(0, prefix.size(), prefix) == 0) {name.erase(0, prefix.size());}
    for (const auto & rule : rules_) {
      if (fnmatch(rule.pattern.c_str(), name.c_str(), 0) == 0) {return rule.index;}
    }
    return 0;
  }

  uint16_t Decode(uint16_t index) const {return decode_.at(index);}

private:
  struct Rule {std::string pattern; int index;};
  std::vector<Rule> rules_;
  std::map<int, int> indices_;
  std::vector<uint16_t> decode_;
};
}  // namespace robot_sensors_description
