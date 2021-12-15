#pragma once
#include "parameters_list.h"
#include <string>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include<iostream>
#include<safety_checks.h>

inline void set_parameters(const std::string setpoint_path) {
  // Safety check, see if file exists
  safety_checks::yaml_file_check(setpoint_path);


  // Load yaml file containing gains
  YAML::Node commands_yaml = YAML::LoadFile(setpoint_path);

  // Set setpoints
  parameters::objects = commands_yaml["objects"].as<std::vector<std::string>>();
  parameters::topic_prefix = commands_yaml["topic_prefix"].as<std::string>();
  // parameters::topics = commands_yaml["topics"].as<std::vector<std::string>>();

}