// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <list>
#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rcl_yaml_param_parser/parser.h"

#include "ros1_bridge/bridge.hpp"


void print_usage(void) {
  printf("Usage for ros2_parameter_bridge\n");
  printf("ros2_parameter_bridge ros2_parameter_file [-h]\n");
  printf("  ros2_parameter_file\n");
  printf("  ROS 2 parameter file containing a dictionary that can have the folowing keys: 'topics', 'services_2_to_1', 'services_1_to_2'\n");
  printf("    topics: an array of dictionnaries with the following keys:\n");
  printf("      topic: the name of the topic to bridge\n");
  printf("      type: the ROS 2 type of the topic to bridge\n");
  printf("      queue_size: the queue size to use (default: 100)\n");
  printf("    e.g. [{\"topic\": \"/chatter\", \"type\": \"std_msgs/msg/String\", \"queue_size\": \"2\"}]\n\n");

  printf("    services_1_to_2: an array of dictionnaries with the following keys:\n");
  printf("      service: the name of the service to bridge\n");
  printf("      package_name: the name of the ROS 2 package containung the service definition\n");
  printf("      type: the ROS 2 type of the service to bridge\n");
  printf("    e.g. [{\"topic\": \"/chatter\", \"type\": \"std_msgs/msg/String\", \"queue_size\": \"2\"}]\n\n");
  printf("    services_2_to_1: Same as services_1_to_2\n");

  printf("-h: Print this help message\n");
}

int main(int argc, char * argv[])
{

  std::list<ros1_bridge::BridgeHandles> all_handles;
  std::list<ros1_bridge::ServiceBridge1to2> service_bridges_1_to_2;
  std::list<ros1_bridge::ServiceBridge2to1> service_bridges_2_to_1;

  // bridge all topics listed in a ROS 1 parameter
  // the parameter needs to be an array
  // and each item needs to be a dictionary with the following keys;
  // topic: the name of the topic to bridge
  // type: the ROS 2 type of the topic to bridge
  // queue_size: the queue size to use (default: 100)
  const char * topics_parameter_name = "topics";
  const char * services_1_to_2_parameter_name = "services_1_to_2";
  const char * services_2_to_1_parameter_name = "services_2_to_1";
  char * bridge_parameter_file;
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Argument parsing
  if (argc < 2 || rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }
  if (argc >= 2) {
    bridge_parameter_file = argv[1];
  }
  
  rcl_params_t * params = rcl_yaml_node_struct_init(rcl_get_default_allocator());
  if (!rcl_parse_yaml_file(bridge_parameter_file, params)) {
    fprintf(stderr, "Could not parse provided yaml file '%s'", bridge_parameter_file);
    return 1;
  }

  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  // Topics
  for(auto topic : ros2_node.get_param("topics") {
    
  }

  // XmlRpc::XmlRpcValue topics;
  // if (
  //   ros1_node.getParam(topics_parameter_name, topics) &&
  //   topics.getType() == XmlRpc::XmlRpcValue::TypeArray)
  // {
  //   for (size_t i = 0; i < static_cast<size_t>(topics.size()); ++i) {
  //     std::string topic_name = static_cast<std::string>(topics[i]["topic"]);
  //     std::string type_name = static_cast<std::string>(topics[i]["type"]);
  //     size_t queue_size = static_cast<int>(topics[i]["queue_size"]);
  //     if (!queue_size) {
  //       queue_size = 100;
  //     }
  //     printf(
  //       "Trying to create bidirectional bridge for topic '%s' "
  //       "with ROS 2 type '%s'\n",
  //       topic_name.c_str(), type_name.c_str());

  //     try {
  //       ros1_bridge::BridgeHandles handles = ros1_bridge::create_bidirectional_bridge(
  //         ros1_node, ros2_node, "", type_name, topic_name, queue_size);
  //       all_handles.push_back(handles);
  //     } catch (std::runtime_error & e) {
  //       fprintf(
  //         stderr,
  //         "failed to create bidirectional bridge for topic '%s' "
  //         "with ROS 2 type '%s': %s\n",
  //         topic_name.c_str(), type_name.c_str(), e.what());
  //     }
  //   }
  // } else {
  //   fprintf(
  //     stderr,
  //     "The parameter '%s' either doesn't exist or isn't an array\n", topics_parameter_name);
  // }

  // // ROS 1 Services in ROS 2
  // XmlRpc::XmlRpcValue services_1_to_2;
  // if (
  //   ros1_node.getParam(services_1_to_2_parameter_name, services_1_to_2) &&
  //   services_1_to_2.getType() == XmlRpc::XmlRpcValue::TypeArray)
  // {
  //   for (size_t i = 0; i < static_cast<size_t>(services_1_to_2.size()); ++i) {
  //     std::string service_name = static_cast<std::string>(services_1_to_2[i]["service"]);
  //     std::string package_name = static_cast<std::string>(services_1_to_2[i]["package"]);
  //     std::string type_name = static_cast<std::string>(services_1_to_2[i]["type"]);
  //     printf(
  //       "Trying to create bridge for ROS 2 service '%s' "
  //       "with package '%s' and type '%s'\n",
  //       service_name.c_str(), package_name.c_str(), type_name.c_str());

  //     auto factory = ros1_bridge::get_service_factory(
  //       "ros2", package_name, type_name);
  //     if (factory) {
  //       try {
  //         service_bridges_1_to_2.push_back(
  //           factory->service_bridge_1_to_2(ros1_node, ros2_node, service_name));
  //         printf("Created 1 to 2 bridge for service %s\n", service_name.c_str());
  //       } catch (std::runtime_error & e) {
  //         fprintf(
  //           stderr,
  //           "failed to create bridge ROS 1 service '%s' "
  //           "with package '%s' and type '%s': %s\n",
  //           service_name.c_str(), type_name.c_str(), type_name.c_str(), e.what());
  //       }
  //     } else {
  //       fprintf(
  //         stderr,
  //         "failed to create bridge ROS 1 service '%s' "
  //         "no conversion for package '%s' and type '%s'\n",
  //         service_name.c_str(), package_name.c_str(), type_name.c_str());
  //     }
  //   }

  // } else {
  //   fprintf(
  //     stderr,
  //     "The parameter '%s' either doesn't exist or isn't an array\n",
  //     services_1_to_2_parameter_name);
  // }

  // // ROS 2 Services in ROS 1
  // XmlRpc::XmlRpcValue services_2_to_1;
  // if (
  //   ros1_node.getParam(services_2_to_1_parameter_name, services_2_to_1) &&
  //   services_2_to_1.getType() == XmlRpc::XmlRpcValue::TypeArray)
  // {
  //   for (size_t i = 0; i < static_cast<size_t>(services_2_to_1.size()); ++i) {
  //     std::string service_name = static_cast<std::string>(services_2_to_1[i]["service"]);
  //     std::string package_name = static_cast<std::string>(services_2_to_1[i]["package"]);
  //     std::string type_name = static_cast<std::string>(services_2_to_1[i]["type"]);
  //     printf(
  //       "Trying to create bridge for ROS 1 service '%s' "
  //       "with package '%s' and type '%s'\n",
  //       service_name.c_str(), package_name.c_str(), type_name.c_str());

  //     auto factory = ros1_bridge::get_service_factory(
  //       "ros1", package_name, type_name);
  //     if (factory) {
  //       try {
  //         service_bridges_2_to_1.push_back(
  //           factory->service_bridge_2_to_1(ros1_node, ros2_node, service_name));
  //         printf("Created 2 to 1 bridge for service %s\n", service_name.c_str());
  //       } catch (std::runtime_error & e) {
  //         fprintf(
  //           stderr,
  //           "failed to create bridge ROS 2 service '%s' "
  //           "with package '%s' and type '%s': %s\n",
  //           service_name.c_str(), type_name.c_str(), type_name.c_str(), e.what());
  //       }
  //     } else {
  //       fprintf(
  //         stderr,
  //         "failed to create bridge ROS 2 service '%s' "
  //         "no conversion for package '%s' and type '%s'\n",
  //         service_name.c_str(), package_name.c_str(), type_name.c_str());
  //     }
  //   }

  // } else {
  //   fprintf(
  //     stderr,
  //     "The parameter '%s' either doesn't exist or isn't an array\n",
  //     services_2_to_1_parameter_name);
  // }

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
