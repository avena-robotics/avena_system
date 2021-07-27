// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef COMPOSE_ITEMS_DEBUG__COMPONENT_HPP_
#define COMPOSE_ITEMS_DEBUG__COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <mysql_connector.h>
#include <assert.h>
#include <sstream>

#include "custom_interfaces/msg/items.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"

#include <memory>

namespace ros2mysql
{

  class ComposeItems : public rclcpp::Node
  {
  public:
    COMPOSITION_PUBLIC
    explicit ComposeItems(const rclcpp::NodeOptions &options);

  private:
    /**;
   * Configures component.
   *
   * Declares parameters and configures video capture.
   */
    COMPOSITION_PUBLIC
    void _configure();

    /**;
      * Declares the parameter using rcl_interfaces.
      */
    COMPOSITION_PUBLIC
    void _initializeParameters();

    uint32_t _getItemsIdx();

    std::unique_ptr<MysqlConnector> _db;
    std::string _host;
    std::string _port;
    std::string _db_name;
    std::string _username;
    std::string _password;
    bool _debug;
    rclcpp::Subscription<custom_interfaces::msg::Items>::SharedPtr _sub;

    // std::map<uint32_t, std::string> _labels;
  };

} // namespace ros2mysql

#endif // DETECT_DEBUG__COMPONENT_HPP_
