#include "path_buffer/path_buffer.hpp"

namespace path_buffer
{
  PathBuffer::PathBuffer(const rclcpp::NodeOptions &options)
      : Node("path_buffer", options)
  {
    RCLCPP_INFO(get_logger(), "Initialization of path buffer.");
    rclcpp::QoS qos_latching = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    auto _input_data_callback = [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr input_data) -> void {
      // RCLCPP_INFO(get_logger(), "Received path data.");
      _input_msg_data = input_data;
    };

    _pub_path = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("generated_path", qos_latching);
    _sub_path = create_subscription<trajectory_msgs::msg::JointTrajectory>("generated_path", qos_latching, _input_data_callback);

    _action_server_set = rclcpp_action::create_server<custom_interfaces::action::PathBuffer>(this,
                                                                                             "path_buffer_set",
                                                                                             std::bind(&PathBuffer::_handle_goal_set, this, std::placeholders::_1, std::placeholders::_2),
                                                                                             std::bind(&PathBuffer::_handle_cancel_set, this, std::placeholders::_1),
                                                                                             std::bind(&PathBuffer::_handle_accepted_set, this, std::placeholders::_1));

    _action_server_get = rclcpp_action::create_server<custom_interfaces::action::PathBuffer>(this,
                                                                                             "path_buffer_get",
                                                                                             std::bind(&PathBuffer::_handle_goal_get, this, std::placeholders::_1, std::placeholders::_2),
                                                                                             std::bind(&PathBuffer::_handle_cancel_get, this, std::placeholders::_1),
                                                                                             std::bind(&PathBuffer::_handle_accepted_get, this, std::placeholders::_1));
    // _watchdog = std::make_shared<helpers::Watchdog>(this, "system_monitor");
    std::cout << "...path buffer init done" << std::endl;
  }

  rclcpp_action::GoalResponse PathBuffer::_handle_goal_set(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PathBufferAction::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Goal ACCEPTED");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse PathBuffer::_handle_cancel_set(const std::shared_ptr<GoalHandlePathBuffer> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal canceled");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void PathBuffer::_handle_accepted_set(const std::shared_ptr<GoalHandlePathBuffer> goal_handle)
  {

    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    const auto goal = goal_handle->get_goal();
    std::string path_name = goal_handle.get()->get_goal()->path_name;
    trajectory_msgs::msg::JointTrajectory path_to_save = goal_handle.get()->get_goal()->generated_path;
    auto feedback = std::make_shared<PathBufferAction::Feedback>();
    auto result = std::make_shared<PathBufferAction::Result>();
    if (path_name == "list")
    {
      RCLCPP_INFO(get_logger(), "Saved path data :");
      for (auto path : _paths)
        RCLCPP_INFO(get_logger(), "Path name : %s", path.first.c_str());
      goal_handle->succeed(result);
      return;
    }
    if (path_to_save == trajectory_msgs::msg::JointTrajectory())
    {
      RCLCPP_INFO(get_logger(), "No path was passed in request, reading data from subsrcriber");
      if (_validateInput())
      {
        RCLCPP_ERROR(this->get_logger(), "Goal aborted invalid data");
        goal_handle->abort(result);
        return;
      }
      // saving the path with name passed in goal
      _paths[path_name] = *_input_msg_data;
      RCLCPP_INFO(get_logger(), "Path successfully saved as %s ", path_name.c_str());
      if (rclcpp::ok())
      {
        // get from subscriber
        // store in map
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded.");
        return;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Saving data passsed from request");
      // saving the path with name passed in goal
      _paths[path_name] = path_to_save;
      RCLCPP_INFO(get_logger(), "Path successfully saved as %s ", path_name.c_str());
      if (rclcpp::ok())
      {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded.");
        return;
      }
    }

    // throw std::runtime_error("Problem with ROS");

    RCLCPP_INFO(this->get_logger(), "finished");
  }

  rclcpp_action::GoalResponse PathBuffer::_handle_goal_get(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PathBufferAction::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse PathBuffer::_handle_cancel_get(const std::shared_ptr<GoalHandlePathBuffer> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal canceled");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void PathBuffer::_handle_accepted_get(const std::shared_ptr<GoalHandlePathBuffer> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    const auto goal = goal_handle->get_goal();
    std::string path_name = goal_handle.get()->get_goal()->path_name;
    auto feedback = std::make_shared<PathBufferAction::Feedback>();
    auto result = std::make_shared<PathBufferAction::Result>();
    if (path_name == "list")
    {
      RCLCPP_INFO(get_logger(), "Saved path data :");
      for (auto path : _paths)
        RCLCPP_INFO(get_logger(), "Path name : %s", path.first.c_str());
      goal_handle->succeed(result);
      return;
    }
    if (_validateInput())
    {
      RCLCPP_ERROR(this->get_logger(), "Goal canceled");
      goal_handle->abort(result);
      return;
    }
    auto path = _paths.find(path_name);
    if (path != _paths.end())
      RCLCPP_INFO(get_logger(), "Selected path name is valid .");
    else
    {
      RCLCPP_ERROR(get_logger(), "Selected path is invalid valid . %s", path_name.c_str());
      goal_handle->abort(result);
      return;
    }
    if (rclcpp::ok())
    {
      // get from map
      _pub_path->publish(path->second);
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded.");
    }
    else
      throw std::runtime_error("Problem with ROS");

    RCLCPP_INFO(this->get_logger(), "finished");
  }
  int PathBuffer::_validateInput()
  {
    if (!_input_msg_data)
    {
      RCLCPP_WARN(get_logger(), "Module has not received data yet.");
      return 1;
    }
    if (_input_msg_data->header.stamp == builtin_interfaces::msg::Time())
    {
      RCLCPP_WARN(get_logger(), "Invalid input message header.");
      return 1;
    }
    return 0;
  }

} // namespace path_buffer
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_buffer::PathBuffer)
