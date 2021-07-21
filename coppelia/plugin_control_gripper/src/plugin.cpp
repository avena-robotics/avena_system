#include "plugin.h"

ControlGripper::ControlGripper()
    : Node("control_gripper"),
      _parameters_read(false),
      _scene_prepared(false)
{
}

ControlGripper::~ControlGripper() {}

void ControlGripper::onStart()
{
    if (!registerScriptStuff())
        throw std::runtime_error("script stuff initialization failed");
    setExtVersion("Plugin for controling gripper in Coppelia Brain. Functionalities: change gripper, open gripper, close gripper");
    setBuildDate(BUILD_DATE);

    _change_tool_srv = create_service<ChangeToolSrv>("change_tool", std::bind(&ControlGripper::_changeToolRequest, this, std::placeholders::_1, std::placeholders::_2));
    _open_gripper_srv = create_service<OpenGripperSrv>("brain_open_gripper", std::bind(&ControlGripper::_openGripperRequest, this, std::placeholders::_1, std::placeholders::_2));
    _close_gripper_srv = create_service<CloseGripperSrv>("brain_close_gripper", std::bind(&ControlGripper::_closeGripperRequest, this, std::placeholders::_1, std::placeholders::_2));

    _readParametersFromServer();
    // _watchdog = std::make_shared<helpers::Watchdog>(this, "system_monitor");
}

void ControlGripper::onInstancePass(const sim::InstancePassFlags &flags)
{
    rclcpp::spin_some(get_node_base_interface());
}

int ControlGripper::_readParametersFromServer()
{
    if (_parameters_read)
        return 0;
    RCLCPP_INFO(get_logger(), "Reading parameters from the server");

    nlohmann::json parameters = helpers::commons::getParameter("robot");
    if (parameters.empty())
        return 1;

    const std::string working_side = parameters["working_side"];
    _robot_info = helpers::commons::getRobotInfo(working_side);

    // Get calibration mat dimensions
    _calibration_mat_dims.x_dim = parameters["calibration_mat"]["dims"]["x"].get<float>();
    _calibration_mat_dims.y_dim = parameters["calibration_mat"]["dims"]["y"].get<float>();
    _calibration_mat_dims.z_dim = parameters["calibration_mat"]["dims"]["z"].get<float>();

    // Update scene info by adding robot prefix name
    _scene_info.updatePrefixForNames(_robot_info.robot_prefix);

    _parameters_read = true;
    RCLCPP_INFO(get_logger(), "Parameters read successfully...");
    return 0;
}

void ControlGripper::_prepareScene()
{
    if (_scene_prepared)
        return;

    _scene_info.placeholder_handle = _getDummyHandle(_scene_info.placeholder_name);
    std::vector<float> placeholder_position = {0, 0, -1}; // to hide it under the floor
    simSetObjectPosition(_scene_info.placeholder_handle, -1, placeholder_position.data());
    _scene_info.calibration_mat_handle = _createCalibrationMat(_scene_info.calibration_mat_name, _calibration_mat_dims, _scene_info.placeholder_handle);
    _scene_info.connection_handle = simGetObjectHandle(_robot_info.connection.c_str());

    // Initialize gripper
    _gripper = _getGripper("franka");
    if (!_gripper)
        throw std::runtime_error("Error occured while getting gripper");
    
    _scene_prepared = true;
}

void ControlGripper::_changeToolRequest(const std::shared_ptr<ChangeToolSrv::Request> request, std::shared_ptr<ChangeToolSrv::Response> response)
{
    if (_readParametersFromServer())
    {
        RCLCPP_ERROR(get_logger(), "Parameters not read from server. Service FAILED");
        response->success = false;
        return;
    }

    try
    {
        _prepareScene();
    }
    catch(const std::runtime_error& e)
    {
        RCLCPP_ERROR(get_logger(), e.what());
        response->success = false;
        return;
    }
    
    const std::string request_tool_name = request->tool_name;
    const std::string full_name_request_tool = _robot_info.robot_prefix + "_" + request_tool_name;
    RCLCPP_INFO_STREAM(get_logger(), "Changing tool to \"" << request_tool_name << "\"");
    if (_isToolNameValid(request_tool_name))
    {
        RCLCPP_ERROR(get_logger(), "Invalid tool name. Service FAILED");
        response->success = false;
        return;
    }

    ToolPose tool_pose;
    try
    {
        if (_detachCurrentTool(full_name_request_tool, tool_pose))
        {
            RCLCPP_INFO(get_logger(), "Requested tool is already attached");
            response->success = true;
            return;
        }
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_WARN(get_logger(), e.what());
        response->success = false;
        return;
    }

    try
    {
        _attachTool(full_name_request_tool, tool_pose);
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_WARN(get_logger(), e.what());
        response->success = false;
        return;
    }

    response->success = true;
    RCLCPP_INFO(get_logger(), "Successfully change tool");
}

void ControlGripper::_openGripperRequest(const std::shared_ptr<OpenGripperSrv::Request> request, std::shared_ptr<OpenGripperSrv::Response> response)
{
    if (_readParametersFromServer())
    {
        RCLCPP_ERROR(get_logger(), "Parameters not read from server. Service FAILED");
        response->message = "Parameters not read from server";
        response->success = false;
        return;
    }

    try
    {
        _prepareScene();
    }
    catch(const std::runtime_error& e)
    {
        RCLCPP_ERROR(get_logger(), "Error occured while preparing scene for changing tool. Service FAILED");
        response->message = "Error occured while preparing scene for changing tool";
        response->success = false;
        return;
    }

    RCLCPP_INFO(get_logger(), "Opening gripper request");
    if (_gripper->open())
    {
        RCLCPP_ERROR(get_logger(), "Error occured while opening gripper");
        response->message = "Error occured while opening gripper";
        response->success = false;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Gripper opened successfully");
        response->message = "Gripper opened successfully";
        response->success = true;
    }
}

void ControlGripper::_closeGripperRequest(const std::shared_ptr<CloseGripperSrv::Request> request, std::shared_ptr<CloseGripperSrv::Response> response)
{
    if (_readParametersFromServer())
    {
        RCLCPP_ERROR(get_logger(), "Parameters not read from server. Service FAILED");
        response->message = "Parameters not read from server";
        response->success = false;
        return;
    }

    try
    {
        _prepareScene();
    }
    catch(const std::runtime_error& e)
    {
        RCLCPP_ERROR(get_logger(), "Error occured while preparing scene for changing tool. Service FAILED");
        response->message = "Error occured while preparing scene for changing tool";
        response->success = false;
        return;
    }

    RCLCPP_INFO(get_logger(), "Closing gripper request");
    if (_gripper->close())
    {
        RCLCPP_ERROR(get_logger(), "Error occured while closing gripper");
        response->message = "Error occured while closing gripper";
        response->success = false;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Gripper closed successfully");
        response->message = "Gripper closed successfully";
        response->success = true;
    }
}

Gripper::SharedPtr ControlGripper::_getGripper(const std::string &gripper_name)
{
    RCLCPP_INFO_STREAM(get_logger(), "Getting gripper with name " << gripper_name);
    Gripper::SharedPtr gripper;
    if (gripper_name == "franka")
        gripper = std::make_shared<FrankaGripper>(InputOptions{_robot_info.robot_prefix});
    else
    {
        RCLCPP_FATAL(get_logger(), "Invalid gripper name to get");
        gripper = nullptr;
    }
    return gripper;
}

int ControlGripper::_isToolNameValid(const std::string &tool_name)
{
    auto tool_name_it = std::find(_tool_names.begin(), _tool_names.end(), tool_name);
    return tool_name_it == _tool_names.end();
}

Handle ControlGripper::_getDummyHandle(const std::string &name)
{
    Handle handle = simGetObjectHandle(name.c_str());
    if (handle == -1)
    {
        handle = simCreateDummy(0.001, nullptr);
        simSetObjectName(handle, name.c_str());
    }
    return handle;
}

Handle ControlGripper::_createCalibrationMat(const std::string &calibration_mat_name, const CalibrationMatDims &calibration_mat_dims, const Handle &parent_handle)
{
    Handle calibration_mat_handle = simGetObjectHandle(calibration_mat_name.c_str());
    if (calibration_mat_handle == -1)
    {
        std::vector<float> calibration_mat_dimensions = {
            calibration_mat_dims.x_dim + _calibration_mat_dims_safety, 
            calibration_mat_dims.y_dim, 
            calibration_mat_dims.z_dim + _calibration_mat_dims_safety
        };
        calibration_mat_handle = simCreatePureShape(0, 16, calibration_mat_dimensions.data(), 0, nullptr);
        simSetObjectName(calibration_mat_handle, calibration_mat_name.c_str());
        simSetObjectSpecialProperty(calibration_mat_handle, sim_objectspecialproperty_collidable + sim_objectspecialproperty_measurable + sim_objectspecialproperty_detectable_all);
        simSetObjectParent(calibration_mat_handle, parent_handle, true);
    }
    return calibration_mat_handle;
}

int ControlGripper::_detachCurrentTool(const std::string &request_tool_name, ToolPose &tool_pose)
{
    int objects_count;
    int options = 3; // bit 0 and bit 1 // Docs: https://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetObjectsInTree.htm version. 4.1.0
    Handle *connection_children_handles = simGetObjectsInTree(_scene_info.connection_handle, sim_handle_all, options, &objects_count);

    if (!connection_children_handles)
        throw std::runtime_error("Problem occured when retrieving robot connection current tool");

    if (objects_count == 0)
    {
        RCLCPP_INFO(get_logger(), "There is currently no tool attached to arm");
        return 0;
    }

    for (int obj_nr = 0; obj_nr < objects_count; ++obj_nr)
    {
        Handle current_tool_handle = connection_children_handles[obj_nr];
        simChar *child_name_ptr = simGetObjectName(current_tool_handle);
        std::string child_name(child_name_ptr);
        simReleaseBuffer(child_name_ptr);

        // Check whether current tool is requested one
        if (child_name == request_tool_name)
            return 1;

        bool valid_tool_found = false;
        for (size_t i = 0; i < _tool_names.size(); ++i)
        {
            auto pos = child_name.find(_tool_names[i]);
            if (pos != std::string::npos)
            {
                valid_tool_found = true;
                break;
            }
        }

        if (valid_tool_found)
        {
            simGetObjectPosition(current_tool_handle, _scene_info.connection_handle, tool_pose.position.data());
            simGetObjectOrientation(current_tool_handle, _scene_info.connection_handle, tool_pose.orientation.data());
            simSetObjectParent(current_tool_handle, _scene_info.placeholder_handle, false);
        }
        else
            throw std::runtime_error("Error occured while getting currently attached tool");
    }

    simReleaseBuffer(reinterpret_cast<const char *>(connection_children_handles));
    return 0;
}

void ControlGripper::_attachTool(const std::string &name, const ToolPose &tool_pose)
{
    Handle tool_to_attach_handle = simGetObjectHandle(name.c_str());
    if (tool_to_attach_handle == -1)
        throw std::runtime_error("Error occured when getting handle to tool to attach");

    if (simSetObjectPosition(tool_to_attach_handle, _scene_info.connection_handle, tool_pose.position.data()) == -1)
        throw std::runtime_error("Error occured when setting position of tool to attach");

    if (simSetObjectOrientation(tool_to_attach_handle, _scene_info.connection_handle, tool_pose.orientation.data()) == -1)
        throw std::runtime_error("Error occured when setting orientation of tool to attach");

    if (simSetObjectParent(tool_to_attach_handle, _scene_info.connection_handle, true) == -1)
        throw std::runtime_error("Error occured when setting parent of tool to attach");
}

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, ControlGripper)
#include "stubsPlusPlus.cpp"
