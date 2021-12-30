#include "kinematics/forward_kinematics.hpp"

namespace kinematics
{
    ForwardKinematics::ForwardKinematics(physics_client_handler::PhysicsClientHandler::SharedPtr physics_server_handler,
                                         helpers::commons::RobotInfo &robot_info,
                                         const rclcpp::Logger &logger)
        : _physics_server_handler(physics_server_handler),
          _robot_info(robot_info),
          _logger(logger)
    {
        if (!_physics_server_handler)
            throw IkError("Physics client handler is not initialized");

        // Get transform to base link
        if (auto robot_base_tf = helpers::vision::getTransformAffine("world", _robot_info.base_link_name))
        {
            _reference_frame = robot_base_tf->cast<double>();
            RCLCPP_INFO(_logger, "[IK]: Using \"world\" as reference frame for end effector pose");
        }
        else
            throw IkError("Cannot read transform for robot base");
    }

    EEConfiguration ForwardKinematics::computeFk(const ArmConfiguration &joint_states,
                                                 bool in_robot_base_frame)
    {
        std::function<void(const IkReal *, IkReal *, IkReal *)> fk_func;
        if (_robot_info.robot_name == "franka")
        {
            fk_func = ik_franka::ComputeFk;
        }
        else if (_robot_info.robot_name == "avena")
        {
            fk_func = ik_avena::ComputeFk;
        }
        else
        {
            throw IkError("Unsupported robot to calculate IK");
        }

        EEConfiguration out;
        std::vector<IkReal> ee_trans(3);
        std::vector<IkReal> ee_rot(9);
        fk_func(joint_states.data(), ee_trans.data(), ee_rot.data());
        Eigen::Translation3d position(ee_trans[0], ee_trans[1], ee_trans[2]);
        Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Map(ee_rot.data());
        rotation_matrix.transposeInPlace();
        Eigen::Affine3d end_effector_pose_robot_frame = position * Eigen::Quaterniond(rotation_matrix);

        Eigen::Affine3d end_effector_pose;

        // Convert end effector pose to "world" frame because IK Fast
        // returns pose in robot base frame of reference
        if (!in_robot_base_frame)
            end_effector_pose = _reference_frame * end_effector_pose_robot_frame;
        else
            end_effector_pose = end_effector_pose_robot_frame;

        out.trans = end_effector_pose;
        out.rot = rotation_matrix;
        return out;
    }

    void ForwardKinematics::setReferenceFrame(const Eigen::Affine3d &reference_frame)
    {
        _reference_frame = reference_frame;
    }

} // namespace kinematics
