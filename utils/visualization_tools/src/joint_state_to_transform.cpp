#include "visualization_tools/joint_state_to_transform.hpp"

namespace visualization_tools
{
    namespace
    {
        inline geometry_msgs::msg::TransformStamped kdlToTransform(const KDL::Frame &k)
        {
            geometry_msgs::msg::TransformStamped t;
            t.transform.translation.x = k.p.x();
            t.transform.translation.y = k.p.y();
            t.transform.translation.z = k.p.z();
            k.M.GetQuaternion(t.transform.rotation.x, t.transform.rotation.y,
                              t.transform.rotation.z, t.transform.rotation.w);
            return t;
        }

    } // namespace

    void JointStateToTransform::initModel(const std::string &urdf_xml)
    {
        _setupURDF(urdf_xml);
        _fixed_transforms = _getFixedTransformsImpl();
    }

    std::vector<geometry_msgs::msg::TransformStamped> JointStateToTransform::getMovingTransforms(
        std::map<std::string, double> &joint_positions, const builtin_interfaces::msg::Time &time)
    {
        std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

        for (const std::pair<const std::string, urdf::JointMimicSharedPtr> &i : _mimic)
        {
            if (joint_positions.find(i.second->joint_name) != joint_positions.end())
            {
                double pos = joint_positions[i.second->joint_name] * i.second->multiplier + i.second->offset;
                joint_positions.insert(std::make_pair(i.first, pos));
            }
        }

        // loop over all joints
        for (const std::pair<const std::string, double> &jnt : joint_positions)
        {
            std::map<std::string, SegmentPair>::iterator seg = _segments.find(jnt.first);
            if (seg != _segments.end())
            {
                geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg->second.segment.pose(jnt.second));
                tf_transform.header.stamp = time;
                tf_transform.header.frame_id = seg->second.root;
                tf_transform.child_frame_id = seg->second.tip;
                tf_transforms.push_back(tf_transform);
            }
        }
        return tf_transforms;
    }

    void JointStateToTransform::_setupURDF(const std::string &urdf_xml)
    {
        _model = std::make_unique<urdf::Model>();

        // Initialize the model
        if (!_model->initString(urdf_xml))
        {
            throw std::runtime_error("Unable to initialize urdf::model from robot description");
        }

        // Initialize the KDL tree
        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(*_model, tree))
        {
            throw std::runtime_error("Failed to extract kdl tree from robot description");
        }

        // Initialize the mimic map
        _mimic.clear();
        for (const std::pair<const std::string, urdf::JointSharedPtr> &i : _model->joints_)
        {
            if (i.second->mimic)
            {
                _mimic.insert(std::make_pair(i.first, i.second->mimic));
            }
        }

        KDL::SegmentMap segments_map = tree.getSegments();
        for (const std::pair<const std::string, KDL::TreeElement> &segment : segments_map)
        {
            RCLCPP_INFO(JOINT_STATE_TO_TF_LOGGER, "got segment %s", segment.first.c_str());
        }

        // walk the tree and add segments to segments_
        _segments.clear();
        _segments_fixed.clear();
        _addChildren(tree.getRootSegment());
    }

    // add children to correct maps
    void JointStateToTransform::_addChildren(const KDL::SegmentMap::const_iterator segment)
    {
        const std::string &root = GetTreeElementSegment(segment->second).getName();

        std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
        for (unsigned int i = 0; i < children.size(); i++)
        {
            const KDL::Segment &child = GetTreeElementSegment(children[i]->second);
            SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
            if (child.getJoint().getType() == KDL::Joint::None)
            {
                if (_model->getJoint(child.getJoint().getName()) &&
                    _model->getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
                {
                    RCLCPP_INFO(JOINT_STATE_TO_TF_LOGGER, "Floating joint. Not adding segment from %s to %s.",
                                root.c_str(), child.getName().c_str());
                }
                else
                {
                    _segments_fixed.insert(make_pair(child.getJoint().getName(), s));
                    RCLCPP_DEBUG(JOINT_STATE_TO_TF_LOGGER, "Adding fixed segment from %s to %s",
                                 root.c_str(), child.getName().c_str());
                }
            }
            else
            {
                _segments.insert(make_pair(child.getJoint().getName(), s));
                RCLCPP_DEBUG(JOINT_STATE_TO_TF_LOGGER, "Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
            }
            _addChildren(children[i]);
        }
    }

    std::vector<geometry_msgs::msg::TransformStamped> JointStateToTransform::_getFixedTransformsImpl()
    {
        RCLCPP_DEBUG(JOINT_STATE_TO_TF_LOGGER, "Get transforms for fixed joints");
        std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

        // loop over all fixed segments
        for (const std::pair<const std::string, SegmentPair> &seg : _segments_fixed)
        {
            geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg.second.segment.pose(0));
            tf_transform.header.frame_id = seg.second.root;
            tf_transform.child_frame_id = seg.second.tip;
            tf_transforms.push_back(tf_transform);
        }

        return tf_transforms;
    }

} // namespace visualization_tools
