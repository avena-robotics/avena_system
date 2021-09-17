#include "spawn_collision_items/spawn_methods/spawn_primitive.hpp"

namespace spawn_collision_items
{
    SpawnPrimitive::SpawnPrimitive(const rclcpp::Logger &logger)
        : ISpawnMethod(logger)
    {
    }

    SpawnPrimitive::~SpawnPrimitive()
    {
    }

    Handle SpawnPrimitive::spawn(const json &part_description, const PartAdditionalData &additional_data)
    {
        if (part_description.empty())
            return -1;

        auto obj_size = _assignObjectSizes(part_description);
        if (obj_size.empty())
            return -1;

        Handle shape_handle = -1;

        // TODO: Create primitive shape

        json part_position = part_description["pose"]["position"];
        json part_orientation = part_description["pose"]["orientation"];

        Eigen::Translation3f translation(part_position["x"], part_position["y"], part_position["z"]);
        Eigen::Quaternionf quaternion(part_orientation["w"], part_orientation["x"], part_orientation["y"], part_orientation["z"]);
        Eigen::Affine3f pose = translation * quaternion;

        // TODO: Set pose

        return shape_handle;
    }

    std::vector<float> SpawnPrimitive::_assignObjectSizes(const json &part_description)
    {
        // TODO: Check how bullet spawn items
        ShapeType_e shape = _shape_type.at(part_description["spawn_method"]);
        const json json_dims = part_description["dims"];
        std::vector<float> dims;
        if (shape == ShapeType_e::BOX)
        {
            dims = {json_dims["x"].get<float>(), json_dims["y"].get<float>(), json_dims["z"].get<float>()};
        }
        else if (shape == ShapeType_e::SPHERE)
        {
            dims = {json_dims["radius"].get<float>() * 2.0f, json_dims["radius"].get<float>() * 2.0f, json_dims["radius"].get<float>() * 2.0f};
        }
        else if (shape == ShapeType_e::CYLINDER)
        {
            dims = {json_dims["radius"].get<float>() * 2.0f, json_dims["radius"].get<float>() * 2.0f, json_dims["height"].get<float>()};
        }
        else
        {
            RCLCPP_WARN(_logger, "Unsupported shape type");
        }
        return dims;
    }
}
