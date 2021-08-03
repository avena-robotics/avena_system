#include "estimate_shape_fit_spatula/fit_spatula.hpp"

namespace estimate_shape
{
    FitSpatula::FitSpatula(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
        : IFitMethod(camera_params, args, default_parameters)
    {
        spatula_center_to_handle_distance = 9.1637e-02;
    }

    FitSpatula::~FitSpatula()
    {
    }

    int FitSpatula::fit(Item &item)
    {
        Timer timer("FitSpatula::fit", LOGGER);

        _sortParts(item);
        if (item.item_elements[0].element_label == "handle")
        { // we can perform fit if we have only handle
            if (_fitTool(item))
                return 1;

            Eigen::Vector3f spatula_position(item.pose.translation());
            Eigen::Quaternionf spatula_orientation(item.pose.rotation());
            Eigen::Vector3f z_shift = spatula_orientation.toRotationMatrix().col(2) * spatula_center_to_handle_distance;
            spatula_position = spatula_position + z_shift;
            pcl::transformPoint(spatula_position, spatula_position, item.pose.inverse()); // get transform in frame of item (which is pose of handle)

            json &part_description = item.item_elements[HANDLE].parts_description[0];
            part_description["spawn_method"] = "tool";
            part_description["fit_method"] = "spatula";
            part_description["part_name"] = item.label;
            part_description["pose"]["position"] = helpers::vision::assignJsonFromPosition(spatula_position);
            part_description["pose"]["orientation"] = {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"w", 1.0}};

            // No need for data about spatula tool part (all which is necessary is inside whole knife pose)
            item.item_elements[SPATULA_TOOL].parts_description.clear();


            return 0;
        }
        else
            return 1;
    }
    int FitSpatula::_sortParts(Item &item)
    {
        std::map<std::string, int> order;
        order[_labels[HANDLE]] = HANDLE;
        order[_labels[SPATULA_TOOL]] = SPATULA_TOOL;

        std::sort(item.item_elements.begin(), item.item_elements.end(), [&order](ItemElement &a, ItemElement &b) {
            return (order[a.element_label] < order[b.element_label]);
        });

        return 0;
    }

    int FitSpatula::setLabelData(const std::vector<Label> &labels)
    {
        for (size_t i = 0; i < labels.size(); ++i)
        {
            if (labels[i].fit_method == "spatula")
            {
                std::vector<std::string> components = labels[i].components;
                for (auto &component : components)
                    _labels.push_back(component);
            }
        }

        return 0;
    }

    int FitSpatula::_fitTool(Item &item)
    {
        Item handle_item;
        _getHandleAsItem(item, handle_item);
        FitTool fit_tool(_camera_params, _args, _default_parameters);
        if (fit_tool.fit(handle_item))
            return 1;
        item.pose = handle_item.pose;
        item.item_elements[HANDLE] = handle_item.item_elements[0];
        // FitTool fit_tool(_camera_params, _args, _default_parameters);
        // if (fit_tool.fit(item))
        //     return 1;
        return 0;
    }

    int FitSpatula::_getHandleAsItem(Item &item, Item &handle)
    {
        handle.item_elements = {item.item_elements[HANDLE]};
        return 0;
    }
} // namespace estimate_shape
