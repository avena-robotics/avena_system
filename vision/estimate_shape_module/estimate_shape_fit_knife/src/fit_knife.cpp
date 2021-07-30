#include "estimate_shape_fit_knife/fit_knife.hpp"

namespace estimate_shape
{
    FitKnife::FitKnife(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
        : IFitMethod(camera_params, args, default_parameters)
    {
        knife_center_to_handle_distance = 1.0382e-01;
    }

    FitKnife::~FitKnife()
    {
    }

    int FitKnife::fit(Item &item)
    {
        Timer timer("FitKnife::fit", LOGGER);

        _sortParts(item);
        if (item.item_elements[0].element_label == "handle")
        { // we can perform fit if we have only handle
            if (_fitTool(item))
                return 1;

            Eigen::Vector3f knife_position(item.pose.translation());
            Eigen::Quaternionf knife_orientation(item.pose.rotation());
            Eigen::Vector3f z_shift = knife_orientation.toRotationMatrix().col(2) * knife_center_to_handle_distance;
            knife_position = knife_position + z_shift;
            pcl::transformPoint(knife_position, knife_position, item.pose.inverse()); // get transform in frame of item (which is pose of handle) 

            json &part_description = item.item_elements[HANDLE].parts_description[0];
            part_description["spawn_method"] = "tool";
            part_description["fit_method"] = "knife";
            part_description["part_name"] = item.label;
            part_description["pose"]["position"] = helpers::vision::assignJsonFromPosition(knife_position);
            part_description["pose"]["orientation"] = {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"w", 1.0}};

            // No need for data about blade part (all which is necessary is inside whole knife pose)
            if (item.item_elements.size() == 2)
                item.item_elements[BLADE].parts_description.clear();
            return 0;
        }
        else
            return 1;
    }

    int FitKnife::setLabelData(const std::vector<Label> &labels)
    {
        for (size_t i = 0; i < labels.size(); ++i)
        {
            if (labels[i].fit_method == "knife")
            {
                std::vector<std::string> components = labels[i].components;
                for (auto &component : components)
                    _labels.push_back(component);
            }
        }

        return 0;
    }

    int FitKnife::_sortParts(Item &item)
    {
        std::map<std::string, int> order;
        order[_labels[HANDLE]] = HANDLE;
        order[_labels[BLADE]] = BLADE;

        std::sort(item.item_elements.begin(), item.item_elements.end(), [&order](ItemElement &a, ItemElement &b) {
            return (order[a.element_label] < order[b.element_label]);
        });

        return 0;
    }

    int FitKnife::_fitTool(Item &item)
    {
        Item handle_item;
        _getHandleAsItem(item, handle_item);
        FitTool fit_tool(_camera_params, _args, _default_parameters);
        if (fit_tool.fit(handle_item))
        {
            LOG_WARN_STREAM("Error occured while fitting tool to knife handle with item ID " << item.id);
            return 1;
        }
        item.pose = handle_item.pose;
        item.item_elements[HANDLE] = handle_item.item_elements[HANDLE];
        return 0;
    }

    int FitKnife::_getHandleAsItem(Item &item, Item &handle)
    {
        handle.item_elements = {item.item_elements[HANDLE]};
        return 0;
    }

} // namespace estimate_shape
