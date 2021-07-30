#include "estimate_shape_manager/estimate_shape_manager.hpp"

namespace estimate_shape
{
    EstimateShapeManager::EstimateShapeManager(const std::vector<CameraParameters> &cams_params, const std::vector<Label> &labels)
        : _camera_parameters(cams_params),
          _labels(labels)
    {
        Timer timer("Initialization", LOGGER);
        LOG_INFO("Loading information about fitting methods.");

        for (size_t i = 0; i < _labels.size(); ++i)
        {
            if (_labels[i].item)
            {
                _fit_parameters[_labels[i].label] = _labels[i].fit_method_parameters;
                _supported_labels.insert(_labels[i].label);
                _valid_fit_methods.insert(_labels[i].fit_method);
                _default_fit_methods[_labels[i].label] = _labels[i].fit_method;
            }
        }
    }

    EstimateShapeManager::~EstimateShapeManager()
    {
    }

    int EstimateShapeManager::estimateShape(std::vector<Item> &items, const std::string &item_label, const std::string &fit_method)
    {
        Timer timer("Estimate shape", LOGGER);
        LOG_INFO("Called estimate shape");

        // Validate item label
        if (_supported_labels.find(item_label) == _supported_labels.end())
            _user_specified_item_label = "all";
        else
            _user_specified_item_label = item_label;
        LOG_INFO_STREAM("Processing " << _user_specified_item_label << " items.");

        // Validate fit method
        if (_valid_fit_methods.find(fit_method) == _valid_fit_methods.end())
            _fit_method = "default";
        else
            _fit_method = fit_method;
        LOG_INFO_STREAM("Processing items with " << _fit_method << " fit method.");

        // Process items
        for (auto &item : items)
        {
            if (_fitItem(item) != FitReturnState_e::SUCCESS)
                item.isEstimationValid = false;
        }

        return 0;
    }

    FitReturnState_e EstimateShapeManager::_fitItem(Item &item)
    {
        Timer timer("EstimateShapeManager::_fitItem: item ID: " + std::to_string(item.id), LOGGER);
        LOG_INFO_STREAM("Processing item with ID: " << item.id << " label: " << item.label);

        std::string label = item.label;
        std::string current_fit_method = _fit_method == "default" ? _default_fit_methods.at(label) : _fit_method;
        auto fit_method_algorithm = FactoryFitMethod::createFitMethod(current_fit_method, _camera_parameters, _args, _fit_parameters[label]);

        if (fit_method_algorithm == nullptr)
        {
            LOG_WARN_STREAM("Unsupported fitting method: " << current_fit_method);
            return FitReturnState_e::SKIP;
        }

        if (fit_method_algorithm->setLabelData(_labels))
            return FitReturnState_e::ERROR;

        if (item.item_elements.size() == 0)
        {
            LOG_WARN("Could not perform fit because item_element vector is empty");
            return FitReturnState_e::SKIP;
        }

        if (fit_method_algorithm->fit(item))
            LOG_WARN_STREAM("Error occured while fitting item with ID: " << item.id << ", label: \"" << item.label << "\", fit method: \"" << current_fit_method << "\"");

        LOG_INFO_STREAM("Checking correctness of estimation format for item with ID: " << item.id << ", label: \"" << item.label << "\", fit method: \"" << current_fit_method << "\"");
        if (fit_method_algorithm->checkEstimation(item))
        {
            LOG_WARN_STREAM("Estimation for \"" << item.label << "\" has errors. Item will not be published.");
            return FitReturnState_e::ERROR;
        }

        return FitReturnState_e::SUCCESS;
    }

    // std::map<std::string, std::string> EstimateShapeManager::getDefaultFitMethods()
    // {
    //     Timer timer("EstimateShapeManager::getDefaultFitMethods", _debug);
    //     std::map<std::string, std::string> default_fit_methods;
    //     for (size_t i = 0; i < _labels.size(); ++i)
    //         default_fit_methods[_labels[i].label] = _labels[i].fit_method;
    //     return default_fit_methods;
    // }
} // namespace estimate_shape
