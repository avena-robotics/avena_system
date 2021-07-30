#include "estimate_shape_ifit_method/ifit_method.hpp"

namespace estimate_shape
{
    IFitMethod::IFitMethod(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
        : _camera_params(camera_params),
          _args(args),
          _default_parameters(default_parameters)
    {
        _camera_poses[CamerasFrames::camera_1] = _camera_params[0].translation * _camera_params[0].orientation;
        _camera_poses[CamerasFrames::camera_2] = _camera_params[1].translation * _camera_params[1].orientation;
    }

    IFitMethod::~IFitMethod()
    {
    }

    int IFitMethod::setLabelData(const std::vector<Label> &labels)
    {
        _labels_data = labels;
        return 0;
    }

    int IFitMethod::validateNumericParameter(const std::string &arg, float &param)
    {
        float temp_val;
        try
        {
            temp_val = std::stof(arg);
        }
        catch (const std::invalid_argument &e)
        {
            std::cerr << "fitmethod::init: invalid argument. Please pass floating point value.\n";
            return 1;
        }
        catch (const std::out_of_range &e)
        {
            std::cerr << "fitmethod::init: out of range error. Please pass floating point value in range: ["
                      << std::numeric_limits<float>::min() << ", " << std::numeric_limits<float>::max() << '\n';
            return 1;
        }
        catch (...)
        {
            std::cerr << "fitmethod::init: other error while parsing. Reseting arguments to default value.\n";
            return 1;
        }
        if (temp_val > 0)
            param = temp_val;
        else
            return 1;

        return 0;
    }

    int IFitMethod::validateNumericParameter(const std::string &arg, int &param)
    {
        int temp_val;
        try
        {
            temp_val = std::stoi(arg);
        }
        catch (const std::invalid_argument &e)
        {
            std::cerr << "fitmethod::init: invalid argument. Please pass integer point value.\n";
            return 1;
        }
        catch (const std::out_of_range &e)
        {
            std::cerr << "fitmethod::init: out of range error. Please pass signed integer point value in range: ["
                      << std::numeric_limits<int>::min() << ", " << std::numeric_limits<int>::max() << '\n';
            return 1;
        }
        catch (...)
        {
            std::cerr << "fitmethod::init: other error while parsing. Reseting arguments to default value.\n";
            return 1;
        }
        if (temp_val > 0)
            param = temp_val;
        else
            return 1;

        return 0;
    }

    int IFitMethod::checkEstimation(Item &item)
    {
        for (auto &element : item.item_elements)
        {
            if (element.parts_description.size() != 0)
                return 0;
        }
        return 1;
    }
} // namespace estimate_shape
