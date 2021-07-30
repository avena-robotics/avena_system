#ifndef ESTIMATE_SHAPE_FACTORY_FIT_METHOD_HPP
#define ESTIMATE_SHAPE_FACTORY_FIT_METHOD_HPP

#include <memory>
#include "estimate_shape_fit_cylinder/fit_cylinder.hpp"
#include "estimate_shape_fit_sphere/fit_sphere.hpp"
#include "estimate_shape_fit_box/fit_box.hpp"
#include "estimate_shape_fit_bowl/fit_bowl.hpp"
#include "estimate_shape_fit_plate/fit_plate.hpp"
#include "estimate_shape_fit_cutting_board/fit_cutting_board.hpp"
#include "estimate_shape_fit_knife/fit_knife.hpp"
#include "estimate_shape_fit_spatula/fit_spatula.hpp"
// #include "fit_broccoli/fit_broccoli.hpp"
// #include "fit_box/fit_box.hpp"
// #include "fit_banana/fit_banana.hpp"
// #include "fit_octosphere/fit_octosphere.hpp"

namespace estimate_shape
{
  class FactoryFitMethod
  {
  public:
    FactoryFitMethod() = delete;
    virtual ~FactoryFitMethod() = delete;
    static std::shared_ptr<IFitMethod> createFitMethod(const std::string &type, const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
    {
      if (type == "box")
        return std::make_shared<FitBox>(camera_params, args, default_parameters);
      if (type == "cylinder")
        return std::make_shared<FitCylinder>(camera_params, args, default_parameters);
      else if (type == "sphere")
        return std::make_shared<FitSphere>(camera_params, args, default_parameters);
      else if (type == "bowl")
        return std::make_shared<FitBowl>(camera_params, args, default_parameters);
      else if (type == "plate")
        return std::make_shared<FitPlate>(camera_params, args, default_parameters);
      else if (type == "cuttingboard")
        return std::make_shared<FitCuttingBoard>(camera_params, args, default_parameters);
      else if (type == "knife")
        return std::make_shared<FitKnife>(camera_params, args, default_parameters);
      else if (type == "spatula")
        return std::make_shared<FitSpatula>(camera_params, args, default_parameters);
      // else if (type == "broccoli")
      //   return std::make_shared<FitBroccoli>(cam1_params, cam2_params, args, default_parameters, debug);
      // else if (type == "cylindersegments")
      //   return std::make_shared<FitBanana>(cam1_params, cam2_params, args, default_parameters, debug);
      // else if (type == "octosphere")
      //   return std::make_shared<FitOctoSphere>(cam1_params, cam2_params, args, default_parameters, debug);
      else
        return nullptr;
    }
  };
} // namespace estimate_shape

#endif
