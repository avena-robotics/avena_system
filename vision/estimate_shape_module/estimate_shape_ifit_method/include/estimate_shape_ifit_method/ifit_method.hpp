#ifndef ESTIMATE_SHAPE_IFITMETHOD_HPP
#define ESTIMATE_SHAPE_IFITMETHOD_HPP

// ___CPP___
#include <map>
#include <memory>

// ___Avena___
#include "estimate_shape_ifit_method/visibility_control.h"
#include "helpers_vision/helpers_vision.hpp"
#include "estimate_shape_commons/commons.hpp"

namespace estimate_shape
{
  class IFitMethod
  {
  public:
    IFIT_METHOD_PUBLIC
    /**
     * @brief Construct a new IFitMethod object
     * 
     * @param camera_params 
     * @param args 
     * @param default_parameters 
     */
    IFitMethod(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters);

    IFIT_METHOD_PUBLIC
    /**
     * @brief Destroy the IFitMethod object
     * 
     */
    virtual ~IFitMethod();

    IFIT_METHOD_PUBLIC
    /**
     * @brief Estimate shape of scene item with shape specific algorithm.
     * 
     * @param item 
     * @return int 
     */
    virtual int fit(Item &item) = 0;

    IFIT_METHOD_PUBLIC
    /**
     * @brief Set the Label Data object
     * 
     * @param labels 
     * @return int 
     */
    virtual int setLabelData(const std::vector<Label> &labels);

    IFIT_METHOD_PUBLIC
    /**
     * @brief Default implementaion to check whether estimation has corrent data. There is a check only if there is at least one parts_description fiels which is not empty.
     * 
     * @param item 
     * @return int 
     */
    virtual int checkEstimation(Item &item);

  protected:
    IFIT_METHOD_PUBLIC
    /**
     * @brief 
     * 
     * @param arg 
     * @param param 
     * @return int 
     */
    int validateNumericParameter(const std::string &arg, float &param);

    IFIT_METHOD_PUBLIC
    /**
     * @brief 
     * 
     * @param arg 
     * @param param 
     * @return int 
     */
    int validateNumericParameter(const std::string &arg, int &param);

    std::map<std::string, Eigen::Affine3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3f>>> _camera_poses;
    std::vector<Label> _labels_data;
    std::vector<CameraParameters> _camera_params;
    std::vector<std::string> _args;
    json _default_parameters;
    size_t _cameras_amount = 0;
    std::string _camera_prefix;
  };
} // namespace estimate_shape

#endif
