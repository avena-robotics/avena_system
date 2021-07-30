#ifndef ESTIMATE_SHAPE_MANAGER_HPP
#define ESTIMATE_SHAPE_MANAGER_HPP

// ___CPP___
#include <set>

// ___Package___
#include "estimate_shape_manager/visibility_control.h"
#include "estimate_shape_manager/factory_fit_method.hpp"
#include "estimate_shape_commons/commons.hpp"

namespace estimate_shape
{
  enum FitReturnState_e
  {
    SUCCESS,
    SKIP,
    ERROR,
    OTHER = 128
  };

  class EstimateShapeManager
  {
  public:
    ESTIMATE_SHAPE_MANAGER_PUBLIC
    explicit EstimateShapeManager(const std::vector<CameraParameters> &cams_params, const std::vector<Label> &labels);

    ESTIMATE_SHAPE_MANAGER_PUBLIC
    EstimateShapeManager(const EstimateShapeManager &other) = delete;

    ESTIMATE_SHAPE_MANAGER_PUBLIC
    virtual ~EstimateShapeManager();

    ESTIMATE_SHAPE_MANAGER_PUBLIC
    virtual int estimateShape(std::vector<Item> &items, const std::string &item_label = "all", const std::string &fit_method = "default");
    
    // ESTIMATE_SHAPE_MANAGER_PUBLIC
    // std::map<std::string, std::string> getDefaultFitMethods();

    using UniquePtr = std::unique_ptr<EstimateShapeManager>;
    using SharedPtr = std::shared_ptr<EstimateShapeManager>;

  private:
    std::string _user_specified_item_label;
    std::string _fit_method;
    std::vector<std::string> _args;
    std::map<std::string, json> _fit_parameters;
    std::set<std::string> _valid_fit_methods;
    std::set<std::string> _supported_labels;
    std::map<std::string, std::string> _default_fit_methods;
    std::vector<CameraParameters> _camera_parameters;
    std::vector<int32_t> _invalid_estimations_item_ids;

    int _getCameraParametersFromDb();
    int _getLabelsFromDb();
    int _saveItemElementsToDb();
    // std::set<std::string> _getAvailableItemsLabels(std::vector<Item> &items);
    // std::vector<uint32_t> _getItemIdsToProcess(std::string item_label);
    // int _fitLabel(std::string label);
    FitReturnState_e _fitItem(Item &item);

    // std::vector<uint32_t> _labels_ids;
    std::vector<Label> _labels;
    // std::vector<uint32_t> _item_ids;
    // std::vector<Item> _items;
    // std::vector<uint32_t> _item_elements_ids;
    // std::vector<ItemElement> _item_elements;
    // std::vector<ItemElement> _item_elements_results;
    // std::unique_ptr<helpers::Logger> _logger;
    // rclcpp::Logger _logger = rclcpp::get_logger("estimate_shape_manager");
  };

} // namespace estimate_shape

#endif