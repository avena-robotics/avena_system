#ifndef _DATA_TYPES_HPP
#define _DATA_TYPES_HPP
#include "data_element.hpp"
//rgb
#include "custom_interfaces/srv/data_store_rgb_data_insert.hpp"
#include "custom_interfaces/srv/data_store_rgb_data_delete.hpp"
#include "custom_interfaces/srv/data_store_rgb_data_select.hpp"
#include "custom_interfaces/srv/data_store_rgb_data_get_list.hpp"

//item_cam1
#include "custom_interfaces/srv/data_store_item_cam1_insert.hpp"
#include "custom_interfaces/srv/data_store_item_cam1_delete.hpp"
#include "custom_interfaces/srv/data_store_item_cam1_select.hpp"
#include "custom_interfaces/srv/data_store_item_cam1_get_list.hpp"

//item_cam2
#include "custom_interfaces/srv/data_store_item_cam2_insert.hpp"
#include "custom_interfaces/srv/data_store_item_cam2_delete.hpp"
#include "custom_interfaces/srv/data_store_item_cam2_select.hpp"
#include "custom_interfaces/srv/data_store_item_cam2_get_list.hpp"

//tracker
#include "custom_interfaces/srv/data_store_tracker_insert.hpp"
#include "custom_interfaces/srv/data_store_tracker_delete.hpp"
#include "custom_interfaces/srv/data_store_tracker_select.hpp"
#include "custom_interfaces/srv/data_store_tracker_get_list.hpp"

// add other data elements here

namespace data_store
{
    //rgb
    using RgbDataInsert = custom_interfaces::srv::DataStoreRgbDataInsert;
    using RgbDataDelete = custom_interfaces::srv::DataStoreRgbDataDelete;
    using RgbDataSelect = custom_interfaces::srv::DataStoreRgbDataSelect;
    using RgbDataGetList = custom_interfaces::srv::DataStoreRgbDataGetList;
    using RgbData = DataElement<RgbDataSelect, RgbDataInsert, RgbDataGetList, RgbDataDelete>;

    //item_cam1
    using ItemCam1Insert = custom_interfaces::srv::DataStoreItemCam1Insert;
    using ItemCam1Delete = custom_interfaces::srv::DataStoreItemCam1Delete;
    using ItemCam1Select = custom_interfaces::srv::DataStoreItemCam1Select;
    using ItemCam1GetList = custom_interfaces::srv::DataStoreItemCam1GetList;
    using ItemCam1 = DataElement<ItemCam1Select, ItemCam1Insert, ItemCam1GetList, ItemCam1Delete>;

    //item_cam2
    using ItemCam2Insert = custom_interfaces::srv::DataStoreItemCam2Insert;
    using ItemCam2Delete = custom_interfaces::srv::DataStoreItemCam2Delete;
    using ItemCam2Select = custom_interfaces::srv::DataStoreItemCam2Select;
    using ItemCam2GetList = custom_interfaces::srv::DataStoreItemCam2GetList;
    using ItemCam2 = DataElement<ItemCam2Select, ItemCam2Insert, ItemCam2GetList, ItemCam2Delete>;

    //tracker
    using TrackerInsert = custom_interfaces::srv::DataStoreTrackerInsert;
    using TrackerDelete = custom_interfaces::srv::DataStoreTrackerDelete;
    using TrackerSelect = custom_interfaces::srv::DataStoreTrackerSelect;
    using TrackerGetList = custom_interfaces::srv::DataStoreTrackerGetList;
    using Tracker = DataElement<TrackerSelect, TrackerInsert, TrackerGetList, TrackerDelete>;

    // add other data elements here

}

#endif
