#ifndef _DATA_TYPES_HPP
#define _DATA_TYPES_HPP
#include "data_element.hpp"

//tracker
#include "custom_interfaces/srv/data_store_tracker_insert.hpp"
#include "custom_interfaces/srv/data_store_tracker_delete.hpp"
#include "custom_interfaces/srv/data_store_tracker_select.hpp"
#include "custom_interfaces/srv/data_store_tracker_get_list.hpp"

//detectron
#include "custom_interfaces/srv/data_store_detectron_insert.hpp"
#include "custom_interfaces/srv/data_store_detectron_delete.hpp"
#include "custom_interfaces/srv/data_store_detectron_select.hpp"
#include "custom_interfaces/srv/data_store_detectron_get_list.hpp"

//items
#include "custom_interfaces/srv/data_store_items_insert.hpp"
#include "custom_interfaces/srv/data_store_items_delete.hpp"
#include "custom_interfaces/srv/data_store_items_select.hpp"
#include "custom_interfaces/srv/data_store_items_get_list.hpp"

//cameras_data
#include "custom_interfaces/srv/data_store_cameras_data_insert.hpp"
#include "custom_interfaces/srv/data_store_cameras_data_delete.hpp"
#include "custom_interfaces/srv/data_store_cameras_data_select.hpp"
#include "custom_interfaces/srv/data_store_cameras_data_get_list.hpp"

// add other data elements here

namespace data_store
{

    //tracker
    using TrackerInsert = custom_interfaces::srv::DataStoreTrackerInsert;
    using TrackerDelete = custom_interfaces::srv::DataStoreTrackerDelete;
    using TrackerSelect = custom_interfaces::srv::DataStoreTrackerSelect;
    using TrackerGetList = custom_interfaces::srv::DataStoreTrackerGetList;
    using Tracker = DataElement<TrackerSelect, TrackerInsert, TrackerGetList, TrackerDelete>;

    //cameras_data
    using CamerasDataInsert = custom_interfaces::srv::DataStoreCamerasDataInsert;
    using CamerasDataDelete = custom_interfaces::srv::DataStoreCamerasDataDelete;
    using CamerasDataSelect = custom_interfaces::srv::DataStoreCamerasDataSelect;
    using CamerasDataGetList = custom_interfaces::srv::DataStoreCamerasDataGetList;
    using CamerasData = DataElement<CamerasDataSelect, CamerasDataInsert, CamerasDataGetList, CamerasDataDelete>;

    //detectron
    using DetectronInsert = custom_interfaces::srv::DataStoreDetectronInsert;
    using DetectronDelete = custom_interfaces::srv::DataStoreDetectronDelete;
    using DetectronSelect = custom_interfaces::srv::DataStoreDetectronSelect;
    using DetectronGetList = custom_interfaces::srv::DataStoreDetectronGetList;
    using Detectron = DataElement<DetectronSelect, DetectronInsert, DetectronGetList, DetectronDelete>;

    //items
    using ItemsInsert = custom_interfaces::srv::DataStoreItemsInsert;
    using ItemsDelete = custom_interfaces::srv::DataStoreItemsDelete;
    using ItemsSelect = custom_interfaces::srv::DataStoreItemsSelect;
    using ItemsGetList = custom_interfaces::srv::DataStoreItemsGetList;
    using Items = DataElement<ItemsSelect, ItemsInsert, ItemsGetList, ItemsDelete>;

    // add other data elements here

}

#endif
