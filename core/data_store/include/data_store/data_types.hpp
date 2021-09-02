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

//rgbd_sync
#include "custom_interfaces/srv/data_store_rgbd_sync_insert.hpp"
#include "custom_interfaces/srv/data_store_rgbd_sync_delete.hpp"
#include "custom_interfaces/srv/data_store_rgbd_sync_select.hpp"
#include "custom_interfaces/srv/data_store_rgbd_sync_get_list.hpp"


//scene
#include "custom_interfaces/srv/data_store_scene_insert.hpp"
#include "custom_interfaces/srv/data_store_scene_delete.hpp"
#include "custom_interfaces/srv/data_store_scene_select.hpp"
#include "custom_interfaces/srv/data_store_scene_get_list.hpp"

//path planning
#include "custom_interfaces/srv/data_store_movement_sequence_insert.hpp"
#include "custom_interfaces/srv/data_store_movement_sequence_delete.hpp"
#include "custom_interfaces/srv/data_store_movement_sequence_select.hpp"
#include "custom_interfaces/srv/data_store_movement_sequence_get_list.hpp"

//trajectory
#include "custom_interfaces/srv/data_store_trajectory_insert.hpp"
#include "custom_interfaces/srv/data_store_trajectory_delete.hpp"
#include "custom_interfaces/srv/data_store_trajectory_select.hpp"
#include "custom_interfaces/srv/data_store_trajectory_get_list.hpp"


// add other data elements here

namespace data_store
{

    //tracker
    using TrackerInsert = custom_interfaces::srv::DataStoreTrackerInsert;
    using TrackerDelete = custom_interfaces::srv::DataStoreTrackerDelete;
    using TrackerSelect = custom_interfaces::srv::DataStoreTrackerSelect;
    using TrackerGetList = custom_interfaces::srv::DataStoreTrackerGetList;
    using Tracker = DataElement<TrackerSelect, TrackerInsert, TrackerGetList, TrackerDelete>;

    //rgbd_sync
    using RgbdSyncInsert = custom_interfaces::srv::DataStoreRgbdSyncInsert;
    using RgbdSyncDelete = custom_interfaces::srv::DataStoreRgbdSyncDelete;
    using RgbdSyncSelect = custom_interfaces::srv::DataStoreRgbdSyncSelect;
    using RgbdSyncGetList = custom_interfaces::srv::DataStoreRgbdSyncGetList;
    using RgbdSync = DataElement<RgbdSyncSelect, RgbdSyncInsert, RgbdSyncGetList, RgbdSyncDelete>;

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

    
    //scene
    using SceneInsert = custom_interfaces::srv::DataStoreSceneInsert;
    using SceneDelete = custom_interfaces::srv::DataStoreSceneDelete;
    using SceneSelect = custom_interfaces::srv::DataStoreSceneSelect;
    using SceneGetList = custom_interfaces::srv::DataStoreSceneGetList;
    using Scene = DataElement<SceneSelect, SceneInsert, SceneGetList, SceneDelete>;

    //motion planning
    using MovementSequenceInsert = custom_interfaces::srv::DataStoreMovementSequenceInsert;
    using MovementSequenceDelete = custom_interfaces::srv::DataStoreMovementSequenceDelete;
    using MovementSequenceSelect = custom_interfaces::srv::DataStoreMovementSequenceSelect;
    using MovementSequenceGetList = custom_interfaces::srv::DataStoreMovementSequenceGetList;
    using MovementSequence = DataElement<MovementSequenceSelect, MovementSequenceInsert, MovementSequenceGetList, MovementSequenceDelete>;

    //trajectory
    using TrajectoryInsert = custom_interfaces::srv::DataStoreTrajectoryInsert;
    using TrajectoryDelete = custom_interfaces::srv::DataStoreTrajectoryDelete;
    using TrajectorySelect = custom_interfaces::srv::DataStoreTrajectorySelect;
    using TrajectoryGetList = custom_interfaces::srv::DataStoreTrajectoryGetList;
    using Trajectory = DataElement<TrajectorySelect, TrajectoryInsert, TrajectoryGetList, TrajectoryDelete>;

    // add other data elements here

}

#endif
