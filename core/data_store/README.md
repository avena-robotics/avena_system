## Data sources

ros2 run data_store rgb_insert_pub
ros2 run data_store cam1_insert_pub
ros2 run data_store cam2_insert_pub

## Data store server

ros2 launch data_store data_store.launch.py 

## Data store client

 python3 src/avena_system/core/data_store/test/scripts/rgb/get_lists.py 
 python3 src/avena_system/core/data_store/test/scripts/cam1/get_lists.py 
 python3 src/avena_system/core/data_store/test/scripts/cam2/get_lists.py 


 python3 src/avena_system/core/data_store/test/scripts/rgb/insert.py 
 python3 src/avena_system/core/data_store/test/scripts/cam1/insert.py 
 python3 src/avena_system/core/data_store/test/scripts/cam2/insert.py 


 python3 src/avena_system/core/data_store/test/scripts/rgb/delete.py 2.0
 python3 src/avena_system/core/data_store/test/scripts/cam1/delete.py 2.0
 python3 src/avena_system/core/data_store/test/scripts/cam2/delete.py 2.0


 python3 src/avena_system/core/data_store/test/scripts/rgb/select.py 1.0
 python3 src/avena_system/core/data_store/test/scripts/cam1/select.py 1.0
 python3 src/avena_system/core/data_store/test/scripts/cam2/select.py 1.0
