
#!bin/bash

tmux new-session -d -s aufzeichnungen "cd &&\
source /opt/ros/humble/setup.bash &&\
cd Documents/Sensorik_Astra/Valeo_Scala_2/lidar_ws &&\
colcon build --symlink-install &&\
source install/setup.bash &&\
ros2 run scala_decoder_sdk_publisher data_processing_voxel_first_node &&\
bash"

tmux split-window -h "cd &&\
source /opt/ros/humble/setup.bash &&\
cd Documents/Sensorik_Astra/Valeo_Scala_2/lidar_ws &&\
source install/setup.bash &&\
ros2 run scala_decoder_sdk_publisher video_downscaling_node &&\
bash"

#================== OUSTER OS1 (ouster_cpp) ==================

tmux split-window -h "cd &&\
source /opt/ros/humble/setup.bash && \
cd Documents/Sensorik_Astra/Ouster_OS1/ros2_ws && \
colcon build --symlink-install &&\
source install/setup.bash && \
ros2 launch ouster_cpp benchmark.launch.py && \
bash"

#==========BOSCH Radar Processing Nodes für Aufzeichnungsmanipulation==========

# Radar_Daten (PCL Processing)
tmux split-window -h "source /opt/ros/humble/setup.bash &&\
cd &&\
cd Documents/Sensorik_Astra/BOSCH_Radar/Data_Process_ws/ &&\
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release' &&\
source install/setup.bash &&\
ros2 run pcl_process_pkg Radar_Data &&\
bash"

# Radar Ziel_Nummerierung
tmux split-window -h "source /opt/ros/humble/setup.bash &&\
cd &&\
cd Documents/Sensorik_Astra/BOSCH_Radar/Data_Process_ws/ &&\
source install/setup.bash &&\
ros2 run Ziel_Nummerierung targets_num &&\
bash"

# Nahfeld-Processing-First-Node
tmux split-window -h "source /opt/ros/humble/setup.bash && \
cd && \
cd Documents/Sensorik_Astra/BOSCH_Radar/Data_Process_ws && \
source install/setup.bash && \
ros2 run radar_process_pkg radar_processing_first_node && \
bash"

# Object Detection Node
tmux split-window -h "source /opt/ros/humble/setup.bash && \
cd && \
cd Documents/Sensorik_Astra/BOSCH_Radar/Data_Process_ws && \
source install/setup.bash && \
ros2 run radar_process_pkg object_detection_node && \
bash"

# Tracking Node
tmux split-window -h "source /opt/ros/humble/setup.bash && \
cd && \
cd Documents/Sensorik_Astra/BOSCH_Radar/Data_Process_ws && \
source install/setup.bash && \
ros2 run radar_process_pkg tracking_node && \
bash"

# Echtzeit Objektliste
tmux split-window -h "source /opt/ros/humble/setup.bash && \
cd && \
cd Documents/Sensorik_Astra/BOSCH_Radar/Data_Process_ws && \
source install/setup.bash && \
ros2 run object_list Echtzeit_Objektliste && \
bash"

# Trackliste
tmux split-window -h "source /opt/ros/humble/setup.bash && \
cd && \
cd Documents/Sensorik_Astra/BOSCH_Radar/Data_Process_ws && \
source install/setup.bash && \
ros2 run radar_process_pkg track_list_node && \
bash"
