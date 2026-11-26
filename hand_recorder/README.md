# CLI Commands
All CLI commands assume <package_name>:=hand_recorder and <node_name>:=recorder_node

## Running
```
ros2 launch hand_recorder recorder.launch.py
```

## Testing 
```
# Run burger image publisher
ros2 run image_tools cam2image --ros-args -p burger_mode:=true

# Run image viewer
ros2 run rqt_image_view rqt_image_view

# Run recorder node
ros2 run hand_recorder recorder_node
```
*List of Parameters*
- 'image_topic': Subscribe to the correct image topic. Defaults to burger publisher '/camera/camera/color/image_raw' (camera node). 
- 'log_every_n': How often the node publishes that an image has been recieved. Defaults to 30Hz (1 sec)
## Parameter
```
# Getting list of parameters
ros2 param list /hand_recorder

# Getting value of a parameter
ros2 param get /hand_recorder <param_name>

# Change individual parameter value + run
ros2 run hand_recorder recorder_node --ros-args -p <param_name>:=<value>

# Change parameters with the yaml vile + run
ros2 run hand_recorder recorder_node --ros-args --params-file hand_recorder/config/recorder.yaml
```