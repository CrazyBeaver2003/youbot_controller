You need install dependencies:
1. slam-toolbox:
    sudo apt install ros-humble-slam-toolbox

Save map into maps folder:
ros2 service call /slam_toolbox/save_map  slam_toolbox/srv/SaveMap "name: {data: "src/youbot_control/maps/map"}"

ros2 service call /navigate_to_object youbot_interfaces/srv/NavigateToObject "{waypoint_name: 'ball'}"

не запускался рвиз без этих команд 
export 
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0


ros2 topic pub /gripper_target_gap std_msgs/Float64 "data: 0.05"

ros2 topic pub --once /gripper_target_gap std_msgs/Float64 "data: 0.071"
ros2 topic pub --once /gripper_target_gap std_msgs/Float64 "data: 0.021"

