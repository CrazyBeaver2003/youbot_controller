You need install dependencies:
1. slam-toolbox:
    sudo apt install ros-humble-slam-toolbox

Save map into maps folder:
ros2 service call /slam_toolbox/save_map  slam_toolbox/srv/SaveMap "name: {data: "src/youbot_control/maps/map"}"