# dory_nav2
 
## Install

Make a ROS workspace and `src` folder in the workspace

Clone this repo into `src`

Clone https://github.com/RKinDLab/Reach_alpha_Blue_heavy_Sim.git into the `src` folder as well.

Build using `colcon build`

## Launch simulator

Source the workspace `. install/setup.bash`

To launch the simulator, run `ros2 launch dory_nav2 dory_nav2.launch.py`

If you want to record the states and forces being published, run `ros2 launch dory_nav2 dory_nav2.launch.py record_bag:=true`

## Process bag
ros2bag-convert filepath/filename.db3

## Plot bag
python3 `src/dory_nav2/dory_nav2/plot_bag_response.py`