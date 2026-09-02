import sys
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path+"/../")

import time

submodule_path = dir_path + "/ros2_ws/install"
print("submodule_path:", submodule_path)
if not os.path.exists(submodule_path):
    print("Building the HAL/msgs first..")
    os.system("bash -c 'bash ros2_ws/src/agx_arm_ros/scripts/can_activate.sh && cd ros2_ws && colcon build --packages-select dls2_interface && cd .. && source ros2_ws/install/setup.bash && python3 ros2_ws/src/dls2_ros2_piper_hal/piper_hal.py'")
else:
    print("\n\n")
    print("msg already built - if you have any modifications, please delete the build folder in the submodule")
    print("\n\n")
    time.sleep(2)
    os.system("bash -c 'bash ros2_ws/src/agx_arm_ros/scripts/can_activate.sh && source ros2_ws/install/setup.bash && python3 ros2_ws/src/dls2_ros2_piper_hal/piper_hal.py'")

