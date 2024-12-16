#!/bin/bash
terminator -e  " source ~/.bashrc && source install/setup.bash && ros2 launch simulator simulation.launch.xml " &
sleep 3
terminator -e  " source ~/.bashrc && source install/setup.bash && ros2 launch mission_manager mission_manager.launch.xml " &
sleep 3
terminator -e  " source ~/.bashrc && source install/setup.bash && ros2 launch evaluation evaluation.launch.xml " &
sleep 3
terminator -e  " bash -c 'source ~/.bashrc && source install/setup.bash && ros2 launch autonomous_driving autonomous_driving.launch.xml; if [ \$? -ne 0 ]; then echo \"Press Enter to close...\"; read; fi' "