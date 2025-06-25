#!/bin/bash

# === CONFIG SECTION ===
CONDITION_TOPIC="/simulation_control"  # Topic to monitor

# Directories
DIR1=~/Documents/Jan_Work/Multi-UAV-System-for-Forest-SAR/sjtu_drone_ros2
DIR2=~/Documents/Jan_Work/Multi-UAV-System-for-Forest-SAR/FASTLIOROS2
DIR3=~/Documents/Jan_Work/Multi-UAV-System-for-Forest-SAR/octomap_server2_latest_2
DIR4=~/Documents/Jan_Work/Multi-UAV-System-for-Forest-SAR/ompl_example_2d_latest_6
DIR5=~/Documents/Jan_Work/Multi-UAV-System-for-Forest-SAR/Thermal_Node

# Commands
COMMAND1="ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py"
COMMAND2="ros2 launch fast_lio mapping.launch.py"
COMMAND3="ros2 launch octomap_server2 octomap_server_launch.py"
COMMAND4="ros2 launch ompl_example_2d ompl_example_2d.launch.py"
COMMAND5="./path_follower.py"  # This might be the node that publishes /simulation_control
COMMAND6="python3 ./humandetection.py"  # Another possible node to publish to /simulation_control

# === LAUNCH FUNCTION ===
launch_all() {
    echo "Launching ROS 2 packages..."

    # Launch commands in new Terminator tabs and capture the PIDs of each tab
    terminator --new-tab -e "bash -c 'cd $DIR1 && source install/setup.bash && $COMMAND1; exec bash'" &
    TERMINATOR_PID1=$!
    terminator --new-tab -e "bash -c 'cd $DIR2 && source install/setup.bash && $COMMAND2; exec bash'" &
    TERMINATOR_PID2=$!
    terminator --new-tab -e "bash -c 'cd $DIR3 && source install/setup.bash && $COMMAND3; exec bash'" &
    TERMINATOR_PID3=$!
    terminator --new-tab -e "bash -c 'cd $DIR4 && source install/setup.bash && $COMMAND4; exec bash'" &
    TERMINATOR_PID4=$!
    terminator --new-tab -e "bash -c 'cd $DIR5 && $COMMAND6; exec bash'" &
    TERMINATOR_PID6=$!

    # Output the PIDs of the launched processes
    echo "Processes started in new Terminator tabs with PIDs: $TERMINATOR_PID1, $TERMINATOR_PID2, $TERMINATOR_PID3, $TERMINATOR_PID4, $TERMINATOR_PID6"
}


launch_follower(){

    terminator --new-tab -e "bash -c 'cd $DIR1 && source install/setup.bash && $COMMAND5; exec bash'" &
    TERMINATOR_PID5=$!
}

# === CLOSE FUNCTION ===
close_terminator_tabs() {
    echo "Closing all Terminator tabs except the main one..."

    # Close all Terminator windows by killing their processes using pkill (kill by name 'terminator')
    pkill -f "terminator"  # This kills all terminator processes
    pkill -f "xterm" 

    # Wait to ensure processes are terminated
    sleep 2

    echo "Terminator tabs closed."
}

# === MONITOR FUNCTION ===
monitor_condition() {
    echo "Monitoring topic: $CONDITION_TOPIC"

    # Listen to the /simulation_control topic and check if the message is True (case-insensitive)
    result=$(ros2 topic echo $CONDITION_TOPIC | grep -i -m 1 'true' | head -n 1)

    if [[ "$result" == *"true"* ]]; then
        echo "Received True on $CONDITION_TOPIC. Simulation can be stopped."
        return 0  # Simulation should be stopped
    fi

    return 1  # No condition met, continue monitoring
}


# === MAIN LOOP ===
while true; do
    # Launch all processes in separate Terminator tabs
    launch_follower
    
    sleep 2
    
    launch_all

    # Monitor the /simulation_control topic for True
    while true; do
        # Monitor condition (it checks if True is received)
        monitor_condition

        if [[ $? -eq 0 ]]; then
            echo "Simulation stopped, closing the terminator windows..."
            # Close the terminator windows when 'True' is received
            close_terminator_tabs
            break  # Exit the inner loop and restart the process
        fi

        sleep 0.01  # Check every second
    done
done


