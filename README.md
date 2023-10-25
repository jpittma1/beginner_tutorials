# beginner_tutorials
ENPM808X UMD-CP MAGE Fall 2023 Homework - ROS 2 Beginner Tutorials (C++)

## Overview
Modified Beginner_tutorial for EMPM808X. Uses C++ and ROS2 Humble. <br>
Adds custom message to the publisher. <br>
Makes the ros_tutorials meet Google C++ standards, passes cpplint, and cppcheck. <br>
Dependencies are: std_msgs, rclcpp. <br>
Adds service client. <br>
Adds launch file. <br>
Modify Talker node to broadcast a tf frame called /talk with parent /world <br>
Using gtest (GoogleTest) to create a Level 2 integration tests. Test Publisher and Subscriber.

## Author
Jerry Pittman, MBA, PMP - Naval Submarine Officer and USNA Instructor

## Download and Dependencies Instructions

```bash
# Download the code in workspace's src folder:
  mkdir -p ~/ros2_ws/src
  git clone https://github.com/jpittma1/beginner_tutorials.git src/beginner_tutorials -b humble
# Resolve Dependencies using rosdep:
  rosdep install -i --from-path src --rosdistro humble -y

  ```

## Build & Run ROS2 Package Instructions (Using ros2 run)

```bash
# Build the code:
    cd ~/ros2_ws
    colcon build --packages-select beginner_tutorials

#Terminal 1:
    source install/setup.bash
    ros2 run beginner_tutorials listener

#Terminal 2:
    source install/setup.bash
    ros2 run beginner_tutorials talker
```

## Service/Client to Change String in Talker
```bash
#Terminal 3: 
    ros2 service call /change_string beginner_tutorials/srv/ChangeString "{after: <new_string>}"
```

## Run ROS2 Package Instructions Using Launch files

* `message` *#the new message to be published*

* `message_freq` *#time between successive messages in ms*


To do this, execute the following command:
```
ros2 launch beginner_tutorials _launch.py message:=<desired_message> message_freq:=<desired_message_frequency>
```

## Running rqt_console
```
ros2 run rqt_console rqt_console
```

## Clang format for Google, cpplint, and cppcheck Instructions
```bash
# run clang-format
  clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")
# run cppcheck 
  mkdir results -p && cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cppcheck
#run cpplint
  mkdir results -p && cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cpplint

```

## tf2 Tools
```bash
# Using tf2 echo
ros2 run tf2_ros tf2_echo world talk
# tf2 View Frames
ros2 run tf2_tools view_frames
```

## Build and run tests using Colcon
```bash
# Build
    colcon build --packages-select beginner_tutorials
# run test (from workspace i.e. ros2_ws); without verbose
  colcon test --packages-select beginner_tutorials
# with verbose
  colcon test --event-handlers console_direct+ --packages-select beginner_tutorials

# examine test results
  cat log/latest_test/beginner_tutorials/stdout_stderr.log
```