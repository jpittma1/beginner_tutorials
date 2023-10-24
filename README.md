# beginner_tutorials
ENPM808X UMD-CP MAGE Fall 2023 Homework - ROS 2 Beginner Tutorials (C++)

## Overview
Modified Beginner_tutorial for EMPM808X. Uses C++ and ROS2 Humble. <br>
Adds custom message to the publisher. <br>
Makes the ros_tutorials meet Google C++ standards, passes cpplint, and cppcheck. <br>
Dependencies are: std_msgs, rclcpp.

## Author
Jerry Pittman, MBA, PMP - Naval Submarine Officer and USNA Instructor

## Download, Doxygen Documentation, cpplint, cppcheck, and cmake build Instructions

```bash
# Download the code in workspace's src folder:
  mkdir -p ~/ros2_ws/src
  git clone https://github.com/jpittma1/beginner_tutorials.git src/beginner_tutorials -b humble
# Resolve Dependencies using rosdep:
  rosdep install -i --from-path src --rosdistro humble -y

  ```

## Build & Run ROS2 Package Instructions

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

## Clang format for Google, cpplint, and cppcheck Instructions
```bash
# run clang-format
  clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")
# run cppcheck 
  mkdir results -p && cppcheck --enable=all --std=c++17 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cppcheck
#run cpplint
  mkdir results -p && cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cpplint

```
