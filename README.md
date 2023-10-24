# beginner_tutorials
ENPM808X UMD-CP MAGE Fall 2023 Homework - ROS 2 Beginner Tutorials (C++)

## Overview
Modified Beginner_tutorial for EMPM808X. Uses C++ and ROS2 Humble.

## Author
Jerry Pittman, MBA, PMP - Naval Submarine Officer and USNA Instructor

## Download, Doxygen Documentation, cpplint, cppcheck, and cmake build Instructions

```bash
# Download the code in workspace's src folder:
  mkdir -p ~/ros2_ws/src
  git clone https://github.com/jpittma1/beginner_tutorials.git src/beginner_tutorials -b humble
# Resolve Dependencies:
  rosdep install -i --from-path src --rosdistro humble -y
  ```
## Build & Run ROS2 Package Instructions

```bash
# Build the code:
    cd ~/ros2_ws
    colcon build --packages-select beginner_tutorials
# Source setup.bash inside of install folder:
    source install/setup.bash

#Terminal 1:
    ros2 run beginner_tutorials listener

#Terminal 2:
    ros2 run beginner_tutorials talker

```
## Doxygen Documentation, cpplint, cppcheck, and cmake build Instructions
```bash
# Install dependencies:
  sudo apt install libeigen3-dev
# Configure the project and generate a native build system:
  # Must re-run this command whenever any CMakeLists.txt file has been changed.
  cmake -S ./ -B build/
# Compile and build the project:
  # rebuild only files that are modified since the last build
  cmake --build build/
  # or rebuild everything from scracth
  cmake --build build/ --clean-first
  # to see verbose output, do:
  cmake --build build/ --verbose
# Build docs:
  cmake --build build/ --target docs
  # open a web browser to browse the doc
  open docs/html/index.html
# Clean
  cmake --build build/ --target clean
# Clean and start over:
  rm -rf build/
  rm -rf .cache/
  rm -rf docs/
  rm -rf Doxyfile
  rm -rf compile_commands.json
# run clang-format
  clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")
# run cppcheck 
  mkdir results -p && cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cppcheck
#run cpplint
  mkdir results -p && cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cpplint

```

## Building for code coverage

```bash
# if you don't have gcovr or lcov installed, do:
  sudo apt-get install gcovr lcov
# Set the build type to Debug and WANT_COVERAGE=ON
  cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
# Now, do a clean compile, run unit test, and generate the covereage report
  cmake --build build/ --clean-first --target all test_coverage
# open a web browser to browse the test coverage report
  open build/test_coverage/index.html

This generates a index.html page in the build/test_coverage sub-directory that can be viewed locally in a web browser.
```

You can also get code coverage report for the *shell-app* target, instead of unit test. Repeat the previous 2 steps but with the *app_coverage* target:

``` bash
# Now, do another clean compile, run shell-app, and generate its covereage report
  cmake --build build/ --clean-first --target all app_coverage
# open a web browser to browse the test coverage report
  open build/app_coverage/index.html

This generates a index.html page in the build/app_coverage sub-directory that can be viewed locally in a web browser.
```
