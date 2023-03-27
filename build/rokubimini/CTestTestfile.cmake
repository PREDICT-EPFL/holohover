# CMake generated Testfile for 
# Source directory: /home/acl3-ubuntu/holohover_ws/src/holohover/bota_driver/rokubimini
# Build directory: /home/acl3-ubuntu/holohover_ws/src/holohover/build/rokubimini
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_rokubimini "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/acl3-ubuntu/holohover_ws/src/holohover/build/rokubimini/test_results/rokubimini/test_rokubimini.gtest.xml" "--package-name" "rokubimini" "--output-file" "/home/acl3-ubuntu/holohover_ws/src/holohover/build/rokubimini/ament_cmake_gtest/test_rokubimini.txt" "--command" "/home/acl3-ubuntu/holohover_ws/src/holohover/build/rokubimini/test_rokubimini" "--gtest_output=xml:/home/acl3-ubuntu/holohover_ws/src/holohover/build/rokubimini/test_results/rokubimini/test_rokubimini.gtest.xml")
set_tests_properties(test_rokubimini PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/acl3-ubuntu/holohover_ws/src/holohover/build/rokubimini/test_rokubimini" TIMEOUT "60" WORKING_DIRECTORY "/home/acl3-ubuntu/holohover_ws/src/holohover/build/rokubimini" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/acl3-ubuntu/holohover_ws/src/holohover/bota_driver/rokubimini/CMakeLists.txt;66;ament_add_gtest;/home/acl3-ubuntu/holohover_ws/src/holohover/bota_driver/rokubimini/CMakeLists.txt;0;")
subdirs("gtest")
