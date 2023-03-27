# CMake generated Testfile for 
# Source directory: /home/acl3-ubuntu/holohover_ws/src/holohover/bota_driver/bota_node/bota_worker
# Build directory: /home/acl3-ubuntu/holohover_ws/src/holohover/build/bota_worker
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_bota_worker "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/acl3-ubuntu/holohover_ws/src/holohover/build/bota_worker/test_results/bota_worker/test_bota_worker.gtest.xml" "--package-name" "bota_worker" "--output-file" "/home/acl3-ubuntu/holohover_ws/src/holohover/build/bota_worker/ament_cmake_gtest/test_bota_worker.txt" "--command" "/home/acl3-ubuntu/holohover_ws/src/holohover/build/bota_worker/test_bota_worker" "--gtest_output=xml:/home/acl3-ubuntu/holohover_ws/src/holohover/build/bota_worker/test_results/bota_worker/test_bota_worker.gtest.xml")
set_tests_properties(test_bota_worker PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/acl3-ubuntu/holohover_ws/src/holohover/build/bota_worker/test_bota_worker" TIMEOUT "60" WORKING_DIRECTORY "/home/acl3-ubuntu/holohover_ws/src/holohover/build/bota_worker" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/acl3-ubuntu/holohover_ws/src/holohover/bota_driver/bota_node/bota_worker/CMakeLists.txt;40;ament_add_gtest;/home/acl3-ubuntu/holohover_ws/src/holohover/bota_driver/bota_node/bota_worker/CMakeLists.txt;0;")
subdirs("gtest")
