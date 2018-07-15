cmake_minimum_required(VERSION 2.8)

add_compile_options(-std=c++17)

project(imu_host)

add_library(imuhost SHARED quaternion.cpp hid.cpp hid_mpu.cpp)
target_include_directories(imuhost PRIVATE /home/vladimir/src/hidapi/hidapi/)
target_include_directories(imuhost PUBLIC .)
target_link_libraries(imuhost /home/vladimir/src/hidapi/linux/.libs/libhidapi-hidraw.so armadillo)

project(imuhost_test)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} imuhost)