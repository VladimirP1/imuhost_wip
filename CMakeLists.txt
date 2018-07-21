cmake_minimum_required(VERSION 2.8)

add_compile_options(-std=c++17)

project(imu_host)

add_library(imuhost SHARED quaternion.cpp hid.cpp hid_mpu.cpp calibrated_imu.cpp imu.cpp)
target_include_directories(imuhost PRIVATE /home/vladimir/src/hidapi/hidapi/)
target_include_directories(imuhost PUBLIC .)
set(HIDLIB /home/vladimir/src/hidapi/linux/.libs/libhidapi-hidraw.so)
#set(HIDLIB /home/vladimir/src/hidapi/libusb/.libs/libhidapi-libusb.so)
target_link_libraries(imuhost ${HIDLIB} armadillo)

project(imuhost_test)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} imuhost)