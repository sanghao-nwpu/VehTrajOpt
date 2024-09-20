cmake_minimum_required(VERSION 3.10)
project(NhcTrajectoryOptimization)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# 添加Eigen库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加Ceres Solver库
find_package(Ceres REQUIRED)
if(Ceres_FOUND)
    message(STATUS "Ceres Solver found!")
else()
    message(FATAL_ERROR "Ceres Solver not found!")
endif()
include_directories(${CERES_INCLUDE_DIRS})

# 添加json库
# find_package(nlohmann_json REQUIRED)
# if(nlohmann_json_FOUND)
#     message(STATUS "nlohmann_json found!")
# else()
#     message(FATAL_ERROR "nlohmann_json not found!")
# endif()
# include_directories(${nlohmann_json_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
if(yaml-cpp_FOUND)
    message(STATUS "yaml-cpp found!")
else()
    message(FATAL_ERROR "yaml-cpp not found!")
endif()
include_directories(${yaml-cpp_INCLUDE_DIRS})

# 添加头文件目录
# include_directories(include)

# 添加源文件目录
# file(GLOB_RECURSE SRC_FILES src/*.cpp)

# 添加源文件
# add_executable(vto_test demo/vto_test.cpp)
# target_link_libraries(vto_test ${CERES_LIBRARIES})

# #
add_executable(auto_diff_demo demo/auto_diff_demo.cpp)
target_link_libraries(auto_diff_demo ${CERES_LIBRARIES})

add_executable(ceres_test demo/ceres_test.cpp)
target_link_libraries(ceres_test ${CERES_LIBRARIES})

add_executable(ceres_example demo/ceres_example.cpp)
target_link_libraries(ceres_example ${CERES_LIBRARIES})

add_executable(curve_fitting demo/curve_fitting.cpp)
target_link_libraries(curve_fitting ${CERES_LIBRARIES})


add_executable(vto_test demo/vto_test.cpp)
target_link_libraries(vto_test ${CERES_LIBRARIES})
target_link_libraries(vto_test yaml-cpp)

add_executable(vto_develop demo/vto_develop.cpp)
target_link_libraries(vto_develop ${CERES_LIBRARIES})
target_link_libraries(vto_develop yaml-cpp)

# 编译动态库
# add_library(NhcTrajectoryOptimizationLib SHARED ${SRC_FILES})

# 链接库
# message(STATUS "Linking libraries: ${CERES_LIBRARIES}")
# target_link_libraries(NhcTrajectoryOptimization ${CERES_LIBRARIES})