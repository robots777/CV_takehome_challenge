cmake_minimum_required(VERSION 3.5.0)
project(cv_takehome_challenge VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

add_executable(cv_takehome_challenge main.cpp groundFit.cpp project_Camera.cpp parse_json.cpp)
add_executable(cv_pcl main_pcl.cpp groundFit.cpp project_Camera.cpp parse_json.cpp)
add_subdirectory(ext/eigen)
add_subdirectory(ext/nlohmann_json)
find_package(PCL REQUIRED COMPONENTS common io segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(cv_takehome_challenge Eigen3::Eigen nlohmann_json::nlohmann_json)
target_link_libraries(cv_pcl Eigen3::Eigen nlohmann_json::nlohmann_json ${PCL_LIBRARIES})


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
