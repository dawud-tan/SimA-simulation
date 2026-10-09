cmake_minimum_required(VERSION 3.18)
project(innova_test CXX)
find_package(Chrono REQUIRED COMPONENTS Vehicle CONFIG
             PATHS /opt/chrono/lib64/cmake/Chrono /opt/chrono/lib/cmake/Chrono)
add_executable(test_innova test_innova.cpp)
target_link_libraries(test_innova PRIVATE ${CHRONO_TARGETS})
target_compile_definitions(test_innova PRIVATE
    CHRONO_DATA_DIR="${CHRONO_DATA_DIR}"
    VEHICLE_DATA_DIR="${VEHICLE_DATA_DIR}")
set_target_properties(test_innova PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
