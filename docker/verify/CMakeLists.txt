cmake_minimum_required(VERSION 3.18)
project(chrono_verify CXX)

# find_package pulls in Chrono's exported targets, which carry the compile
# definitions and flags the library was built with. Using the imported targets
# rather than hand-rolled -I/-l flags is what keeps the ABI consistent.
find_package(Chrono REQUIRED
             COMPONENTS Vehicle
             CONFIG
             PATHS /opt/chrono/lib64/cmake/Chrono /opt/chrono/lib/cmake/Chrono)

add_executable(verify_chrono verify_chrono.cpp)

# ChronoConfig.cmake exports namespaced targets under their full library names
# (Chrono::Chrono_core, Chrono::Chrono_vehicle, ...) and collects the set for
# the requested components in CHRONO_TARGETS. Use that variable rather than
# naming targets by hand: the build-tree ALIAS `Chrono::core` is NOT exported
# to the install tree, so hardcoding it fails at generate time.
target_link_libraries(verify_chrono PRIVATE ${CHRONO_TARGETS})

set_target_properties(verify_chrono PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

message(STATUS "Chrono ${CHRONO_VERSION}; targets: ${CHRONO_TARGETS}")
