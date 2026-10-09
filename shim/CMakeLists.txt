cmake_minimum_required(VERSION 3.18)
project(chrono_shim CXX)

find_package(Chrono REQUIRED COMPONENTS Vehicle CONFIG
             PATHS /opt/chrono/lib64/cmake/Chrono /opt/chrono/lib/cmake/Chrono)

add_library(chrono_shim SHARED chrono_shim.cpp)
target_include_directories(chrono_shim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(chrono_shim PRIVATE ${CHRONO_TARGETS})

set_target_properties(chrono_shim PROPERTIES
    CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

# NOTE: do NOT also set CXX_VISIBILITY_PRESET=hidden here. That marks symbols
# hidden at COMPILE time, and a version script cannot promote a hidden symbol
# back to global -- the result is a .so that exports nothing at all. The
# version script below is sufficient on its own: it keeps Chs_* global and
# makes everything else (including all 28574 Chrono C++ symbols) local.

# Chrono is built -march=skylake (AVX2). The shim MUST match: it embeds Eigen
# types in Chrono classes, and a mismatch changes their layout silently.
target_compile_options(chrono_shim PRIVATE -march=skylake -mtune=skylake)

# A linker version script re-exports exactly the Chs_* entry points and hides
# everything else, without needing visibility macros scattered through the
# header. This is what keeps the .so's dynamic symbol table tiny.
target_link_options(chrono_shim PRIVATE
    "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/chrono_shim.map")

# RPATH of $ORIGIN so the shim finds libChrono_*.so sitting BESIDE it, wherever
# Unreal copies the set to. Without this the linker bakes in /opt/chrono/lib --
# the CONTAINER path, which does not exist on the host -- and the module fails
# to load with "libChrono_core.so => not found".
#
# $ORIGIN must reach the linker literally, hence the escaped \$: CMake would
# otherwise try to expand it as a variable.
# RPATH not RUNPATH (--disable-new-dtags): RPATH is inherited by transitive
# dependencies, RUNPATH is not.
set_target_properties(chrono_shim PROPERTIES
    INSTALL_RPATH "\$ORIGIN"
    BUILD_WITH_INSTALL_RPATH ON)
target_link_options(chrono_shim PRIVATE "-Wl,--disable-new-dtags")
