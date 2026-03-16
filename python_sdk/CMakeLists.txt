cmake_minimum_required(VERSION 3.16)

project(sailor_python_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

set(BUILD_EXAMPLES OFF CACHE BOOL "Disable dqrobotics bundled examples" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static dqrobotics for the standalone solver" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Disable third-party tests" FORCE)
set(BUILD_WITH_PINOCCHIO OFF CACHE BOOL "Disable pinocchio in toppra" FORCE)
set(BUILD_WITH_qpOASES OFF CACHE BOOL "Disable qpOASES in toppra" FORCE)
set(BUILD_WITH_GLPK OFF CACHE BOOL "Disable GLPK in toppra" FORCE)
set(PYTHON_BINDINGS OFF CACHE BOOL "Disable toppra python bindings" FORCE)
set(TOPPRA_DEBUG_ON OFF CACHE BOOL "Disable toppra debug logs" FORCE)
set(TOPPRA_WARN_ON OFF CACHE BOOL "Disable toppra warnings" FORCE)
set(OPT_MSGPACK OFF CACHE BOOL "Disable toppra msgpack serialization" FORCE)

include(FetchContent)

FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG 9.0.0
)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(tinyxml2)

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dqrobotics-cpp
    ${CMAKE_CURRENT_BINARY_DIR}/dqrobotics
)

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/toppra/cpp/src
    ${CMAKE_CURRENT_BINARY_DIR}/toppra_src
)

add_executable(
    sailor_nullspace_solver
    src/sailor_nullspace_solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dqrobotics-cpp/examples/urdf_humanoid_chain_loader.cpp
)

target_include_directories(
    sailor_nullspace_solver
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dqrobotics-cpp/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dqrobotics-cpp/examples
        ${tinyxml2_SOURCE_DIR}
)

target_compile_definitions(
    sailor_nullspace_solver
    PRIVATE
        SAILOR_SDK_DEFAULT_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../public/robots/sailor_r1_pro_description/urdf/sailor_r1_pro_description.urdf"
)

target_link_libraries(
    sailor_nullspace_solver
    PRIVATE
        dqrobotics
        toppra
        tinyxml2
)
