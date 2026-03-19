cmake_minimum_required(VERSION 3.18)
project(dqrobotics_wasm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Emscripten 编译标志
if(EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".js")
    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        # 避免第三方子项目 install 到 emsdk 系统目录。
        set(CMAKE_INSTALL_PREFIX "/tmp/dq_vizlab_wasm_install" CACHE PATH "Install prefix" FORCE)
    endif()
    
    # 添加 DQ Robotics 源目录
    set(DQROBOTICS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/dqrobotics-cpp)
    set(TOPPRA_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/toppra/cpp)
    set(NLOPT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/nlopt)
    
    # 查找 Eigen3
    # 注意: 在 Emscripten 环境中，你需要确保 Eigen 可用
    # 可以通过设置 EIGEN3_INCLUDE_DIR 或使用 emscripten 的 ports
    find_package(Eigen3 QUIET)
    
    if(NOT EIGEN3_FOUND)
        # 如果找不到系统的 Eigen，使用本地副本
        set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/eigen)
        if(NOT TARGET Eigen3::Eigen)
            add_library(Eigen3::Eigen INTERFACE IMPORTED)
            set_target_properties(Eigen3::Eigen PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
            )
        endif()
        if(NOT DEFINED EIGEN3_VERSION_STRING OR EIGEN3_VERSION_STRING STREQUAL "")
            set(EIGEN3_VERSION_STRING "3.4.0")
        endif()
        message(STATUS "Using local Eigen at: ${EIGEN3_INCLUDE_DIR}")
    endif()

    set(HAS_TOPPRA OFF)
    if(EXISTS "${TOPPRA_CPP_DIR}/src/toppra/toppra.hpp")
        # 仅编译核心 C++ 库，不启用 Python / tests / pinocchio / 第三方 QP 求解器。
        set(BUILD_TESTS OFF CACHE BOOL "Build toppra tests" FORCE)
        set(BUILD_WITH_PINOCCHIO OFF CACHE BOOL "Build toppra with pinocchio" FORCE)
        set(BUILD_WITH_qpOASES OFF CACHE BOOL "Build toppra with qpOASES" FORCE)
        set(BUILD_WITH_GLPK OFF CACHE BOOL "Build toppra with GLPK" FORCE)
        set(PYTHON_BINDINGS OFF CACHE BOOL "Build toppra python bindings" FORCE)
        set(TOPPRA_DEBUG_ON OFF CACHE BOOL "Enable toppra debug logging" FORCE)
        set(TOPPRA_WARN_ON OFF CACHE BOOL "Enable toppra warnings" FORCE)
        set(OPT_MSGPACK OFF CACHE BOOL "Enable toppra msgpack serialization" FORCE)
        set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build toppra shared libraries" FORCE)

        add_subdirectory(${TOPPRA_CPP_DIR}/src ${CMAKE_BINARY_DIR}/toppra_src)
        set(HAS_TOPPRA ON)
        message(STATUS "Configured TOPPRA C++ core from: ${TOPPRA_CPP_DIR}/src")
    else()
        message(WARNING "TOPPRA source not found at ${TOPPRA_CPP_DIR}/src, building without TOPPRA")
    endif()

    set(HAS_NLOPT OFF)
    if(EXISTS "${NLOPT_DIR}/CMakeLists.txt")
        # 仅构建核心静态 NLopt C API 库，不启用语言绑定 / tests / shared libs。
        # 当前 WASM 集成先避免引入 Luksan 的 LGPL 算法与额外的 C++ 算法源。
        set(NLOPT_CXX OFF CACHE BOOL "Enable NLopt C++ routines" FORCE)
        set(NLOPT_FORTRAN OFF CACHE BOOL "Enable NLopt Fortran support" FORCE)
        set(NLOPT_PYTHON OFF CACHE BOOL "Build NLopt Python bindings" FORCE)
        set(NLOPT_OCTAVE OFF CACHE BOOL "Build NLopt Octave bindings" FORCE)
        set(NLOPT_MATLAB OFF CACHE BOOL "Build NLopt MATLAB bindings" FORCE)
        set(NLOPT_GUILE OFF CACHE BOOL "Build NLopt Guile bindings" FORCE)
        set(NLOPT_JAVA OFF CACHE BOOL "Build NLopt Java bindings" FORCE)
        set(NLOPT_SWIG OFF CACHE BOOL "Use SWIG for NLopt bindings" FORCE)
        set(NLOPT_LUKSAN OFF CACHE BOOL "Enable LGPL Luksan solvers" FORCE)
        set(NLOPT_TESTS OFF CACHE BOOL "Build NLopt tests" FORCE)
        set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build third-party libraries as shared libraries" FORCE)

        add_subdirectory(${NLOPT_DIR} ${CMAKE_BINARY_DIR}/nlopt_src)
        set(HAS_NLOPT ON)
        message(STATUS "Configured NLopt core from: ${NLOPT_DIR}")
    else()
        message(WARNING "NLopt source not found at ${NLOPT_DIR}, building without NLopt")
    endif()
    
    # 添加 DQ Robotics 库的源文件
    set(DQROBOTICS_SOURCES
        ${DQROBOTICS_DIR}/src/DQ.cpp
        ${DQROBOTICS_DIR}/src/internal/_dq_linesegment.cpp
        ${DQROBOTICS_DIR}/src/utils/DQ_Geometry.cpp
        ${DQROBOTICS_DIR}/src/utils/DQ_LinearAlgebra.cpp
        ${DQROBOTICS_DIR}/src/utils/DQ_Math.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_Kinematics.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulator.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulatorDH.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulatorMDH.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulatorURDF.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_HumanoidURDF.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_MobileBase.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_HolonomicBase.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_DifferentialDriveRobot.cpp
        ${DQROBOTICS_DIR}/src/robots/FrankaEmikaPandaRobot.cpp
        ${DQROBOTICS_DIR}/src/robots/KukaLw4Robot.cpp
    )
    
    # 创建 WASM 模块
    add_executable(dqrobotics
        bindings.cpp
        nlopt_humanoid_solver.cpp
        ${DQROBOTICS_SOURCES}
    )
    
    # 包含目录
    target_include_directories(dqrobotics PRIVATE
        ${DQROBOTICS_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
    )

    if(HAS_TOPPRA)
        target_link_libraries(dqrobotics PRIVATE toppra)
        target_compile_definitions(dqrobotics PRIVATE TOPPRA_AVAILABLE=1)
    else()
        target_compile_definitions(dqrobotics PRIVATE TOPPRA_AVAILABLE=0)
    endif()

    if(HAS_NLOPT)
        target_link_libraries(dqrobotics PRIVATE nlopt)
        target_compile_definitions(dqrobotics PRIVATE NLOPT_AVAILABLE=1)
    else()
        target_compile_definitions(dqrobotics PRIVATE NLOPT_AVAILABLE=0)
    endif()
    
    # Emscripten 链接标志
    set(EMSCRIPTEN_LINK_FLAGS
        "-lembind"
        "-s WASM=1"
        "-s ALLOW_MEMORY_GROWTH=1"
        "-s MODULARIZE=1"
        "-s EXPORT_NAME='createDQRoboticsModule'"
        "-s ENVIRONMENT='web,worker'"
        "-s SINGLE_FILE=1"
        "-s DISABLE_EXCEPTION_CATCHING=0"
        "-s ASSERTIONS=1"
        "--bind"
        "-O3"
    )
    
    # 将标志转换为字符串
    string(REPLACE ";" " " EMSCRIPTEN_LINK_FLAGS_STR "${EMSCRIPTEN_LINK_FLAGS}")
    
    # 设置链接器标志
    set_target_properties(dqrobotics PROPERTIES
        LINK_FLAGS "${EMSCRIPTEN_LINK_FLAGS_STR}"
    )
    
    # 安装规则
    install(TARGETS dqrobotics
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../public/wasm
    )
    
    message(STATUS "Configured DQ Robotics WASM build")
    message(STATUS "DQ Robotics directory: ${DQROBOTICS_DIR}")
    message(STATUS "Eigen3 include: ${EIGEN3_INCLUDE_DIR}")
    message(STATUS "TOPPRA enabled: ${HAS_TOPPRA}")
    message(STATUS "NLopt enabled: ${HAS_NLOPT}")
    
else()
    message(FATAL_ERROR "This project must be built with Emscripten")
endif()
