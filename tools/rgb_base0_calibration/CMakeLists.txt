cmake_minimum_required(VERSION 3.20)
project(rgb_base0_calibration_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT WIN32)
  message(FATAL_ERROR "These tools require Windows, Orbbec SDK v1.10.18, and HIWIN HRSDK.")
endif()

set(ORBBEC_SDK_ROOT "C:/Program Files/OrbbecSDK 1.10.18/SDK" CACHE PATH
    "Exact Orbbec SDK v1.10.18 SDK directory")
set(PROJECT_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH
    "camera_test repository root")
get_filename_component(PROJECT_REPO_ROOT_ABS "${PROJECT_REPO_ROOT}" ABSOLUTE)

set(ORBBEC_INCLUDE_DIR "${ORBBEC_SDK_ROOT}/include")
set(ORBBEC_LIBRARY "${ORBBEC_SDK_ROOT}/lib/OrbbecSDK.lib")
set(HRSDK_INCLUDE_DIR "${PROJECT_REPO_ROOT_ABS}/include" CACHE PATH
    "Directory containing HRSDK.h")
set(HRSDK_LIBRARY "${PROJECT_REPO_ROOT_ABS}/lib/HRSDK.lib" CACHE FILEPATH
    "Path to the x64 HRSDK import library")
set(HRSDK_RUNTIME_DLL "${PROJECT_REPO_ROOT_ABS}/bin/HRSDK.dll" CACHE FILEPATH
    "Path to the matching x64 HRSDK runtime DLL")

foreach(required_path IN ITEMS
    "${ORBBEC_INCLUDE_DIR}/libobsensor/ObSensor.hpp"
    "${ORBBEC_LIBRARY}"
    "${HRSDK_INCLUDE_DIR}/HRSDK.h"
    "${HRSDK_LIBRARY}"
    "${HRSDK_RUNTIME_DLL}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "Required dependency not found: ${required_path}")
  endif()
endforeach()

add_library(rgb_base0_core STATIC
  src/brown_projection.cpp
  src/camera_profile.cpp
  src/calibration_io.cpp
  src/geometry.cpp
  src/runtime_utils.cpp
)
target_include_directories(rgb_base0_core PUBLIC include)
target_compile_definitions(rgb_base0_core PUBLIC RGB_BASE0_REPO_ROOT="${PROJECT_REPO_ROOT_ABS}")
target_compile_options(rgb_base0_core PRIVATE /W4 /permissive- /EHsc)

add_library(rgb_base0_hardware STATIC
  src/orbbec_camera.cpp
  src/robot_pose_reader.cpp
)
target_include_directories(rgb_base0_hardware PUBLIC
  include
  "${ORBBEC_INCLUDE_DIR}"
  "${HRSDK_INCLUDE_DIR}"
)
target_link_libraries(rgb_base0_hardware PUBLIC rgb_base0_core "${ORBBEC_LIBRARY}" "${HRSDK_LIBRARY}")
target_compile_definitions(rgb_base0_hardware PRIVATE _CRT_SECURE_NO_WARNINGS)
target_compile_options(rgb_base0_hardware PRIVATE /W4 /wd4251 /permissive- /EHsc)

add_executable(rgb_base0_calibrate src/calibrate_main.cpp)
target_link_libraries(rgb_base0_calibrate PRIVATE rgb_base0_hardware)
target_compile_options(rgb_base0_calibrate PRIVATE /W4 /permissive- /EHsc)

add_executable(rgb_base0_validate src/validate_main.cpp)
target_link_libraries(rgb_base0_validate PRIVATE rgb_base0_hardware)
target_compile_options(rgb_base0_validate PRIVATE /W4 /permissive- /EHsc)

add_executable(rgb_base0_geometry_tests tests/geometry_tests.cpp)
target_link_libraries(rgb_base0_geometry_tests PRIVATE rgb_base0_core)
target_compile_options(rgb_base0_geometry_tests PRIVATE /W4 /permissive- /EHsc)
enable_testing()
add_test(NAME rgb_base0_geometry_tests COMMAND rgb_base0_geometry_tests)

set(ORBBEC_RUNTIME_DLLS
  OrbbecSDK.dll
  ob_usb.dll
  live555.dll
)
foreach(target IN ITEMS rgb_base0_calibrate rgb_base0_validate)
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${HRSDK_RUNTIME_DLL}"
      "$<TARGET_FILE_DIR:${target}>"
    VERBATIM)
  foreach(runtime_dll IN LISTS ORBBEC_RUNTIME_DLLS)
    if(EXISTS "${ORBBEC_SDK_ROOT}/lib/${runtime_dll}")
      add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${ORBBEC_SDK_ROOT}/lib/${runtime_dll}"
          "$<TARGET_FILE_DIR:${target}>"
        VERBATIM)
    endif()
  endforeach()
endforeach()
