cmake_minimum_required(VERSION 3.16)
project(g2_torque_subscriber CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Threads REQUIRED)

# ---- Dynamics model (Pinocchio: gravity, Coriolis, inertia), used by the node ----
add_library(g2_dynamics_model STATIC src/dynamics_model.cpp)
set_target_properties(g2_dynamics_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(g2_dynamics_model PUBLIC include)
target_link_libraries(g2_dynamics_model PUBLIC pinocchio::pinocchio)

# ---- The node. It does NOT link the GDK: the GDK is dlopen()ed from a backend module ----
add_executable(torque_subscriber_node src/torque_subscriber_node.cpp)
target_link_libraries(torque_subscriber_node g2_dynamics_model ${CMAKE_DL_LIBS} Threads::Threads)
ament_target_dependencies(torque_subscriber_node ament_index_cpp rclcpp sensor_msgs std_msgs)

# ---- Fake backend: simulated arms for offline tests (backend:=fake) ----
add_library(g2_fake_backend MODULE src/fake_backend.cpp)
target_include_directories(g2_fake_backend PRIVATE include)
target_link_libraries(g2_fake_backend pinocchio::pinocchio)

# ---- GDK backend: needs the AGIBOT GDK (headers + libgdk_adapter.so) at build time ----
# GDK_APP_DIR is the GDK "app" directory (the one holding env.sh, lib/ and gdk/).
set(_gdk_candidates "$ENV{GDK_APP_DIR}" "/opt/agibot/app" "$ENV{HOME}/.cache/agibot/app")
set(GDK_APP_DIR "" CACHE PATH "AGIBOT GDK app directory (contains env.sh, lib/, gdk/)")
if(NOT GDK_APP_DIR)
  foreach(_dir ${_gdk_candidates})
    if(_dir AND EXISTS "${_dir}/gdk/build_dep/cpp")
      set(GDK_APP_DIR "${_dir}")
      break()
    endif()
  endforeach()
endif()
set(_gdk_dep "${GDK_APP_DIR}/gdk/build_dep/cpp/${CMAKE_SYSTEM_PROCESSOR}")
if(GDK_APP_DIR AND EXISTS "${_gdk_dep}/lib/libgdk_adapter.so")
  message(STATUS "GDK: ${GDK_APP_DIR} (${CMAKE_SYSTEM_PROCESSOR})")
  add_library(g2_gdk_backend MODULE src/gdk_backend.cpp)
  target_include_directories(g2_gdk_backend PRIVATE include "${_gdk_dep}/include")
  target_link_libraries(g2_gdk_backend "${_gdk_dep}/lib/libgdk_adapter.so")
  # libgdk_adapter's own dependencies (libgdk_core, bundled Fast DDS 3, ...) live in <app>/lib.
  # DT_RPATH (--disable-new-dtags) also applies to those indirect dependencies; the GDK env.sh
  # (LD_LIBRARY_PATH) must still be sourced for the GDK's network configuration.
  target_link_options(g2_gdk_backend PRIVATE
    "-Wl,-rpath-link,${GDK_APP_DIR}/lib" "-Wl,--disable-new-dtags"
    "-Wl,-rpath,${GDK_APP_DIR}/lib:${GDK_APP_DIR}/gdk/lib")
  install(TARGETS g2_gdk_backend LIBRARY DESTINATION lib/${PROJECT_NAME}
          PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
else()
  message(WARNING "AGIBOT GDK not found (set -DGDK_APP_DIR=<gdk>/app or env GDK_APP_DIR): "
                  "building without the GDK backend; only backend:=fake will work")
endif()

install(TARGETS torque_subscriber_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS g2_fake_backend LIBRARY DESTINATION lib/${PROJECT_NAME}
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)  # not listed as executables

# ---- Python node ----
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/torque_subscriber_node.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})  # urdf/: the confirmed G2 model
install(DIRECTORY test/fake_gdk DESTINATION share/${PROJECT_NAME})

ament_package()
