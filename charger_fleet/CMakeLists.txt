cmake_minimum_required(VERSION 3.5.0)
project(charger_fleet VERSION 0.0.1 LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

include(GNUInstallDirs)

find_package(CycloneDDS REQUIRED)

# -----------------------------------------------------------------------------

add_library(charger_fleet SHARED
  src/Client.cpp
  src/ClientImpl.cpp
  src/configs/ClientConfig.cpp
  src/Server.cpp
  src/ServerImpl.cpp
  src/configs/ServerConfig.cpp
  src/messages/FleetMessages.c
  src/messages/message_utils.cpp
  src/dds_utils/common.cpp
)
target_include_directories(charger_fleet
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(charger_fleet
  CycloneDDS::ddsc
  ssl
  crypto
)

# -----------------------------------------------------------------------------

set(testing_targets
  test_dds_pub_mode_request
  test_dds_sub_state
)

foreach(target ${testing_targets})
  add_executable(${target}
    src/tests/${target}.cpp
    src/dds_utils/common.cpp
    src/messages/FleetMessages.c
  )
  target_link_libraries(${target}
    CycloneDDS::ddsc
    ssl
    crypto
  )
endforeach()

install(
  TARGETS ${testing_targets}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# -----------------------------------------------------------------------------

# Mark executables and/or libraries for installation
list(APPEND PACKAGE_LIBRARIES
  charger_fleet
)
install(
  TARGETS ${PACKAGE_LIBRARIES}
  EXPORT charger_fleet-targets
  DESTINATION lib
)
install(
  EXPORT charger_fleet-targets 
  DESTINATION lib/cmake/charger_fleet
)

# Mark cpp header files for installation
install(
  DIRECTORY include/charger_fleet
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
)
install(
  FILES package.xml 
  DESTINATION share/charger_fleet
)

# Create cmake config files
# See doc: https://cmake.org/cmake/help/latest/module/CMakePackageConfigHelpers.html#example-generating-package-files
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/charger_fleet-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/charger_fleet-config.cmake
  INSTALL_DESTINATION lib/cmake/charger_fleet
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/charger_fleet-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/charger_fleet-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/charger_fleet-config-version.cmake
  DESTINATION lib/cmake/charger_fleet
)
export(
  EXPORT charger_fleet-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/charger_fleet-targets.cmake
)
