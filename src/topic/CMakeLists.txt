cmake_minimum_required(VERSION 3.8)
project(topic)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT APPLE AND NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CCAPI_LEGACY_USE_WEBSOCKETPP)
  message(STATUS "Using Boost Beast for WebSocket")
  include(ExternalProject)
  if(NOT BOOST_INCLUDE_DIR)
    ExternalProject_Add(boost
      URL https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz
      SOURCE_DIR "${CMAKE_BINARY_DIR}/boost"
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      TEST_COMMAND ""
    )
    set(BOOST_INCLUDE_DIR "${CMAKE_BINARY_DIR}/boost")
  endif()

  if(NOT RAPIDJSON_INCLUDE_DIR)
    ExternalProject_Add(rapidjson
      URL https://github.com/Tencent/rapidjson/archive/refs/tags/v1.1.0.tar.gz
      SOURCE_DIR "${CMAKE_BINARY_DIR}/rapidjson"
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      TEST_COMMAND ""
    )
    set(RAPIDJSON_INCLUDE_DIR "${CMAKE_BINARY_DIR}/rapidjson/include")
  endif()

  if(NOT HFFIX_INCLUDE_DIR)
    ExternalProject_Add(hffix
      URL https://github.com/jamesdbrock/hffix/archive/refs/tags/v1.3.0.tar.gz
      SOURCE_DIR "${CMAKE_BINARY_DIR}/hffix"
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      TEST_COMMAND ""
    )
    set(HFFIX_INCLUDE_DIR "${CMAKE_BINARY_DIR}/hffix/include")
  endif()

else()
  message(STATUS "Using legacy WebSocket++")
  set(RAPIDJSON_INCLUDE_DIR ${CCAPI_PROJECT_DIR}/dependency/rapidjson/include)
  set(WEBSOCKETPP_INCLUDE_DIR ${CCAPI_PROJECT_DIR}/dependency/websocketpp)
  set(BOOST_INCLUDE_DIR ${CCAPI_PROJECT_DIR}/dependency/boost)
  set(HFFIX_INCLUDE_DIR ${CCAPI_PROJECT_DIR}/dependency/hffix/include)
endif()


# Set the Apache Arrow installation path
set(CMAKE_PREFIX_PATH "/usr/local")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")


# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(Arrow REQUIRED)  # Finds Apache Arrow
find_package(Parquet REQUIRED)  # Find Parquet separately if needed
find_package(OpenSSL REQUIRED)
link_libraries(
  OpenSSL::Crypto
  OpenSSL::SSL
)


# If linking Parquet separately is needed (comment out if not)
# find_package(Parquet REQUIRED)
# include_directories(${PARQUET_INCLUDE_DIR})

# Include directories for both Arrow and Parquet
include_directories(${ARROW_INCLUDE_DIR})
include_directories(${PARQUET_INCLUDE_DIR})
include(FetchContent)
include_directories(
  ${CCAPI_PROJECT_DIR}/include
  ${BOOST_INCLUDE_DIR}
  ${RAPIDJSON_INCLUDE_DIR}
  ${HFFIX_INCLUDE_DIR}
  ${OPENSSL_INCLUDE_DIR}
)

FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.10.5
)

FetchContent_MakeAvailable(json)

add_compile_options(-Wno-deprecated -Wno-nonnull -Wno-deprecated-declarations)

get_filename_component(CCAPI_PROJECT_DIR ../ ABSOLUTE)
message(STATUS "CCAPI_PROJECT_DIR: ${CCAPI_PROJECT_DIR}")
# Enable multiple exchanges
add_compile_definitions(CCAPI_ENABLE_SERVICE_MARKET_DATA)
add_compile_definitions(CCAPI_ENABLE_EXCHANGE_BINANCE)
add_compile_definitions(CCAPI_ENABLE_EXCHANGE_BITMEX)
# Linter settings
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare executables
add_executable(data_pull src/data_pull.cpp)
add_executable(data_storage src/data_storage.cpp)

# Declare dependencies of executables
ament_target_dependencies(data_pull rclcpp std_msgs interfaces)
ament_target_dependencies(data_storage rclcpp std_msgs interfaces)

# Ensure the external projects are built before your executable
add_dependencies(${NAME} boost rapidjson)
include_directories($ENV{HOME}/Desktop/ccapi/include)

# Link libraries with executables
# Link Arrow (and Parquet if separate) with your executables
# Link both Arrow and Parquet with your executables
target_link_libraries(data_pull ${ARROW_SHARED_LIB} ${PARQUET_SHARED_LIB})
target_link_libraries(data_storage ${ARROW_SHARED_LIB} ${PARQUET_SHARED_LIB})
target_link_libraries(data_pull ${ARROW_SHARED_LIB})  # Link Arrow to publisher
target_link_libraries(data_storage ${ARROW_SHARED_LIB})  # Link Arrow to subscriber
target_include_directories(data_pull PRIVATE ${json_SOURCE_DIR}/single_include)


# If Parquet is separate, uncomment the following lines


# Install the executables

install(TARGETS data_pull data_storage DESTINATION lib/${PROJECT_NAME})

# Package configuration
ament_package()