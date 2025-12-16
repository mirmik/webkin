cmake_minimum_required(VERSION 3.16)
project(webkin)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Threads REQUIRED)

# Optional: MQTT support via mosquitto
find_library(MOSQUITTO_LIB mosquitto)
if(MOSQUITTO_LIB)
    message(STATUS "Found mosquitto library: ${MOSQUITTO_LIB}")
    add_definitions(-DHAVE_MOSQUITTO)
else()
    message(WARNING "mosquitto library not found, MQTT support disabled")
endif()

# Optional: Crow protocol support
find_library(CROW_LIB crow)
if(CROW_LIB)
    message(STATUS "Found crow library: ${CROW_LIB}")
    add_definitions(-DHAVE_CROW)
else()
    message(WARNING "crow library not found, Crow protocol support disabled")
endif()

# Source files
set(SOURCES
    src/main.cpp
    src/mqtt_listener.cpp
    src/crow_listener.cpp
)

# Include directories
set(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../nos
    ${CMAKE_CURRENT_SOURCE_DIR}/../igris
    ${CMAKE_CURRENT_SOURCE_DIR}/../crow
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIRS})

# Link with system libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    Threads::Threads
    nos
    igris
)

if(MOSQUITTO_LIB)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${MOSQUITTO_LIB})
endif()

if(CROW_LIB)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${CROW_LIB})
endif()

# CrowHTTP/Asio defines (for the web server, not crow protocol)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    ASIO_STANDALONE
    CROW_USE_BOOST=0
    CROW_DISABLE_STATIC_DIR
)

# Install
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
