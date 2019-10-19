cmake_minimum_required(VERSION 3.5)

project(myapp_apis)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

set(custom_messages
    "msg/AppCommand.msg"
    "msg/AppData.msg"
)

set(custom_services
    "srv/GetAppData.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${custom_messages}
    ${custom_services}
    DEPENDENCIES std_msgs
)

ament_package()
