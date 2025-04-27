add_library(robot_controller STATIC)

target_sources(robot_controller
    PRIVATE
        robot_controller.cpp
        robot_controller.hpp
)

target_include_directories(robot_controller
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robot_controller
    PUBLIC
        stm32cubemx
        servo_manager
)

target_compile_options(robot_controller
    PUBLIC
        -std=c++23
        -Wall
        -Wextra
        -Wconversion
        -Wshadow
        -Wpedantic
        -Wnarrowing
        -Waddress
        -pedantic
        -Wdeprecated
        -Wsign-conversion
        -Wduplicated-cond
        -Wduplicated-branches
        -Wlogical-op
        -Wnull-dereference
        -Wdouble-promotion
        -Wimplicit-fallthrough
        -Wcast-align
        -fconcepts
)
