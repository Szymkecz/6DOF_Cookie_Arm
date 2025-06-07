add_library(callbacks STATIC)

target_sources(callbacks
    PRIVATE
        callbacks.cpp
        callbacks.hpp
)

target_include_directories(callbacks
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(callbacks
    PUBLIC
        stm32cubemx
        kinematics
        servo_manager
)

target_compile_options(callbacks
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
