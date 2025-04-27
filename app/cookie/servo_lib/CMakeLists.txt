add_library(servo_lib STATIC)

target_sources(servo_lib
    PRIVATE
        servo.cpp
        servo.hpp
)

target_include_directories(servo_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(servo_lib
    PUBLIC
        stm32cubemx
)

target_compile_options(servo_lib
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
