add_library(servo_manager STATIC)

target_sources(servo_manager
    PRIVATE
        servo_manager.cpp
        servo_manager.hpp
)

target_include_directories(servo_manager
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(servo_manager
    PUBLIC
        stm32cubemx      
        servo_lib        
)

target_compile_options(servo_manager
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
