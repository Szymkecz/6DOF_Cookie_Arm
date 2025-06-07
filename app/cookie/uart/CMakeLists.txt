add_library(uart STATIC)

target_sources(uart
    PRIVATE
        uart.cpp
        uart.hpp
)

target_include_directories(uart
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(uart
    PUBLIC
        stm32cubemx
)

target_compile_options(uart
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
