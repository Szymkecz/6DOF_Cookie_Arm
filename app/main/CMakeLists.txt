add_executable(app)

target_sources(app PRIVATE 
    "main.cpp"
)

target_include_directories(app PRIVATE 
    "." 
)

target_link_libraries(app PRIVATE
    stm32cubemx
    servo_lib
    servo_manager
    robot_controller
    kinematics
    callbacks
    uart
    )

target_compile_options(app PUBLIC
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
