add_executable(make_motion make_motion.cpp)
target_link_libraries(make_motion PRIVATE rtctrl::rtctrl)

add_executable(x7_wave x7_wave.cpp)
target_link_libraries(x7_wave PRIVATE rtctrl::rtctrl)
