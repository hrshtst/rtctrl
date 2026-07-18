add_executable(dxl_emu dxl_emu.cpp)
target_link_libraries(dxl_emu PRIVATE rtctrl::rtctrl)

add_executable(dxl_inspect dxl_inspect.cpp)
target_link_libraries(dxl_inspect PRIVATE rtctrl::rtctrl)
