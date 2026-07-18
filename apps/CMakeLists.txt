add_executable(dxl_emu dxl_emu.cpp)
target_link_libraries(dxl_emu PRIVATE rtctrl::rtctrl)

add_executable(dxl_inspect dxl_inspect.cpp)
target_link_libraries(dxl_inspect PRIVATE rtctrl::rtctrl)

foreach(app x7_onoff x7_read x7_set_param x7_move_simple)
  add_executable(${app} ${app}.cpp)
  target_link_libraries(${app} PRIVATE rtctrl::rtctrl)
endforeach()
