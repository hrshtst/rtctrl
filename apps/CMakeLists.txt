add_executable(dxl_emu dxl_emu.cpp)
target_link_libraries(dxl_emu PRIVATE rtctrl::rtctrl)

add_executable(dxl_inspect dxl_inspect.cpp)
target_link_libraries(dxl_inspect PRIVATE rtctrl::rtctrl)

foreach(app x7_onoff x7_read x7_set_param x7_move_simple x7_float x7_track
        x7_track_sim x7_ident x7_ident_sim x7_pose)
  add_executable(${app} ${app}.cpp)
  target_link_libraries(${app} PRIVATE rtctrl::rtctrl)
endforeach()

# Offline EFL study runner (docs/ENVELOPE_STABILITY_PLAN.md). The git
# provenance source is regenerated on EVERY build — a configure-time
# value goes stale after later commits (review finding); the dirty
# flag counts untracked files.
set(EFL_GIT_REV_SRC ${CMAKE_BINARY_DIR}/generated/efl_git_rev.cpp)
add_custom_target(efl_git_rev
  COMMAND ${CMAKE_COMMAND} -DOUT=${EFL_GIT_REV_SRC}
          -DSRC_DIR=${CMAKE_SOURCE_DIR}
          -P ${CMAKE_SOURCE_DIR}/cmake/GitRev.cmake
  BYPRODUCTS ${EFL_GIT_REV_SRC})
add_executable(x7_efl_study x7_efl_study.cpp ${EFL_GIT_REV_SRC})
add_dependencies(x7_efl_study efl_git_rev)
target_compile_definitions(x7_efl_study PRIVATE
  RTCTRL_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
target_link_libraries(x7_efl_study PRIVATE rtctrl::rtctrl)
