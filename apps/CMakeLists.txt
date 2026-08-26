# Sources are grouped by domain (bus/ bringup/ gravity/ track/ plan/
# ident/ study/, cross-domain headers in common/); binaries stay FLAT at
# build/apps/ — the bring-up checklist, operator shell history, and
# archived session notes all invoke ./build/apps/<name>, so no
# add_subdirectory() here. App-layer includes are subdir-qualified and
# resolve from this directory as the include root.

add_executable(dxl_emu bus/dxl_emu.cpp)
target_link_libraries(dxl_emu PRIVATE rtctrl::rtctrl)

add_executable(dxl_inspect bus/dxl_inspect.cpp)
target_link_libraries(dxl_inspect PRIVATE rtctrl::rtctrl)
target_include_directories(dxl_inspect PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(dxl_load_params bus/dxl_load_params.cpp)
target_link_libraries(dxl_load_params PRIVATE rtctrl::rtctrl)
target_include_directories(dxl_load_params PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

set(PTP_GIT_REV_SRC ${CMAKE_BINARY_DIR}/generated/ptp_git_rev.cpp)
add_custom_target(ptp_git_rev
  COMMAND ${CMAKE_COMMAND} -DOUT=${PTP_GIT_REV_SRC}
          -DSRC_DIR=${CMAKE_SOURCE_DIR}
          -P ${CMAKE_SOURCE_DIR}/cmake/GitRev.cmake
  BYPRODUCTS ${PTP_GIT_REV_SRC})
add_executable(x7_plan_ptp plan/x7_plan_ptp.cpp plan/ptp_bundle.cpp
  ${PTP_GIT_REV_SRC})
add_dependencies(x7_plan_ptp ptp_git_rev)
target_link_libraries(x7_plan_ptp PRIVATE rtctrl::rtctrl)
target_include_directories(x7_plan_ptp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

set(FOLLOW_GIT_REV_SRC ${CMAKE_BINARY_DIR}/generated/follow_git_rev.cpp)
add_custom_target(follow_git_rev
  COMMAND ${CMAKE_COMMAND} -DOUT=${FOLLOW_GIT_REV_SRC}
          -DSRC_DIR=${CMAKE_SOURCE_DIR}
          -P ${CMAKE_SOURCE_DIR}/cmake/GitRev.cmake
  BYPRODUCTS ${FOLLOW_GIT_REV_SRC})

set(FOLLOW_BUNDLE_SOURCES follow/follow_bundle.cpp plan/ptp_bundle.cpp)
add_executable(x7_follow_sim follow/x7_follow_sim.cpp
  ${FOLLOW_BUNDLE_SOURCES} ${FOLLOW_GIT_REV_SRC})
add_dependencies(x7_follow_sim follow_git_rev)
target_link_libraries(x7_follow_sim PRIVATE rtctrl::rtctrl)
target_include_directories(x7_follow_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(x7_follow follow/x7_follow.cpp
  ${FOLLOW_BUNDLE_SOURCES} ${FOLLOW_GIT_REV_SRC})
add_dependencies(x7_follow follow_git_rev)
target_link_libraries(x7_follow PRIVATE rtctrl::rtctrl)
target_include_directories(x7_follow PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

foreach(src bringup/x7_onoff bringup/x7_read bringup/x7_set_param
        bringup/x7_move_simple bringup/x7_pose gravity/x7_float
        gravity/x7_gravity_demo track/x7_track track/x7_track_sim
        ident/x7_ident_sim)
  get_filename_component(app ${src} NAME)
  add_executable(${app} ${src}.cpp)
  target_link_libraries(${app} PRIVATE rtctrl::rtctrl)
  target_include_directories(${app} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

# Offline EFL study runner (docs/records/history.md (EFL study)). The git
# provenance source is regenerated on EVERY build — a configure-time
# value goes stale after later commits (review finding); the dirty
# flag counts untracked files.
set(EFL_GIT_REV_SRC ${CMAKE_BINARY_DIR}/generated/efl_git_rev.cpp)
add_custom_target(efl_git_rev
  COMMAND ${CMAKE_COMMAND} -DOUT=${EFL_GIT_REV_SRC}
          -DSRC_DIR=${CMAKE_SOURCE_DIR}
          -P ${CMAKE_SOURCE_DIR}/cmake/GitRev.cmake
  BYPRODUCTS ${EFL_GIT_REV_SRC})
add_executable(x7_efl_study study/x7_efl_study.cpp ${EFL_GIT_REV_SRC})
add_dependencies(x7_efl_study efl_git_rev)
target_compile_definitions(x7_efl_study PRIVATE
  RTCTRL_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
target_link_libraries(x7_efl_study PRIVATE rtctrl::rtctrl)
target_include_directories(x7_efl_study PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
