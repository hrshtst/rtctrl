include(Catch)

add_executable(unit_tests
  unit/smoke_test.cpp
  unit/model_test.cpp)
target_link_libraries(unit_tests PRIVATE rtctrl::rtctrl Catch2::Catch2WithMain)
# Model files reference meshes by repo-root-relative paths (zeo resolves
# shape imports against the CWD), so tests run from the source root.
catch_discover_tests(unit_tests
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  PROPERTIES LABELS unit)
