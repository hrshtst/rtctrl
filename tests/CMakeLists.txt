include(Catch)

add_executable(unit_tests
  unit/smoke_test.cpp)
target_link_libraries(unit_tests PRIVATE rtctrl::rtctrl Catch2::Catch2WithMain)
catch_discover_tests(unit_tests PROPERTIES LABELS unit)
