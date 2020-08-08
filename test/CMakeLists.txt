add_executable(Test Test.cpp)
target_link_libraries(Test PUBLIC
    gtest_main
)

add_test(
    NAME Test
    COMMAND Test
)