set(BINARY vcu_core)

file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES true *.cpp)

add_executable(
        ${BINARY}
        ${TEST_SOURCES}
)
target_link_libraries(
        ${BINARY}
        vcu_core_lib
)
