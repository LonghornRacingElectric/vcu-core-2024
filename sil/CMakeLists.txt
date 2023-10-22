set(BINARY vcu_core)

file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true *.cpp)
set(SOURCES ${SOURCES})

add_executable(
        ${BINARY}
        ${SOURCES}
)
target_link_libraries(
        ${BINARY}
        -static
        vcu_core_lib
)
