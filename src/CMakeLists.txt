file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true *.h *.cpp)
set(SOURCES ${SOURCES})

add_library(vcu_core_lib STATIC ${SOURCES})
add_library(vcu_core SHARED ${SOURCES}
        util/math/Position.cpp
        util/math/Position.h)
