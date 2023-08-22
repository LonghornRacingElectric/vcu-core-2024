set(BINARY vcu_core_lib)

file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true *.h *.cpp)
#list(FILTER SOURCES EXCLUDE REGEX ".*PyBindWrapper.cpp$")
set(SOURCES ${SOURCES})

#find_package(Python COMPONENTS Interpreter Development)
#pybind11_add_module(vcu_core ${SOURCES} PyBindWrapper.cpp)
add_library(${BINARY} STATIC ${SOURCES})
