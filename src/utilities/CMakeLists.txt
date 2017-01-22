ADD_DEFINITIONS(-std=c++11)
add_library(utilities matrix_printer.cpp)

install (TARGETS utilities DESTINATION bin)
