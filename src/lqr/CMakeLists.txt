ADD_DEFINITIONS(-std=c++11)
add_library(lqr lqr.cpp)

install (TARGETS lqr DESTINATION bin)
install (FILES lqr.hpp DESTINATION include)
