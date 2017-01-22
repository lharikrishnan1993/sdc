ADD_DEFINITIONS(-std=c++11)
add_library(rrt naive_rrt.cpp)

install (TARGETS rrt DESTINATION bin)
install (FILES rrt.h DESTINATION include)
install (FILES rrt_utilities.cpp DESTINATION include)
