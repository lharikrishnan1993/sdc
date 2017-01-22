ADD_DEFINITIONS(-std=c++11)
add_library(car car_utilities.cpp)

install (TARGETS car DESTINATION bin)
install (FILES car.hpp DESTINATION include)
