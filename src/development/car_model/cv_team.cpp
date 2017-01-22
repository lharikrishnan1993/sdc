/**
*    @author Harikrishnan Lakshmanan
*    @file cv_team.cpp
*    @date 11/29/2016
*    @ref http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf
*
*    @brief WAVe Lab, A sample program go get the crux on how to use the projection functions.
*
*    @section Updates to make
*    NIL
*
*    @section Optimization Issues
*    These functions don't demand optimality as it does not run in real time
*    and hence optimality was not a concern while development of the concerned functions.
*
*    @section How to use
*    Create a object -> Car c (Dubin's Car Model), kinematic_bicycle b (Kinematic Bicycle Car Model)
*    Create a std::vector to hold the start position -> std::vector <float> pose = {10,0,0}; (Assumes X = 10 m, Y = 0 m, Orientation = 0 Radians)
*    Create a std::vector to hold the returned positions -> std::vector <float> returned_pose;
*    Call the [object].step function along with the speed, stering angle(radians) and the desired delta time -> returned_pose = c.step(pose, vel, phi, del_t);
*    pose = c.step(Vector of current pose, Speed of the vehicle (m/s), Steering angle in radians, Delta time in seconds);
*    Store the value in returned_pose and get the values in order. X(m) in index 0, Y(m) in index 1, Theta(rad) in index 2.
*/

#include <iostream>
#include "car_utilities.cpp"

int main()
{
    Car c;
    kinematic_bicycle b;
    float del_t = 0.5;
    float time = 10;
    float i = 0;
    float phi = d2r(10.0);
    float vel = 5.0;

    std::vector <float> pose = {0,0,0};
//    std::vector <float>::iterator iter;
//    while (i < time)
//    {
        pose = b.step(pose, vel, phi, del_t);
//        for (iter = pose.begin(); iter != pose.end(); iter++) std::cout<<*iter<<" ";
//        std::cout<<std::endl;
//        i += del_t;
//    }
    return 0;
}

