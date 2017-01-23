/**
*    @author Harikrishnan Lakshmanan
*    @file sample.cpp
*    @date 11/24/2017
*
*    @brief WAVe Lab, ***WARNING*** This is a sample code. This does not compile.
*
*    @section Updates to make
*    Looks like, it's pretty much alright.
*
*    @section Optimization Issues
*    I don't see any potential drawback
*/

#include "ekf_utilities.cpp"
#include "car_utilities.cpp"

int main()
{
    kinematic_bicycle c;

    matrix<double> Xi(5,1);
    Xi(0,0) = 0;
    Xi(1,0) = 10;
    Xi(2,0) = 0;
    Xi(3,0) = 0;
    Xi(4,0) = 0;

    identity_matrix<double> Pi (5, 5);
    zero_matrix<double> Hi (5, 5);

    /// Tune the values below as you find fit
    matrix<double> Qi(5,5);
    for (unsigned i = 0; i < Qi.size1 (); ++ i)
        for (unsigned j = 0; j < Qi.size2 (); ++ j)
            if (i==j) Qi (i, j) = 0.0000004;
            else Qi(i,j) = 0;

    matrix<double> Ri(5,5);
    for (unsigned i = 0; i < Ri.size1 (); ++ i)
        for (unsigned j = 0; j < Ri.size2 (); ++ j)
            if (i==j) Ri (i, j) = 0.000005;
            else Ri(i,j) = 0;

    acc = /// Function to get the desired control action from ROS
    phi = /// Function to get the desired steering angle from ROS in radians
    matrix<double> control_vectori(2,1);
    control_vectori(0,0) = acc;
    control_vectori(1,0) = phi;

    matrix<double> xxi(5,1);
    matrix<double> measurement_vectori(5,1);
    ekf filteri(Xi, Pi, Hi, Qi, Ri);

    while(true)
    {
        filteri.get_state(xxi);
        measurement_vectori = /// Function to get the sensor measurements from ROS
        c.update(acc, phi, del_t);
        filteri.step(control_vectori, measurement_vectori, del_t);
    }
}
