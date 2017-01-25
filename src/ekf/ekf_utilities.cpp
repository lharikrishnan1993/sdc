/**
*    @author Harikrishnan Lakshmanan
*    @file ekf_utilities.cpp
*    @date 11/21/2016
*
*    @brief WAVe Lab, State Estimator using EKF.
*
*
*    @section Updates to make
*
*    1) Try to make the function ekf::get_F( matrix <double>& control_vector, float del_t)
*    automated as the Jacobian is currently being calculated manually.
*        a) Possible hacks as far as I have seen is to use Boost Python module and take it
*           to the python interface and calculate the Jacobian using sympy as can be seen in
*           Matt's Python version of EKF.
*        b) Explore libraries such as Eigen, Autodiff.
*
*    2) Currently the system assumes that, it obtains sensors readings on [x, y, theta, v, theta_dot].
*    This has be changed to [x_double_dot, y_double_dot, v, theta_dot, phi] and the same changes have to be
*    made in getting the Jacobian and in the H matrix.
*
*
*    @section Optimization Issues
*
*    1) Pointer manipulation seems to be an issue in boost::numeric::ublas::matrix
*    Unless the matrix is completely copied into another matrix object,
*    the elements camnnot be accessed
*
*    The trouble can be seen in the following test code.
*    Check out the code in the folder sdc/src/tests/matrix_check.cpp
*
*    2) In function ekf::ekf(matrix <double>& x, matrix <double> p,
*    matrix <double> ih, matrix <double>& q, matrix <double>& r)
*    the variables 'p' and 'ih' are not taken by reference, as for some reason,
*    identity_matrix and zero matrix are not accepted as references.
*/

#include "ekf.hpp"

using namespace std;
using namespace boost::numeric::ublas;
const double L = 4.5;
const double PI = 3.14159;
double lr = 2.25;
double lf = 2.25;
/*
ekf::ekf(matrix <double>& x, matrix <double> p, matrix <double> ih, matrix <double>& q, matrix <double>& r)
{
    X = x;
    P = p;
    h = ih;
    Q = q;
    R = r;
}

ekf::~ekf() {}

void ekf::get_state(matrix<double>& xx) { xx = X;}

matrix<double> ekf::get_F( matrix <double>& control_vector, float del_t)
{
    double x = X(0,0);
    double y = X(1,0);
    double theta = X(2,0);
    double v = X(3,0);
    double acc = control_vector(0,0);
    double phi = control_vector(1,0);

    matrix<double> F(5,5);
    F(0,0) = 0; F(0,1) = 0; F(0,2) = (-v*del_t*sin(theta) - (acc*sin(theta)*del_t*del_t)*0.5); F(0,3) = cos(theta)*del_t; F(0,4) = 0;
    F(1,0) = 0; F(1,1) = 0; F(1,2) = (v*del_t*cos(theta) + (acc*cos(theta)*del_t*del_t)*0.5); F(1,3) = sin(theta)*del_t; F(1,4) = 0;
    F(2,0) = 0; F(2,1) = 0; F(2,2) = (tan(phi)*del_t)/L; F(2,3) = 0; F(2,4) = 0;
    F(3,0) = 0; F(3,1) = 0; F(3,2) = 0; F(3,3) = 0; F(3,4) = 0;
    F(4,0) = 0; F(4,1) = 0; F(4,2) = 0; F(4,3) = (tan(phi)*del_t)/L; F(4,4) = 0;

    return F;
}

matrix <double> ekf::predicted_X( matrix <double> &control_vector, float del_t)
{
    double x = X(0,0);
    double y = X(1,0);
    double theta = X(2,0);
    double vel = X(3,0);
    double theta_dot = X(4,0);
    double acc = control_vector(0,0);
    double steering = control_vector(1,0);

    x += vel*cos(theta)*del_t + 0.5*acc*cos(theta)*del_t*del_t;
    y += vel*sin(theta)*del_t + 0.5*acc*sin(theta)*del_t*del_t;
    theta += fmod((vel/L)*tan(steering)*del_t, 2*PI);
    theta_dot += (vel/L)*tan(steering);
    vel += acc*del_t;

    matrix <double> new_X(5,1);
    new_X(0,0) = x;
    new_X(1,0) = y;
    new_X(2,0) = fmod(theta, 2*PI);
    new_X(3,0) = vel;
    new_X(4,0) = theta_dot;
    return new_X;
}

void ekf::step( matrix<double>& control_vector, matrix<double>& measurement_vector, float del_t)
{
    matrix<double> predicted_x = predicted_X(control_vector, del_t);
    matrix<double> F = get_F(control_vector, del_t);
    matrix<double> predicted_P(5,5);
    predicted_P = matrix<double> (prod(matrix<double> (prod(F, P)),trans(F))) + Q;

    matrix<double> diff = measurement_vector - predicted_x;
    identity_matrix<double> H (5);
    matrix<double> S = matrix<double> (prod(matrix<double> (prod(H,predicted_P)),trans(H))) + R;

    matrix<double> inverted_S(5,5);
    bool inversion = InvertMatrix(S, inverted_S);
    matrix<double> K = matrix<double> (prod(matrix<double> (prod(predicted_P, trans(H))),inverted_S));
    X = (predicted_x + matrix<double> (prod(K, diff)));
    identity_matrix<double> place_holder (5);

    matrix<double> G = place_holder - matrix<double> (prod(K, H));
    P = matrix<double> (prod(G,predicted_P));
}
*/
ekf::ekf(matrix <double>& x, matrix <double> p, matrix <double> ih, matrix <double>& q, matrix <double>& r)
{
    X = x;
    P = p;
    h = ih;
    Q = q;
    R = r;
}

ekf::~ekf() {}

void ekf::get_state(matrix<double>& xx) {xx = X;}

matrix<double> ekf::get_F( matrix <double>& control_vector, float del_t)
{
    double x = X(0,0);
    double y = X(1,0);
    double theta = X(2,0);
    double v = X(3,0);
    double beta = X(4,0);
    double acc = control_vector(0,0);
    double phi = control_vector(1,0);

    matrix<double> F(5,5);
    F(0,0) = 0; F(0,1) = 0; F(0,2) = (-v*del_t*sin(theta+beta) - (acc*sin(theta+beta)*del_t*del_t)*0.5); F(0,3) = cos(theta+beta)*del_t; F(0,4) = -(v*del_t*sin(theta+beta)+(acc*del_t*del_t*sin(theta+beta)));
    F(1,0) = 0; F(1,1) = 0; F(1,2) = (v*del_t*cos(theta+beta) + (acc*cos(theta+beta)*del_t*del_t)*0.5); F(1,3) = sin(theta+beta)*del_t; F(1,4) = v*del_t*cos(theta+beta)+(acc*del_t*del_t*cos(theta+beta));
    F(2,0) = 0; F(2,1) = 0; F(2,2) = 0; F(2,3) = (sin(beta)*del_t)/L; F(2,4) = (cos(beta)*del_t*v)/L;
    F(3,0) = 0; F(3,1) = 0; F(3,2) = 0; F(3,3) = 0; F(3,4) = 0;
    F(4,0) = 0; F(4,1) = 0; F(4,2) = 0; F(4,3) = 0; F(4,4) = 0;

    return F;
}

matrix<double> ekf::get_H(float del_t)
{
    double x = X(0,0);
    double y = X(1,0);
    double theta = X(2,0);
    double v = X(3,0);
    double beta = X(4,0);

    matrix<double> H(5,5);
    H(0,0) = 2/std::pow(del_t, 2); H(0,1) = 0; H(0,2) = -(2*v*cos(theta+beta))/(del_t*(theta+beta)); H(0,3) = 0; H(0,4) = (2*v*cos(theta+beta))/(theta+beta);
    H(1,0) = 0; H(1,1) = 2/std::pow(del_t, 2); H(1,2) = -(2*v*sin(theta+beta))/(del_t*(theta+beta)); H(1,3) = 0; H(1,4) = (2*v*sin(theta+beta))/(theta+beta);
    H(2,0) = 0; H(2,1) = 0; H(2,2) = 0; H(2,3) = 0; H(2,4) = 1;
    H(3,0) = 0; H(3,1) = 0; H(3,2) = 0; H(3,3) = 1/del_t; H(3,4) = 0;
    H(4,0) = 0; H(4,1) = 0; H(4,2) = 1/(theta+beta); H(4,3) = 0; H(4,4) = -del_t/(theta+beta);

    return H;
}

matrix <double> ekf::predicted_X( matrix <double> &control_vector, float del_t)
{
    double x = X(0,0);
    double y = X(1,0);
    double theta = X(2,0);
    double vel = X(3,0);
    double beta = X(4,0);
    double acc = control_vector(0,0);
    double steering = control_vector(1,0);

    beta = atan((lr/(lr+lf))*tan(steering));
    x += vel*cos(theta + beta)*del_t + 0.5*acc*cos(theta + beta)*del_t*del_t;
    y += vel*sin(theta + beta)*del_t + 0.5*acc*sin(theta + beta)*del_t*del_t;
    theta += fmod((vel/L)*sin(beta)*del_t, 2*PI);
    vel += acc*del_t;

    matrix <double> new_X(5,1);
    new_X(0,0) = x;
    new_X(1,0) = y;
    new_X(2,0) = fmod(theta, 2*PI);
    new_X(3,0) = vel;
    new_X(4,0) = beta;
    return new_X;
}

matrix <double> ekf::get_sensor_projected_state(matrix <double> &measurement_vector, float del_t)
{

    double theta = X(2,0);
    double v = X(3,0);
    double beta = X(4,0);

    matrix<double> projected_state(5, 0);
    matrix<double> hi (5, 5);
    hi(0,0) = 0.5*std::pow(del_t, 2); hi(0,1) = 0; hi(0,2) = 0; hi(0,3) = 0; hi(0,4) = v*cos(theta+beta)*del_t;
    hi(1,0) = 0; hi(1,1) = 0.5*std::pow(del_t, 2); hi(1,2) = 0; hi(0,4) = 0; hi(0,5) = v*sin(theta+beta)*del_t;
    hi(2,0) = 0; hi(2,1) = 0; hi(2,2) = del_t; hi(2,3) = 0; hi(2,4) = theta+beta;
    hi(3,0) = 0; hi(3,1) = 0; hi(3,2) = 0; hi(3,3) = del_t; hi(3,4) = 0;
    hi(4,0) = 0; hi(4,1) = 0; hi(4,2) = 1; hi(4,3) = 0; hi(4,4) = 0;

    projected_state = matrix<double> (prod(hi, measurement_vector));
    return projected_state;
}


void ekf::step( matrix<double>& control_vector, matrix<double>& measurement_vector, float del_t)
{
    matrix<double> predicted_x = predicted_X(control_vector, del_t);
    matrix<double> F = get_F(control_vector, del_t);
    matrix<double> predicted_P(5,5);
    predicted_P = matrix<double> (prod(matrix<double> (prod(F, P)),trans(F))) + Q;

    matrix<double> projected_state_vector = get_sensor_projected_state(measurement_vector, del_t);
    matrix<double> diff = projected_state_vector - predicted_x;
//    identity_matrix<double> H (5);
    matrix <double> H = get_H(del_t);
    matrix<double> S = matrix<double> (prod(matrix<double> (prod(H,predicted_P)),trans(H))) + R;

    matrix<double> inverted_S(5,5);
    bool inversion = InvertMatrix(S, inverted_S);
    matrix<double> K = matrix<double> (prod(matrix<double> (prod(predicted_P, trans(H))),inverted_S));
    X = (predicted_x + matrix<double> (prod(K, diff)));
    identity_matrix<double> place_holder (5);

    matrix<double> G = place_holder - matrix<double> (prod(K, H));
    P = matrix<double> (prod(G,predicted_P));
}
