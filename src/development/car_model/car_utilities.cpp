/**
*    @author Harikrishnan Lakshmanan
*    @file car_utilities.cpp
*    @date 11/29/2016
*    @ref http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf
*
*    @brief WAVe Lab, A car simulator. Models a car for running/testing various packages such as state estimation, motion planning and so on.
*
*    @section Updates to make
*    1) Change the software architecture to Hierarchical inheritance, redefinition of many over lapping class methods is required.
*       There is no other better design for the scenario. The current implementation is just a hack and is not useful for the long run.
*
*    @section Optimization Issues
*    1) Looks pretty decent. I don't see any issues
*/

#include "car.hpp"
#include <iostream>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;
std::default_random_engine generator;
using namespace std;
//extern const double PI;
const double PI = 3.14159;



float d2r(float deg)
{
    float f = deg*0.0174533;
    return fmod(f, 2*PI);
}

/**
Dubins car model.
*/

Car::Car(float a = 0.0, float b = 0.0, float c = 0.0, float d = 0.0, float e = 0.0)
{
    x = a;
    y = b;
    theta = c;
    vel = d;
    theta_dot = e;

    *state_vector = x;
    *(state_vector+1) = y;
    *(state_vector+2) = theta;
    *(state_vector+3) = vel;
    *(state_vector+4) = theta_dot;
}

Car::~Car() {}

float Car::get_x_pos() {return x;}
float Car::get_y_pos() {return y;}
float Car::get_theta() {return fmod(theta, PI);}
float Car::get_vel() {return vel;}
float Car::get_theta_dot() {return theta_dot;}
float Car::get_L() {return L;}

float Car::get_noisy_x_pos(float &noise)
{
    std::normal_distribution<double> distribution(get_x_pos(), noise);
    return distribution(generator);
}
float Car::get_noisy_y_pos(float &noise)
{
    std::normal_distribution<double> distribution(get_y_pos(), noise);
    return distribution(generator);
}
float Car::get_noisy_theta(float &noise)
{
    std::normal_distribution<double> distribution(get_theta(),noise);
    return distribution(generator);
}
float Car::get_noisy_vel(float &noise)
{
    std::normal_distribution<double> distribution(get_vel(),noise);
    return distribution(generator);
}
float Car::get_noisy_theta_dot(float &noise)
{
    std::normal_distribution<double> distribution(get_theta_dot(),noise);
    return distribution(generator);
}

float* Car::get_state()
{
    *state_vector = x;
    *(state_vector+1) = y;
    *(state_vector+2) = theta;
    *(state_vector+3) = vel;
    *(state_vector+4) = theta_dot;
    return state_vector;
}

float* Car::get_noisy_state(float &noise)
{
    *noisy_state_vector = get_noisy_x_pos(noise);
    *(noisy_state_vector+1) = get_noisy_y_pos(noise);
    *(noisy_state_vector+2) = get_noisy_theta(noise);
    *(noisy_state_vector+3) = get_noisy_vel(noise);
    *(noisy_state_vector+4) = get_noisy_theta_dot(noise);
    return noisy_state_vector;
}

void Car::update(float &acc, float &steering, float &del_t)
{
    x += vel*cos(theta)*del_t + 0.5*acc*cos(theta)*del_t*del_t;
    y += vel*sin(theta)*del_t + 0.5*acc*sin(theta)*del_t*del_t;
    theta += fmod((vel/L)*tan(steering)*del_t, 2*PI);
    theta_dot += (vel/L)*tan(steering);
    vel += acc*del_t;
}

/**
Computer Vision Team -> Motion Projection Request
Not required anywhere else. DO NOT USE for State Estimation
*/
std::vector <float> Car::step(std::vector<float> values, float &vel, float &steering, float &del_t)
{
    float new_x = values[0];
    float new_y = values[1];
    float new_theta = values[2];

    float desired_t = 0.01;
    float i = 0.0;
    while (i <= del_t)
    {
        new_x += vel*cos(new_theta)*desired_t;
        new_y += vel*sin(new_theta)*desired_t;
        new_theta += fmod(((vel/L)*tan(steering)*desired_t), 2*PI);
        std::cout<<new_x<<" "<<new_y<<" "<<new_theta<<std::endl;
        i += desired_t;
    }
    std::vector <float> new_values = {new_x, new_y, (float) fmod(new_theta, 2*PI)};
    return new_values;
}

/**
Kinematic Bicycle model
*/

kinematic_bicycle::kinematic_bicycle(float g = 0.0, float r = 2.25, float f = 2.25)
{
    beta = g;
    lr = r;
    lf = f;
}

kinematic_bicycle::~kinematic_bicycle() {}

void kinematic_bicycle::update(float &acc, float &steering, float &del_t)
{
    beta = atan((lr/(lr+lf))*tan(steering));
    x += vel*cos(theta + beta)*del_t + 0.5*acc*cos(theta + beta)*del_t*del_t;
    y += vel*sin(theta + beta)*del_t + 0.5*acc*sin(theta + beta)*del_t*del_t;
    theta += fmod((vel/lr)*sin(beta)*del_t, 2*PI);
    vel += acc*del_t;
}

/**
Computer Vision Team -> Motion Projection Request.
Not required anywhere else. DO NOT USE for State Estimation
*/
std::vector <float> kinematic_bicycle::step(std::vector<float> values, float &vel, float &steering, float &del_t)
{
    float new_x = values[0];
    float new_y = values[1];
    float new_theta = values[2];
    float i = 0.0;
    float desired_t = 0.01;
    beta = atan((lr/(lr+lf))*tan(steering));

    while (i < del_t)
    {
        new_x += vel*cos(new_theta + beta)*desired_t;
        new_y += vel*sin(new_theta + beta)*desired_t;
        new_theta += fmod((vel/lr)*sin(beta)*desired_t, 2*PI);
        std::cout<<new_x<<" "<<new_y<<" "<<new_theta<<std::endl;
        i += desired_t;
    }
    std::vector <float> new_values = {new_x, new_y, (float) fmod(new_theta, 2*PI)};
    return new_values;
}

float kinematic_bicycle::get_beta() {return beta;}

float kinematic_bicycle::get_noisy_beta(float &noise)
{
    std::normal_distribution<double> distribution(get_beta(), noise);
    return distribution(generator);
}

float* kinematic_bicycle::get_state()
{
    *state_vector = x;
    *(state_vector+1) = y;
    *(state_vector+2) = theta;
    *(state_vector+3) = vel;
    *(state_vector+4) = beta;
    return state_vector;
}

float* kinematic_bicycle::get_noisy_state(float &noise)
{
    *noisy_state_vector = get_noisy_x_pos(noise);
    *(noisy_state_vector+1) = get_noisy_y_pos(noise);
    *(noisy_state_vector+2) = get_noisy_theta(noise);
    *(noisy_state_vector+3) = get_noisy_vel(noise);
    *(noisy_state_vector+4) = get_noisy_beta(noise);
    return noisy_state_vector;
}
