#include <iostream>
#include "car.hpp"
#include <math.h>

Car::Car()
{
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    vel = 0.0;
    theta_dot = 0.0;
    L = 4.5;    //meters

    *state_vector = x;
    *(state_vector+1) = y;
    *(state_vector+2) = theta;
    *(state_vector+3) = vel;
    *(state_vector+4) = theta_dot;
}

Car::~Car() {}

float Car::get_x_pos() {return x;}
float Car::get_y_pos() {return y;}
float Car::get_theta() {return theta;}
float Car::get_vel() {return vel;}
float Car::get_theta_dot() {return theta_dot;}
float Car::get_L() {return L;}

float* Car::get_state()
{
    *state_vector = x;
    *(state_vector+1) = y;
    *(state_vector+2) = theta;
    *(state_vector+3) = vel;
    *(state_vector+4) = theta_dot;
    return state_vector;
}

void Car::step(float &acc, float &steering, float &del_t)
{
    x += vel*cos(theta)*del_t + 0.5*acc*cos(theta)*del_t*del_t;
    y += vel*sin(theta)*del_t + 0.5*acc*sin(theta)*del_t*del_t;
    theta += (vel/L)*tan(steering)*del_t;
    theta_dot += (vel/L)*tan(steering);
    vel += acc*del_t;
}

int main()
{
    Car c;
    for (int i = 0; i < 5; i++) std::cout<<*(c.get_state()+i)<<std::endl;
    float acc = 1, steering = 10, time = 1;
    c.step(acc, steering, time);
    std::cout<<std::endl;
    for (int i = 0; i < 5; i++) std::cout<<*(c.get_state()+i)<<std::endl;
}
