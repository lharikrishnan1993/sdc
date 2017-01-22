#include <iostream>
#include "car_utilities.cpp"

int main()
{
    Car c;
    kinematic_bicycle b;
    float del_t = 0.1;
    float time = 10;
    float i = 0;
    float *state;
    float acc = 0.1;
    float phi = d2r(10.0);

    while (i < time)
    {
        state = b.kinematic_bicycle::get_state();
        phi = sin(d2r(50.0*i));
        for (int j = 0; j < 5; j++) std::cout<<*(state+j)<<" ";
        std::cout<<i<<std::endl;
        b.update(acc, phi, del_t);
        i += del_t;
    }

    i = 0;
    while (i < time)
    {
        state = c.Car::get_state();
        phi = sin(d2r(50.0*i));
        for (int j = 0; j < 5; j++) std::cout<<*(state+j)<<" ";
        std::cout<<i<<std::endl;
        c.update(acc, phi, del_t);
        i += del_t;
    }
    return 0;
}
