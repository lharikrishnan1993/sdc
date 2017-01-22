#include <iostream>
#include "car_utilities.cpp"

int main()
{
    Car c;
    kinematic_bicycle b;
    Car *simple_model = &c;
    Car *bicycle_model = &b;
    float del_t = 0.1;
    float time = 20;
    float i = 0;
    float *state;
    float acc = 0.3;
    float phi = d2r(30.0);

    while (i < time)
    {
        sensor s(simple_model, 0.5);
        state = simple_model->get_state();
//        phi = sin(d2r(5.0*i));
        for (int j = 0; j < 5; j++) std::cout<<*(state+j)<<" ";
        std::cout<<i<<std::endl;
        simple_model->update(acc, phi, del_t);
        i += del_t;
    }

    i = 0;
    while (i < time)
    {
        sensor sen(bicycle_model, 0.5);
        state = bicycle_model->get_state();
//        phi = sin(d2r(5.0*i));
        for (int j = 0; j < 5; j++) std::cout<<*(state+j)<<" ";
        std::cout<<i<<std::endl;
        bicycle_model->update(acc, phi, del_t);
        i += del_t;
    }
    return 0;
}
