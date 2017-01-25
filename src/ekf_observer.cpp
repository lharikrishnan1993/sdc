/**
*    @author Harikrishnan Lakshmanan
*    @file ekf_observer.cpp
*    @date 11/24/2016
*
*    @brief WAVe Lab, Main code which fetches the system dynamics and uses EKF state estimator to
*           estimate the state of the system.
*
*    @section Updates to make
*    Looks like, it's pretty much alright.
*
*    @section Optimization Issues
*    I don't see any potential drawback
*/

#include "ekf_utilities.cpp"
#include "car_utilities.cpp"
//std::default_random_engine generator;

double get_noisy_acc(double acc, double noise)
{
    std::normal_distribution<double> distribution(acc, noise);
    return distribution(generator);
}

double get_noisy_phi(double phi, double noise)
{
    std::normal_distribution<double> distribution(phi, noise);
    return distribution(generator);
}

matrix<double> get_measurement_vector(double acc, double phi, double noisy_orientation, double noise, double del_t)
{
    double noisy_acc = get_noisy_acc(acc, noise);
    double noisy_phi = get_noisy_phi(phi, noise);
    matrix <double> sensor_values(5,1);
    sensor_values(0,0) = noisy_acc*cos(noisy_orientation);
    sensor_values(1,0) = noisy_acc*sin(noisy_orientation);
    sensor_values(2,0) = noisy_phi;
    sensor_values(3,0) = noisy_acc;
    sensor_values(4,0) = 1;
    return sensor_values;
}

int main()
{
/*    Car s;

    matrix<double> X(5,1);
    X(0,0) = 0;
    X(1,0) = 10;
    X(2,0) = 0;
    X(3,0) = 0;
    X(4,0) = 0;

    zero_matrix<double> P (5, 5);
    zero_matrix<double> H (5, 5);

    matrix<double> Q(5,5);
    for (unsigned i = 0; i < Q.size1 (); ++ i)
        for (unsigned j = 0; j < Q.size2 (); ++ j)
            if (i==j) Q (i, j) = 0.0000004;
            else Q(i,j) = 0;

    matrix<double> R(5,5);
    for (unsigned i = 0; i < R.size1 (); ++ i)
        for (unsigned j = 0; j < R.size2 (); ++ j)
            if (i==j) R (i, j) = 0.0000075;
            else R(i,j) = 0;

    float time = 20.0;
    float del_t = 0.01;
    float acc = 0.3;
    float phi = d2r(10.0);
    matrix<double> control_vector(2,1);
    control_vector(0,0) = acc;
    control_vector(1,0) = phi;

    matrix<double> measurement_vector(5,1);
    matrix<double> true_vector(5,1);
    ekf filter(X, P, H, Q, R);

    float i = 0;
    float noise = 0.5;
    while(i < time)
    {
        matrix<double> xx(5,1);
        filter.get_state(xx);
        cout<<"EKF State(K): "<<xx<<endl;
        cout<<endl;
        phi = sin(d2r(50.0*i));
        for (int j =0; j<5; j++) true_vector(j, 0) = (*(s.get_state()+j));
        for (int j =0; j<5; j++) measurement_vector(j, 0) = (*(s.get_noisy_state(noise)+j));
        cout<<"Measurement(K): "<<measurement_vector<<endl;
        cout<<"True_State(K): "<<true_vector<<endl;
        s.update(acc, phi, del_t);
        cout<<control_vector(1,0)<<endl;
        cout<<control_vector<<endl;
        filter.step(control_vector, measurement_vector, del_t);
        i += del_t;
    }
*/

    kinematic_bicycle c;

    float time = 20.0;
    float del_t = 0.01;
    float acc = 0.3;
    float phi = d2r(10.0);

    matrix<double> Xi(5,1);
    Xi(0,0) = 0;
    Xi(1,0) = 10;
    Xi(2,0) = 0;
    Xi(3,0) = 0;
    Xi(4,0) = 0;

    zero_matrix<double> Pi (5, 5);
    zero_matrix<double> hi (5, 5);

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

    matrix<double> control_vectori(2,1);
    control_vectori(0,0) = acc;
    control_vectori(1,0) = phi;

    matrix<double> measurement_vectori(5,1);
    matrix<double> true_vectori(5,1);
    ekf filteri(Xi, Pi, hi, Qi, Ri);

    float i = 0;
    float noise = 0.5;
    while(i < time)
    {
        matrix<double> xxi(5,1);
        filteri.get_state(xxi);
        cout<<"EKF State(S): "<<xxi<<endl;
        cout<<endl;
        phi = sin(d2r(50.0*i));
        for (int j =0; j<5; j++) true_vectori(j, 0) = (*(c.get_state()+j));
        float orientation = c.get_noisy_theta(noise);
        measurement_vectori = get_measurement_vector(acc, phi, orientation, noise, del_t);
//        for (int j =0; j<5; j++) measurement_vectori(j, 0) = (*(c.get_noisy_state(noise)+j));
        cout<<"Measurement(S): "<<measurement_vectori<<endl;
        cout<<"True_State(S): "<<true_vectori<<endl;
        c.update(acc, phi, del_t);
        cout<<control_vectori(1,0)<<endl;
        cout<<control_vectori<<endl;
        filteri.step(control_vectori, measurement_vectori, del_t);
        i += del_t;
    }
}
