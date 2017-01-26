/**
*    @author Harikrishnan Lakshmanan
*    @file ekf.hpp
*    @date 11/21/2016
*
*    @brief WAVe Lab, Header File for State Estimator using EKF.
*
*    @section Updates to make
*    Check the updates to be made in ekf_utilities.cpp
*
*    @section Optimization Issues
*    Check the updates to be made in ekf_utilities.cpp
*/

#ifndef EKF_HPP
#define EKF_HPP

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <cmath>
#include "invert_matrix.hpp"

using namespace std;
using namespace boost::numeric::ublas;

class ekf
{
    private:
        matrix<double> X;
        matrix<double> P;
        matrix<double> h;
        matrix<double> Q;
        matrix<double> R;

    public:
        ekf(matrix <double> &x, matrix <double> p, matrix <double> ih, matrix <double>& q, matrix <double>& r);
        ~ekf();
        void get_state(matrix<double>& xx);
        matrix<double> get_F( matrix <double>& control_vector, float del_t);
//        matrix<double> get_H( float del_t);
//        matrix<double> get_sensor_projected_state(matrix <double>& measurement_vector, float del_t);
        matrix<double> predicted_X( matrix <double>& control_vector, float del_t);
        void step( matrix<double>& control_vector, matrix<double>& measurement_vector, float del_t);
};

#endif
