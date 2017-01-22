/**
*    @author Harikrishnan Lakshmanan
*    @file car.hpp
*    @date 11/21/2016
*    @ref http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf
*
*    @brief WAVe Lab, A car simulator. Models a car for running/testing various packages such as state estimation, motion planning and so on.
*
*    @section Updates to make
*    Check the updates to be made in car_utilities.cpp
*    step(), might be useful. Look into it.
*
*    @section Optimization Issues
*    Check the updates to be made in car_utilities.cpp
*/

#ifndef CAR_H
#define CAR_H

//Simple Car
class Car
{
    protected:
        float L = 4.5;                                  // Length of vehicle (4.5m)
        float x;                                        // X Position
        float y;                                        // Y Position
        float theta;                                    // Orientation of vehicle (Chi in the paper)
        float vel;                                        // Linear Velocity
        float theta_dot;                                // Angular Velocity (Chi dot)

        float state_vector[5];              // State vector [x, y, v, theta, theta_dot]
        float noisy_state_vector[5];        // State vector [x, y, v, theta, theta_dot]

    public:
        Car(float, float, float, float, float);         // Constructor
        ~Car();                                         // Destructor

        // Return true values
        float get_L();
        float get_x_pos();
        float get_y_pos();
        float get_theta();
        float get_vel();
        float get_theta_dot();

        // Return noisy values
        float get_noisy_x_pos(float &noise);
        float get_noisy_y_pos(float &noise);
        float get_noisy_theta(float &noise);
        float get_noisy_vel(float &noise);
        float get_noisy_theta_dot(float &noise);

        // Return state vector
        float* get_state();
        float* get_noisy_state(float &noise);

        // Model of the vehicle, update for timestep
        void update(float &acc, float &steering, float &del_t);
        void step(float &acc, float &steering, float &del_t);
};

//http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf
class kinematic_bicycle : public Car
{
    private:
        float beta;                                         // Angle of current linear velocity

    protected:
        float lr = L/2;                                     // Distance from CoM to Rear wheel
        float lf = L/2;                                     // Distance from CoM to Front wheel

    public:
        kinematic_bicycle(float, float, float);             // Constructor
        ~kinematic_bicycle();                               // Destructor

        // Return true values
        float get_beta();
        float get_lr();
        float get_lf();

        float get_noisy_beta(float &noise);

        float* get_state();
        float* get_noisy_state(float &noise);

        // Model of the vehicle, update for timestep
        void update(float &acc, float &steering, float &del_t);
};
#endif
