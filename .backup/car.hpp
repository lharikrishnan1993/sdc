#ifndef CAR_H
#define CAR_H

//Simple Car
class Car
{
    private:
        float x;                    // X Position
        float y;                    // Y Position
        float theta;                // Orientation of vehicle (Chi in the paper)
        float vel;                  // Linear Velocity
        float theta_dot;            // Angular Velocity (Chi dot)
        float L;                    // Length of the vehicle
        float state_vector[5];      // State vector [x, y, v, theta, theta_dot]

    public:
        Car();                      // Constructor
        ~Car();                     // Destructor

        // Return true values
        float get_x_pos();
        float get_y_pos();
        float get_theta();
        float get_vel();
        float get_theta_dot();
        float get_L();
/*
        // Return noisy values
        float get_noisy_x_pos(float &noise);
        float get_noisy_y_pos(float &noise);
        float get_noisy_theta(float noise);
        float get_noisy_vel(float &noise);
        float get_noisy_theta_dot(float &noise);
*/
        // Return state vector
        float* get_state();

        // Model of the vehicle, update for timestep
        void step(float &acc, float &steering, float &del_t);
};
/*
//http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf
class kinematic_bicycle : public Car
{
    private:
        float lr;                   // CoM to Rear wheel
        float lf;                   // CoM to Front wheel
        float beta;                 // Angle of current linear velocity
        float acc;                  // Acceleration
        float del_r = 0;            // Steered angle at rear default 0
        float del_f;                // Steered angle in the front

    public:
        kinematic_bicycle();        // Constructor
        ~kinematic_bicycle();       // Destructor

        // Return true values
        float get_beta();
        float get_del_f();
        float get_acc();

        // Return noisy values
//        float get_noisy_beta(float &noise);
//        float get_noisy_del_f(float &noise);
//        float get_noisy_acc(float &noise);

        // Return the state vector
        float* get_state();

        // Model of the vehicle, update for timestep
        void step();
};
*/
#endif
