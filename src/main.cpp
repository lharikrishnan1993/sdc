#include <iostream>
#include "controller.cpp"
#include <eigen3/Eigen/Dense>

class create_trajectory
{
    private:
        double time;
        double disc;
        double time_stamp;
        bool flag = false;

        std::vector<double> x_pos;
        std::vector<double> y_pos;
        std::vector<double> x_vel;
        std::vector<double> y_vel;
        std::vector<double> x_acc;
        std::vector<double> y_acc;

    public:
        create_trajectory(double t, double discrete):time(t), disc(discrete)
        {
            time_stamp = disc;
        }

        void straight_line_traj()
        {
            flag = true;
            while (time_stamp < time)
            {
                x_pos.push_back(2*disc);
                y_pos.push_back(0);
                x_vel.push_back(2);
                y_vel.push_back(0);
                x_acc.push_back(0);
                y_acc.push_back(0);
                time_stamp += disc;
            }
        }

        void clear_data()
        {
            x_pos.clear();
            y_pos.clear();
            x_vel.clear();
            y_vel.clear();
            x_acc.clear();
            y_acc.clear();
        }

        std::vector<std::vector<double>> get_trajectory()
        {
            if (flag != false)
            {
                std::vector<std::vector<double>> data;
                data = {x_pos, y_pos, x_vel, y_vel, x_acc, y_acc};
                return data;
            }
            else
            {
                throw std::invalid_argument( "Error->Trajectory was not created" );
            }
        }
};

int main()
{
    Eigen::Matrix4d A;
    A << 0,0,1,0,
         0,0,0,1,
         0,0,0,0,
         0,0,0,0;

    Eigen::MatrixXd B(4,2);
    B << 0,0,
         0,0,
         1,0,
         0,1;

    Eigen::Matrix4d Q;
    Q << 1,0,0,0,
         0,1,0,0,
         0,0,1,0,
         0,0,0,1;

    Eigen::Matrix2d R;
    R << 1,0,
         0,1;

    double time = 10.0;
    double discrete = 0.01;
    double current_time = 0.01;

    create_trajectory trajectory(time, discrete);
    trajectory.straight_line_traj();
    std::vector<std::vector<double>> traj = trajectory.get_trajectory();

    int counter = 0;
    std::vector<double> desired_state;
    std::vector<double> current_state;

    while (current_time < time)
    {
        desired_state.clear();
        desired_state = {0.0,0.0,0.0,0.0,0.0,0.0};
        /**
            Get the current state of the system. In the future, get the sensor readings, call EKF and use the estimate as the current state.
//          current_state = {0,0,0};
        */
        current_state.clear();
        control_actions data = controller(A, B, Q, R, desired_state, current_state);
        std::cout<<data.lin_velocity<<std::endl;
        std::cout<<data.ang_velocity<<std::endl;
        /**
            Apply control action to system
        */
        current_time += discrete;
        counter++;
    }

    return 0;
}
