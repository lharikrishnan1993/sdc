#include <lqr.hpp>
#include <math.h>

class control_actions
{
    public:
        float lin_velocity;
        float ang_velocity;
};

control_actions controller( const Eigen::Ref<const Eigen::MatrixXd>& A, \
                            const Eigen::Ref<const Eigen::MatrixXd>& B, \
                            const Eigen::Ref<const Eigen::MatrixXd>& Q, \
                            const Eigen::Ref<const Eigen::MatrixXd>& R, \
                            const std::vector<double> trajectory, \
                            const std::vector<double> state)
{
    Eigen::MatrixXd K = solve_care(A, B, Q, R);

    double x_pos = trajectory[0];
    double y_pos = trajectory[1];
    double x_vel = trajectory[2];
    double y_vel = trajectory[3];
    double x_acc = trajectory[4];
    double y_acc = trajectory[5];

    /**
    double current_x_pos = 0.0;
    double current_y_pos = 0.0;
    double current_x_vel = 0.0;
    double current_y_vel = 0.0;
    Get the current values and update the above variables
    */

    Eigen::MatrixXd linearized_state(4,1);
    linearized_state(0,0) = current_x_pos - x_pos;
    linearized_state(1,0) = current_y_pos - y_pos;
    linearized_state(2,0) = current_x_vel - x_vel;
    linearized_state(3,0) = current_x_vel - x_vel;

    Eigen::MatrixXd linearized_control_actions = -K*linearized_state;
    double v1 = linearized_control_actions(0,0);
    double v2 = linearized_control_actions(1,0);

    control_actions controls;
    controls.lin_velocity = (linearized_state(2,0) + x_vel)*cos(state[2]) + (linearized_state(3,0) + y_vel)*sin(state[2]);
    controls.ang_velocity = ((v2 + y_acc)*cos(state[2]) - (v1 + x_acc)*sin(state[2]))/controls.lin_velocity;
    return controls;
}
