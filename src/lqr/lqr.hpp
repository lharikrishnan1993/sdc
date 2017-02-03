#ifndef LQR_H
#define LQR_H

#include <eigen3/Eigen/Dense>

Eigen::MatrixXd solve_care( const Eigen::Ref<const Eigen::MatrixXd>& A, \
                            const Eigen::Ref<const Eigen::MatrixXd>& B, \
                            const Eigen::Ref<const Eigen::MatrixXd>& Q, \
                            const Eigen::Ref<const Eigen::MatrixXd>& R);

Eigen::MatrixXd solve_care( const Eigen::Ref<const Eigen::MatrixXd>& A, \
                            const Eigen::Ref<const Eigen::MatrixXd>& B, \
                            const Eigen::Ref<const Eigen::MatrixXd>& Q, \
                            const Eigen::LLT<Eigen::MatrixXd>& R_cholesky);

#endif
