#define NDEBUG
#include <iostream>
#include <assert.h>
#include <eigen3/Eigen/Cholesky>
#include "lqr.hpp"

Eigen::MatrixXd solve_care( const Eigen::Ref<const Eigen::MatrixXd>& A, \
                            const Eigen::Ref<const Eigen::MatrixXd>& B, \
                            const Eigen::Ref<const Eigen::MatrixXd>& Q, \
                            const Eigen::Ref<const Eigen::MatrixXd>& R)
{
    Eigen::LLT<Eigen::MatrixXd> L(R);
    if (L.info() != Eigen::Success) throw std::runtime_error("R must be positive definite");
    return solve_care(A, B, Q, L);
}

Eigen::MatrixXd solve_care( const Eigen::Ref<const Eigen::MatrixXd>& A, \
                            const Eigen::Ref<const Eigen::MatrixXd>& B, \
                            const Eigen::Ref<const Eigen::MatrixXd>& Q, \
                            const Eigen::LLT<Eigen::MatrixXd>& R_cholesky)
{
    assert(A.rows() == A.cols());
    assert(Q.rows() == Q.cols());
    assert(R_cholesky.matrixL().rows() == R_cholesky.matrixL().cols());

    const int n = B.rows();
    const int m = B.cols();

    Eigen::MatrixXd H(2*n, 2*n);
    H << A, B * R_cholesky.solve(B.transpose()),
         Q, -A.transpose();

    Eigen::MatrixXd Z = H;
    Eigen::MatrixXd Z_old;

    const double tolerance = 1e-9;
    const double max_iterations = 100;

    double relative_norm;
    size_t iteration = 0;

    const double p = static_cast<double>(Z.rows());

    do
    {
        Z_old = Z;
        double ck = std::pow(std::abs(Z.determinant()), -1.0 / p);
        Z *= ck;
        Z = Z - 0.5 * (Z - Z.inverse());
        relative_norm = (Z - Z_old).norm();
        iteration++;
    } while (iteration < max_iterations && relative_norm > tolerance);

    Eigen::MatrixXd W11 = Z.block(0, 0, n, n);
    Eigen::MatrixXd W12 = Z.block(0, n, n, n);
    Eigen::MatrixXd W21 = Z.block(n, 0, n, n);
    Eigen::MatrixXd W22 = Z.block(n, n, n, n);

    Eigen::MatrixXd lhs(2 * n, n);
    Eigen::MatrixXd rhs(2 * n, n);
    Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(n, n);
    lhs << W12, W22 + eye;
    rhs << W11 + eye, W21;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::MatrixXd S = svd.solve(rhs);
    Eigen::MatrixXd K = R_cholesky.solve(B.transpose()*S);

    return K;
}
