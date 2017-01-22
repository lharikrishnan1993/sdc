#ifndef INVERT_MATRIX_HPP
#define INVERT_MATRIX_HPP

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

namespace ublas = boost::numeric::ublas;
template<class T>
bool InvertMatrix (const ublas::matrix<T>& input, ublas::matrix<T>& inverse) {
    using namespace boost::numeric::ublas;
    typedef permutation_matrix<std::size_t> pmatrix;
    // create a working copy of the input
    matrix<T> A(input);
    // create a permutation matrix for the LU-factorization
    pmatrix pm(A.size1());
    // perform LU-factorization
    int res = lu_factorize(A,pm);
    if( res != 0 )
        return false;
    // create identity matrix of "inverse"
    inverse.assign(ublas::identity_matrix<T>(A.size1()));
    // backsubstitute to get the inverse
    lu_substitute(A, pm, inverse);
    return true;
}

#endif //INVERT_MATRIX_HPP
