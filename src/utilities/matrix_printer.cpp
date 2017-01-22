#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;

double get_element(const matrix<double> &m,int i,int j) {return m(i,j);}
