#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "../../sdc/src/matrix_printer.cpp"

using namespace std;
using namespace boost::numeric::ublas;

int main()
{
    matrix<double> mat(2,2);
    mat(0, 0) = 1; mat(0, 1) = 2;
    mat(1, 0) = 3; mat(1, 1) = 4;
    cout<<"Matrix: "<<mat<<endl;

    matrix<double> *mat_pointer;
//    cout<<get_element(*mat_pointer, 1, 0)<<endl;      //Run time Error -> Results in seg fault
    mat_pointer = &mat;

    cout<<"Matrix Pointer: "<<*mat_pointer<<endl;
//    cout<<"Matrix Pointer: "<<*mat_pointer(0,0)<<endl;    //Compilation Error -> ‘mat_pointer’ cannot be used as a function
}
