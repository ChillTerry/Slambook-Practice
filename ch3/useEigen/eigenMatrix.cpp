#include <iostream>
using namespace std;
#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 50


int main( int argc, char** argv )
{
    Eigen::Matrix<float, 2, 3> matrix_23;
    
    Eigen::Matrix<float, 3, 1> matrix_3d;
    Eigen::Vector3d v_3d;

    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    Eigen::MatrixXd matrix_x;

    matrix_23 << 1,2,3,4,5,6;
    cout << matrix_23 << endl;

    for(int i=0; i<2; i++)
    {
        for(int j=0; j<3; j++)
            cout << matrix_23(i,j) << "\t";
        cout<<endl;
    }

    v_3d << 3,2,1;
    matrix_3d << 4,5,6;
    Eigen::Matrix<double,2,1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;

    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl;

    cout << matrix_33.transpose() << endl;
    cout << matrix_33.sum() << endl;
    cout << matrix_33.trace() << endl;
    cout << matrix_33.inverse() << endl;
    cout << matrix_33.determinant() << endl;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose()*matrix_33);
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = \n" << eigen_solver.eigenvalues() << endl;

    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE,1);
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE,1);

    clock_t start_time = clock();
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse()*v_Nd;
    cout <<"time use in normal inverse is " << 1000*(clock()-start_time)/(double)CLOCKS_PER_SEC << "ms" << endl;

    start_time = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout <<"time use in Qr decomposition is " <<1000*  (clock() - start_time)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    return 0;
}
