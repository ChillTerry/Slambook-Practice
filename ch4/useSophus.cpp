#include "iostream"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char ** argv)
{
    Matrix3d R = AngleAxisd(M_PI/2, Vector3d(0,0,1)).toRotationMatrix();
    Quaterniond q(R);
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);
    cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
    cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;

    Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    cout << "so3 hat = " << Sophus::SO3d::hat(so3) << endl;
    cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3).transpose()) << endl;

    Vector3d update_so3(1e-4, 0 ,0);
    Sophus::SO3d SO3_update = Sophus::SO3d::exp(update_so3)*SO3_R;
    cout << "SO3 update = \n" << SO3_update.matrix() << endl;

    Vector3d t(0,0,1);
    Sophus::SE3d SE3_Rt(R,t);
    Sophus::SE3d SE3_qt(q,t);
    cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
    cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    cout << "se3 hat = " << Sophus::SE3d::hat(se3) << endl;
    cout << "se3 vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0,0) = 1e-4;
    Sophus::SE3d SE3_update = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 update = " << endl << SE3_update.matrix() << endl;
}