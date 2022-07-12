#include "iostream"
#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "chrono"

using namespace std;
using namespace Eigen;

int main(int argc, char ** argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;
    int N = 100;
    double w_sigma = 1.0;
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;

    vector<double> x_data, y_data;
    for (int i=0; i<N; i++)
    {
        double x = i / 100.0;
        double y = exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma);
        x_data.push_back(x);
        y_data.push_back(y);
    }

    int iterations = 100;
    double cost = 0.0, lastCost = 0.0;
    
    for (int iter=0; iter<iterations; iter++)
    {
        Matrix3d H = Matrix3d::Zero();
        Vector3d b = Vector3d::Zero();
        cost = 0;

        for(int i=0; i<N; i++)
        {
            double xi = x_data[i], yi = y_data[i];
            double error = yi - exp(ae * xi * xi + be * xi + ce);
            Vector3d J;
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);  // de/da
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);  // de/db
            J[2] = -exp(ae * xi * xi + be * xi + ce);  // de/dc

            H +=  inv_sigma * inv_sigma * J * J.transpose();
            b += - inv_sigma * inv_sigma * error * J;

            cost += error * error;
        }

        Vector3d dx = H.ldlt().solve(b);
        
        if(iter>0 && cost > lastCost)
        {
            cout << "cost > lastCost" << endl;
        }

        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;   
    }
    cout << "estimate abc = " << ae << "," << be << "," << ce << endl;
}