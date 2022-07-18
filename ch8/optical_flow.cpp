#include "iostream"
#include "string"
#include "chrono"
#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"

using namespace std;
using namespace cv;

string file_1 = "../LK1.png";
string file_2 = "../LK2.png";

class OpticalFlowTracker 
{
public:
    OpticalFlowTracker(
        const Mat &img1_,
        const Mat &img2_,
        const vector<KeyPoint> &kp1_,
        vector<KeyPoint> &kp2_,
        vector<bool> &success_,
        bool inverse_ = true, bool has_initial_ = false) :
        img1(img1_), img2(img2_), kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_),
        has_initial(has_initial_) {}

    void calculateOpticalFlow(const Range &range);

private:
    const Mat &img1;
    const Mat &img2;
    const vector<KeyPoint> &kp1;
    vector<KeyPoint> &kp2;
    vector<bool> &success;
    bool inverse = true;
    bool has_initial = false;    
};


int main(int argc, char **argv)
{
    chrono::steady_clock::time_point t1;
    chrono::steady_clock::time_point t2;
    chrono::duration<double> time_used;

    Mat img1 = imread(file_1, 0);
    Mat img2 = imread(file_2, 0);
    assert(img1.data != nullptr && img2.data != nullptr);

    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20);
    detector->detect(img1, kp1);

    vector<Point2f> pt1, pt2;
    vector<uchar> status;
    vector<float> error;
    for (auto &kp : kp1) pt1.push_back(kp.pt);
    t1 = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by opencv: " << time_used.count() << endl;

    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++)
    {
        if (status[i]) 
        {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }

    cv::imshow("tracked by opencv", img2_CV);
    cv::waitKey(0);

}


