#include "iostream"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "chrono"

using namespace std;
using namespace cv;

int main(int argc, char ** argv)
{
    Mat img1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img1.data != nullptr && img2.data != nullptr);

    vector<KeyPoint> KeyPoints1, KeyPoints2;
    Mat descriptors1, descriptors2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img1, KeyPoints1);
    detector->detect(img2, KeyPoints2);
    
    descriptor->compute(img1, KeyPoints1, descriptors1);
    descriptor->compute(img2, KeyPoints2, descriptors2);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);
    cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

    Mat outimg1;
    drawKeypoints(img1, KeyPoints1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB features", outimg1);

    vector<DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors1, descriptors2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

    auto min_max = minmax_element(matches.begin(), matches.end(),
                                  [](const DMatch &m1, const DMatch &m2) {return m1.distance < m2.distance;});
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    vector<DMatch> goodMatches;
    for (int i=0; i<descriptors1.rows; i++)
    {
        if(matches[i].distance <= max(2*min_dist, 30.0))
            goodMatches.push_back(matches[i]);        
    }

    Mat img_matches;
    Mat img_goodMatches;
    drawMatches(img1, KeyPoints1, img2, KeyPoints2, matches, img_matches);
    drawMatches(img1, KeyPoints1, img2, KeyPoints2, goodMatches, img_goodMatches);
    imshow("all matches", img_matches);
    imshow("good matches", img_goodMatches);
    waitKey(0);
}