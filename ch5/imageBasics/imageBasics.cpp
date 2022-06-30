#include "iostream"
#include "chrono"

using namespace std;

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

int main(int argc, char ** argv)
{
    cv::Mat image;
    image = cv::imread(argv[1]);

    if(image.data == nullptr)
    {
        cerr << "file " << argv[1] << "is not exsit." << endl;
        return 0;
    }

    cout << "width: " << image.cols << ", hight: " << image.rows \
    << ", channel: " << image.channels() << endl;
    cv::imshow("image", image);
    cv::waitKey(0);

    if(image.type() != CV_8UC1 && image.type() != CV_8UC3)
    {
        cerr << "please input a rgb or gray picture." << endl;
        return 0;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y=0; y<image.rows; y++)
    {
        unsigned char *row_ptr = image.ptr<unsigned char> (y);
        for(size_t x=0; x<image.cols; x++)
        {
            unsigned char *data_ptr = &row_ptr[x*image.channels()];
            for(int c=0; c<image.channels(); c++)
            {
                unsigned char data = data_ptr[c];
            }
        }
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast <chrono::duration<double>> (t2-t1);
    cout << "遍历图像用时：" << time_used.count() << " 秒。" << endl;

    cv::Mat image_another = image;
    image_another(cv::Rect(0,0,100,100)).setTo(0);
    cv::imshow("image", image);
    cv::imshow("image_another", image_another);
    cv::waitKey(0);

    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0,0,100,100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("iamge_clone", image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;

}