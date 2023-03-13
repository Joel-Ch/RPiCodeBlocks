#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <map>

using namespace cv;
using namespace std;

int euclidDistance(Scalar A,int* B){
    return sqrt((A[0]-B[0])*(A[0]-B[0]) +(A[1]-B[1])*(A[1]-B[1]) +(A[2]-B[2])*(A[2]-B[2]));
}

Mat doKMeans(Mat3b img)
{
    int K = 4;
    int n = img.rows * img.cols;
    Mat data = img.reshape(1, n);
    data.convertTo(data, CV_32F);

    vector<int> labels;
    Mat1f centres;
    kmeans(data, K, labels, cv::TermCriteria(), 1, cv::KMEANS_PP_CENTERS, centres);

    for (int i = 0; i < n; ++i)
    {
        data.at<float>(i, 0) = centres(labels[i], 0);
        data.at<float>(i, 1) = centres(labels[i], 1);
        data.at<float>(i, 2) = centres(labels[i], 2);
    }

    Mat reduced = data.reshape(3, img.rows);
    reduced.convertTo(reduced, CV_8U);

    //find the colour most different from the others

    return reduced;
}


int main()
{
    Mat3b img = imread("C:/OpenCV/OpenCV Task/Images/RedCar.bmp");
    imshow("Image",img);
    img = doKMeans(img);
    imshow("Kmeans",img);
    waitKey(0);
    return 0;
}
