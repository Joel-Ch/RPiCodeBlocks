#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <vector>
#include <algorithm>
#include <iostream>

//#define TEST

using namespace cv;
using namespace cv::ml;
using namespace std;



Mat getMask(Mat src){

    // Convert the input image to grayscale
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

    // Run the Canny edge detector
    cv::Mat edges;
    cv::Canny(blurred, edges, 100, 200, 3);
    cv::imshow("H", edges);

    // Find contours in the edge map
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find the largest contour
    int largestContourIndex = -1;
    double largestContourArea = 0;
    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > largestContourArea) {
            largestContourArea = area;
            largestContourIndex = i;
        }
    }

    // Extract the largest contour
    cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);
    cv::drawContours(mask, contours, largestContourIndex, cv::Scalar(255), -1);

    cv::imshow("D",mask);

    // Create a black background image
    cv::Mat bg = cv::Mat::zeros(src.size(), src.type());

    // Copy the masked image to the black background
    src.copyTo(bg, mask);

    // Show the resulting image
    cv::imshow("Result", bg);

    return bg;
}

Vec3b getMostCommonColour(Mat &img, Mat &mask)
{
    // Resize the mask to the same size as the image
    cv::resize(mask, mask, img.size(), 0, 0, INTER_NEAREST);

    // Convert the mask to a binary image, where all pixels with a value of 0 are set to 0 and all other pixels are set to 255
    threshold(mask, mask, 0, 255, THRESH_BINARY);

    // Apply the mask to the image
    Mat masked_img;
    bitwise_and(img, img, masked_img, mask);

    // Convert the masked image to HSV color space
    Mat hsv_masked_img;
    cvtColor(masked_img, hsv_masked_img, COLOR_BGR2HSV);

    // Reshape the masked image into a single column matrix
    Mat samples;
    hsv_masked_img.reshape(3, hsv_masked_img.rows * hsv_masked_img.cols).convertTo(samples, CV_32F);

    // Perform k-means clustering on the reshaped masked image
    int clusterCount = 7;
    Mat labels, centers;
    kmeans(samples, clusterCount, labels, TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);


    // Display the color palette
    for (int i = 0; i < clusterCount; i++)
    {
        Vec3f color = centers.at<Vec3f>(i);
        cout << "Color " << i << ": " << color << endl;
    }

    int frequency[clusterCount] = {0};

    for (int y = 0; y < masked_img.rows; y++) {
        for (int x = 0; x < masked_img.cols; x++) {
            if (masked_img.at<Vec3b>(y, x) != Vec3b(0, 0, 0)) {
                int clusterIndex = labels.at<int>(y + x * masked_img.rows, 0);
                frequency[clusterIndex]++;
            }
        }
    }

    int mostCommonIndex = 0;
    for (int i = 1; i < clusterCount; i++) {
        if (frequency[i] > frequency[mostCommonIndex]) {
            mostCommonIndex = i;
        }
    }

    Vec3b mostCommonColour = centers.at<Vec3f>(mostCommonIndex);



    cout << "Most common color: " << mostCommonColour << endl;
    cv::Mat averageColour(480, 640, CV_8UC3, mostCommonColour);
    imshow("Average", averageColour);
    return mostCommonColour;
}


int main()
{
    Mat img = imread("C:/OpenCV Task/Images/GreenApple.bmp");
    imshow("src", img);

    Mat mask = getMask(img);
    /*imshow("mask", mask);
    Mat maskedImg;
    img.copyTo(maskedImg, mask);
    imshow("maskedImg", maskedImg);

    Vec3b mostCommonColour = getMostCommonColour(img, mask);

    cout << endl << "Most common colour: " << mostCommonColour << endl;*/

    waitKey(0);

    return 0;
}
