#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <vector>
#include <algorithm>
#include <iostream>

//#define TEST

using namespace cv;
using namespace cv::ml;
using namespace std;

Mat doKMeans(Mat img)
{

    // Reshape the masked image into a single column matrix
    Mat samples;
    int n = img.rows * img.cols;
    img.reshape(1, n).convertTo(samples, CV_32F);

    // Perform k-means clustering on the reshaped masked image
    int clusterCount = 8;
    vector<int> labels;
    Mat1f colours;
    kmeans(samples, clusterCount, labels, TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, colours);

    for (int i = 0; i < n; ++i)
    {
        samples.at<float>(i, 0) = colours(labels[i], 0);
        samples.at<float>(i, 1) = colours(labels[i], 1);
        samples.at<float>(i, 2) = colours(labels[i], 2);
    }

    Mat KMeansImg;
    samples.reshape(3,img.rows).convertTo(KMeansImg,CV_8U);

    return KMeansImg;
}

Mat getMask(Mat src){

    // Convert the input image to grayscale
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    imshow("Grey", gray);

    // Apply Gaussian blur to reduce noise
    //cv::Mat blurred;
    //cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

    //imshow("Blur", blurred);

    // Run the Canny edge detector:


    Mat sorted;
    cv::sort(gray, sorted, SORT_ASCENDING);
    double v = 0.0;
    int num_pixels = gray.rows * gray.cols;
    if (num_pixels % 2 == 0) {
        // even number of pixels
        int idx = num_pixels / 2;
        v = (sorted.at<uchar>(idx-1) + sorted.at<uchar>(idx)) / 2.0;
    } else {
        // odd number of pixels
        int idx = (num_pixels - 1) / 2;
        v = sorted.at<uchar>(idx);
    }

    Mat edges;

    // Apply automatic Canny edge detection using the computed median
    double sigma = 0.33;
    int lower = max(0, static_cast<int>((1.0 - sigma) * v));
    int upper = min(255, static_cast<int>((1.0 + sigma) * v));
    cv::Canny(gray,edges, lower, upper);
    cv::imshow("H", edges);

    	//4. Dilate
	Mat dilateGrad = edges;
	int dilateType = MORPH_ELLIPSE;
	int dilateSize = 3;
	Mat elementDilate = getStructuringElement(dilateType,
		Size(2*dilateSize + 1, 2*dilateSize+1),
		Point(dilateSize, dilateSize));
	dilate(edges, dilateGrad, elementDilate);

	#ifdef TEST
	imshow("4. Dilate", dilateGrad);
    #endif // TEST

	//5. Floodfill
	Mat floodFilled = cv::Mat::zeros(dilateGrad.rows+2, dilateGrad.cols+2, CV_8U);
	floodFill(dilateGrad, floodFilled, cv::Point(0, 0), 0, 0, cv::Scalar(), cv::Scalar(), 4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
	floodFilled = cv::Scalar::all(255) - floodFilled;
	Mat temp;
	floodFilled(Rect(1, 1, dilateGrad.cols-2, dilateGrad.rows-2)).copyTo(temp);
	floodFilled = temp;

	#ifdef TEST
	imshow("5. Floodfill", floodFilled);
    #endif // TEST

	//6. Erode
	int erosionType = MORPH_ELLIPSE;
	int erosionSize = 4;
	Mat erosionElement = getStructuringElement(erosionType,
		Size(2*erosionSize+1, 2*erosionSize+1),
		Point(erosionSize, erosionSize));
	erode(floodFilled, floodFilled, erosionElement);
	#ifdef TEST
	imshow("6. Erode", floodFilled);
    #endif // TEST


    // Find contours in the edge map
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(floodFilled, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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

    #ifdef TEST
    cv::imshow("mask",mask);
    #endif // TEST

    // Create a black background image
    cv::Mat bg = cv::Mat::zeros(src.size(), src.type());

    // Copy the masked image to the black background
    src.copyTo(bg, mask);

    // Show the resulting image
    #ifdef TEST
    cv::imshow("Result", bg);
    #endif // TEST

    return bg;
}

int main()
{
    cv::namedWindow("src");
    cv::namedWindow("KMeans");
    Mat img = imread("C:/OpenCV Task/Images/BlueApple.bmp");
    cv::imshow("src", img);

    //Mat KMeansImg = doKMeans(img);
    //imshow("KMeans", KMeansImg);

    /*Mat mask = getMask(img);

    Mat KMeansImgMasked;
    KMeansImg.copyTo(KMeansImgMasked, mask);
    imshow("KMeansMasked",KMeansImgMasked);

    //find/display most common colour

    Mat imgMasked;
    img.copyTo(imgMasked, mask);
    imshow("ImgMasked",imgMasked);*/

    waitKey(0);

    return 0;
}
