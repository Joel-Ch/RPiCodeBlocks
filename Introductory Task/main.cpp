#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv\cv.h"
#include <opencv2/opencv.hpp>

#define TEST
#define COLOUR
#define RESULT

using namespace cv;
using namespace std;

int euclidDistance(Scalar A,int* B){
    return sqrt((A[0]-B[0])*(A[0]-B[0]) +(A[1]-B[1])*(A[1]-B[1]) +(A[2]-B[2])*(A[2]-B[2]));
}

Mat sobel(Mat gray){
	Mat edges;

	int scale = 1;
	int delta = 1;
	int ddepth = CV_16S;
	Mat edges_x, edges_y;
	Mat abs_edges_x, abs_edges_y;
	cv::Sobel(gray, edges_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs( edges_x, abs_edges_x );
	cv::Sobel(gray, edges_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(edges_y, abs_edges_y);
	addWeighted(abs_edges_x, 0.5, abs_edges_y, 0.5, 0, edges);

	return edges;
}

Mat canny(Mat src){
	Mat detected_edges;

	int lowThreshold = 1500;
	int highThreshold = 4500;
	int kernel_size = 5;
	cv::Canny(src, detected_edges, lowThreshold, highThreshold, kernel_size);

	return detected_edges;
 }

String DetermineColour(Scalar average){//can find yellow,cyan and purple (also extendable)
    int Colours[6][3] = {{255,0,0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};
    String ColourName[6] = {"Blue", "Green", "Red", "Cyan", "Yellow", "Purple"};
    int distance; int minimumDistance = 400; int minimumDistanceIndex; int i;

    for (i=0; i < 6;i++)
    {
        distance = euclidDistance(average,Colours[i]);
        cout << "Colour: " << ColourName[i] << "\t Distance: " << distance << endl;

        if (distance < minimumDistance)
        {
            minimumDistanceIndex = i;
            minimumDistance = distance;
        }
    }
    return ColourName[minimumDistanceIndex];
 }



int main( int argc, char** argv )
{
    String imageName("C:/OpenCV/OpenCV Task/Images/RedCar.bmp"); // by default
    if (argc > 1)
    {
        imageName = argv[1];
    }
    Mat src = imread(imageName, IMREAD_COLOR);
    if (src.empty())
    {
        cerr << "No image supplied ..." << endl;
        return -1;
    }

	//windows
	/*
	namedWindow("src", WINDOW_NORMAL);
	namedWindow("1. \"Remove Shadows\"", WINDOW_NORMAL);
	namedWindow("2. Grayscale", WINDOW_NORMAL);
	namedWindow("3. Edge Detector", WINDOW_NORMAL);
	namedWindow("4. Dilate", WINDOW_NORMAL);
	namedWindow("5. Floodfill", WINDOW_NORMAL);
	namedWindow("6. Erode", WINDOW_NORMAL);
	namedWindow("7. Largest Contour", WINDOW_NORMAL);
	namedWindow("8. Masked Source", WINDOW_NORMAL);
	namedWindow("src boxed", WINDOW_NORMAL);
	*/

	//0. Source Image
    #ifdef TEST
	imshow("src", src);
    #endif //TEST

	//1. Remove Shadows
	//Convert to HSV
	Mat hsvImg;
	cvtColor(src, hsvImg, CV_BGR2HSV);
	Mat channel[3];
	split(hsvImg, channel);
	channel[2] = Mat(hsvImg.rows, hsvImg.cols, CV_8UC1, 200);//Set V
	//Merge channels
	merge(channel, 3, hsvImg);
	Mat rgbImg;
	cvtColor(hsvImg, rgbImg, CV_HSV2BGR);

	#ifdef TEST
	imshow("1. \"Remove Shadows\"", rgbImg);
    #endif // TEST

	//2. Convert to gray and normalize
	Mat gray(rgbImg.rows, src.cols, CV_8UC1);
	cvtColor(rgbImg, gray, CV_BGR2GRAY);
	normalize(gray, gray, 0, 255, NORM_MINMAX, CV_8UC1);
	#ifdef TEST
	imshow("2. Grayscale", gray);
	#endif // TEST

	//3. Edge detector
	GaussianBlur(gray, gray, Size(3,3), 0, 0, BORDER_DEFAULT);
	Mat edges;
	bool useCanny = true;
	if(useCanny){
		edges = canny(gray);
	} else {
		//Use Sobel filter and thresholding.
		edges = sobel(gray);
		//Automatic thresholding
		threshold(edges, edges, 0, 255, cv::THRESH_OTSU);
		//Manual thresholding
		//threshold(edges, edges, 25, 255, cv::THRESH_BINARY);
	}

	#ifdef TEST
	imshow("3. Edge Detector", edges);
    #endif // TEST

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

	//7. Find largest contour
	int largestArea = 0;
	int largestContourIndex = 0;
	Rect boundingRectangle;
	Mat largestContour(src.rows, src.cols, CV_8UC1, Scalar::all(0));
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(floodFilled, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	for(int i=0; i<contours.size(); i++)
	{
		double a = contourArea(contours[i], false);
		if(a > largestArea)
		{
			largestArea = a;
			largestContourIndex = i;
			boundingRectangle = boundingRect(contours[i]);
		}
	}

	Scalar color(255, 255, 255);
	drawContours(largestContour, contours, largestContourIndex, color, CV_FILLED, 8, hierarchy); //Draw the largest contour using previously stored index.
	#ifdef TEST
	imshow("7. Largest Contour", largestContour);
    #endif // TEST

	// Compute the mean with the computed mask

    Scalar average = mean(src, largestContour);
    std::cout << average << std::endl;
    cv::Mat averageColour(480, 640, CV_8UC3, average);
    #ifdef COLOUR
    imshow("Average", averageColour);
    #endif // COLOUR

    String colour = DetermineColour(average);
    cout << colour <<endl;
	//Determine colour name

	//improvements: use dominant colour not average
	//remove some colours before processing?

	//8. Mask original image
	Mat maskedSrc;
	src.copyTo(maskedSrc, largestContour);

	#ifdef TEST
	imshow("8. Masked Source", maskedSrc);
	#endif // TEST

	//Source with largest contour boxed
	rectangle(src, boundingRectangle, Scalar(0, 255, 0), 1, 8, 0);
	#ifdef TEST
	imshow("src boxed", src);
	#endif // TEST
    Mat enddisp;
	cv::vconcat(src, maskedSrc, enddisp);
	#ifdef RESULT
	imshow("Result", enddisp);
	#endif // RESULT


	waitKey(0);
}


