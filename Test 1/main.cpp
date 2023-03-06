#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;
Mat src; Mat src_gray;
int maxArea;

int main( int argc, char** argv )
{
    String imageName("C:/OpenCV Task/Images/BlueCar.bmp"); // by default
    if (argc > 1)
    {
        imageName = argv[1];
    }
    src = imread(imageName, IMREAD_COLOR);
    if (src.empty())
    {
        cerr << "No image supplied ..." << endl;
        return -1;
    }

    cvtColor( src, src_gray, COLOR_BGR2GRAY );//convert to grayscale
    blur( src_gray, src_gray, Size(3,3) );//blur
    const char* source_window = "Source";//create window to display original image
    namedWindow( source_window, WINDOW_AUTOSIZE );//^
    imshow( source_window, src );//display ^ window


    Mat canny_output;//create mat to hold output of edging
    vector<vector<Point> > contours;//variable to store list of contours
    vector<Vec4i> hierarchy;//variable to store topography data
    Canny( src_gray, canny_output, 100, 200, 3 );// use canny to find outline of shape
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );//find contours from outline

    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );//create output image
    Mat contour = Mat::zeros( canny_output.size(), CV_8UC3 );

    for( size_t i = 0; i< contours.size(); i++ )//draw contours onto output image
    {
        drawContours( drawing, contours, (int)i, Scalar(0,0,255-i), 2, 8, hierarchy, 0, Point() );
        int area = cv::contourArea(contours[i]); // Calculate the area of a contour
        if (area > maxArea)
            maxArea=i;
        cout << area << endl;
    }
    cout << maxArea << endl;
    drawContours(contour,contours,maxArea,Scalar(0,0,255),2,8,hierarchy,0,Point());

    namedWindow( "Contours", WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );
    namedWindow( "Canny", WINDOW_AUTOSIZE );
    imshow( "Canny", canny_output );
    namedWindow( "Grayscale", WINDOW_AUTOSIZE );
    imshow( "Grayscale", src_gray );
    namedWindow( "Contour", WINDOW_AUTOSIZE );
    imshow( "Contour", contour );

    waitKey(0);
    return(0);
}
