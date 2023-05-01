// Include files for required libraries
#include <stdio.h>
#include "opencv_aee.hpp"

#include <string>//string manipulation

using namespace cv;
using namespace std;

const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;

int main(int argc, char **argv)
{
    setupCamera(FRAME_WIDTH, FRAME_HEIGHT); // Enable the camera for OpenCV

    while (true)
    {
        // Capture a frame from the camera
        Mat frame;
        while (frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        cv::flip(frame,frame,0);
        cv::flip(frame,frame,1);//flip image both vertically and horizontally to make it the correct way up
        cv::imshow("Photo", frame); // Display the image in the window

        Mat frameHSV;
        cvtColor(frame, frameHSV, COLOR_BGR2HSV);

        inRange(frameHSV, Scalar(111, 10, 0), Scalar(179, 148, 255), frameHSV);
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(frameHSV, frameHSV, MORPH_OPEN, kernel);
        imshow("HSV", frameHSV); // Display the image in the window

        std::vector<std::vector<cv::Point>> contours; // Variable for list of contours std
        findContours(frameHSV, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
        float Area, LACurrent = 0;
        int LAI;

        if (contours.size() > 0)
        {
            for (i = 0; i < contours.size(); i++)
            {
                Area = cv::contourArea(contours[i]);
                if (Area > LAcurrent)
                {
                    LAcurrent = Area;
                    LAI = i; // Finding the largest contour in the image through comparing the areas, storing the index of our largest contour for later use.
                }
            }
        }
        RotatedRect rect = minAreaRect(contours[LAI]); // Find the rectangle of minimal area which encompasses our largest contour.
        Point2f srcPoints[4];
        rect.points(srcPoints); // 145 & 146 define srcPoints using the points function. This function sequentially assigns the corners to array elements starting
        // At the bottom left [Min x & y values] and continues to label counterclockwise around the rectangle
        cv::drawContours(frameHSV, contours, LAI, scalar(0, 0, 255), 2);
        imshow("Largest contour", frameHSV); // Draw the largest contour to the HSV Image

        Point2f dstPoints[4] = {cv : Point2f(0, 0), cv::Point2f(349, 0), cv::Point2f(349, 349), cv::Point2f(0, 349)};
        // define points to be used for the warping of image

        Mat M = cv::getPerspectiveTransform(srcPoints, dstPoints); // Using the srcPoints[Points found on the rectangle as seen by the camera] and the desired points [dstPoints]
        // we return the transformation needed into Mat M.

        Mat Warped; // Create variable for the warped image

        warpPerspective(SymbolHSV, Warped, M, Size(350, 350)); // Applying the transformation and warping the image
        imshow("Transformed view", Warped);

        Mat shape;
        string ShapeName[4] = {"CircleR.png", "TriangleB.png", "UmbrellaY.png", "StarG.png"};
        float match[4];

        for (int j = 0; j < 4; j++) // Compares both images
        {
            shape = imread(ShapeName[j]);
            cvtColor(shape, shape, COLOR_BGR2HSV);                               // converts to HSV
            inRange(shape, Scalar(139, 115, 115), Scalar(150, 255, 255), shape); // So its in the same format as the input image [Warped]
            match[j] = compareImages(shape, Warped);                             // Returns a % match as a float value
        }
        float maxMatch = 0;
        int maxMatchIndex = 0;
        for (int i = 0; i < 4; ++i) // Compares both images
        {
            if (match[i] > maxMatch)
            {
                maxMatch = match[i];
                maxMatchIndex = i;
            }
        }

        cout << ShapeName[maxMatchIndex]; // displays the color and shape from the position of the largest value
        printf("\n");

        int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)
        key = (key == 255) ? -1 : key; // Check if the ESC key has been pressed
        if (key == 27)
            break;
    }
    return 0;
}
