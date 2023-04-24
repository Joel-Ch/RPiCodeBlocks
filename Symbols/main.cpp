// Include files for required libraries
#include <stdio.h>

#include "opencv_aee.hpp"
//#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"//i2c library

#include <string>//string manipulation
#include <unistd.h>//sleep

Pi2c arduino(4);//set up i2c communication


using namespace cv;
using namespace std;

//ROI etc
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
const int ROI_TOP = FRAME_HEIGHT*0.33;
const int ROI_BOTTOM = FRAME_HEIGHT;
const int ROI_HEIGHT = ROI_BOTTOM-ROI_TOP;
const int ROI_LEFT = FRAME_WIDTH*0.1;
const int ROI_RIGHT = FRAME_WIDTH*0.9;
const int ROI_WIDTH = ROI_RIGHT-ROI_LEFT;

// Tuning parameters for line following
const double LINE_FOLLOW_KP = 1.5;
const double LINE_FOLLOW_KI = 0;
const double LINE_FOLLOW_KD = 0;
const int BASESPEED = 150;

float Max(float values[])
{
    float maxv=0;
    for (int i=0; i<sizeof(values); i++)
    {
        if (maxv<values[i])
            maxv=values[i];
    }
    return maxv;
}

string SymbolCompareAndRect(Mat Warped)
{
    Mat shape;
    string ShapeName[4] = {"CircleR.png", "TriangleB.png", "UmbrellaY.png", "StarG.png"};
    string ShapeColour[4] = {"red", "blue", "yellow", "green"};
    float match[4];

    for (int j = 0; j < 4; j++)                            //Compares both images
    {
        shape = imread(ShapeName[j]);
        cvtColor(shape, shape, COLOR_BGR2HSV);                  //converts to HSV
        inRange(shape, Scalar(139,115,115), Scalar(150,255,255), shape);       //So its in the same format as the input image [Warped]
        match[j] = compareImages(shape, Warped);                        //Returns a % match as a float value
    }

    int ShapeMatch = 0, s = 0;
    float ShapeValue = Max(match);                            //Finds the largest % match

    while (ShapeMatch == 0)                                   //Finds the position of the largest match value
    {

        if (ShapeValue == match[s])
            ShapeMatch = 1;
        else
            s = s + 1;
    }

    cout<<ShapeName[s];                 //displays the color and shape from the position of the largest value
    printf("\t");

    cout<<ShapeColour[s];
    printf("\n");

    return ShapeColour[s];			//Returns color of line to be followed.
}


// Function to adjust steering angle based on error
void PID(int *error,float *sumError, float *lastError)
{
    *error -= 90;
    // Adjust steering using proportional, integral and derivative control
    *sumError += *error;

    *error * LINE_FOLLOW_KP + *sumError * LINE_FOLLOW_KI +(*error - *lastError) * LINE_FOLLOW_KD;

    // Update last error
    *lastError = *error;

    *error += 90;

}

string getColour(Mat frame)
{
    string colour;
    Mat frameHSV, kernel;

    cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    inRange(frameHSV, Scalar(111, 10, 0), Scalar(179, 148, 255), frameHSV);     // Produce a binary image to work with

    int numNonZero = 0;
    numNonZero = countNonZero(frameHSV);        // Find any pixels which fell in the specified range.

    if(numNonZero > 1000)           // Trial and error this value too
    {
        usleep(250000);        //Adjust with trial and error
        char dataString[10] = {};
        sprintf(dataString, "0/0/90");      // this is a string print.
        arduino.i2cWrite(dataString, 10);

        Mat Symbol = captureFrame();
        Mat SymbolHSV;
        cvtColor(Symbol, Symbolhsv, COLOR_BGR2HSV);

        inRange(SymbolHSV, Scalar(111, 10, 0), Scalar(179, 148, 255), SymbolHSV);
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(SymbolHSV, SymbolHSV, MORPH_OPEN, kernel);                 // Lines 120 & 121 are in place to filter out noise and add clarity to the image


        std::vector< std::vector<cv::Point> > contours;                             // Variable for list of contours std
        findContours(SymbolHSV, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        if(contours.size() == 0)
        {
            colour = "black";
            return colour;          // This prevents crashing if no contours are found
        }

        int i, LAcurrent, LAI, Area;        // LA [Largest Area], LAI - Largest area index
        for(i=0 ; i < contours.size() ; i++)
        {
            Area = cv::contourArea(contours[i]);
            if(Area > LAcurrent)
            {
                LAcurrent = Area;
                LAI = i;                                // Finding the largest contour in the image through comparing the areas, storing the index of our largest contour for later use.
            }
        }

        RotatedRect rect = minAreaRect(contours[LAI]);          // Find the rectangle of minimal area which encompasses our largest contour.
        Point2f srcPoints[4];
        rect.points(srcPoints);                     //145 & 146 define srcPoints using the points function. This function sequentially assigns the corners to array elements starting
                                                    //At the bottom left [Min x & y values] and continues to label counterclockwise around the rectangle

        cv::drawContours(SymbolHSV, contours, LAI, scalar(0, 0, 255), 2);
        imshow("Largest contour", SymbolHSV);                               //Draw the largest contour to the HSV Image

        cv::Point2f dstPoints [4] = (cv::Point2f(0, 0), cv::Point2f(349, 0), cv::Point2f(349, 349), cv::Point2f(0, 349));

        Mat M = cv::getPerspectiveTransform(srcPoints, dstPoints);      // Using the srcPoints[Points found on the rectangle as seen by the camera] and the desired points [dstPoints]
        // we return the transformation needed into Mat M.

        Mat Warped;                                 // Create variable for the warped image

        warpPerspective(SymbolHSV, Warped, M, Size(350, 350));              //Applying the transformation and warping the image
        imshow("Transformed view", Warped);


        colour = SymbolCompareAndRect(Warped);                              //Pass the transformed 350 x 350 image to the comparing function to determine the shape and corresponding color
    }

    return colour;
}


cv::Mat getBinaryImage(cv::Mat frame,string colour)
{
    cv::Mat binary;
    if (colour == "blackold")
    {
        cv::Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY); // this is the old version
        threshold(gray, binary, 100, 255, THRESH_BINARY_INV);
    }
    else
    {
        cv::Mat frameHsv, kernel;
        cvtColor(frame, frameHsv, COLOR_BGR2HSV);
        if (colour == "red")
        {
            // two masks for red
            cv::Mat binary2;
            inRange(frameHsv, Scalar(0, 70, 50), Scalar(10, 255, 255), binary); // red has 2 masks as it needs to select both 160-180 and 0-10 ranges
            inRange(frameHsv, Scalar(160, 70, 50), Scalar(180, 255, 255), binary2);
            // combine masks
            cv::bitwise_or(binary, binary2, binary);
            // reduce noise
            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(binary, binary, MORPH_OPEN, kernel);
        }
        else if (colour == "green")
        {
            inRange(frameHsv, Scalar(36, 70, 70), Scalar(86, 255, 255), binary);
            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(binary, binary, MORPH_OPEN, kernel);
        }
        else if (colour == "blue")
        {
            inRange(frameHsv, Scalar(100, 150, 0), Scalar(140, 255, 255), binary);
            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(binary, binary, MORPH_OPEN, kernel);
        }
        else if (colour == "yellow")
        {
            inRange(frameHsv, Scalar(20, 100, 100), Scalar(50, 255, 255), binary);
            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(binary, binary, MORPH_OPEN, kernel);
        }
        else if (colour == "black")
        {
            inRange(frameHsv, Scalar(0, 0, 0), Scalar(180, 255, 70), binary);
            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(binary, binary, MORPH_OPEN, kernel);
        }
    }
    return binary;
}

cv::Rect getMaxRect(vector<vector<Point>> contours,int *maxArea)
{
    Rect maxRect;
    for (const auto &contour : contours)
    {
        Rect rect = boundingRect(contour);
        int area = rect.width * rect.height;
        if (area > *maxArea)
        {
            *maxArea = area;
            maxRect = rect;
        }
    }
    return maxRect;
}

int getAngle(Point centre)
{
    Point centreBottom = Point(ROI_WIDTH / 2, ROI_HEIGHT);

    // calculating tan breaks the code when the centres are above each other (divide by zero!)
    if (centreBottom.x == centre.x)
        centre.x += 1;

    float gradient = -((float)centreBottom.y - (float)centre.y) / ((float)centreBottom.x - (float)centre.x);
    float rawAngle = atan(gradient) * (180 / 3.14159265);
    // this function returns an angle from 90 -> -90 but -90 and 90 are both almost vertical so this needs some editing to enable a useful angle output
    int angle;
    if (rawAngle > 0)
    {
        angle = 180 - rawAngle;
    }
    else if (rawAngle < 0)
    {
        angle = -rawAngle;
    }
    return angle;
}


int main(int argc, char **argv)
{
    setupCamera(FRAME_WIDTH, FRAME_HEIGHT); // Enable the camera for OpenCV



    // Initialize ROI for line following
    Rect roi(ROI_LEFT, ROI_TOP, ROI_WIDTH, ROI_HEIGHT);

    float lastError = 0, lastAngle = 0, sumAngle = 0;//declare variables
    int rightSpeed = 0, leftSpeed = 0;
    bool Reverse = true;

    while (true)
    {
        // Capture a frame from the camera
        Mat frame;
        while (frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        cv::flip(frame,frame,0);
        cv::flip(frame,frame,1);//flip image both vertically and horizontally to make it the correct way up
        cv::imshow("Photo", frame); // Display the image in the window

        // Apply ROI for line following
        //Mat roi_frame = frame;
        Mat roiFrame = frame(roi);
        string colour = getColour(frame);
        printf("%s", colour);

        cv::Mat binary = getBinaryImage(roiFrame, colour);//get binary image

        cv::imshow("Binary", binary);

        // Find contours in binary image
        vector<vector<Point>> contours;//vector of vectors
        cv::findContours(binary, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        // Find contour with largest area + draw a rectangle around it
        int maxArea = 0;
        Rect maxRect = getMaxRect(contours, &maxArea);

        //find rectangle centre coordinates
        Point centre = Point(maxRect.x + maxRect.width / 2,maxRect.y + maxRect.height / 2);

        if (maxArea == 0)//if there is no rectangle
        {
            centre = Point(ROI_WIDTH/2,ROI_HEIGHT/2);
            Reverse = true;
        }
        else if (maxArea > 0)
        {
            Reverse = false;
        }

        int angle = getAngle(centre);//get angle

        PID(&angle,&sumAngle,&lastAngle);//PID!

        int rightSpeed, leftSpeed;
        if (Reverse == true)
        {
            angle = 90;
            rightSpeed = leftSpeed = -BASESPEED;
        }
        else if (Reverse == false)
        {
            //set wheel speeds
            rightSpeed = BASESPEED - ((angle - 90) * 0.5);
            leftSpeed = BASESPEED + ((angle - 90) * 0.5);

            //reduce speed if angle is too large
            if (abs(angle - 90 >= 20))
            {
                rightSpeed = rightSpeed * 0.8;
                leftSpeed = leftSpeed * 0.8;
            }
            else if (abs(angle - 90) >= 30)
            {
                rightSpeed = rightSpeed * 0.5;
                leftSpeed = leftSpeed * 0.5;
            }
        }

        char dataString[8] = {};
        sprintf(dataString, "%-3i/%-4i/%-4i", angle, rightspeed, leftspeed); // this is String printf (prints to a string)
        arduino.i2cWrite(dataString, 13);      // write the string via i2c

        // draw an output image
        cv::Mat contourDisp = roiFrame.clone();                          // clone roi_frame (image) into contourDisp
        cv::drawContours(contourDisp, contours, -1, Scalar(0, 0, 255), 2);//draw contours on the output image
        cv::circle(contourDisp,centre,5,Scalar(0,255,0),3);//draw green circle at the centre of the rectangle
        line(contourDisp, centre, Point(ROI_WIDTH / 2, ROI_HEIGHT), Scalar(0, 255, 0), 2); // draw a line on the output image
        imshow("Contour", contourDisp);//display output image

        printf("Centre: %i %i\tAngle: %i\t%s\n", centre.x, centre.y, angle, dataString);//print results

        int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)

        key = (key == 255) ? -1 : key; // Check if the ESC key has been pressed
        if (key == 27)
            break;

        usleep(15000);//delay (servo can only update at 50Hz so must delay until servo can update again)
    }

    return 0;
}
