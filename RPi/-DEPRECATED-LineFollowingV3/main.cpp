// Include files for required libraries
#include <stdio.h>

#include "opencv_aee.hpp"
//#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

#include <string>
#include <unistd.h>


using namespace cv;
using namespace std;


// Tuning parameters for line following
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
const int ROI_TOP = FRAME_HEIGHT*0.66;
const int ROI_BOTTOM = FRAME_HEIGHT;
const double LINE_FOLLOW_KP = 0.5;
const double LINE_FOLLOW_KI = 0.1;
const double LINE_FOLLOW_KD = 0.1;

// Function to adjust steering angle based on erfror
float PID(float error,float *sumError, float *lastError);

int main(int argc, char **argv)
{
    setupCamera(FRAME_WIDTH, FRAME_HEIGHT); // Enable the camera for OpenCV
    Pi2c arduino(4);//set up i2c communication

    cv::namedWindow("Photo"); // Create a GUI window called photo
    //cv::namedWindow("Grey");
    //cv::namedWindow("Binary");
    cv::namedWindow("Contour");
    string colour;
    while(1)//this only lets  you out when a valid colour is entered
    {
        cout << "enter colour to follow (black, red, green, blue, yellow): ";
        cin >> colour;

        if (colour == "red" || colour == "green" || colour == "blue" || colour == "yellow" || colour == "black" || colour == "black2")
            break;
    }

    // Initialize ROI for line following
    Rect roi(0, ROI_TOP, FRAME_WIDTH, ROI_BOTTOM - ROI_TOP);

    float lastError = 0, lastAngle=0, sumAngle=0;//declare variables
    int rightSpeed=0, leftSpeed=0;
    bool Reverse=true;

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
        Mat roi_frame = frame(roi);

        //convert to HSV
        cv::Mat roi_frame_hsv;
        cvtColor(roi_frame,roi_frame_hsv,COLOR_BGR2HSV);

        // Convert to grayscale and apply thresholding
        cv::Mat gray,binary,kernel;

        if(colour=="black")
        {
            cvtColor(roi_frame, gray, COLOR_BGR2GRAY);//this is the old version
            threshold(gray, binary, 100, 255, THRESH_BINARY_INV);
        }
        else if(colour=="red")
        {
            //two maks for red
            cv::Mat binary2;
            inRange(roi_frame_hsv,Scalar(0,70,50),Scalar(10,255,255),binary);//red has 2 masks as it needs to select both 160-180 and 0-10 ranges
            inRange(roi_frame_hsv,Scalar(160,70,50),Scalar(180,255,255),binary2);
            //combine masks
            cv::bitwise_or(binary,binary2,binary);
            //reduce noise
            kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
            cv::morphologyEx(binary,binary,MORPH_OPEN,kernel);

        }
        else if(colour=="green")
        {
            inRange(roi_frame_hsv,Scalar(36,70,70),Scalar(86,255,255),binary);
            kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
            cv::morphologyEx(binary,binary,MORPH_OPEN,kernel);
        }
        else if(colour=="blue")
        {
            inRange(roi_frame_hsv,Scalar(100,150,0),Scalar(140,255,255),binary);
            kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
            cv::morphologyEx(binary,binary,MORPH_OPEN,kernel);
        }
        else if(colour=="yellow")
        {
            inRange(roi_frame_hsv,Scalar(20,100,100),Scalar(50,255,255),binary);
            kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
            cv::morphologyEx(binary,binary,MORPH_OPEN,kernel);
        }
        else if (colour == "black2")
        {
            inRange(roi_frame_hsv,Scalar(0,0,0),Scalar(180,255,70),binary);
            kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
            cv::morphologyEx(binary,binary,MORPH_OPEN,kernel);
        }
        else
        {
            cerr <<"invalid colour";
            return 1;
        }
        //cv::imshow("HSV", roi_frame_hsv);//display the various useful images
        //cv::imshow("Grey",gray);
        cv::imshow("Binary", binary);

        // Find contours in binary image
        vector<vector<Point>> contours;//vector of vectors
        cv::findContours(binary, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        //draw contours on the original image
        Mat contourDisp = roi_frame.clone();//clone roi_frame (image) into contourDisp
        cv::drawContours(contourDisp, contours, -1, Scalar(0, 0, 255),2);

        // Find contour with largest area + draw a rectangle around it
        int max_area = 0;
        Rect max_rect;
        for (const auto &contour : contours)
        {
            Rect rect = boundingRect(contour);
            int area = rect.width * rect.height;
            if (area > max_area)
            {
                max_area = area;
                max_rect = rect;
            }
        }
        //find rectangle centre coordinates
        Point centre = Point(max_rect.x + max_rect.width / 2,max_rect.y + max_rect.height / 2);

        if (max_area == 0)//if there is no rectangle
        {
            centre = Point(FRAME_WIDTH/2,FRAME_HEIGHT/2);
            Reverse=true;
        }
        else if (max_area > 0)
        {
            Reverse=false;
        }
        cv::circle(contourDisp,centre,5,Scalar(0,255,0),3);//draw green circle at the centre

        //this is the old function - checks the x coordinate of the centre and uses it to find the error.

        /*float error = centre.x - FRAME_WIDTH / 2;
        // Adjust steering based on error
        PID(&error, &lastError);
        //change error into angle
        int errorInt = 90+error/6;*/

        //this is the new function, finding the angle between the bottom centre and the point.

        Point centreBottom = Point(FRAME_WIDTH/2,FRAME_HEIGHT);

        //calculating tan breaks the code when the centres are above each other (divide by zero!)
        if (centreBottom.x==centre.x)
            centre.x +=1;

        float gradient = -((float)centreBottom.y-(float)centre.y)/((float)centreBottom.x-(float)centre.x);
        float rawAngle = atan(gradient) * (180/3.14159265);
        //this function returns an angle from 90 -> -90 but -90 and 90 are both almost vertical so this needs some editing to enable a useful angle output
        int angle;
        if (rawAngle > 0)
        {
            angle=180-rawAngle;
        }
        else if (rawAngle < 0)
        {
            angle = -rawAngle;
        }

        float correctedAngle = PID(angle-90,&sumAngle,&lastAngle);//PID!

        line(contourDisp,centre,centreBottom,Scalar(0,255,0),2);//draw a line on the output image

        imshow("Contour", contourDisp);//display output image

        if (Reverse == true)
        {
            angle=90;
            rightSpeed=leftSpeed=-150;
        }
        else if (Reverse == false)
        {
            rightSpeed=150-((angle-90)*0.5);
            leftSpeed=150+((angle-90)*0.5);
        }
        char dataString[15] = {};//convert the 3 numbers to a string
        sprintf(dataString, "%-3i/%-4i/%-4i", angle,rightSpeed,leftSpeed);// this is String printf (prints to a string)
        arduino.i2cWrite(dataString,13);//write the string via i2c

        //arduino.i2cWriteArduinoInt(rightSpeed);
        //arduino.i2cWriteArduinoInt(leftSpeed);
        //cout << "X: " << centre.x<< "\tY: " << centre.y << "\tGradient: " << gradient << "\tRAWANGLE: " << rawAngle <<"\tANGLE: " << angle <<endl;
        printf("X: %i\tY: %i\tGradient: %-8.2fRaw Angle: %-8.2fAngle: %i\tRightSpeed: %-5iLeftSpeed: %-5i\n", centre.x, centre.y, gradient, rawAngle, angle, rightSpeed,leftSpeed);//print results

        int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)

        key = (key == 255) ? -1 : key; // Check if the ESC key has been pressed
        if (key == 27)
            break;

        usleep(15000);//delay (servo can only update at 50Hz so must delay until servo can update again)
    }

    return 0;
}

float PID(float error,float *sumError, float *lastError)
{

    // Adjust steering using proportional, integral and derivative control
    *sumError += error;

    float output = error * LINE_FOLLOW_KP + *sumError * LINE_FOLLOW_KI +(error - *lastError) * LINE_FOLLOW_KD;

    // Update last error
    *lastError = error;

    return output;
}
