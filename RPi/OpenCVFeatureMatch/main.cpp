// Include files for required libraries
#include <stdio.h>
#include <string>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
}

int main(int argc, char **argv)
{
    // Initialize camera capture
    setupCamera(320, 240); // Enable the camera for OpenCV

    cv::namedWindow("No-Entry"); // Create a GUI window
    cv::namedWindow("Camera"); // Create a GUI window
    cv::namedWindow("Selection"); // Create a GUI window

    Mat noEntry = cv::imread("/home/pi/Desktop/no-entry-sign.png");
    cv::imshow("No-Entry", noEntry);

    while (true)
    {
        // Capture a frame from the camera
        Mat frame;
        while (frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        cv::imshow("Camera", frame); // Display the image in the window

        Point centre = featureMatch(frame, noEntry, 400, 0.75, 10);

        if (centre.x<0 && centre.y < 0)
            std::cout << "ERROR" << std::endl;
        else
        {
            std::cout << "Object found at X:" << centre.x << "\tY:" << centre.y << std::endl;
            cv::circle(frame,centre,5,(0,0,255),-1);
        }

        imshow("Selection", frame);


        int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)

        key = (key == 255) ? -1 : key; // Check if the ESC key has been pressed
        if (key == 27)
            break;
    }

    closeCV();

    return 0;
}




