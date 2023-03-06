// Include files for required libraries
#include <stdio.h>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

#include <string>

using namespace cv;
using namespace std;


// Tuning parameters for line following
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int ROI_TOP = 320;
const int ROI_BOTTOM = 480;
const double LINE_FOLLOW_KP = 0.5;
const double LINE_FOLLOW_KD = 0.1;

// Function to adjust steering angle based on error
void adjust_steering(int error)
{
    // Adjust steering using proportional and derivative control
    int output = error * LINE_FOLLOW_KP + (error - last_error) * LINE_FOLLOW_KD;

    // Update last error
    last_error = error;

    // Send output to steering servo
}

int main(int argc, char **argv)
{
    // Initialize camera capture
    setupCamera(320, 240); // Enable the camera for OpenCV
    setup();               // Call a setup function to prepare IO and devices

    cv::namedWindow("Photo"); // Create a GUI window called photo

    // Initialize ROI for line following
    Rect roi(0, ROI_TOP, FRAME_WIDTH, ROI_BOTTOM - ROI_TOP);

    // Initialize error and last error for steering control
    int error = 0;
    int last_error = 0;

    while (true)
    {
        // Capture a frame from the camera
        Mat frame;
        while (frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        cv::imshow("Photo", frame); // Display the image in the window

        // Apply ROI for line following
        Mat roi_frame = frame(roi);

        // Convert to grayscale and apply thresholding
        Mat gray, binary;
        cvtColor(roi_frame, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 100, 255, THRESH_BINARY);

        // Find contours in binary image
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        Mat contour = frame.clone();
        drawContours(contour, contours, -1, Scalar(0, 0, 255), 2);

        // Find contour with largest area and compute error
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
        int line_center = max_rect.x + max_rect.width / 2;
        error = line_center - FRAME_WIDTH / 2;

        // Adjust steering based on error
        //adjust_steering(error);
        cout << "ERROR: " << error << endl;

        int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)

        key = (key == 255) ? -1 : key; // Check if the ESC key has been pressed
        if (key == 27)
            break;
    }

    return 0;
}
