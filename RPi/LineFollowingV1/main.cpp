// Include files for required libraries
#include <stdio.h>

#include "opencv_aee.hpp"
//#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"
//#include "ArduinoComms.h"

#include <string>


using namespace cv;
using namespace std;


// Tuning parameters for line following
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;
const int ROI_TOP = FRAME_HEIGHT*0.66;
const int ROI_BOTTOM = FRAME_HEIGHT;
const double LINE_FOLLOW_KP = 0.5;
const double LINE_FOLLOW_KD = 0.1;

// Function to adjust steering angle based on erfror
void PID(float error, float *lastError);

int main(int argc, char **argv)
{
    setupCamera(320, 240); // Enable the camera for OpenCV
    Pi2c arduino(4);

    cv::namedWindow("Photo"); // Create a GUI window called photo
    cv::namedWindow("Grey");
    cv::namedWindow("Binary");
    cv::namedWindow("Contour");

    // Initialize ROI for line following
    Rect roi(0, ROI_TOP, FRAME_WIDTH, ROI_BOTTOM - ROI_TOP);

    float lastError = 0;

    while (true)
    {
        // Capture a frame from the camera
        Mat frame;
        while (frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

        cv::flip(frame,frame,0);
        cv::flip(frame,frame,1);
        cv::imshow("Photo", frame); // Display the image in the window

        // Apply ROI for line following
        Mat roi_frame = frame(roi);

        // Convert to grayscale and apply thresholding
        Mat gray, binary;
        cvtColor(roi_frame, gray, COLOR_BGR2GRAY);//replace this line with colour selection
        threshold(gray, binary, 100, 255, THRESH_BINARY_INV);

        cv::imshow("Grey", gray);

        cv::imshow("Binary", binary);

        // Find contours in binary image
        vector<vector<Point>> contours;
        cv::findContours(binary, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        Mat contour = roi_frame.clone();
        cv::drawContours(contour, contours, -1, Scalar(0, 0, 255), 2);
        imshow("Contour", contour);

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
        int lineCentre = max_rect.x + max_rect.width / 2;
        float error = lineCentre - FRAME_WIDTH / 2;

        // Adjust steering based on error
        PID(error, &lastError);
        arduino.i2cWriteArduinoInt(150);
        arduino.i2cWriteArduinoInt(150);
        arduino.i2cWriteArduinoInt(error/3);
        cout << "Line: " << lineCentre<<"\tERROR: " << error << endl;

        int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)

        key = (key == 255) ? -1 : key; // Check if the ESC key has been pressed
        if (key == 27)
            break;
    }

    return 0;
}

void PID(float error, float *lastError)
{

    // Adjust steering using proportional and derivative control
    float output = error * LINE_FOLLOW_KP + (error - *lastError) * LINE_FOLLOW_KD;

    // Update last error
    *lastError = error;

}
