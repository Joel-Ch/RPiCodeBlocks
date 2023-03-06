#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv\cv.h"
#include <opencv2/opencv.hpp>

//#define TEST
//#define COLOUR
//#define RESULT

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

	int lowThreshold = 250;
	int highThreshold = 750;
	int kernel_size = 5;
	cv::Canny(src, detected_edges, lowThreshold, highThreshold, kernel_size);

	return detected_edges;
 }

Mat getMask(Mat src){
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
	bool useCanny = false;
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
		}
	}

	Scalar color(255, 255, 255);
	drawContours(largestContour, contours, largestContourIndex, color, CV_FILLED, 8, hierarchy); //Draw the largest contour using previously stored index.
	#ifdef TEST
	imshow("7. Largest Contour", largestContour);
    #endif // TEST

    return largestContour;
}

void getPalette(Mat img, Mat mask){
    Mat samples(img.rows * img.cols, 3, CV_32F);
    int idx = 0;
    for (int row = 0; row < img.rows; row++) {
        for (int col = 0; col < img.cols; col++) {
            if (mask.at<uchar>(row, col) == 255) {
                Vec3b color = img.at<Vec3b>(row, col);
                samples.at<float>(idx, 0) = color[0];
                samples.at<float>(idx, 1) = color[1];
                samples.at<float>(idx, 2) = color[2];
                idx++;
            }
        }
    }

    int K = 8;
    Mat labels;
    TermCriteria criteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1.0);
    Mat centers;
    kmeans(samples, K, labels, criteria, 10, KMEANS_RANDOM_CENTERS, centers);

    cout << "Color Palette:" << endl;
    int count[K];
    memset(count, 0, sizeof count);
    for (int i = 0; i < labels.rows; i++) {
        count[labels.at<int>(i, 0)]++;
    }

    float total_pixels = labels.rows;
    int max_color = 0;
    float max_percentage = 0.0f;
    for (int i = 0; i < K; i++) {
        float percentage = (count[i] / total_pixels) * 100;
        if (percentage > max_percentage) {
            max_percentage = percentage;
            max_color = i;
        }
        cout << "Color " << i + 1 << ": ";
        cout << (int)centers.at<float>(i, 0) << "\t";
        cout << (int)centers.at<float>(i, 1) << "\t";
        cout << (int)centers.at<float>(i, 2) << "\t";
        cout << percentage << "%" << endl;
    }
    cout << "The most common color is Color " << max_color + 1 << " with " << max_percentage << "%" << endl;
    return;
}

Mat GetPalette(Mat img) {
    cv::Mat1f color_mat;
    img.convertTo(color_mat, CV_32FC3);
    color_mat = color_mat.reshape(1, color_mat.total());

    cv::Mat1f color_mat_float;
    color_mat.convertTo(color_mat_float, CV_32F, 1.0 / 255.0);

    int cluster_count = 8;
    cv::Mat1i labels;
    cv::Mat1f centers;
    cv::kmeans(color_mat_float, cluster_count, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);

    centers.convertTo(centers, CV_8UC1, 255.0);

    std::vector<int> histogram(cluster_count, 0);
    for (int i = 0; i < labels.total(); i++) {
        histogram[labels(i)]++;
    }

    int max_value = *std::max_element(histogram.begin(), histogram.end());
    cv::Mat3b color_palette(100, cluster_count * 50, cv::Vec3b(255, 255, 255));
    for (int i = 0; i < cluster_count; i++) {
        cv::Rect color_rect(i * 50, 100 - (100 * histogram[i] / max_value), 50, 100 * histogram[i] / max_value);
        cv::Mat3b color_roi = color_palette(color_rect);
        cv::Mat3b color_fill = cv::Mat3b(cv::Vec3b(centers(i, 0), centers(i, 1), centers(i, 2)));
        cv::addWeighted(color_fill, 1.0, color_roi, 0.0, 0.0, color_roi);
    }
    return color_palette;
}


int main( int argc, char** argv )
{
    String imageName("C:/OpenCV Task/Images/RedApple.bmp"); // by default
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

	//0. Source Image
    #ifdef TEST
	imshow("src", src);
    #endif //TEST

    Mat mask = getMask(src);

    //getPalette(src,mask);

    Mat palette = GetPalette(src);

    imshow("Palette", palette);

	//8. Mask original image
	Mat maskedSrc;
	src.copyTo(maskedSrc, mask);

	#ifdef TEST
	imshow("8. Masked Source", maskedSrc);
	#endif // TEST



	waitKey(0);
}


