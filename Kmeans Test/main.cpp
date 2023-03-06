#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

int main()
{
    Mat img = imread("C:/Users/poely/Downloads/lenna.jpg");
    Mat samples(img.rows * img.cols, 3, CV_32F);
    int idx = 0;
    for (int row = 0; row < img.rows; row++) {
        for (int col = 0; col < img.cols; col++) {
            Vec3b color = img.at<Vec3b>(row, col);
            samples.at<float>(idx, 0) = color[0];
            samples.at<float>(idx, 1) = color[1];
            samples.at<float>(idx, 2) = color[2];
            idx++;
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

    float total_pixels = img.rows * img.cols;
    int max_color = 0;
    float max_percentage = 0.0f;
    for (int i = 0; i < K; i++) {
        float percentage = (count[i] / total_pixels) * 100;
        if (percentage > max_percentage) {
            max_percentage = percentage;
            max_color = i;
        }
        cout << "Color " << i + 1 << ": ";
        cout << (int)centers.at<float>(i, 0) << " ";
        cout << (int)centers.at<float>(i, 1) << " ";
        cout << (int)centers.at<float>(i, 2) << " ";
        cout << percentage << "%" << endl;
    }
    cout << "The most common color is Color " << max_color + 1 << " with " << max_percentage << "%" << endl;
    return 0;
}
