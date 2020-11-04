#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void gradientSobel()
{
    const int nrows = 3, ncols = 3; 
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img2gray.png");

    // TODO: Based on the image gradients in both x and y, compute an image 
    // which contains the gradient magnitude according to the equation at the 
    // beginning of this section for every pixel position. Also, apply different 
    // levels of Gaussian blurring before applying the Sobel operator and compare the results.
    cv::Mat blurredImg;
    // cv::blur(img, blurredImg, cv::Size(ncols, nrows));
    cv::GaussianBlur(img, blurredImg, cv::Size(nrows,ncols), 0, 0, cv::BORDER_DEFAULT);

    float sobelData[nrows][ncols] = { {-1,-2,-1}, {0,0,0}, {1,2,1} };
    cv::Mat kernel = cv::Mat(nrows, ncols, CV_32F, sobelData);

    cv::Mat res;
    cv::filter2D(blurredImg, res, -1, kernel);
    
    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, res);
    cv::waitKey(0); // wait for keyboard input before continuing


}

int main()
{
    gradientSobel();
}