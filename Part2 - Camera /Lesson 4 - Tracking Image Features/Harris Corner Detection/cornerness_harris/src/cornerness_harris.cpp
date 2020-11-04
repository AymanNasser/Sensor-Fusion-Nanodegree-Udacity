#include <iostream>
#include <time.h>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)


    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    auto startTime = std::chrono::steady_clock::now();
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled); // Scales, calculates absolute values, and converts the result to 8-bit
    auto endTime = std::chrono::steady_clock::now();

    // KeyPoints is a data structure for salient point detectors.
    // The class instance stores a keypoint, i.e. a point feature found by one of many available keypoint detectors, such as Harris corner detector
    // The keypoint is characterized by the 2D position, scale (proportional to the diameter of the neighborhood that needs to be taken into account), 
    // orientation and some other parameters.
    std::vector<cv::KeyPoint> detectedKeyPoints;

    float IoU_Threshold = 0;
    for(int rows=0; rows<dst_norm.rows; rows++){

        for(int cols=0; cols<dst_norm.cols; cols++){
            
            int pixelIntensity = dst_norm.at<float>(rows,cols);
            
            // If pixel intensity higher than the intensity threshold ==> take into account 
            if(pixelIntensity > minResponse)
            {
                cv::KeyPoint tempKeyPoint;
                tempKeyPoint.pt = cv::Point2f(cols,rows); // Columns --> change in x coord, Rows --> change in y coord  
                tempKeyPoint.response = pixelIntensity;
                tempKeyPoint.size = 2*apertureSize;

                // NMS procedure 
                bool isOverlapOccurred = false;
                for(auto it = detectedKeyPoints.begin(); it != detectedKeyPoints.end(); it++){
                    
                    /* This method computes overlap for pair of keypoints. Overlap is the ratio between area of keypoint regions' 
                    * intersection and area of keypoint regions' union (considering keypoint region as circle). If they don't overlap, we get zero. 
                    * If they coincide at same location with same size, we get 1
                    */
                    float overlapArea = cv::KeyPoint::overlap(tempKeyPoint, *it);

                    if(overlapArea > IoU_Threshold)
                    {   
                        isOverlapOccurred = true;
                        if(tempKeyPoint.response > it->response)
                            *it = tempKeyPoint; // Replacing the old keypoint with the new keypoint due to higher intensity of the new keypoint 
                    }
                }

                if(! isOverlapOccurred )
                    detectedKeyPoints.push_back(tempKeyPoint);
            }

        }

    }

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Corner Harris KeyPoint Detection Took: " << elapsedTime.count() << " milliseconds" << std::endl;

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);

    cv::Scalar RANDOM_COLOR = cv::Scalar::all(-1);
    cv::Scalar PURPLE = cv::Scalar(255,0,255);

    cv::Mat visImg = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, detectedKeyPoints, visImg, RANDOM_COLOR, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); // For each keypoint the circle around keypoint with keypoint size and orientation will be drawn.
    cv::imshow(windowName, visImg);
    cv::waitKey(0);

}

int main()
{
    cornernessHarris();
}