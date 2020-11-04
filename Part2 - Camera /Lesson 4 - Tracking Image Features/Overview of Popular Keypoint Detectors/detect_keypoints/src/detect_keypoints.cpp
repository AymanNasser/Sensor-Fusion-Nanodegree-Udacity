#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void detKeypoints1()
{
    // load image from file and convert to grayscale
    cv::Mat imgGray;
    cv::Mat img = cv::imread("../images/img1.png");
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // Shi-Tomasi detector
    int blockSize = 6;       //  size of a neighborhood arout the keypoint for computing a derivative covariation matrix over each pixel 
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, ranges from 0.0 to 1.0

    // minimum distance between any 2 keypoints neighborhood size = (blockSize) if there is no overlapping 
    // e.g --> if maxOverlap = 25% therefore, minDistance = 75% of the blocksize 
    double minDistance = (1.0 - maxOverlap) * blockSize;  

    // 	Maximum number of corners to return. If there are more corners than are found, the strongest of them is returne
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); 

    // Parameter characterizing the minimal accepted quality of image corners. 
    // The parameter value is multiplied by the best corner quality measure, which is the minimal eigenvalue   
    // The corners with the quality measure less than the product are rejected.
    double qualityLevel = 0.01;                                   
    double k = 0.04;
    bool useHarris = false; // Parameter indicating whether to use a Harris detector or cornerMinEigenVal.

    vector<cv::KeyPoint> kptsShiTomasi;
    vector<cv::Point2f> corners;
    double t = (double)cv::getTickCount();
    // goodFeaturesToTrack finds the most prominent corners in the image or in the specified image region
    /* 
     * Function calculates the corner quality measure at every source image pixel using the cornerMinEigenVal or cornerHarris.
     * Function performs a non-maximum suppression.
     * The corners with the minimal eigenvalue less than qualityLevel are rejected.
     * The remaining corners are sorted by the quality measure in the descending order.
     * Function throws away each corner for which there is a stronger corner at a distance less than maxDistance.
    */
    cv::goodFeaturesToTrack(imgGray, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarris, k);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi with n = " << corners.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    for (auto it = corners.begin(); it != corners.end(); ++it)
    { // add corners to result vector

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        kptsShiTomasi.push_back(newKeyPoint);
    }

    // visualize results
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, kptsShiTomasi, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Shi-Tomasi Results";
    cv::namedWindow(windowName, 1);
    imshow(windowName, visImage);
    cv::waitKey(0);

    // Features from accelerated segment test (FAST) "Published in 2006" is a corner detection method, 
    // which could be used to extract feature points and later used to track and map objects in many computer vision tasks.
    /* 
     * Select a pixel p in the image which is to be identified as an interest point or not. Let its intensity be Ip.
     * Select appropriate threshold value t.
     * Consider a circle of 16 pixels around the pixel under test. (This is a Bresenham circle of radius 3.)
     * Now the pixel p is a corner if there exists a set of n contiguous pixels in the circle (of 16 pixels) which are all brighter than Ip + t, or all darker than Ip - t. (The authors have used n= 12 in the first version of the algorithm)
     * To make the algorithm fast, first compare the intensity of pixels 1, 5, 9 and 13 of the circle with Ip. As evident from the figure above, at least three of these four pixels should satisfy the threshold criterion so that the interest point will exist.
     * If at least three of the four-pixel values — I1, I5, I9, I13 are not above or below Ip + t, then p is not an interest point (corner). In this case reject the pixel p as a possible interest point. Else if at least three of the pixels are above or below Ip + t, then check for all 16 pixels and check if 12 contiguous pixels fall in the criterion.
     * Repeat the procedure for all the pixels in the image.
     * 
    */
   
    // FAST detector
    int threshold = 10;
    bool nonMaxSuppression = true; 
    std::vector<cv::KeyPoint> fastKeyPoints;

    double t_2 = (double)cv::getTickCount();

    auto fastDetector = cv::FastFeatureDetector::create(threshold, nonMaxSuppression, cv::FastFeatureDetector::TYPE_9_16);
    fastDetector->detect(imgGray, fastKeyPoints);

    t_2 = ((double)cv::getTickCount() - t_2) / cv::getTickFrequency();
    cout << "FAST with n = " << fastKeyPoints.size() << " keypoints in " << 1000 * t_2 / 1.0 << " ms" << endl;

    // visualize results
    cv::Mat visImage_2 = img.clone();
    cv::drawKeypoints(img, fastKeyPoints, visImage_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName_2 = "FAST Results";
    cv::namedWindow(windowName_2 , cv::WINDOW_AUTOSIZE);
    imshow(windowName_2, visImage_2);
    cv::waitKey(0);

}

int main()
{
    detKeypoints1();
}