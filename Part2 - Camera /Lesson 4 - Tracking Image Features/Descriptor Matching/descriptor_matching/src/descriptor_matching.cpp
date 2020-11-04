#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "structIO.hpp"

#define LOAD_LARGE_DATASET 0
#define LOAD_BRISK_DATASET 0

namespace const_hyperparams
{
    std::string MATCHER_BF = "MAT_BF";
    std::string MATCHER_FLANN = "MAT_FLANN";
    std::string DESC_BINARY = "DES_BIN";
    std::string DESC_NOT_BINARY = "DES_NBIN";
    std::string SELECT_NN = "SEL_NN";
    std::string SELECT_KNN = "SEL_KNN";
}

using namespace std;

void matchDescriptors(cv::Mat &imgSource, cv::Mat &imgRef, vector<cv::KeyPoint> &kPtsSource, vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      vector<cv::DMatch> &matches, string descriptorType, string matcherType, string selectorType)
{

    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {

        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        cout << "BF matching cross-check=" << crossCheck << endl;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { 
            // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::FlannBasedMatcher::create();
        cout << "FLANN matching" << endl;
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { 
        // nearest neighbor (best match)
        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor from a query set (descSource)
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { 
        // k nearest neighbors (k=2), finds the k best matches for each descriptor from a query set.
        int k = 2;
        float DESC_distanceThreshold = 0.8;
        std::vector<std::vector<cv::DMatch>> knnMatches;
        double t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knnMatches, k);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << knnMatches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

        // Filtering matches using descriptor distance ratio test
        for(auto it = knnMatches.begin(); it!= knnMatches.end(); ++it){

            double distanceRatio = (*it)[0].distance / (*it)[1].distance;
            if(distanceRatio < DESC_distanceThreshold)
                matches.push_back((*it)[0]);
        }
        cout << "Keypoints removed = " << knnMatches.size() - matches.size() << endl;
    }

    // visualize results
    cv::Mat matchImg = imgRef.clone();
    cv::drawMatches(imgSource, kPtsSource, imgRef, kPtsRef, matches,
                    matchImg, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    string windowName = "Matching keypoints between two camera images (best 50)";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::imshow(windowName, matchImg);
    cv::waitKey(0);
}

int main()
{
    cv::Mat imgSource = cv::imread("../images/img1gray.png");
    cv::Mat imgRef = cv::imread("../images/img2gray.png");


    vector<cv::KeyPoint> kptsSource, kptsRef; 
    cv::Mat descSource, descRef; 
    
    if(LOAD_BRISK_DATASET)
    {
        if(LOAD_LARGE_DATASET)
        {
            readKeypoints("../dat/C35A5_KptsSource_BRISK_large.dat", kptsSource);
            readKeypoints("../dat/C35A5_KptsRef_BRISK_large.dat", kptsRef);

            readDescriptors("../dat/C35A5_DescSource_BRISK_large.dat", descSource);
            readDescriptors("../dat/C35A5_DescRef_BRISK_large.dat", descRef);
        }
        else
        {
            readKeypoints("../dat/C35A5_KptsSource_BRISK_small.dat", kptsSource);
            readKeypoints("../dat/C35A5_KptsRef_BRISK_small.dat", kptsRef);

            readDescriptors("../dat/C35A5_DescSource_BRISK_small.dat", descSource);
            readDescriptors("../dat/C35A5_DescRef_BRISK_small.dat", descRef);
        }
    }
    else
    {
        readKeypoints("../dat/C35A5_KptsSource_SIFT.dat", kptsSource);
        readKeypoints("../dat/C35A5_KptsRef_SIFT.dat", kptsRef);

        readDescriptors("../dat/C35A5_DescSource_SIFT.dat", descSource);
        readDescriptors("../dat/C35A5_DescRef_SIFT.dat", descRef);   
    }    

    vector<cv::DMatch> matches;
    string matcherType = const_hyperparams::MATCHER_BF; 
    string descriptorType = const_hyperparams::DESC_BINARY; 
    string selectorType = const_hyperparams::SELECT_KNN;

    matchDescriptors(imgSource, imgRef, kptsSource, kptsRef, descSource, descRef, matches, descriptorType, matcherType, selectorType);
}