#ifndef ARUCO_MARKER_DETECTION_H
#define ARUCO_MARKER_DETECTION_H

#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


namespace ORB_SLAM2
{

struct ORBParameters
{
    // general parameters for the ORB detector
    int maxFrames, nFeatures, nLevels, iniThFAST, minThFAST;
    bool RGB;
    float scaleFactor, depthMapFactor, thDepth;
    // camera parameters
    float fx, fy, cx, cy, baseline;
    float k1, k2, p1, p2, k3;
};


class AruCoMarkerDetection {
public:
    AruCoMarkerDetection(ORBParameters& parameters) 
    {
        // Constructor implementation
        // Initialize member variables
        mMarkerDetected = false;
        if (loadCameraParameters(parameters))
        {
            std::cout << "Camera parameters have been loaded." << std::endl;
        }
        else
        {
            std::cerr << "Failed to load camera parameters." << std::endl;
        }
        
        std::cout << "AruCoMarkerDetection has been initialized." << std::endl;
    }

    ~AruCoMarkerDetection() {
        // Destructor implementation
        // Clean up resources
    }

    bool loadCameraParameters(ORBParameters& parameters) 
    {
        // Implementation for loading camera parameters
        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = parameters.fx;
        K.at<float>(1,1) = parameters.fy;
        K.at<float>(0,2) = parameters.cx;
        K.at<float>(1,2) = parameters.cy;
        K.copyTo(mCameraMatrix);

        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = parameters.k1;
        DistCoef.at<float>(1) = parameters.k2;
        DistCoef.at<float>(2) = parameters.p1;
        DistCoef.at<float>(3) = parameters.p2;
        if(parameters.k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = parameters.k3;
        }
        DistCoef.copyTo(mDistCoeffs);

        mBaseline = parameters.baseline;

        // Set dictionary
        cv::Ptr<cv::aruco::Dictionary> mpDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        // Set detector parameters
        mpDetectorParams = cv::aruco::DetectorParameters::create();
        mpDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        mpDetectorParams->cornerRefinementWinSize = 5;
        mpDetectorParams->cornerRefinementMaxIterations = 30;
        mpDetectorParams->cornerRefinementMinAccuracy = 0.1;

        return true;
    }

    // Implementation for marker detection
    bool DetectMarkers(const cv::Mat& image) 
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mMarkerDetected = false; 
        // Set Current Image
        if( SetImage(image))
        {
            std::cout << "Image has been set." << std::endl;
        }
        else
        {
            std::cerr << "Failed to set image." << std::endl;
            return false;
        }

        // Detect markers
        cv::aruco::detectMarkers(mCurrentImage, mpDictionary, mvvp2fMarkerCorners, mviMarkerIds, mpDetectorParams);
        if (mviMarkerIds.size() >0)
        {
            // Estimate camera pose
            EstimateCameraPose();
            // Return true if markers are detected
            return mMarkerDetected;
        }
        else
        {
            // Return false if markers are not detected
            return false;
        }
        
    }

    void EstimateCameraPose() {
        // Implementation for camera pose estimation
        // Set pose member variable
        
        std::lock_guard<std::mutex> lock(mMutex);
        if (mviMarkerIds.size() > 0)
        {
            mMarkerDetected = true;
            cv::aruco::estimatePoseSingleMarkers(mvvp2fMarkerCorners, 0.05, mCameraMatrix, mDistCoeffs, mRotVec, mTransVec);
            cv::aruco::drawDetectedMarkers(mCurrentImage, mvvp2fMarkerCorners, mviMarkerIds);
            cv::aruco::drawAxis(mCurrentImage, mCameraMatrix, mDistCoeffs, mRotVec, mTransVec, 0.1);
            cv::Mat R;
            cv::Rodrigues(mRotVec, R); // 회전 벡터를 회전 행렬로 변환

            cv::Mat mPose = (cv::Mat_<double>(4, 4) << 
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), mTransVec.at<double>(0),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), mTransVec.at<double>(1),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), mTransVec.at<double>(2),
            0, 0, 0, 1);

            cv::Mat mPoseInverse = mPose.inv(); // 동차 변환 행렬의 역변환
            std::cout << "Marker estimation has done." << std::endl;
        }
        else
        {
            mMarkerDetected = false;
        }
    }

    bool SetImage(const cv::Mat& image) {
        // Implementation for setting the image
        // Call detectMarkers and estimateCameraPose methods
        std::lock_guard<std::mutex> lock(mMutex);
        
        if (image.empty())
        {
            std::cerr << "Failed to load image" << std::endl;
            return false;
        }
        
        mCurrentImage = image;
        return true;
    }

    bool isMarkerDetected() {
        // Implementation for getting the detection result
        return mMarkerDetected;
    }

    cv::Mat GetPose() {
        // Implementation for getting the pose
        return mPose;
    }
    cv::Mat GetPoseInverse() {
        // Implementation for getting the pose
        return mPoseInverse;
    }

private:
    cv::Ptr<cv::aruco::Dictionary> mpDictionary;
    cv::Ptr<cv::aruco::DetectorParameters> mpDetectorParams;
    
    cv::Mat mCameraMatrix;
    cv::Mat mDistCoeffs;
    float mBaseline;
    cv::Mat mPoseInverse;

    std::vector<int> mviMarkerIds;
    std::vector<std::vector<cv::Point2f>> mvvp2fMarkerCorners;
    cv::Mat mRotVec;
    cv::Mat mTransVec;
    cv::Mat mCurrentImage;
    bool mMarkerDetected;
    cv::Mat mPose;

    std::mutex mMutex;
}; // Class AruCoMarkerDetection
}  // Namespace ORB_SLAM2

#endif // ARUCO_MARKER_DETECTION_H
