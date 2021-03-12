/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"include/System.h"

#include"include/ViewerAR.h"

using namespace std;


ORB_SLAM2::ViewerAR viewerAR;
bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

//    if(argc != 3)
//    {
//        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
//        ros::shutdown();
//        return 1;
//    }
    
    ORB_SLAM2::ORBParameters parameters;
//    string strSettingPath = argv[2];
    string strSettingPath = "/home/michael/catkin_ws/src/orb_slam_2_ros/Asus.yaml";
    string vocPath = "/home/michael/catkin_ws/src/orb_slam_2_ros/orb_slam2/Vocabulary/ORBvoc.txt";
    string mapFile = "/home/michael/catkin_ws/src/orb_slam_2_ros/Map.bin";

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    parameters.maxFrames = fps;
    int rgb = fSettings["Camera.RGB"];
    parameters.RGB = rgb == 1 ? true : false;
    parameters.thDepth = fSettings["ThDepth"];
    parameters.nFeatures = fSettings["ORBextractor.nFeatures"];
    parameters.scaleFactor = fSettings["ORBextractor.scaleFactor"];
    parameters.nLevels = fSettings["ORBextractor.nLevels"];
    parameters.iniThFAST = fSettings["ORBextractor.iniThFAST"];
    parameters.minThFAST = fSettings["ORBextractor.minThFAST"];
    parameters.depthMapFactor = fSettings["DepthMapFactor"];

    parameters.fx = fSettings["Camera.fx"];
    parameters.fy = fSettings["Camera.fy"];
    parameters.cx = fSettings["Camera.cx"];
    parameters.cy = fSettings["Camera.cy"];

    parameters.k1 = fSettings["Camera.k1"];
    parameters.k2 = fSettings["Camera.k2"];
    parameters.p1 = fSettings["Camera.p1"];
    parameters.p2 = fSettings["Camera.p2"];
    parameters.k3 = fSettings["Camera.k3"];
    parameters.baseline = fSettings["Camera.bf"];

    parameters.mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    parameters.mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    parameters.mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    parameters.mPointSize = fSettings["Viewer.PointSize"];
    parameters.mCameraSize = fSettings["Viewer.CameraSize"];
    parameters.mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    parameters.mImageWidth = fSettings["Camera.width"];
    parameters.mImageHeight = fSettings["Camera.height"];
    parameters.mViewpointX = fSettings["Viewer.ViewpointX"];
    parameters.mViewpointY = fSettings["Viewer.ViewpointY"];
    parameters.mViewpointZ = fSettings["Viewer.ViewpointZ"];
    parameters.mViewpointF = fSettings["Viewer.ViewpointF"];
    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocPath, ORB_SLAM2::System::MONOCULAR, parameters, mapFile, true, false);


    cout << endl << endl;
    cout << "-----------------------" << endl;
    cout << "Augmented Reality Demo" << endl;
    cout << "1) Translate the camera to initialize SLAM." << endl;
    cout << "2) Look at a planar region and translate the camera." << endl;
    cout << "3) Press Insert Cube to place a virtual cube in the plane. " << endl;
    cout << endl;
    cout << "You can place several cubes in different planes." << endl;
    cout << "-----------------------" << endl;
    cout << endl;


    viewerAR.SetSLAM(&SLAM);

    cout << "debug 1" << endl;

    ImageGrabber igb(&SLAM);
    cout << "debug 2" << endl;
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/tello/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    cout << "debug 3" << endl;
    viewerAR.SetFPS(parameters.maxFrames);
    viewerAR.SetCameraCalibration(parameters.fx,parameters.fy,parameters.cx,parameters.cy);
	cout << "debug 4" << endl;
    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = parameters.fx;
    K.at<float>(1,1) = parameters.fy;
    K.at<float>(0,2) = parameters.cx;
    K.at<float>(1,2) = parameters.cy;

    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = parameters.k1;
    DistCoef.at<float>(1) = parameters.k2;
    DistCoef.at<float>(2) = parameters.p1;
    DistCoef.at<float>(3) = parameters.p2;
    if(parameters.k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = parameters.k3;
    }

    thread tViewer = thread(&ORB_SLAM2::ViewerAR::Run,&viewerAR);
    cout << "debug 5" << endl;
    ros::spin();
    cout << "debug 6" << endl;
    // Stop all threads
    SLAM.Shutdown();
    cout << "debug 7" << endl;
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    cout << "debug 8" << endl;
    ros::shutdown();
    cout << "debug 9" << endl;
    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat im = cv_ptr->image.clone();
    cv::Mat imu;
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    cv::Mat Tcw = mpSLAM->GetCurrentPosition();
    int state = mpSLAM->GetTrackingState();
    vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
    vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();

    cv::undistort(im,imu,K,DistCoef);

    if(bRGB)
        viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
    else
    {
        cv::cvtColor(imu,imu,CV_RGB2BGR);
        viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
    }    
}


