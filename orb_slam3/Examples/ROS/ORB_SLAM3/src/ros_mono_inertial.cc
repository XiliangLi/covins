/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"
#include"../include/Converter.h"

// COVINS
#include <covins/covins_base/config_comm.hpp> //for covins_params

//coxgraph
#include <coxgraph_mod/vio_interface.h>

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe, ros::NodeHandle nh, ros::NodeHandle nh_private): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe) {
      // ros::NodeHandle n;
      

      nh_private.param<std::string>("origin_frame_name", 
                            ORIGIN_FRAME_NAME, 
                            "odom");
      nh_private.param<std::string>("imu_frame_name", 
                            IMU_FRAME_NAME, 
                            "imu");
      nh_private.param<std::string>("camera_frame_name", 
                            CAMERA_FRAME_NAME, 
                            "cam");

      pubKeyframePose = nh_private.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
      
      // coxgraph
      // vio_interface = new coxgraph::mod::VIOInterface(nh, nh_private);
      // mpSLAM->getLoopCloser()->vio_interface = vio_interface;
    }

    void pubTF(cv::Mat& Tcw, nav_msgs::Odometry& keyframePose_msg);
    void PublishKeyframePose(cv::Mat Tcw, nav_msgs::Odometry &keyframePose_msg);
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg, nav_msgs::Odometry &keyframePose_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    ros::Publisher pubKeyframePose;
    std::__cxx11::string ORIGIN_FRAME_NAME;
    std::__cxx11::string IMU_FRAME_NAME;
    std::__cxx11::string CAMERA_FRAME_NAME;

    // coxgraph
    coxgraph::mod::VIOInterface* vio_interface;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

void ImageGrabber::pubTF(cv::Mat& Tcw, nav_msgs::Odometry& keyframePose_msg) {
  if(!Tcw.empty()) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    cv::Mat Tbc = mpSLAM->getTracker()->getImuCalib()->Tbc;
    cv::Mat Tbw = Tbc * Tcw;

    cv::Mat Rwb = Tbw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twb = - Rwb * Tbw.rowRange(0, 3).col(3);
    vector<float> q1 = ORB_SLAM3::Converter::toQuaternion(Rwb);

    // imu frame
    // cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
    transform.setOrigin(tf::Vector3(twb.at<float>(0), twb.at<float>(1), twb.at<float>(2)));
    // vector<float> q1 = ORB_SLAM3::Converter::toQuaternion(Tcw.rowRange(0, 3).colRange(0, 3));
    q.setW(q1[3]);
    q.setX(q1[0]);
    q.setY(q1[1]);
    q.setZ(q1[2]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, keyframePose_msg.header.stamp, ORIGIN_FRAME_NAME, IMU_FRAME_NAME));
    
    // camera frame
    transform.setOrigin(tf::Vector3(Tbc.at<float>(0, 3), Tbc.at<float>(1, 3), Tbc.at<float>(2, 3)));
    cv::Mat Rbc = Tbc.rowRange(0, 3).colRange(0, 3);
    vector<float> q2 = ORB_SLAM3::Converter::toQuaternion(Rbc);
    q.setW(q2[3]);
    q.setX(q2[0]);
    q.setY(q2[1]);
    q.setZ(q2[2]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, keyframePose_msg.header.stamp, IMU_FRAME_NAME, CAMERA_FRAME_NAME));
    
    
  } else {
    std::cout << "====================================== no Tcw ============" << std::endl;
  }
}

void ImageGrabber::PublishKeyframePose(cv::Mat Tcw, nav_msgs::Odometry &keyframePose_msg) {
  // nav_msgs::Odometry keyframePose_msg;
  
  if(!Tcw.empty()) {
    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = - Rwc * Tcw.rowRange(0, 3).col(3);
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);

    keyframePose_msg.pose.pose.position.x = twc.at<float>(0);
    keyframePose_msg.pose.pose.position.y = twc.at<float>(1);
    keyframePose_msg.pose.pose.position.z = twc.at<float>(2);
    keyframePose_msg.pose.pose.orientation.w = q[3];
    keyframePose_msg.pose.pose.orientation.x = q[0];
    keyframePose_msg.pose.pose.orientation.y = q[1];
    keyframePose_msg.pose.pose.orientation.z = q[2];
    pubKeyframePose.publish(keyframePose_msg); 
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 3 || argc > 4)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }


  if(argc==4)
  {
    std::string sbEqual(argv[3]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, covins_params::orb::activate_visualization);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,bEqual, nh, nh_private); // TODO
  
  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = nh_private.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img0 = nh_private.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg, nav_msgs::Odometry &keyframePose_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    keyframePose_msg.header = img_msg->header;
    keyframePose_msg.header.stamp = img_msg->header.stamp;
    // keyframePose_msg.header.stamp = ros::Time::now();
    keyframePose_msg.header.frame_id = ORIGIN_FRAME_NAME;
    
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;

    cv::Mat Tcw;
    nav_msgs::Odometry keyframePose_msg;

    if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.toSec();
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      {
      this->mBufMutex.lock();
      im = GetImage(img0Buf.front(), keyframePose_msg);
      img0Buf.pop();
      this->mBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
        mClahe->apply(im,im);

      Tcw = mpSLAM->TrackMonocular(im,tIm,vImuMeas);

      // start send tf time
      double tpub = mpSLAM->getTracker()->t0IMU;
      // std::cout << "==================== tpub:" << tpub << std::endl;
      
      // if(keyframePose_msg.header.stamp.toSec() >= double(tpub + 15)) {
        // std::cout << "==================== keyframe:" << keyframePose_msg.header.stamp.toSec() << std::endl;
      // if(mpSLAM->getLocalMapper()->GetCurrKF()->GetMap()->GetIniertialBA2()) {         // 发生锁错误
      // if(mpSLAM->getTracker()->getAtlaser()->GetCurrentMap()->GetIniertialBA2()) {
      // if(mpSLAM->getLocalMapper()->mbVIBA2) {
        pubTF(Tcw, keyframePose_msg);
        PublishKeyframePose(Tcw, keyframePose_msg);
      // }
      
    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}


