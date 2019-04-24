/*
 *a sample to use tof in ros with smart_tof
 *
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/types.h>
#include <getopt.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <sstream>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/distortion_models.h"
#include "sensor_msgs/CameraInfo.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//#include "smart_tof/change_intg.h"
//#include "smart_tof/change_power.h"

using namespace std;

//pcl::visualization::CloudViewer pclviewer("Cloud Viewer");
boost::shared_ptr<pcl::visualization::PCLVisualizer> obsviewer (new pcl::visualization::PCLVisualizer ("Obstacle Viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
int vv1=5, last_sz_size = 0;

//data source
//1-tof,rgbd
//2-3dlidar
int data_type = 2;

//void img_gray_callback(const sensor_msgs::ImageConstPtr& msg)
//{
//    //for dist img you may replace "mono8" with sensor_msgs::image_encodings::TYPE_32FC1
//    cv::Mat img = cv_bridge::toCvCopy(msg, "mono8")->image;
//    cv::imshow("image gray got", img);
//    cv::waitKey(1);
//}
void initPclViewer()
{
    obsviewer->initCameraParameters();
    obsviewer->createViewPort(0.0,0.0,1.0,1.0,vv1);
    obsviewer->setBackgroundColor (255, 255, 255, vv1);
    obsviewer->addCoordinateSystem(0.2, "Obstacle Viewer", vv1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(pc, 255, 0, 0);
    obsviewer->addPointCloud(pc, colorHandler, "obstacle");
    pcl::PointXYZ lp0(0,-0.3,0),lp1(0,-0.3,2);
    obsviewer->addLine(lp0, lp1, 0, 250, 0, "oridir", vv1);
    obsviewer->addLine(lp0, lp1, 0, 250, 160, "moddir", vv1);
    pcl::PointXYZ basic_point(0,0,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ppcc(new pcl::PointCloud<pcl::PointXYZ>);
    if(data_type == 1){
        for(float i=0;i<=2*M_PI;i+=M_PI/360){
            basic_point.x = 2*cos(i);
            basic_point.z = 2*sin(i);
            ppcc->points.push_back(basic_point);
        }
    }
    else if(data_type == 2){
        for(float i=0;i<=2*M_PI;i+=M_PI/360){
            basic_point.x = 2*cos(i);
            basic_point.y = 2*sin(i);
            ppcc->points.push_back(basic_point);
        }
    }
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler0(ppcc, 0, 200, 0);
    obsviewer->addPointCloud(ppcc, colorHandler0, "2m");
    //pc->clear();
//    lp1.x = sqrt(3.0);
//    obsviewer->addLine(lp0, lp1, 0, 0, 0, "120", vv1);
//    lp1.x = 2/sqrt(3.0);
//    obsviewer->addLine(lp0, lp1, 250, 0, 0, "60", vv1);
}
void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    //obsviewer.showCloud(pc);
    /*
    double maxxyz[3]={0,0,0}, minxyz[3]={100,100,100};
    for(int i=0; i<pc->size();++i){
        if(pc->points[i].x>maxxyz[0]){
            maxxyz[0] = pc->points[i].x;
        }
        if(pc->points[i].y>maxxyz[1]){
            maxxyz[1] = pc->points[i].y;
        }
        if(pc->points[i].z>maxxyz[2]){
            maxxyz[2] = pc->points[i].z;
        }
        if(pc->points[i].x<minxyz[0]){
            minxyz[0] = pc->points[i].x;
        }
        if(pc->points[i].y<minxyz[1]){
            minxyz[1] = pc->points[i].y;
        }
        if(pc->points[i].z<minxyz[2]){
            minxyz[2] = pc->points[i].z;
        }
    }
    cout<<maxxyz[0]<<","<<maxxyz[1]<<","<<maxxyz[2]<<","<<",size="<<pc->size()
        <<","<<minxyz[0]<<","<<minxyz[1]<<","<<minxyz[2]<<","<<endl;
    */
    obsviewer->removePointCloud("obstacle");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(pc, 180, 0, 0);
    obsviewer->addPointCloud(pc, colorHandler, "obstacle");
}
void disang_to_xz(float dis, float ang, float &x, float &z){
    z = dis*cos(ang);
    x = -dis*sin(ang);
}
void disang_to_xy(float dis, float ang, float &x, float &y){
    y = dis*sin(ang);
    x = dis*cos(ang);
}

void sz_callback(const std_msgs::Float32MultiArrayConstPtr& msg){
    int sz_size = (msg->data.size()-2)/4;
    string lname;
    string l="line";
    char num[5];
    stringstream stream;
    for(int i=0; i<last_sz_size; ++i){
        stream<<i;
        lname = l+stream.str();
        obsviewer->removeShape(lname, vv1);
        lname.clear();
        stream.clear();
        stream.str("");
    }
    pcl::PointXYZ lp0(0,0,0),lp1(0,0,0);
    // if(data_type == 1){
    //     lp1.y = 0.3;
    // }
    // else if(data_type == 2){
    //     lp1.z = 0.3;
    // }
    string n="dir";
    if(data_type == 1){
        disang_to_xz(1.5, msg->data[msg->data.size()-1], lp0.x, lp0.z);
    }
    else if(data_type == 2){
        disang_to_xy(1.5, msg->data[msg->data.size()-1], lp0.x, lp0.y);
    }
    obsviewer->removeShape("oridir", vv1);
    obsviewer->removeShape("moddir", vv1);
    obsviewer->addLine(lp0, lp1, 0, 250, 160, "moddir", vv1);
    if(data_type == 1){
        disang_to_xz(1.5, msg->data[msg->data.size()-2], lp0.x, lp0.z);
    }
    else if(data_type == 2){
        disang_to_xy(1.5, msg->data[msg->data.size()-2], lp0.x, lp0.y);
    }
    obsviewer->addLine(lp0, lp1, 0, 250, 0, "oridir", vv1);
    for(int i=0; i<sz_size; ++i){
        stream<<i;
        lname = l+stream.str();
        if(data_type == 1){
            disang_to_xz(msg->data[4*i], msg->data[4*i+1], lp0.x, lp0.z);
            disang_to_xz(msg->data[4*i+2], msg->data[4*i+3], lp1.x, lp1.z);
        }
        else if(data_type == 2){
            disang_to_xy(msg->data[4*i], msg->data[4*i+1], lp0.x, lp0.y);
            disang_to_xy(msg->data[4*i+2], msg->data[4*i+3], lp1.x, lp1.y);
        }
        obsviewer->addLine(lp0, lp1, 0, 0, 250, lname, vv1);
        lname.clear();
        stream.clear();
        stream.str("");
    }
    last_sz_size = sz_size;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pclviewer");
    ros::NodeHandle nh;
    //setting running rate as 30Hz
    ros::Rate loop_rate(30);
    //service client for intg time
    //ros::ServiceClient client = nh.serviceClient<smart_tof::change_intg>("/smarttof/change_intg");
    //smart_tof::change_intg intg_srv;

    //image subscriber
    //image_transport::ImageTransport it(nh);
    //image_transport::Subscriber imggray_sub = it.subscribe("/smarttof/image_gray", 1, img_gray_callback);

    nh.param("data_type", data_type, 2);

    //point cloud subcribersmarttof/pointcloud
    constexpr char kPointCloud2Topic[] = "/points";
    ros::Subscriber pcl_sub = nh.subscribe(kPointCloud2Topic, 1, pcl_callback);
    ros::Subscriber sz_sub = nh.subscribe("/obstacle/safezone",1,sz_callback);

    initPclViewer();
    switch(data_type){
        case 1:ROS_INFO("[OBS]point cloud viewer start with 3d lidar mode!");break;
        case 2:ROS_INFO("[OBS]point cloud viewer start with depth cam mode!");break;
        default:break;
    }
    while(ros::ok())
    {
        //call service to change intg time every 5 seconds
        // if(++cnt == 150)
        // {
        //     intg_srv.request.a = 100;
        //     client.call(intg_srv);ROS_INFO("set intg 100!");
        // }
        // else if(cnt == 300)
        // {
        //     intg_srv.request.a = 200;
        //     client.call(intg_srv);ROS_INFO("set intg 200!");
        //     cnt = 0;
        // }
        //necessary for running callback
        obsviewer->spinOnce(1);
        ros::spinOnce();
        //sleep to fit the running rate
        loop_rate.sleep();
    }

    return 1;
}
