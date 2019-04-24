/*
 this uses sonar on pioneer-3at as input
*/

#ifndef P3ATOBSAVOID_H
#define P3ATOBSAVOID_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include "ros/ros.h"
#include "ros/time.h"
//#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "Octree.hpp"
//#include "ai_robot_navigation/restart_nav.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include "assistMath.h"

using namespace std;

class safeZone{
public:
    safeZone(){}
    ~safeZone(){}
    bool setSafeZone(float ldis, float lang, float rdis, float rang);
    void setSimpleSZ(float ldis, float lang, float rdis, float rang, int l_level, int r_level, float deld);
    float getdTheta(float dyaw, float& dist, float& dir);
    void getMidxyz(double &x, double &y, double &z);

//    void setSimpleSZxy(float xl, float yl, float xr, float yr);
//    float getdThetaxy(float dyaw, float& dist, float& dir);

//private:
    float leftP[2], rightP[2];
    float *near, *far;
    //safedir:0right,1left
    float safe_direction[2];
};

class p3atObstacleAvoid
{
public:
    p3atObstacleAvoid(ros::NodeHandle &_n);
	~p3atObstacleAvoid();

    //input with origin vel
    //after goal is reached, input findpath=false to reset
    //return true if the vel is modified
    bool modifyVel(float &vx, float &rz, float dyaw, bool findpath);
    void modifyVelBySonar(float &vx, float &rz);
    void modifyVelBySonarSafeZone(float &vx, float &rz);
    bool modifyVelByTof(float &vx, float &rz, float dyaw, bool findpath);
    bool modifyVelByLidar(float &vx, float &rz, float dyaw, bool findpath);
    bool modifyVelByLidar3d(float &vx, float &rz, float dyaw, bool findpath);
    //input: local target in body frame
    //in this way, you donot have to pub a target point to this node
    bool gotoLocalTarget(float &vx, float &rz, float ltpose[],float ltq[], bool findpath);
    void reset();

private:
	ros::NodeHandle nh;
    image_transport::ImageTransport it;
    ros::Subscriber sonarpc2_sub;
    ros::Subscriber tofpcl_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber lidar3d_sub;
    /// this msg use pose as topic, position is the safezone desired point in body frame
    /// and orientation.w is flag that wether exits a safezone(>0 yes, <0 no)
    /// and orientation.x is flag that wether vel is modified(>0 yes, <0 no)
    /// and orientation.y is flag that wether origin target is block(>0 yes, <0 no)
    ros::Publisher replan_pub;
    ros::Subscriber tarP_sub;

    ofstream mylog;

    bool isUsedSonar;
    bool isDanger;
    int justReplan;

    double max_vx, max_rz;

    //sonar
    pcl::PointCloud<pcl::PointXYZ> pc_now;
    vector< pcl::PointCloud<pcl::PointXYZ> > pc_vec;
    vector<double> time_sonar_past;
    double time_sonar_now;
    int cnt_sonar;

    //tof point cloud
    bool isUsedTof;
    bool isViewSFTof;
    pcl::PointCloud<pcl::PointXYZ> pctof_now;
    vector< pcl::PointCloud<pcl::PointXYZ> > pctof_vec;
    vector<double> time_tof_past;
    double time_tof_now;
    double start_height_of_cam;
    double half_fov;
    void pubSafeZone();
    ros::Publisher safezone_pub;
//    void showTofPC();
//    void initPclViewer();

    //lidar
    bool isUsedLidar;
    vector<float> rangelidar_now;
    vector< vector<float> > rangelidar_vec;
    vector<double> time_lidar_past;
    double time_lidar_now;
    vector<safeZone> safezone_vec;
    cv::Mat safezone_view;
    bool isViewSafezoneLidar;
    float ori_dir, mod_dir;
    //sensor_msgs::LaserScan ls_msg;
    void findSafeZone(const sensor_msgs::LaserScanConstPtr& msg);
    //return nearest safezone number in safezone_vec, -1 if vec is null
    int findNearestSafeZone(float dyaw, float &d_dir_safe, float &dist, float& dir, float& ddir_between_now_and_chosen_dir);
    double Tlidarbody[3];
    float last_desired_dir, center_dir_dist;
    double time_of_halfpi;

    //3d lidar point cloud
    bool isUsed3dLidar;
    double start_height_of_3dlidar;
    int detect_layer;
    double deadzone;

    //lidar with octree
    double dangle;
    int timesofdeld;
    double deld;
    double range_step_size;
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarxyzPC;
    unibn::Octree<pcl::PointXYZ> lidarxyzOct;
    vector<float> last_chosen_dir;
    void findSZOctree(const sensor_msgs::LaserScanConstPtr& msg);
    inline void distangleToxy(float dist, float angle, float& x, float& y);
    inline void xyTodistangle(float x, float y, float& dist, float& angle);
    bool suddenChangeExist(float rang,float lang, const sensor_msgs::LaserScanConstPtr& msg, vector<int> &sudden_v, int &start_id_of_sc);

    float a_sonar[8][2];
    float asonar;

    //for time
    double time_begin, time_end, _ti;
    //for calculate time consume of calculate potantial
    double time_calpotan_total, time_maxcalpo;
    int cnt_calpotan;
    bool shouldprinttime;

    //for replan
    pcl::PointXYZ targetP;
    float targetOrien[4];//w,x,y,z
    int targetPisBlocked;

    //smooth velocity
    float last_vx, last_rz;
    void smooth(float &vx, float &rz);

    //for ctrl with orientation
    //vx = c_dist * dist
    //rz = (a_theta+b_alpha)*new_dir - b_alpha*dyaw
    double a_theta, b_alpha, c_dist;

	void initSubscriber();
    void readParam();
    void sonarpc2Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void tofpclCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void lidarCallback(const sensor_msgs::LaserScanConstPtr& msg);
    void lidar3dCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void targetPCallback(const geometry_msgs::PoseConstPtr& msg);
};

#endif
