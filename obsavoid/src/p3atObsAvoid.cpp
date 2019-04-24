#include "p3atObsAvoid.h"

double OA_MAX_ANGULAR_SPEED = M_PI/2;
int row_img = 250, col_img = 500;
//tof 256 0.2
//lidar 10 0.05
//3dlidar 256 0.2
const unibn::OctreeParams& myOctParams = unibn::OctreeParams(10, false, 0.1f);
//100 grid = 1 meters
//boost::shared_ptr<pcl::visualization::PCLVisualizer> obsviewer (new pcl::visualization::PCLVisualizer ("Obstacle Viewer"));

p3atObstacleAvoid::p3atObstacleAvoid(ros::NodeHandle &_n):
    nh(_n), it(_n), cnt_sonar(0), asonar(0), isUsedSonar(false), isDanger(false),shouldprinttime(false), time_of_halfpi(1.25),
    isUsedLidar(false), isUsedTof(false), ori_dir(0), mod_dir(0), justReplan(0), last_desired_dir(0), center_dir_dist(2),
    last_vx(0), last_rz(0), lidarxyzPC(new pcl::PointCloud<pcl::PointXYZ>),
    max_vx(0.5), max_rz(M_PI_2),
    targetOrien{1,0,0,0}
{
    //it = new image_transport::ImageTransport(nh);
    readParam();
    initSubscriber();
    if(isViewSafezoneLidar && isUsedLidar){
    safezone_view = cv::Mat(row_img, col_img, CV_8UC3, cv::Scalar(255,255,255)).clone();
    }
    else if(isViewSFTof && isUsedTof){
//        safezone_view = cv::Mat(row_img, col_img, CV_8UC3, cv::Scalar(255,255,255)).clone();
//        initPclViewer();
        safezone_view = cv::Mat(5, 5, CV_8UC3, cv::Scalar(255,255,255)).clone();
    }
}

p3atObstacleAvoid::~p3atObstacleAvoid()
{
    mylog.close();
}

void p3atObstacleAvoid::reset(){
    float tmp=0;
    modifyVel(tmp,tmp,0,false);
}

bool p3atObstacleAvoid::gotoLocalTarget(float &vx, float &rz, float ltpose[],float ltq[], bool findpath){
    targetP.x = ltpose[0];
    targetP.y = ltpose[1];
    targetP.z = ltpose[2];
    targetOrien[0] = ltq[0];
    targetOrien[1] = ltq[1];
    targetOrien[2] = ltq[2];
    targetOrien[3] = ltq[3];
//    cout<<"px:"<<ltpose[0]<<",py:"<<ltpose[1]<<",pz:"<<ltpose[2]<<
//          ",qw:"<<ltq[0]<<",qx:"<<ltq[1]<<",qy:"<<ltq[2]<<",qz:"<<ltq[3]<<endl;
    double dist = sqrt(pow(ltpose[0],2) + pow(ltpose[1],2) + pow(ltpose[2],2));
    vx = dist / 4;
    if(vx > max_vx){
        vx = max_vx;
    }
    float dyaw = atan2(ltpose[1], ltpose[0]);
    rz = dyaw/M_PI_2*max_rz;
    if(rz > max_rz){
        rz = max_rz;
    }
    else if(rz < -max_rz){
        rz = -max_rz;
    }
    modifyVel(vx, rz, dyaw, findpath);
    if(dist < 0.1)
        return false;
    else
        return true;
}

bool p3atObstacleAvoid::modifyVel(float &vx, float &rz, float dyaw, bool findpath){
    //first is restrict force, second is attract force
    //time_begin = clock();
    int64_t start1=0,end1=0;
    //struct timeval tstart,tend;
    //double timer;
    //gettimeofday(&tstart,NULL);
    //time_begin = ros::Time::now().toSec();
    //int64_t tb = clock();
    start1 = cv::getTickCount();
    bool res;
    if(isUsedLidar){
        //modifyVelBySonar(vx, rz);
        res = modifyVelByLidar(vx, rz, dyaw, findpath);
    }
    else if(isUsedTof){
        //modifyVelBySonarSafeZone(vx, rz);
        res = modifyVelByTof(vx, rz, dyaw, findpath);
    }
    else if(isUsed3dLidar){
        res = modifyVelByLidar3d(vx, rz, dyaw, findpath);
    }
    smooth(vx, rz);
    //time_end = ros::Time::now().toSec();//clock();
    //_ti = time_end - time_begin;//((double)(time_end - time_begin) / CLOCKS_PER_SEC);
    //int64_t te = clock();
    //double ttti = ((double)(te - tb) / CLOCKS_PER_SEC);
    end1 = cv::getTickCount();
    _ti = 1000*double(end1 - start1)/cv::getTickFrequency();
    //gettimeofday(&tend,NULL);
    //timer=(tend.tv_sec-tstart.tv_sec)+(tend.tv_usec-tstart.tv_usec)/1000000;
    if(findpath){
        if(!shouldprinttime){
            shouldprinttime = true;
            time_maxcalpo = 0;
            cnt_calpotan = 0;
            time_calpotan_total = 0;
        }
        else{
            if(_ti > time_maxcalpo){
                time_maxcalpo = _ti;
            }
            //mylog<<"cal obs time = "<<_ti<<"(ros), "<<ttti<<"(oct), "<<tcvi<<"(cv) "<<timer<<"(hp) "
            //<<"second in loop "<<cnt_calpotan<<endl<<endl;
            mylog<<"cal obs time = "<<_ti<<"(cv) "<<"ms in loop "<<cnt_calpotan<<endl<<endl;
            time_calpotan_total += _ti;
            ++cnt_calpotan;
        }
    }
    else{
        if(shouldprinttime){
            cout << "adverage calculate obs time:" << time_calpotan_total/cnt_calpotan
                 << " ms, max calculate time:" << time_maxcalpo <<" ms."<< endl;
            shouldprinttime=false;
            cnt_calpotan = 0;
            time_calpotan_total = 0;
            time_maxcalpo = 0;
            //reset some var
            last_rz = 0;
            last_vx = 0;
            last_desired_dir = 0;
        }
    }
    return res;
}

bool p3atObstacleAvoid::modifyVelByLidar3d(float &vx, float &rz, float dyaw, bool findpath){
    bool res = modifyVelByLidar(vx, rz, dyaw, findpath);
    pubSafeZone();
    return res;
}

bool p3atObstacleAvoid::modifyVelByLidar(float &vx, float &rz, float dyaw, bool findpath){
    geometry_msgs::Pose msg;
    
    //findSafeZone(&ls_msg);
    if(vx<=0){
        vx = 0;
        return false; //avoid move backward
    } 
    
    if(fabs(dyaw) > half_fov){ //move direction is out of detection
        vx = 0.1;
        return true;
    }
    float err_dyaw_safe, dist, newdir, ddir_between_now_and_chosen_dir;
    //ROS_INFO("[OBSAVOID]1");
    int num_of_nearest_safe = findNearestSafeZone(dyaw, err_dyaw_safe, dist, newdir, ddir_between_now_and_chosen_dir);
    //ROS_INFO("[OBSAVOID]2");
    mylog << "modify:"<<endl<<"origin vx="<<vx<<", rz="<<rz
          <<", dyaw="<<dyaw/M_PI*180<<", err_dyaw_safe="<<err_dyaw_safe/M_PI*180<<", near safezone dist="<<dist<<", newdir="<<newdir/M_PI*180<<", ddir_between_now_and_chosen_dir="<<ddir_between_now_and_chosen_dir/M_PI*180<<endl;

    if(isViewSafezoneLidar){
    //cv::namedWindow("obstacleview");
    cv::imshow("obstacleview", safezone_view);
//    if(safezone_vec.size()==2 && safezone_vec[0].leftP[1]>safezone_vec[1].rightP[1]){
//        cv::waitKey();
//    }
    cv::waitKey(1);
    }

    //ROS_INFO("[OBSAVOID]3");
    if(num_of_nearest_safe == -1 && findpath){
        if(justReplan==0){
            //ROS_INFO("[OBSAVOID]no near safe zone!");
            mylog<<ros::Time::now().toSec();
            mylog<<">>>>>>>>>>>>>no near safe zone and research path."<<endl;
            ROS_INFO("[OBS]no near safe zone and research path.");
            //ai_robot::restart_nav srv;
            //srv.request.in = true;
            msg.orientation.w = -1;
            msg.orientation.x = -1;
            msg.orientation.y = targetPisBlocked;
            msg.position.x = 0;
            msg.position.y = 0;
            msg.position.z = 0;
            replan_pub.publish(msg);
            justReplan = 1000;
            return false;//replan_cli.call(srv);
        }
        else{
            if(justReplan>0){
                --justReplan;
            }
            //vx = 0;
            return false;
        }
    }
    else{
        if(justReplan>0){
            --justReplan;
        }
        
        //method 1: no orientation
        if(fabs(err_dyaw_safe) < 0.00001){
            //?
            newdir = newdir + dyaw;
            //mylog<<"no modify"<<endl;
        //    return true;//false;
        }
//        //else{
//            float dlast_new_dir = newdir - last_desired_dir;
//            float ddd = M_PI/100;
//            if(dlast_new_dir > ddd)
//            {
//                newdir = last_desired_dir + ddd;
//            }
//            else if(dlast_new_dir < -ddd)
//            {
//                newdir = last_desired_dir - ddd;
//            }
//            last_desired_dir = newdir;
//            rz = newdir/M_PI*OA_MAX_ANGULAR_SPEED*2;
//            /*if(dist < 0.5 || isDanger){
//                vx = 0;
//            }
//            else*/ //if(dist < 2){
           float tmpvx;
           if(center_dir_dist > 2)
           {
               tmpvx = 0.5;
           }
           else
           {
               tmpvx = center_dir_dist / 4;
           }
           if(tmpvx < vx)
               vx = tmpvx;
//            if(vx < 0.3 && center_dir_dist > 1)//
//                vx = 0.3;
//            vx = vx * (1 - fabs(newdir)*2/M_PI);//(5.5-) *dist/2.5 here should use turning radius
            
//            //}
//            /* not finished
//            msg.orientation.w = 1;
//            msg.orientation.x = 1;
//            msg.orientation.y = targetPisBlocked;
//            safezone_vec[num_of_nearest_safe].getMidxyz(msg.position.x, msg.position.y, msg.position.z);
//            replan_pub.publish(msg);
//            */
//        //}
//        if(fabs(ddir_between_now_and_chosen_dir) > 0.00001 && fabs(dlast_new_dir)<ddd){
//            // chosen dir is not robot's present dir, so it needs a min turning speed
//            if(rz < 0.15 && rz > 0)
//            {
//                rz = 0.15;
//            }
//            else if(rz > -0.15 && rz < 0)
//            {
//                rz = -0.15;
//            }
//        }

        //method 2: with orientation
        float target_yaw, tmproll, tmppitch;
        if(fabs(targetOrien[0])+fabs(targetOrien[1])+fabs(targetOrien[2])+
                fabs(targetOrien[3]) < 0.1){
            target_yaw = 0;
        }
        else{
            qToEuler(tmproll, tmppitch, target_yaw, targetOrien);
        }
        // cout<<dist<<","<<newdir<<","<<target_yaw<<endl;
        //vx = c_dist * center_dir_dist;
        rz = (a_theta + b_alpha)*newdir - b_alpha*target_yaw;
        if(vx > 0.5){
            vx = 0.5;
        }
        else if(vx < 0.0){
            vx = 0.0;
        }
        if(rz > M_PI/2){
            rz = M_PI/2;
        }
        else if(rz < -M_PI/2){
            rz = -M_PI/2;
        }
        mylog<<"after vx="<<vx<<", rz="<<rz<<endl;
    }
    if(isViewSafezoneLidar){
        int imglx = col_img/2 - int(3*sin(newdir)*row_img/5);
        int imgly = row_img- 1 - int(3*cos(newdir)*row_img/5);
        int imgrx = col_img/2;
        int imgry = row_img- 1;
        cv::line(safezone_view, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(123,222,13), 2);
        cv::imshow("obstacleview", safezone_view);
        cv::waitKey(1);
    }
    //ROS_INFO("[OBSAVOID]end");
    return true;
}

void p3atObstacleAvoid::modifyVelBySonarSafeZone(float &vx, float &rz){
    if(!isUsedSonar)
        return;

    float deltaDistance[8], distance_now[8], minD=100;
    bool safezone[8];
    if(vx > 0.01){
        for(int i=0; i<8; ++i){
            distance_now[i] = sqrt(pow(pc_now.points[i].x, 2) + pow(pc_now.points[i].y, 2));
            if(distance_now[i] < minD){
                minD = distance_now[i];
            }
        }
        for(int i=0; i<8; ++i){
            float deltaD = distance_now[i] - minD;
            if(deltaD > 1){
                safezone[i] = true;
            }
            else{
                safezone[i] = false;
            }
        }
    }
}

void p3atObstacleAvoid::modifyVelBySonar(float &vx, float &rz)
{
    if(!isUsedSonar)
        return;

    float deltaDistance[8], distance_now[8], minLeftD=100, minRightD=100;
    bool isGoingNear[8];
    int nearestLeftPoint=-1, nearestRightPoint=-1;
    if(vx > 0.01){
        mylog<<"originvx:"<<vx<<"; rz:"<<rz<<endl;
        //ROS_INFO("OO");
        //printf("%f, %f, %f, %f, %f, %f, %f, %f\n",pc_vec[0].points[0].y,pc_vec[0].points[1].y,pc_vec[0].points[2].y,pc_vec[0].points[3].y,pc_vec[0].points[4].y,pc_vec[0].points[5].y,pc_vec[0].points[6].y,pc_vec[0].points[7].y);
        //printf("%f, %f, %f, %f, %f, %f, %f, %f\n",pc_now.points[0].y,pc_now.points[1].y,pc_now.points[2].y,pc_now.points[3].y,pc_now.points[4].y,pc_now.points[5].y,pc_now.points[6].y,pc_now.points[7].y);
/*mylog<<"move"<<endl;
for(int i=0;i<8;++i){
    mylog<<"("<<pc_vec[0].points[i].x<<","<<pc_vec[0].points[i].y<<")";
}*/

        for(int i=0; i<8; ++i){
            double tt=sqrt(pow(pc_vec[0].points[i].x, 2) + pow(pc_vec[0].points[i].y, 2));
            mylog<<tt<<" ";
        }
        mylog<<endl;
        for(int i=0; i<4; ++i){
            distance_now[i] = sqrt(pow(pc_now.points[i].x, 2) + pow(pc_now.points[i].y, 2));
            deltaDistance[i] = distance_now[i] -
                    sqrt(pow(pc_vec[0].points[i].x, 2) + pow(pc_vec[0].points[i].y, 2));
            float _tmp = fabs(deltaDistance[i]);
            mylog <<distance_now[i]<<" ";
            if(deltaDistance[i] < -0.01 && _tmp < 1.7 && distance_now[i] < 3){
                isGoingNear[i] = true;
                //choose fastest closing obs
                if(_tmp < minLeftD){
                    minLeftD = _tmp;
                    nearestLeftPoint = i;
                }
            }
            else
                isGoingNear[i] = false;
        }
        for(int i=4; i<8; ++i){
            distance_now[i] = sqrt(pow(pc_now.points[i].x, 2) + pow(pc_now.points[i].y, 2));
            deltaDistance[i] = distance_now[i] -
                    sqrt(pow(pc_vec[0].points[i].x, 2) + pow(pc_vec[0].points[i].y, 2));
            float _tmp = fabs(deltaDistance[i]);
            mylog <<distance_now[i]<<" ";
            if(deltaDistance[i] < -0.01 && _tmp < 1.7 && distance_now[i] < 3){
                isGoingNear[i] = true;
                //choose fastest closing obs
                if(_tmp < minRightD){
                    minRightD = _tmp;
                    nearestRightPoint = i;
                }
            }
            else
                isGoingNear[i] = false;
        }
        //deltaV = a * dD / D
        mylog<<endl<<"near l point:"<<nearestLeftPoint<<"; near r point:"<<nearestRightPoint<<endl;
        float delvx = 0, delrz = 0;
        if(nearestLeftPoint!=-1 && (nearestRightPoint==-1 || minLeftD<minRightD)){//distance_now[nearestLeftPoint]<distance_now[nearestRightPoint]
            delvx += asonar*(a_sonar[nearestLeftPoint][0] * deltaDistance[nearestLeftPoint]
                    / distance_now[nearestLeftPoint] / fabs(time_sonar_now - time_sonar_past[0]));
            //vx += delvx;pc_now.points[nearestLeftPoint].xpc_now.points[nearestLeftPoint].y
            delrz += asonar*(a_sonar[nearestLeftPoint][1] * deltaDistance[nearestLeftPoint]
                    / distance_now[nearestLeftPoint] / fabs(time_sonar_now - time_sonar_past[0]));
            //rz += delrz;
            mylog<<"l delta d:"<<deltaDistance[nearestLeftPoint]
                   <<"; time:"<<fabs(time_sonar_now - time_sonar_past[0])<<endl
                   <<"dvx:"<<delvx<<"; drz:"<<delrz<<endl;/**/
        }
        else if(nearestRightPoint!=-1 && (nearestLeftPoint==-1 || minRightD<minLeftD)){//distance_now[nearestRightPoint]<distance_now[nearestLeftPoint]
            delvx += asonar*(a_sonar[nearestRightPoint][0] * deltaDistance[nearestRightPoint]
                    / distance_now[nearestRightPoint] / fabs(time_sonar_now - time_sonar_past[0]));
            //vx += delvx;pc_now.points[nearestRightPoint].xpc_now.points[nearestRightPoint].y
            delrz += asonar*(a_sonar[nearestRightPoint][1] * deltaDistance[nearestRightPoint]
                    / (-distance_now[nearestRightPoint]) / fabs(time_sonar_now - time_sonar_past[0]));
            //rz += delrz;
            mylog<<"r delta d:"<<deltaDistance[nearestRightPoint]
                   <<"; time:"<<fabs(time_sonar_now - time_sonar_past[0])<<endl
                   <<"dvx:"<<delvx<<"; drz:"<<delrz<<endl;/**/
        }
        if(false){//isUsedSonar
        if((delrz*rz)<0){
            rz = -rz;//delrz = asonar*delrz;
        }
        else{
            rz += delrz;
        }
        if(distance_now[0]<distance_now[7] && distance_now[0]<0.45){
            rz = -0.15;
        }
        else if(distance_now[7]<distance_now[0] && distance_now[7]<0.45){
            rz = 0.15;
        }
        vx += delvx;

        if(vx < 0){
            vx = 0;
        }
        }
        if(distance_now[2]<0.7 || distance_now[3]<0.9 || distance_now[4]<0.9
           || distance_now[5]<0.7){// || distance_now[0]<0.4 || distance_now[7]<0.4){
            vx = 0;
            rz = 0;
        }
        mylog<<"finalvx:"<<vx<<"; rz:"<<rz<<endl;
    }
    else if(vx < -0.01){}
    mylog<<endl;
}

bool p3atObstacleAvoid::modifyVelByTof(float &vx, float &rz, float dyaw, bool findpath){
    /*if(vx < 0) return;
    if(fabs(dyaw) > M_PI/3){
        vx = 0.1;
        return;
    }*/
    //ROS_INFO("0000");
//    cout<<"points------"<<isViewSFTof<<endl;
//    if(isViewSFTof){
//        cv::imshow("obstacleview", safezone_view);
//        cv::waitKey(1);
//        showTofPC();
//    }

    bool res = modifyVelByLidar(vx, rz, dyaw, findpath);
    pubSafeZone();
    //ROS_INFO("4444");
    return res;
}

void p3atObstacleAvoid::initSubscriber()
{
    constexpr char kPointCloud2Topic[] = "/points";
    //replan_cli = nh.serviceClient<ai_robot::restart_nav>("/ai_robot/restart_nav");
    replan_pub = nh.advertise<geometry_msgs::Pose>("/ai_robot/restart_nav", 1);
    tarP_sub = nh.subscribe("/ai_robot/findpath/targetP", 1, &p3atObstacleAvoid::targetPCallback, this);
    if(isUsedLidar){
        lidar_sub = nh.subscribe("/base_scan", 1, &p3atObstacleAvoid::lidarCallback, this);
    }
    else if(isUsedTof){
        ///smarttof/pointcloud
        safezone_pub = nh.advertise<std_msgs::Float32MultiArray>("/obstacle/safezone", 1);
        tofpcl_sub = nh.subscribe(kPointCloud2Topic, 1, &p3atObstacleAvoid::tofpclCallback, this);
    }
    else if(isUsedSonar){
        sonarpc2_sub = nh.subscribe("/RosAria/sonar_pointcloud2", 1, &p3atObstacleAvoid::sonarpc2Callback, this);
    }
    else if(isUsed3dLidar){
        safezone_pub = nh.advertise<std_msgs::Float32MultiArray>("/obstacle/safezone", 1);
        lidar3d_sub = nh.subscribe(kPointCloud2Topic, 1, &p3atObstacleAvoid::lidar3dCallback, this);
    }
}

void p3atObstacleAvoid::readParam(){
    nh.getParam("usesonar", isUsedSonar);
    nh.getParam("uselidar", isUsedLidar);
    nh.getParam("usetof", isUsedTof);
    nh.getParam("use3dlidar", isUsed3dLidar);

    nh.getParam("a00", a_sonar[0][0]);
    nh.getParam("a01", a_sonar[0][1]);
    nh.getParam("a10", a_sonar[1][0]);
    nh.getParam("a11", a_sonar[1][1]);
    nh.getParam("a20", a_sonar[2][0]);
    nh.getParam("a21", a_sonar[2][1]);
    nh.getParam("a30", a_sonar[3][0]);
    nh.getParam("a31", a_sonar[3][1]);
    nh.getParam("a30", a_sonar[4][0]);
    nh.getParam("a31", a_sonar[4][1]);
    nh.getParam("a20", a_sonar[5][0]);
    nh.getParam("a21", a_sonar[5][1]);
    nh.getParam("a10", a_sonar[6][0]);
    nh.getParam("a11", a_sonar[6][1]);
    nh.getParam("a00", a_sonar[7][0]);
    nh.getParam("a01", a_sonar[7][1]);
    nh.getParam("asonar", asonar);

    nh.param("max_vx", max_vx, 0.5);
    nh.param("max_rz", max_rz, M_PI_2);

    nh.getParam("viewsafezonelidar", isViewSafezoneLidar);
    nh.getParam("viewsafezonetof", isViewSFTof);
    string logfile;
    nh.getParam("obslog", logfile);
    double a;
    nh.param("lx", a, 0.0);
    Tlidarbody[0] = a;
    nh.param("ly", a, 0.0);
    Tlidarbody[1] = a;
    nh.param("lz", a, 0.0);
    Tlidarbody[2] = 0;
    if(isUsedTof){
        nh.param("toffov", half_fov, 0.52);
        nh.param("sheight_of_cam", start_height_of_cam, -0.42);
        nh.param("obs_timesofdeld_tof", timesofdeld, 4);
        nh.param("obs_dangle_tof", dangle, 0.058);
        nh.param("deld_tof", deld, 0.46);
        nh.param("detect_layer", detect_layer, 1);
        nh.param("deadzone", deadzone, 0.2);
        nh.param("range_step_size", range_step_size, 0.2);
    }
    else if(isUsedLidar){
        half_fov = M_PI/2-0.17;
        nh.param("obs_timesofdeld", timesofdeld, 4);
        nh.param("obs_dangle", dangle, 0.058);
        nh.param("deld", deld, 0.5);
        nh.param("deadzone", deadzone, 0.2);
        nh.param("range_step_size", range_step_size, 0.2);
    }
    else if(isUsed3dLidar){
        half_fov = M_PI/2;
        nh.param("sheight_of_3dlidar", start_height_of_3dlidar, -0.42);
        nh.param("obs_timesofdeld", timesofdeld, 4);
        nh.param("obs_dangle", dangle, 0.058);
        nh.param("deld", deld, 0.5);
        nh.param("detect_layer", detect_layer, 1);
        nh.param("deadzone", deadzone, 0.2);
        nh.param("range_step_size", range_step_size, 0.2);
    }

    nh.param("OA_MAX_ANGULAR_SPEED", OA_MAX_ANGULAR_SPEED, M_PI/2);
    nh.param("a_theta", a_theta, 0.5);
    nh.param("b_alpha", b_alpha, 0.1);
    nh.param("c_dist", c_dist, 0.25);

    mylog.open(logfile);
}
/*
void p3atObstacleAvoid::showTofPC(){
    cout<<"points:"<<lidarxyzPC->size()<<endl;
//    if(firstViewObs){ //&& lidarxyzPC->size()>0
//        firstViewObs = false;
//    }
//    else{
    ROS_INFO("1111");
        obsviewer->removePointCloud("obstacle");
//    }
        ROS_INFO("2222");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(lidarxyzPC, 255, 0, 0);
    obsviewer->addPointCloud(lidarxyzPC, colorHandler, "obstacle");
    ROS_INFO("3333");
//    obsviewer->updatePointCloud<pcl::PointXYZ>(lidarxyzPC, "obstacle");
    obsviewer->spinOnce(1);
    ROS_INFO("4444");
}
void p3atObstacleAvoid::initPclViewer()
{
    obsviewer->initCameraParameters();
    obsviewer->createViewPort(0.0,0.0,1.0,1.0,vv1);
    obsviewer->setBackgroundColor (255, 255, 255, vv1);
    obsviewer->addCoordinateSystem(1.0, "Obstacle Viewer", vv1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(lidarxyzPC, 255, 0, 0);
    obsviewer->addPointCloud(lidarxyzPC, colorHandler, "obstacle");
}
*/
void p3atObstacleAvoid::sonarpc2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pc_now.clear();
    pcl::fromROSMsg(*msg, pc_now);
    time_sonar_now = ros::Time::now().toSec();
    //every time only one of 16 sonar data is updated
    if(++cnt_sonar >= 16){
        cnt_sonar = 0;
        time_sonar_past.push_back(time_sonar_now);
        pc_vec.push_back(pc_now);
/*mylog<<endl<<"callback"<<endl;
for(int i=0;i<8;++i){
    mylog<<"("<<pc_vec[0].points[i].x<<","<<pc_vec[0].points[i].y<<")";
}
ROS_INFO("KK");
printf("%f, %f, %f, %f, %f, %f, %f, %f\n",pc_vec[0].points[0].y,pc_vec[0].points[1].y,pc_vec[0].points[2].y,pc_vec[0].points[3].y,pc_vec[0].points[4].y,pc_vec[0].points[5].y,pc_vec[0].points[6].y,pc_vec[0].points[7].y);
printf("%f, %f, %f, %f, %f, %f, %f, %f\n",pc_now.points[0].y,pc_now.points[1].y,pc_now.points[2].y,pc_now.points[3].y,pc_now.points[4].y,pc_now.points[5].y,pc_now.points[6].y,pc_now.points[7].y);*/
        if(pc_vec.size() > 1){
            pc_vec.erase(pc_vec.begin());
            time_sonar_past.erase(time_sonar_past.begin());
        }
//mylog<<endl;

/*for(int i=0;i<8;++i){
    mylog<<"("<<pc_vec[0].points[i].x<<","<<pc_vec[0].points[i].y<<")";
}*/
    }
}

//3d lidar
void p3atObstacleAvoid::lidar3dCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    //ROS_INFO("HELLO3DLIDAR");
    // init point cloud
    lidarxyzPC->clear();
    safezone_vec.clear();
    pcl::fromROSMsg(*msg, *lidarxyzPC);
    vector<int> nouse;
    pcl::removeNaNFromPointCloud(*lidarxyzPC, *lidarxyzPC, nouse);
    lidarxyzOct.initialize(lidarxyzPC->points, myOctParams);
    //ROS_INFO("init octree");
    pcl::PointXYZ tmpP(0,0,0);
    bool obsexist = false, snstart = false;
    safeZone tmpsz;
    cv::Point2f start_point(2.03,0), end_point(2.03,0);
    int last_dist_level=timesofdeld-1;
    int cnt_safescan=0;
    int i=0, llevel=0, rlevel=0;
    double _angle;
    int closest_level;
    float midangle = 1;
    vector<int> all_levels;
    float targetdir = atan2(targetP.y, targetP.x), minerr = 100;
    int num_of_dir = -1;
    // find safezone
    int cnttest=0;
    for(_angle = -half_fov; _angle <= half_fov; _angle += dangle){
        float delx = deld*cos(_angle), dely = deld*sin(_angle);
        float firstx = deadzone*cos(_angle), firsty = deadzone*sin(_angle);//deadzone
        closest_level = timesofdeld;
        obsexist = false;
        bool jumpout=false;
        tmpP.x = delx/2 - Tlidarbody[0] + firstx;
        tmpP.y = dely/2 - Tlidarbody[1] + firsty;
            //cout<<"search("<<tmpP.x<<","<<tmpP.y<<","<<tmpP.z<<endl;
        for(i=0; i<timesofdeld; ++i){
             for(int j=0; j<detect_layer; ++j){
                //search circle center
                tmpP.z = start_height_of_3dlidar + deld*j*0.8;
                //cout<<"search("<<tmpP.x<<","<<tmpP.y<<","<<tmpP.z<<"),r="<<deld/2<<",flag="<<fff<<endl;
                //mylog<<"testball:"<<tmpP.x<<","<<tmpP.y<<","<<tmpP.z<<";";
                if(lidarxyzOct.radiusNeighborsSimple<unibn::L2Distance<pcl::PointXYZ> >(tmpP, deld/2)){
                    obsexist = true;
                    jumpout = true;
                    if(i<closest_level){
                        closest_level = i;
                    }
                    break;
                }        
            }
            if(jumpout)
                break;
            tmpP.x += delx;
            tmpP.y += dely;
            //cout<<"search balls:"<<i<<" of "<<timesofdeld<<" in dir "<<_angle<<endl;
        }
        if(fabs(_angle) < midangle){
            center_dir_dist = closest_level * range_step_size + deadzone;
            midangle = fabs(_angle);
        }
        all_levels.push_back(closest_level);
        mylog<<_angle/M_PI*180<<":"<<closest_level<<",";
        if(++cnttest%10==0)
            mylog<<endl;
        if(fabs(_angle-targetdir) < minerr){
            num_of_dir = all_levels.size()-1;
            minerr = fabs(_angle-targetdir);
        }
        if(snstart){
            if(cnt_safescan > 0)
                ++cnt_safescan;
            if(closest_level < (last_dist_level) || (last_dist_level>=timesofdeld && closest_level < last_dist_level)){
                //find a safezone
                snstart = false;
                //if(cnt_safescan>2){
                    end_point.x = closest_level*deld + deadzone;
                    end_point.y = _angle - dangle;
                    llevel = closest_level;
                    tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
                    safezone_vec.push_back(tmpsz);
                //}
                
            }
            else if(closest_level > last_dist_level){
                start_point.x = last_dist_level*deld + deadzone;
                start_point.y = _angle;
                rlevel = last_dist_level;
                if(closest_level > (last_dist_level+1)){
                    cnt_safescan=1;
                }
                else
                {
                    cnt_safescan=0;
                }
            }
        }
        else{
            //no obs or sudden start
            if((closest_level > (last_dist_level)) || !obsexist){
                snstart = true;
                start_point.x = last_dist_level*deld + deadzone;
                start_point.y = _angle;
                rlevel = closest_level;
                if(closest_level > (last_dist_level+1)){
                    cnt_safescan=1;
                }
                else
                {
                    cnt_safescan=0;
                }
            }
            else if(obsexist){
                if(closest_level < (last_dist_level-1)){
                    end_point.x = closest_level*deld + deadzone;
                    end_point.y = _angle - dangle;
                    llevel = closest_level;
                    int tmplevel = last_dist_level, tmpcnt = all_levels.size()-2;
                    while(tmplevel >= last_dist_level){
                        tmplevel = all_levels[tmpcnt];
                        --tmpcnt;
                        if(tmpcnt<0)
                            break;
                    }
                    start_point.x = tmplevel*deld + deadzone;
                    start_point.y = dangle*(tmpcnt+1) - M_PI/2;
                    rlevel = tmplevel;
                }
            }
        }
        last_dist_level = closest_level;
        //mylog<<endl<<_angle*180/M_PI<<":"<<closest_level<<endl;
    }
    mylog<<endl;
    if(snstart && last_dist_level>=timesofdeld-1){
        //cout<<"last level:"<<last_dist_level<<endl;
        end_point.x = 2;
        end_point.y = _angle - dangle;
        llevel = i;
        tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
        safezone_vec.push_back(tmpsz);
    }
    pcl::PointXYZ self2(0,0,start_height_of_3dlidar), self1(0,0,start_height_of_3dlidar+deld*0.8),
            tarP2(targetP.x,targetP.y,start_height_of_3dlidar), tarP1(targetP.x,targetP.y,start_height_of_3dlidar+deld*0.8);
    if(lidarxyzOct.isBlock<unibn::L2Distance<pcl::PointXYZ> >(tarP2, self2, 0.6) ||
       lidarxyzOct.isBlock<unibn::L2Distance<pcl::PointXYZ> >(tarP1, self1, 0.6)){
        targetPisBlocked = 1;
    }
    else{
        targetPisBlocked = -1;
        // targetP is not blocked, add a safezone of targetP
        int level = all_levels[num_of_dir], nextlevel = level, step = 0;
        if(level >= 1){
            while(nextlevel == level && step < 15 && (num_of_dir-step)>0){
                nextlevel = all_levels[num_of_dir-step];
                ++step;
            }
            if(level <= nextlevel){
                nextlevel = level-1;
            }
            start_point.x = (0.5 + nextlevel)*deld;
            start_point.y = -M_PI/2 + dangle*(num_of_dir-step+1);
            llevel = level -1;
            step = 0;
            nextlevel = level;
            while(nextlevel == level && step < 15 && (num_of_dir+step)<all_levels.size()){
                nextlevel = all_levels[num_of_dir+step];
                //cout<<"level:"<<level<<",nextlevel:"<<nextlevel<<endl;
                ++step;
            }
            if(level <= nextlevel){
                nextlevel = level-1;
            }
            end_point.x = (0.5 + nextlevel)*deld;
            end_point.y = -M_PI/2 + dangle*(num_of_dir+step-1);
            rlevel = level - 1;
            tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
            //safezone_vec.push_back(tmpsz);
            safezone_vec.insert(safezone_vec.begin(), tmpsz);
        }
    }
    //ROS_INFO("endoct");
}

//camera frame<->body frame
void p3atObstacleAvoid::tofpclCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    lidarxyzPC->clear();
    safezone_vec.clear();
//    safezone_view = cv::Mat(row_img, col_img, CV_8UC3, cv::Scalar(255,255,255)).clone();
//    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 50, cv::Scalar(100,100,0),1);
//    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 100, cv::Scalar(100,100,0),1);
//    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 150, cv::Scalar(100,100,0),1);
//    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 200, cv::Scalar(100,100,0),1);
//    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 400, cv::Scalar(100,100,0),1);
    // ROS_INFO("get one frame");
    pcl::PointCloud<pcl::PointXYZ> tmppc;
    pcl::fromROSMsg(*msg, *lidarxyzPC);//tmppc
    vector<int> nouse;
    vector<int> all_levels;
    pcl::removeNaNFromPointCloud(*lidarxyzPC, *lidarxyzPC, nouse);
    // ROS_INFO("pre deal");
//    cout<<msg->data[0]<<",,,"<<lidarxyzPC->points[0].x<<"---"<<endl;
//    cout<<msg->data.size()<<" "<<lidarxyzPC->size()<<endl;
//    printf("testP(%f,%f,%f)(%f,%f,%f)(%f,%f,%f)(%f,%f,%f)\n",
//           lidarxyzPC->points[0].x, lidarxyzPC->points[0].y, lidarxyzPC->points[0].z,
//            lidarxyzPC->points[1280].x, lidarxyzPC->points[1280].y, lidarxyzPC->points[1280].z,
//            lidarxyzPC->points[640].x, lidarxyzPC->points[640].y, lidarxyzPC->points[640].z,
//            lidarxyzPC->points[960].x, lidarxyzPC->points[960].y, lidarxyzPC->points[960].z);

    //choose less points
    pcl::PointXYZ tmpP(0,0,0);
//    if(isViewSFTof){
//        cout<<"here"<<endl;
//        for(int i=lidarxyzPC->size()-1; i>0; --i){
//            if(fabs(lidarxyzPC->points[i].y)<0.5){
//                int imgx, imgy;
//                imgx = col_img/2 + int(lidarxyzPC->points[i].x*row_img/5);
//                imgy = row_img- 1 - int(lidarxyzPC->points[i].z*row_img/5);
//                if(imgx>=0 && imgx<col_img && imgy>=0 && imgy<row_img){
//                    cv::circle(safezone_view, cv::Point(imgx, imgy), 1, cv::Scalar(0,0,0),2);
//                }
//            }
//        }
//    }
//    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
//    transformation(0, 0) = 0;
//    transformation(0, 2) = 1;
//    transformation(1, 0) = -1;
//    transformation(1, 1) = 0;
//    transformation(2, 1) = -1;
//    transformation(2, 2) = 0;
    //pcl::transformPointCloud(tmppc,*lidarxyzPC, transformation);
    lidarxyzOct.initialize(lidarxyzPC->points, myOctParams);
    // ROS_INFO("init octree");
/**/
    //std::vector<uint32_t> res;
    bool obsexist = false, snstart = false;
    //float deld=0.46;
    //int timesofdeld=4;
    safeZone tmpsz;
    cv::Point2f start_point(2.03,0), end_point(2.03,0);  //this store x and y
    //every 10/3 degree
    //float dangle = 0.174/3;lastobspoint=0,
    int last_dist_level=timesofdeld-1;
    int cnt_safescan=0;
    int i=0, llevel=0, rlevel=0;
    float midangle = 1;
    double _angle;
    int num_of_dir = -1;
    float targetdir = atan2(targetP.y, targetP.x), minerr = 100;
    int closest_level;
    for(_angle = -half_fov; _angle <= half_fov; _angle += dangle){
        // cout<<endl<<"angle:"<<_angle<<", ";
        float delx = deld*cos(_angle), dely = deld*sin(_angle);
        float firstx = (deadzone)*cos(_angle), firsty = (deadzone)*sin(_angle);
        //two layers
        obsexist = false;
        bool jumpout=false;
        closest_level = timesofdeld;
        tmpP.z = delx/2 - Tlidarbody[0] + firstx;
        tmpP.x = -dely/2 - Tlidarbody[1] + firsty;
        for(i=0; i<timesofdeld; ++i){
            //res.clear();
            for(int j=0; j<detect_layer; ++j){
                tmpP.y = -start_height_of_cam - deld*j;
                // cout<<"P("<<tmpP.x<<","<<tmpP.y<<","<<tmpP.z<<")-";
                if(lidarxyzOct.radiusNeighborsSimple<unibn::L2Distance<pcl::PointXYZ> >(tmpP, deld/2)){
                    obsexist = true;
                    jumpout = true;
                    if(i<closest_level){
                        closest_level = i;
                    }
                    break;
                }
//                if(res.size() > 20){
//                    obsexist = true;
//                    if(i<closest_level){
//                        closest_level = i;
//                    }
//                    break;
//                }
            }
            if(jumpout)
                break;
            tmpP.z += delx;
            tmpP.x -= dely;
        }
        if(fabs(_angle) < midangle){
            center_dir_dist = closest_level * range_step_size + deadzone;
            midangle = fabs(_angle);
        }
        all_levels.push_back(closest_level);
        if(fabs(_angle-targetdir) < minerr){
            num_of_dir = all_levels.size()-1;
            minerr = fabs(_angle-targetdir);
        }
        if(snstart){
            if(cnt_safescan > 0)
                ++cnt_safescan;
            if(closest_level < (last_dist_level) || (last_dist_level>=timesofdeld && closest_level < last_dist_level)){
                //find a safezone
                snstart = false;
                //if(cnt_safescan>2){
                    end_point.x = closest_level*deld + deadzone;
                    end_point.y = _angle - dangle;
                    llevel = closest_level;
                    tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
                    safezone_vec.push_back(tmpsz);
                //}
                
            }
            else if(closest_level > last_dist_level){
                start_point.x = last_dist_level*deld + deadzone;
                start_point.y = _angle;
                rlevel = last_dist_level;
                if(closest_level > (last_dist_level+1)){
                    cnt_safescan=1;
                }
                else
                {
                    cnt_safescan=0;
                }
            }
        }
        else{
            //no obs or sudden start
            if((closest_level > (last_dist_level)) || !obsexist){
                snstart = true;
                start_point.x = last_dist_level*deld + deadzone;
                start_point.y = _angle;
                rlevel = closest_level;
                if(closest_level > (last_dist_level+1)){
                    cnt_safescan=1;
                }
                else
                {
                    cnt_safescan=0;
                }
            }
            else if(obsexist){
                if(closest_level < (last_dist_level-1)){
                    end_point.x = closest_level*deld + deadzone;
                    end_point.y = _angle - dangle;
                    llevel = closest_level;
                    int tmplevel = last_dist_level, tmpcnt = all_levels.size()-2;
                    while(tmplevel >= last_dist_level){
                        tmplevel = all_levels[tmpcnt];
                        --tmpcnt;
                        if(tmpcnt<0)
                            break;
                    }
                    start_point.x = tmplevel*deld + deadzone;
                    start_point.y = dangle*(tmpcnt+1) - M_PI/2;
                    rlevel = tmplevel;
                }
            }
        }
        last_dist_level = closest_level;
    }
    if(snstart && last_dist_level>=timesofdeld-1){
        //cout<<"last level:"<<last_dist_level<<endl;
        end_point.x = 2;
        end_point.y = _angle - dangle;// + 0.149
        llevel = i;
        tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
        safezone_vec.push_back(tmpsz);
    }
    /**/
    pcl::PointXYZ self2(0,-start_height_of_cam,0), self1(0,-start_height_of_cam-deld*0.8,0),
            tarP2(-targetP.y,-start_height_of_cam,targetP.x), tarP1(-targetP.y,-start_height_of_cam-deld*0.8,targetP.x);
    if(lidarxyzOct.isBlock<unibn::L2Distance<pcl::PointXYZ> >(tarP2, self2, 0.6) ||
       lidarxyzOct.isBlock<unibn::L2Distance<pcl::PointXYZ> >(tarP1, self1, 0.6)){
        targetPisBlocked = 1;
    }
    else{
        targetPisBlocked = -1;
        // targetP is not blocked, add a safezone of targetP
        int level = all_levels[num_of_dir], nextlevel = level, step = 0;
        if(level >= 1){
            while(nextlevel == level && step < 15 && (num_of_dir-step)>0){
                nextlevel = all_levels[num_of_dir-step];
                ++step;
            }
            if(level <= nextlevel){
                nextlevel = level-1;
            }
            start_point.x = (0.5 + nextlevel)*deld;
            start_point.y = -M_PI/2 + dangle*(num_of_dir-step+1);
            llevel = level -1;
            step = 0;
            nextlevel = level;
            while(nextlevel == level && step < 15 && (num_of_dir+step)<all_levels.size()){
                nextlevel = all_levels[num_of_dir+step];
                //cout<<"level:"<<level<<",nextlevel:"<<nextlevel<<endl;
                ++step;
            }
            if(level <= nextlevel){
                nextlevel = level-1;
            }
            end_point.x = (0.5 + nextlevel)*deld;
            end_point.y = -M_PI/2 + dangle*(num_of_dir+step-1);
            rlevel = level - 1;
            tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
            //safezone_vec.push_back(tmpsz);
            safezone_vec.insert(safezone_vec.begin(), tmpsz);
        }
    }
    // ROS_INFO("end frame");
}

void p3atObstacleAvoid::pubSafeZone(){
    std_msgs::Float32MultiArray mamsgs;
    for(int i=0; i<safezone_vec.size(); ++i){
        mamsgs.data.push_back(safezone_vec[i].leftP[0]);
        mamsgs.data.push_back(safezone_vec[i].leftP[1]);
        mamsgs.data.push_back(safezone_vec[i].rightP[0]);
        mamsgs.data.push_back(safezone_vec[i].rightP[1]);
    }
    mamsgs.data.push_back(ori_dir);
    mamsgs.data.push_back(mod_dir);
    //cout<<"testdata:"<<safezone_vec.size()<<endl;
    safezone_pub.publish(mamsgs);
}

void p3atObstacleAvoid::findSZOctree(const sensor_msgs::LaserScanConstPtr& msg){
    //ROS_INFO("[obstesttime]start");
    safezone_view = cv::Mat(row_img, col_img, CV_8UC3, cv::Scalar(255,255,255)).clone();
    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 50, cv::Scalar(100,100,0),1);
    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 100, cv::Scalar(100,100,0),1);
    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 150, cv::Scalar(100,100,0),1);
    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 200, cv::Scalar(100,100,0),1);
    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 400, cv::Scalar(100,100,0),1);
    lidarxyzPC->clear();
    safezone_vec.clear();
    float _angle = msg->angle_min;
    pcl::PointXYZ tmpP(0,0,0);
    int start_in_msg=0;
    vector<int> sudden_change_p;
    //ROS_INFO("[obstesttime]1");
    for(int i=0; i<msg->ranges.size(); ++i){
        if(_angle > time_of_halfpi*M_PI/2){
            break;
        }
        if(_angle >= -time_of_halfpi*M_PI/2){
            if(isViewSafezoneLidar){
                int imgx, imgy;
                imgx = col_img/2 - int((msg->ranges[i]*sin(_angle)+Tlidarbody[1])*row_img/5);
                imgy = row_img- 1 - int((msg->ranges[i]*cos(_angle)+Tlidarbody[0])*row_img/5);
                if(imgx>=0 && imgx<col_img && imgy>=0 && imgy<row_img){
                    cv::circle(safezone_view, cv::Point(imgx, imgy), 1, cv::Scalar(0,0,0),2);
                }
            }
            /*
            //find sudden change to confirm safezone
            if((msg->ranges[i] > 4.5)
               ||((msg->ranges[i] - msg->ranges[i+1]) < -0.5 && (msg->ranges[i] - msg->ranges[i+2]) < -0.5 && (msg->ranges[i] - msg->ranges[i+3]) < -0.5)
               ||((msg->ranges[i] - msg->ranges[i-1]) < -0.5 && (msg->ranges[i] - msg->ranges[i-2]) < -0.5 && (msg->ranges[i] - msg->ranges[i-3]) < -0.5)){
                sudden_change_p.push_back(i);
                if(isViewSafezoneLidar){
                    int imgx, imgy;
                    float range;
                    if(msg->ranges[i] > 4.5)
                        range = 4.5;
                    else
                        range = msg->ranges[i];
                    imgx = col_img/2 - int(range*sin(_angle)*row_img/5);
                    imgy = row_img-1 - int(range*cos(_angle)*row_img/5);
                    if(imgx>=0 && imgx<col_img && imgy>=0 && imgy<row_img){
                        cv::circle(safezone_view, cv::Point(imgx, imgy), 3, cv::Scalar(100,20,240),2);
                    }
                }
            }
            */
            if(fabs(msg->ranges[i]) < 10)
                distangleToxy(msg->ranges[i], _angle, tmpP.x, tmpP.y);
                lidarxyzPC->push_back(tmpP);
        }
        else{
            start_in_msg = i;
        }
        _angle += msg->angle_increment;
    }
    ++start_in_msg;
    //cout<<"findoctree000"<<endl;
    //ROS_INFO("[obstesttime]2");
    //cout<<"points size="<<lidarxyzPC->size()<<", angle="<<_angle<<endl;
    lidarxyzOct.initialize(lidarxyzPC->points, myOctParams);
    //ROS_INFO("[obstesttime]initdone");
    //cout<<"findoctree111"<<endl;
    std::vector<uint32_t> res;
    bool obsexist = false, snstart = false;
    //float deld=0.46;
    //int timesofdeld=4;
    safeZone tmpsz;
    cv::Point2f start_point(2.03,0), end_point(2.03,0);  //this store dis in x and angle in y
    //every 10/3 degree
    //float dangle = 0.174/3;
    int lastobspoint=0, last_dist_level=4;
    int sudden_change_id_in_v = 0;
    int cnt_safescan=0;
    //cout<<"sudden change points size:"<<sudden_change_p.size()<<endl;
    int i=0, llevel=0, rlevel=0;
    bool last_obs_exist = false;
    float midangle = 1;
    vector<int> all_levels;
    float targetdir = atan2(targetP.y, targetP.x), minerr = 100;
    int num_of_dir = -1;
    for(_angle = -M_PI/2; _angle <= M_PI/2; _angle += dangle){
        float delx = range_step_size*cos(_angle), dely = range_step_size*sin(_angle);
        float firstx = (deld/2+deadzone)*cos(_angle), firsty = (deld/2+deadzone)*sin(_angle);
        tmpP.x = -Tlidarbody[0]+ firstx;
        tmpP.y = -Tlidarbody[1]+ firsty;
        res.clear();
        obsexist = false;
        for(i=0; i<timesofdeld; ++i){
            lidarxyzOct.radiusNeighbors<unibn::L2Distance<pcl::PointXYZ> >(tmpP, deld/2, res);
            if(res.size() > 2){
                obsexist = true;
                break;
            }
//            else if(res.size() > 0){
//                for(int j=0; j<res.size(); ++j){
//                    cout << "point:" <<res[j]<<","<<atan2(lidarxyzPC->points[res[j]].y,lidarxyzPC->points[res[j]].x)<<",";
//                }
//                cout<<endl;
//            }
            tmpP.x += delx;
            tmpP.y += dely;
        }
        if(fabs(_angle) < midangle){
            center_dir_dist = i * range_step_size + deadzone;
            midangle = fabs(_angle);
        }
        all_levels.push_back(i);
        if(fabs(_angle-targetdir) < minerr){
            num_of_dir = all_levels.size()-1;
            minerr = fabs(_angle-targetdir);
        }
        if(snstart){
            if(cnt_safescan > 0)
                ++cnt_safescan;
            if(i < (last_dist_level) || (last_dist_level>=timesofdeld && i < last_dist_level)){
                //end a safezone(obsexist && last_dist_level==4) ||
                snstart = false;
                //because it is the blocked one, so minus to last one
                end_point.x = msg->ranges[start_in_msg+res[0]];//i*deld + deadzone;//
                end_point.y = _angle - dangle;// + 0.149
                llevel = i;
                //if(cnt_safescan > 3){//suddenChangeExist(start_point.y, end_point.y, msg, sudden_change_p, sudden_change_id_in_v) || true
                    tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
                    safezone_vec.push_back(tmpsz);
                //}
                lastobspoint = res[res.size()-1];
            }
            else if(i > last_dist_level){
                if(res.size()>0)
                    lastobspoint = res[res.size()-1];
                if(cnt_safescan > 2){
                    end_point.x = msg->ranges[start_in_msg+lastobspoint];
                    end_point.y = _angle - dangle;
                    llevel = last_dist_level;
                    tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
                    safezone_vec.push_back(tmpsz);
                }
                start_point.x = msg->ranges[start_in_msg+lastobspoint];
                start_point.y = _angle;// - 0.149
                rlevel = i;
                if(i > (last_dist_level+1)){
                    cnt_safescan=1;
                }
                else
                {
                    cnt_safescan=0;
                }
            }
//            else{
//                if(res.size()>0)
//                    lastobspoint = res[res.size()-1];
//            }
        }
        else{
            //no obs or sudden start
            if((i > (last_dist_level)) || !obsexist){//
                snstart = true;
                if(msg->ranges[start_in_msg+lastobspoint]>4)
                    start_point.x = 4;
                else
                    start_point.x = msg->ranges[start_in_msg+lastobspoint];
                start_point.y = _angle;// - 0.149
                rlevel = i;
                if(i > (last_dist_level+1)){
                    cnt_safescan=1;
                }
                else
                {
                    cnt_safescan=0;
                }
            }
            else if(obsexist){
                lastobspoint = res[res.size()-1];
                if(i < (last_dist_level-1)){
                    end_point.x = msg->ranges[start_in_msg+lastobspoint];
                    end_point.y = _angle - dangle;
                    llevel = i;
                    int tmplevel = last_dist_level, tmpcnt = all_levels.size()-2;
                    while(tmplevel >= last_dist_level){
                        tmplevel = all_levels[tmpcnt];
                        --tmpcnt;
                        if(tmpcnt<0)
                            break;
                    }
                    start_point.x = tmplevel*deld + deadzone;
                    start_point.y = dangle*(tmpcnt+1) - M_PI/2;
                    rlevel = tmplevel;
                }
            }
        }
        last_dist_level = i;
    }
    if(snstart && last_dist_level>=timesofdeld){
        //cout<<"last level:"<<last_dist_level<<endl;
        end_point.x = 2;
        end_point.y = _angle - dangle;// + 0.149
        llevel = i;
        tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
        safezone_vec.push_back(tmpsz);
    }
    pcl::PointXYZ self(0,0,0);
    //lidarxyzOct.isBlock<unibn::L2Distance<pcl::PointXYZ> >(targetP, self, 0.6);
    if(lidarxyzOct.isBlock<unibn::L2Distance<pcl::PointXYZ> >(targetP, self, 0.6)){
        targetPisBlocked = 1;
    }
    else{
        targetPisBlocked = -1;
        // targetP is not blocked, add a safezone of targetP
        int level = all_levels[num_of_dir], nextlevel = level, step = 0;
        if(level >= 1){
            while(nextlevel == level && step < 15 && (num_of_dir-step)>0){
                nextlevel = all_levels[num_of_dir-step];
                ++step;
            }
            if(level <= nextlevel){
                nextlevel = level-1;
            }
            start_point.x = (0.5 + nextlevel)*deld;
            start_point.y = -M_PI/2 + dangle*(num_of_dir-step+1);
            llevel = level -1;
            step = 0;
            nextlevel = level;
            while(nextlevel == level && step < 15 && (num_of_dir+step)<all_levels.size()){
                nextlevel = all_levels[num_of_dir+step];
                //cout<<"level:"<<level<<",nextlevel:"<<nextlevel<<endl;
                ++step;
            }
            if(level <= nextlevel){
                nextlevel = level-1;
            }
            end_point.x = (0.5 + nextlevel)*deld;
            end_point.y = -M_PI/2 + dangle*(num_of_dir+step-1);
            rlevel = level - 1;
            tmpsz.setSimpleSZ(end_point.x, end_point.y, start_point.x, start_point.y, llevel, rlevel, deld);
            //safezone_vec.push_back(tmpsz);
            safezone_vec.insert(safezone_vec.begin(), tmpsz);
        }
    }
    //ROS_INFO("[obstesttime]end");
    //cout<<"findoctree"<<endl;
}

bool p3atObstacleAvoid::suddenChangeExist(float rang, float lang, const sensor_msgs::LaserScanConstPtr& msg, vector<int> &sudden_v, int &start_id_of_sc){
    float angle = msg->angle_increment*sudden_v[start_id_of_sc] + msg->angle_min;
    //cout<<"sudden change points size:"<<sudden_v.size()<<endl;
    while(angle < lang && start_id_of_sc < sudden_v.size()){
        if(angle > rang){
            return true;
        }
        angle = msg->angle_increment*sudden_v[start_id_of_sc] + msg->angle_min;
        ++start_id_of_sc;
    }
    return false;
}

void p3atObstacleAvoid::findSafeZone(const sensor_msgs::LaserScanConstPtr& msg){
    safezone_view = cv::Mat(row_img, col_img, CV_8UC3, cv::Scalar(255,255,255)).clone();
    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 200, cv::Scalar(100,100,0),1);
    cv::circle(safezone_view, cv::Point(col_img/2, row_img- 1), 400, cv::Scalar(100,100,0),1);
    float _angle = msg->angle_min;
    rangelidar_now.clear();
    //0-right, max-left
    bool start_safe_zone = false;
    int num_of_range = 0;
    safezone_vec.clear();
    safeZone tmp;
    float ldis, lang, rdis, rang;
    int danger_points=0;
    isDanger = false;
    //method 3
    vector<cv::Point2f> start_points, end_points, over2_points;  //this store dis in x and angle in y
    bool isOver2Exist=false;
    //cout << "get lidar:angle_min="<<msg->angle_min<<", angle_max="<<msg->angle_max<<", angle_increment="<<msg->angle_increment<<", size="<<msg->ranges.size()<<endl;
    //mylog<<"record safezone:min angle="<<_angle<<", +- "<<M_PI/2<<endl;
    for(int i=0; i<msg->ranges.size(); ++i){
//        mylog<<"angle now="<<_angle<<", ";", start angle:"<<sizeof(msg->ranges)<<
        //method (1):find start and end of safezone one by one. when one start
        // point is found, it ignores any start point until end point appears.
        /* the problem: once the end is found it thinks it's over for a safezone, but
         * that may be a fake end point.
         * like this(it will think ab is safezone,while ac, bd are the correct one):
         * |  b|____
         * |    ____
         * |  d|
         *_|   |____
         * c   a
         *  bot
         *//*
        if(_angle > M_PI/2){
            if(start_safe_zone){
                ldis = msg->ranges[i];
                lang = _angle;
                if(tmp.setSafeZone(ldis, lang, rdis, rang)){
                    safezone_vec.push_back(tmp);
                }
            }
            //mylog<<"break"<<endl;
            break;
        }
        if(_angle >= -M_PI/2){
            //for view
            if(isViewSafezoneLidar){
            int imgx, imgy;
            imgx = col_img/2 - int(msg->ranges[i]*sin(_angle)*row_img/5);
            imgy = row_img- 1 - int(msg->ranges[i]*cos(_angle)*row_img/5);
            if(imgx>=0 && imgx<col_img && imgy>=0 && imgy<row_img){
                safezone_view.at<cv::Vec3b>(imgy, imgx)[0] = 0;
                safezone_view.at<cv::Vec3b>(imgy, imgx)[1] = 0;
                safezone_view.at<cv::Vec3b>(imgy, imgx)[2] = 0;
            }
            }
            rangelidar_now.push_back(msg->ranges[i]);
            if(fabs(_angle) < 0.46 && msg->ranges[i] < 0.6) ++danger_points;
            ++num_of_range;
            //mylog<<"start point "<<i<<" range="<<msg->ranges[i]<<", ";
            //start point of safe zone
            //1.distance<2, 2.corner, 3.sudden close to far
            if(!start_safe_zone){
                if(msg->ranges[i] > 2 ||
                   ((msg->ranges[i] - msg->ranges[i-1]) < 0 && (msg->ranges[i] - msg->ranges[i-2]) < 0 && (msg->ranges[i] - msg->ranges[i+1]) < 0 && (msg->ranges[i] - msg->ranges[i+2]) < 0) ||
                   ((msg->ranges[i] - msg->ranges[i-1]) > 0.5 && (msg->ranges[i] - msg->ranges[i-2]) > 0.5) && (msg->ranges[i] - msg->ranges[i-3]) > 0.5){
                    start_safe_zone = true;
                    rdis = msg->ranges[i-1];
                    rang = _angle - msg->angle_increment;
                    //mylog<<"start point:"<<i<<", range="<<rdis<<", ang="<<rang*180/M_PI<<endl;
                }
            }
            //end point of safe zone
            //1.distance<2, 2.corner, 3.sudden far to close
            else{
                if(msg->ranges[i] > 3){}
                //for corner, it maybe the start point of next  safe zone
                else if(((msg->ranges[i] - msg->ranges[i-1]) < 0 && (msg->ranges[i] - msg->ranges[i-2]) < 0 && (msg->ranges[i] - msg->ranges[i+1]) < 0 && (msg->ranges[i] - msg->ranges[i+2]) < 0)){
                    ldis = msg->ranges[i];
                    lang = _angle;
                    if(tmp.setSafeZone(ldis, lang, rdis, rang)){
                        safezone_vec.push_back(tmp);
                    }
                    //mylog<<"maybe end:"<<i<<", range="<<ldis<<", ang="<<lang*180/M_PI<<endl;
                    rdis = msg->ranges[i];
                    rang = _angle;
                    //mylog<<"start point:"<<i<<", range="<<rdis<<", ang="<<rang*180/M_PI<<endl;
                }
                else if(msg->ranges[i] <= 2 ||
                   ((msg->ranges[i] - msg->ranges[i-1]) < -0.5 && (msg->ranges[i] - msg->ranges[i-2]) < -0.5) && (msg->ranges[i] - msg->ranges[i-3]) < -0.5){
                    start_safe_zone = false;
                    ldis = msg->ranges[i];
                    lang = _angle;
                    //mylog<<"maybe end:"<<i<<", range="<<ldis<<", ang="<<lang*180/M_PI<<endl;
                    if(tmp.setSafeZone(ldis, lang, rdis, rang)){
                        safezone_vec.push_back(tmp);
                    }
                }
            }
        }//end of method 1
        */
        //method (2):similar to method(1), but it dosn't ignore start point while replace
        // start point with new one
/*
        if(_angle > M_PI/2){
            break;
        }
        if(_angle >= -M_PI/2){
            //for view
            if(isViewSafezoneLidar){
            int imgx, imgy;
            imgx = col_img/2 - int(msg->ranges[i]*sin(_angle)*row_img/5);
            imgy = row_img- 1 - int(msg->ranges[i]*cos(_angle)*row_img/5);
            if(imgx>=0 && imgx<col_img && imgy>=0 && imgy<row_img){
                cv::circle(safezone_view, cv::Point(imgx, imgy), 1, cv::Scalar(0,0,0),2);
//                safezone_view.at<cv::Vec3b>(imgy, imgx)[0] = 0;
//                safezone_view.at<cv::Vec3b>(imgy, imgx)[1] = 0;
//                safezone_view.at<cv::Vec3b>(imgy, imgx)[2] = 0;
            }
            }
            //judge start
            if(!start_safe_zone){
                if(msg->ranges[i] > 2 ||
                   ((msg->ranges[i] - msg->ranges[i-1]) < 0 && (msg->ranges[i] - msg->ranges[i-2]) < 0 && (msg->ranges[i] - msg->ranges[i+1]) < 0 && (msg->ranges[i] - msg->ranges[i+2]) < 0) ||
                   ((msg->ranges[i] - msg->ranges[i-1]) > 0.5 && (msg->ranges[i] - msg->ranges[i-2]) > 0.5) && (msg->ranges[i] - msg->ranges[i-3]) > 0.5){
                    start_safe_zone = true;
                    rdis = msg->ranges[i-1];
                    rang = _angle - msg->angle_increment;
                    //mylog<<"start point:"<<i<<", range="<<rdis<<", ang="<<rang*180/M_PI<<endl;
                }
            }
            else{
                //rejudge start of safezone condition 3
                if(((msg->ranges[i] - msg->ranges[i-1]) > 0.5 && (msg->ranges[i] - msg->ranges[i-2]) > 0.5) && (msg->ranges[i] - msg->ranges[i-3]) > 0.5){
                    rdis = msg->ranges[i-1];
                    rang = _angle - msg->angle_increment;
                }
                //judge end
                else if(msg->ranges[i] < 4){
                    if(((msg->ranges[i] - msg->ranges[i-1]) < 0 && (msg->ranges[i] - msg->ranges[i-2]) < 0 && (msg->ranges[i] - msg->ranges[i+1]) < 0 && (msg->ranges[i] - msg->ranges[i+2]) < 0)){
                        ldis = msg->ranges[i];
                        lang = _angle;
                        if(tmp.setSafeZone(ldis, lang, rdis, rang)){
                            safezone_vec.push_back(tmp);
                        }
                        //mylog<<"maybe end:"<<i<<", range="<<ldis<<", ang="<<lang*180/M_PI<<endl;
                        rdis = msg->ranges[i];
                        rang = _angle;
                        //mylog<<"start point:"<<i<<", range="<<rdis<<", ang="<<rang*180/M_PI<<endl;
                    }
                    else if(msg->ranges[i] <= 2 ||
                       ((msg->ranges[i] - msg->ranges[i-1]) < -0.5 && (msg->ranges[i] - msg->ranges[i-2]) < -0.5 && (msg->ranges[i] - msg->ranges[i-3]) < -0.5)){
                        start_safe_zone = false;
                        ldis = msg->ranges[i];
                        lang = _angle;
                        //mylog<<"maybe end:"<<i<<", range="<<ldis<<", ang="<<lang*180/M_PI<<endl;
                        if(tmp.setSafeZone(ldis, lang, rdis, rang)){
                            safezone_vec.push_back(tmp);
                        }
                    }
                }
            }
            rangelidar_now.push_back(msg->ranges[i]);
            if(fabs(_angle) < 0.46 && msg->ranges[i] < 0.6) ++danger_points;
        }//end of method 2
*/
        //method (3): find all possible points
        /**/
        if(_angle > M_PI/2){
            if(msg->ranges[i] > 2){
                end_points.push_back(cv::Point2f(2, _angle-msg->angle_increment));
                if(isOver2Exist){
                    over2_points.push_back(cv::Point2f(2, _angle-msg->angle_increment));
                }
                //cout << "endP:("<<msg->ranges[i]<<","<<_angle<<endl;
            }
            break;
        }
        if(_angle >= -M_PI/2){
            rangelidar_now.push_back(msg->ranges[i]);
            if(fabs(_angle) < 0.46 && msg->ranges[i] < 0.6) ++danger_points;
            //for view
            if(isViewSafezoneLidar){
            int imgx, imgy;
            imgx = col_img/2 - int(msg->ranges[i]*sin(_angle)*row_img/5);
            imgy = row_img- 1 - int(msg->ranges[i]*cos(_angle)*row_img/5);
            if(imgx>=0 && imgx<col_img && imgy>=0 && imgy<row_img){
                cv::circle(safezone_view, cv::Point(imgx, imgy), 1, cv::Scalar(0,0,0),2);
            }
            }

            //add distance over 2 possible safezone edge
            /**/
            if(!isOver2Exist){
                if(msg->ranges[i] > 2){
                    isOver2Exist = true;
//                    if(num_of_range == 0){
//                        if(msg->ranges[i] < 4)
//                            start_points.push_back(cv::Point2f(msg->ranges[i], _angle));
//                        else
//                            start_points.push_back(cv::Point2f(4, _angle));
//                    }else
//                    if(msg->ranges[i] < 4){//
//                        over2_points.push_back(cv::Point2f(msg->ranges[i], _angle));
//                    }
//                    else{
//                        over2_points.push_back(cv::Point2f(4, _angle));
//                    }
                    over2_points.push_back(cv::Point2f(msg->ranges[i-1], _angle-msg->angle_increment));
                }
            }
            else{
                if(msg->ranges[i] < 2){
                    isOver2Exist = false;
                    over2_points.push_back(cv::Point2f(msg->ranges[i], _angle));
                }
            }

            if(msg->ranges[i] < 4){
            //add other possible safezone edge
            //corner
            /*if(((msg->ranges[i] - msg->ranges[i-5]) < 0 && (msg->ranges[i] - msg->ranges[i-2]) < 0 && (msg->ranges[i] - msg->ranges[i+5]) < 0 && (msg->ranges[i] - msg->ranges[i+2]) < 0)){
                start_points.push_back(cv::Point2f(msg->ranges[i], _angle));
                end_points.push_back(cv::Point2f(msg->ranges[i], _angle));
            }
            //sudden close to far
            else*/ if((msg->ranges[i] - msg->ranges[i+1]) < -0.5 && (msg->ranges[i] - msg->ranges[i+2]) < -0.5 && (msg->ranges[i] - msg->ranges[i+3]) < -0.5){
                start_points.push_back(cv::Point2f(msg->ranges[i-1], _angle-msg->angle_increment));
//                if(isOver2Exist){
//                    over2_points[over2_points.size()-1].x = msg->ranges[i-1];
//                    over2_points[over2_points.size()-1].y = _angle-msg->angle_increment;
//                }
                //cout << "startP:("<<msg->ranges[i-1]<<","<<_angle-msg->angle_increment<<endl;
            }
            //sudden far to close
            else if((msg->ranges[i] - msg->ranges[i-1]) < -0.5 && (msg->ranges[i] - msg->ranges[i-2]) < -0.5 && (msg->ranges[i] - msg->ranges[i-3]) < -0.5){
                end_points.push_back(cv::Point2f(msg->ranges[i], _angle));
                //cout << "endP:("<<msg->ranges[i]<<","<<_angle<<endl;
//                if(msg->ranges[i] > 2 && isOver2Exist){
//                    //this would skip a over 2 start point, so add one
//                    start_points.push_back(cv::Point2f(msg->ranges[i+1], _angle+msg->angle_increment));
//                }
            }
            }
            ++num_of_range;
        }//end of method 3,not finish

        _angle += msg->angle_increment;
    }
    //method 3, connect start end of safezone
    /**/
    int ii=0, jj=0;
    //cout<<"total startP:"<<start_points.size()<<" ,endP:"<<end_points.size()<<endl;
    if(isViewSafezoneLidar){
        int imgx, imgy;
        for(ii=0;ii<start_points.size();++ii){
            imgx = col_img/2 - int(start_points[ii].x*sin(start_points[ii].y)*row_img/5);
            imgy = row_img- 1 - int(start_points[ii].x*cos(start_points[ii].y)*row_img/5);
            cv::circle(safezone_view, cv::Point(imgx, imgy), 3, cv::Scalar(0,255,0),3);
        }
        for(ii=0;ii<end_points.size();++ii){
            imgx = col_img/2 - int(end_points[ii].x*sin(end_points[ii].y)*row_img/5);
            imgy = row_img- 1 - int(end_points[ii].x*cos(end_points[ii].y)*row_img/5);
            cv::circle(safezone_view, cv::Point(imgx, imgy), 3, cv::Scalar(255,0,0),3);
        }
        for(ii=0;ii<over2_points.size();++ii){
            imgx = col_img/2 - int(over2_points[ii].x*sin(over2_points[ii].y)*row_img/5);
            imgy = row_img- 1 - int(over2_points[ii].x*cos(over2_points[ii].y)*row_img/5);
            cv::circle(safezone_view, cv::Point(imgx, imgy), 3, cv::Scalar(70,120,230),3);
        }
    }
    //sudden start end point safezone
    for(ii=0;ii<start_points.size();++ii){
        //mylog<<"startP "<<ii<<"("<<start_points[ii].x<<","<<start_points[ii].y<<")"<<endl;
        //method 3.1
        /*
        //find the first end point on the left of start point
        while(jj<end_points.size() && end_points[jj].y <= start_points[ii].y){
            //mylog<<"endP "<<jj<<"("<<end_points[jj].x<<","<<end_points[jj].y<<")"<<endl;
            ++jj;
        }
        //find the nearest start point of the end point
        while(ii<start_points.size() && end_points[jj].y > start_points[ii].y){
            //mylog<<"startP "<<ii<<"("<<start_points[ii].x<<","<<start_points[ii].y<<")"<<endl;
            ++ii;
        }
        if(ii<start_points.size() && jj<end_points.size()){
            --ii;
            if(tmp.setSafeZone(end_points[jj].x, end_points[jj].y, start_points[ii].x, start_points[ii].y)){
                safezone_vec.push_back(tmp);
            }
        }//end of method 3.1
        */
        //method 3.2
        //find nearest end point
        while(jj<end_points.size() && end_points[jj].y <= start_points[ii].y){
            ++jj;
        }
        float dist_start_end = 10000, tmpdist;
        int nearest_end = -1;
        for(int tj=jj; tj<end_points.size(); ++tj){
            tmpdist = start_points[ii].x*start_points[ii].x + end_points[tj].x*end_points[tj].x
                    - 2*start_points[ii].x*end_points[tj].x*cos(fabs(start_points[ii].y-end_points[tj].y));
            if(tmpdist < dist_start_end){
                dist_start_end = tmpdist;
                nearest_end = tj;
            }
        }
        int nearest_start = ii;
        dist_start_end = 10000;
        for(int ti=ii; ti<start_points.size() && start_points[ti].y < end_points[nearest_end].y; ++ti){
            tmpdist = start_points[ti].x*start_points[ti].x + end_points[nearest_end].x*end_points[nearest_end].x
                    - 2*start_points[ti].x*end_points[nearest_end].x*cos(fabs(start_points[ti].y-end_points[nearest_end].y));
            if(tmpdist < dist_start_end){
                dist_start_end = tmpdist;
                nearest_start = ti;
            }
        }
        //cout <<"nearest end id:"<<nearest_end<<endl;
        if(nearest_end != -1){
            jj = nearest_end;
            if(ii<start_points.size() && nearest_end<end_points.size()){
                //cout <<"nearest end:"<<nearest_end<<", nearest start:"<<nearest_start<<", ii"<<ii<<endl;
                if(tmp.setSafeZone(end_points[nearest_end].x, end_points[nearest_end].y, start_points[nearest_start].x, start_points[nearest_start].y)){
                    safezone_vec.push_back(tmp);
                }
            }
            while(end_points[jj].y > start_points[ii].y){
                ++ii;
            }
            --ii;
        }
        else{
            break;
        }//end of method 3.2
    }
    //over 2m safezone
    //cout<<"over 2 point size:"<<over2_points.size()<<endl;
    vector<safeZone> over2_vec;
    for(ii=0; ii+1<over2_points.size();){
        //cout<<"point "<<ii<<"=("<<over2_points[ii].x<<", "<<over2_points[ii].y<<"), ("<<over2_points[ii+1].x<<","<<over2_points[ii+1].y<<endl;
        if(tmp.setSafeZone(over2_points[ii+1].x, over2_points[ii+1].y, over2_points[ii].x, over2_points[ii].y)){
            over2_vec.push_back(tmp);
        }
        ii += 2;
    }
    //end of method 3

    //remove near safezone
    jj=0;
    bool need_insert = true;
    int size_of_ori_safezone = safezone_vec.size();
    for(ii=0; ii<over2_vec.size(); ++ii){
        for(; jj<size_of_ori_safezone; ++jj){
            float rerr = over2_vec[ii].safe_direction[0] - safezone_vec[jj].safe_direction[0],
                  lerr = over2_vec[ii].safe_direction[1] - safezone_vec[jj].safe_direction[1],
                  err_dir = (lerr + rerr) / 2;
            if(fabs(err_dir) < 0.05){
                if(lerr < 0 && rerr > 0){
                    safezone_vec.erase(safezone_vec.begin()+jj);
                    safezone_vec.insert(safezone_vec.begin()+jj, over2_vec[ii]);
                    need_insert = false;
                    break;
                }
                else{
                    need_insert = false;
                    break;
                }
            }
            else if(err_dir < 0){
                need_insert = true;
                break;
            }
        }
        if(need_insert){
            safezone_vec.push_back(over2_vec[ii]);
        }
        else{
            need_insert = true;
        }
    }
    //remove overlap zone, keep the smaller one.(not right)
    /*
    jj=0;
    int size_of_ori_safezone = safezone_vec.size();
    bool need_insert = true;
    for(ii=0; ii<over2_vec.size(); ++ii){
        for(; jj<size_of_ori_safezone; ++jj){
            if(safezone_vec[jj].rightP[1] <= over2_vec[ii].rightP[1]){
                //over2's right is on the left of compared zone
                if(safezone_vec[jj].leftP[1] >= over2_vec[ii].leftP[1]){
                    //overlap, over2 is smaller, replace origin
                    need_insert = false;
                    safezone_vec.erase(safezone_vec.begin()+jj);
                    safezone_vec.insert(safezone_vec.begin()+jj, over2_vec[ii]);
                    break;
                }
                else if(safezone_vec[jj].leftP[1] <= over2_vec[ii].rightP[1]){
                    //not overlap, skip to next one
                    //continue;
                }
            }
            else{
                //over2's right is on the right of compared zone
                if(safezone_vec[jj].rightP[1] <= over2_vec[ii].leftP[1]){
                    //the compared one is totally on the left of over2
                    break;
                }
                else if(safezone_vec[jj].leftP[1] <= over2_vec[ii].leftP[1]){
                    //over2 is larger
                    need_insert = false;
                    break;
                }
            }
        }
        if(need_insert){
            safezone_vec.push_back(over2_vec[ii]);
        }
        need_insert = true;
    }
    */
    
    //mylog<<"final angle:"<<_angle*180/M_PI<<endl;
    time_lidar_now = ros::Time::now().toSec();
    time_lidar_past.push_back(time_lidar_now);
    rangelidar_vec.push_back(rangelidar_now);
    if(danger_points > 20) isDanger = true;
    //mylog<<"num of dist<0.5:"<<danger_points<<endl;
    if(rangelidar_vec.size() > 5){
        rangelidar_vec.erase(rangelidar_vec.begin());
        time_lidar_past.erase(time_lidar_past.begin());
    }
}

void p3atObstacleAvoid::lidarCallback(const sensor_msgs::LaserScanConstPtr& msg){
    //ROS_INFO("[obstesttime]start");
    //findSafeZone(msg);
    findSZOctree(msg);
    //ROS_INFO("[obstesttime]end");
/*
    
*/
}

void p3atObstacleAvoid::targetPCallback(const geometry_msgs::PoseConstPtr &msg){
    targetP.x = msg->position.x;
    targetP.y = msg->position.y;
    targetP.z = msg->position.z;
    targetOrien[0] = msg->orientation.w;
    targetOrien[1] = msg->orientation.x;
    targetOrien[2] = msg->orientation.y;
    targetOrien[3] = msg->orientation.z;
    //cout<<"[obs]get targetp:"<<targetP.x<<","<<targetP.y<<","<<targetP.z<<endl;
}

///
/// \brief p3atObstacleAvoid::findNearestSafeZone
/// \param dyaw, origin desired dir
/// \param d_dir_safe, err between origin desired dir and chosen dir
/// \param dist, chosen dir distance
/// \param dir, chosen dir(it is the same as the following one, because they are all in the self coordinate)
/// \param ddir_between_now_and_chosen_dir, err between now dir and chosen dir
/// \return
///
int p3atObstacleAvoid::findNearestSafeZone(float dyaw, float &d_dir_safe, float &dist, float& dir, float& ddir_between_now_and_chosen_dir){
    if(safezone_vec.empty())
        return -1;

    int num_of_nearest = -1;
    float min_dtheta = 360, tmpdtheta, tmpdist, tmpdir;
    mylog <<"all "<<safezone_vec.size()<<" safezone"<<endl;
    //for view
    int imglx, imgly, imgrx, imgry;
    //ROS_INFO("[ininin]1");
    //cout<<"size="<<safezone_vec.size()<<endl;
    for(int i=0; i<safezone_vec.size(); ++i){
        //cout<<"ooo  "<<i<<endl;
        //for view
        if(isViewSafezoneLidar){
        imglx = col_img/2 - int(safezone_vec[i].leftP[0]*sin(safezone_vec[i].leftP[1])*row_img/5);
        imgly = row_img- 1 - int(safezone_vec[i].leftP[0]*cos(safezone_vec[i].leftP[1])*row_img/5);
        imgrx = col_img/2 - int(safezone_vec[i].rightP[0]*sin(safezone_vec[i].rightP[1])*row_img/5);
        imgry = row_img- 1 - int(safezone_vec[i].rightP[0]*cos(safezone_vec[i].rightP[1])*row_img/5);
        cv::line(safezone_view, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(0,255,0), 2);
        }

        tmpdtheta = safezone_vec[i].getdTheta(dyaw, tmpdist, tmpdir);
        //cout<<"zone["<<i<<"]:lp[0]="<<safezone_vec[i].leftP[0]<<", lp[1]="<<safezone_vec[i].leftP[1]*180/M_PI<<", rp[0]="<<safezone_vec[i].rightP[0]<<", rp[1]="<<safezone_vec[i].rightP[1]*180/M_PI<<", safedir:"<<safezone_vec[i].safe_direction[0]*180/M_PI<<", "<<safezone_vec[i].safe_direction[1]*180/M_PI<<endl;
        mylog<<"zone["<<i<<"]:lp[0]="<<safezone_vec[i].leftP[0]<<", lp[1]="<<safezone_vec[i].leftP[1]*180/M_PI<<", rp[0]="<<safezone_vec[i].rightP[0]<<", rp[1]="<<safezone_vec[i].rightP[1]*180/M_PI<<", safedir:"<<safezone_vec[i].safe_direction[0]*180/M_PI<<", "<<safezone_vec[i].safe_direction[1]*180/M_PI<<endl;
        if(fabs(tmpdtheta) < fabs(min_dtheta)){
            min_dtheta = tmpdtheta;
            dist = tmpdist;
            dir = tmpdir;
            num_of_nearest = i;
            ddir_between_now_and_chosen_dir = safezone_vec[i].getdTheta(0, tmpdist, tmpdir);
        }
    }
    //ROS_INFO("[ininin]2");
    d_dir_safe = min_dtheta;
    mylog<<"choose "<<num_of_nearest<<" safezone"<<endl;
    //for view
    if(isViewSafezoneLidar){
    imglx = col_img/2 - int(safezone_vec[num_of_nearest].leftP[0]*sin(safezone_vec[num_of_nearest].leftP[1])*row_img/5);
    imgly = row_img- 1 - int(safezone_vec[num_of_nearest].leftP[0]*cos(safezone_vec[num_of_nearest].leftP[1])*row_img/5);
    imgrx = col_img/2 - int(safezone_vec[num_of_nearest].rightP[0]*sin(safezone_vec[num_of_nearest].rightP[1])*row_img/5);
    imgry = row_img- 1 - int(safezone_vec[num_of_nearest].rightP[0]*cos(safezone_vec[num_of_nearest].rightP[1])*row_img/5);
    cv::line(safezone_view, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(0,0,255), 2);
    imglx = col_img/2 - int(3*sin(dyaw)*row_img/5);
    imgly = row_img- 1 - int(3*cos(dyaw)*row_img/5);
    imgrx = col_img/2;
    imgry = row_img- 1;
    cv::line(safezone_view, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(255,0,0), 2);
    imglx = col_img/2 - int(2*sin(dir)*row_img/5);
    imgly = row_img- 1 - int(2*cos(dir)*row_img/5);
    cv::line(safezone_view, cv::Point(imglx, imgly), cv::Point(imgrx, imgry), cv::Scalar(111,222,111), 2);
    }
    //ROS_INFO("[ininin]3");
    ori_dir = dyaw;
    mod_dir = dir;
    return num_of_nearest;
}

inline void p3atObstacleAvoid::distangleToxy(float dist, float angle, float& x, float& y){
    x = dist * cos(angle);
    y = dist * sin(angle);
}

inline void p3atObstacleAvoid::xyTodistangle(float x, float y, float& dist, float& angle){
    dist = sqrt(x*x + y*y);
    angle = atan2(y, x);
}

void p3atObstacleAvoid::smooth(float &vx, float &rz){
    if(vx - last_vx > 0.01){
        vx = last_vx + 0.01;
    }
    else if(vx - last_vx < -0.01){
        vx = last_vx - 0.01;
    }
    last_vx = vx;

    if(rz - last_rz > 0.01){
        rz = last_rz + 0.01;
    }
    else if(rz - last_rz < -0.01){
        rz = last_rz - 0.01;
    }
    last_rz = rz;
}

bool safeZone::setSafeZone(float ldis, float lang, float rdis, float rang){
    leftP[0] = ldis;
    leftP[1] = lang;
    rightP[0] = rdis;
    rightP[1] = rang;
    if(ldis > rdis){
        near = rightP;
        far = leftP;
    }
    else{
        near = leftP;
        far = rightP;
    }

    float dtheta = far[1]-near[1];
    float width_total = sqrt(near[0]*near[0] + far[0]*far[0] - 2*far[0]*near[0]*cos(fabs(dtheta)));
    if(width_total < 0.6){
        return false;
    }
    else{
        float theta_near = acos(1 - pow(0.6/near[0], 2) / 2);
        if(theta_near > fabs(dtheta)){
            //direction passable space is narrow
            safe_direction[0] = near[1] + fabs(dtheta) / dtheta * theta_near / 2;
            safe_direction[1] = safe_direction[0];
        }
        else{
            float theta_far = acos(1 - pow(0.6/far[0], 2) / 2);
            safe_direction[0] = near[1] + fabs(dtheta) / dtheta * theta_near / 2;
            safe_direction[1] = far[1] - fabs(dtheta) / dtheta *theta_far / 2;
            //keep 1 as left and 0 as right
            if(safe_direction[0] > safe_direction[1]){
                float ex = safe_direction[1];
                safe_direction[1] = safe_direction[0];
                safe_direction[0] = ex;
            }
        }
        return true;
    }
}

void safeZone::setSimpleSZ(float ldis, float lang, float rdis, float rang, int l_level, int r_level, float deld){
    if(ldis < 20){
        leftP[0] = ldis;
    }
    else{
        leftP[0] = 20;
    }
    leftP[1] = lang;
    if(rdis < 20){
        rightP[0] = rdis;
    }
    else{
        rightP[0] = 20;
    }
    rightP[1] = rang;
    safe_direction[0] = rang + acos(1 - pow(0.05/deld/(r_level+1),2) / 2.3);//(0.5-deld)(0.5-deld)
    safe_direction[1] = lang - acos(1 - pow(0.05/deld/(l_level+1),2) / 2.3);
//ROS_INFO("AAAA");
//cout<<"acos="<<(pow(0.05/deld/(r_level+1),2) / 2)<<", rlevel:"<<r_level<<", deld"<<deld<<endl;
//cout<<"setsz:"<<leftP[0]<<", "<<leftP[1]<<"--"<<rightP[0]<<","<<rightP[1]<<"--"<<safe_direction[0]<<","<<safe_direction[1]<<endl;
    if(safe_direction[0]>safe_direction[1]){
        safe_direction[0] = (lang+rang)/2;
        safe_direction[1] = safe_direction[0];
    }
}

///
/// \brief safeZone::getdTheta
/// \param dyaw, origin desired dir
/// \param dist, distance of chosen dir
/// \param dir, chosen dir
/// \return err between origin desired dir and chosen dir
///
float safeZone::getdTheta(float dyaw ,float& dist, float& dir){
    if(dyaw <= safe_direction[1] && dyaw >= safe_direction[0]){
        dist = (rightP[0]+leftP[0])/2;
        dir = 0;
        return 0;
    }
    else if(dyaw < safe_direction[0]){
        dist = rightP[0];
        dir = safe_direction[0];
        return (safe_direction[0] - dyaw);//
    }
    else{
        dist = leftP[0];
        dir = safe_direction[1];
        return (safe_direction[1] - dyaw);//
    }
}

void safeZone::getMidxyz(double &x, double &y, double &z){
    z = 0;
    float mang = (rightP[1]+leftP[1])/2, mdis = (leftP[0]+rightP[0])/2;
    x = mdis*cos(mang);
    y = mdis*sin(mang);
}
