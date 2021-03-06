/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <iostream>
#include "../include/qt_package/qnode.hpp"
#include <pcl/io/pcd_io.h>
#include <QTime>

namespace qt_package {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv),
    rofl(new pcl::PointCloud<pcl::PointXYZ>)
    {
        qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr >("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    }

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}


bool QNode::init() {

    ros::init(init_argc,init_argv,"qt_package_sindre");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    picture_flag = false;
    anglesub = n.subscribe<std_msgs::Int32,QNode>("current_angle",1000,&QNode::angleCallback,this);
    anglepub = n.advertise<std_msgs::Int32>("set_turntable_angle",1000);
	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(400);
    while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

// ********* SLOTS ********* //
// ************************* //
void QNode::findTopics(){
    QStringList list;
    QString tmp;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
        tmp = QString::fromUtf8(info.datatype.c_str());

        // Add more types if needed
        if(QString::compare(tmp, "sensor_msgs/PointCloud2", Qt::CaseInsensitive) == 0){
           list.append(QString::fromUtf8(info.name.c_str()));
        }
    }
    Q_EMIT sendTopics(list);
}

void QNode::subscribeToPointCloud2(QString topic, bool rgb){
    ros::NodeHandle n;
    rgb_enabled = rgb;
    const char *tmp = topic.toUtf8().constData();
    pointCloud2Sub = n.subscribe<sensor_msgs::PointCloud2, QNode>(tmp, 1, &QNode::cloudCallback,this);
}

void QNode::setAngle(double angle)
{
    std_msgs::Int32 msg;
    //convert from 0-360 to 0-12800 here
    msg.data = angle*35.5555555555556;
    anglepub.publish(msg);
}

void QNode::takePicture(int nrPicture, QString url, bool display)
{
    bool running = true;
    int picture_taken = 0;
    QString tmpUrl;
    while(running)
    {
        if(picture_flag)
        {
            tmpUrl = url;
            tmpUrl.append("_");
            tmpUrl.append(QString::number(picture_taken+1));
            tmpUrl.append(".pcd");
            //take picture here
            if(rgb_enabled){
                //pcl::io::savePCDFile(tmpUrl.toUtf8().constData(), cloudRGB);
                pcl::io::savePCDFileBinary(tmpUrl.toUtf8().constData(),cloudRGB);

                picture_taken = picture_taken + 1;
            }
            else{
                pcl::io::savePCDFileASCII(tmpUrl.toUtf8().constData(), cloud);
                picture_taken = picture_taken + 1;
            }
            picture_flag = false;
        }
        if(picture_taken == nrPicture)
        {
            running = false;
        }
    }
    if(display){
        Q_EMIT displayImage(tmpUrl);
    }
}

// ********* CALLBACK METHODS ********* //
// ************************************ //
void QNode::angleCallback(const std_msgs::Int32::ConstPtr& msg){
    double angle = msg->data*0.028125;
    Q_EMIT sendCurrentAngle(angle);
}

void QNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
        // Convert ROS message (PointCloud2) to PCL point cloud (PointCloud(PointXYZ))
        pcl::fromROSMsg(*cloud_msg, cloud);
        if(rgb_enabled){
            pcl::fromROSMsg(*cloud_msg, cloudRGB);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(*cloud_msg, *tmpCloud);
            Q_EMIT setPointCloudRGB(tmpCloud);
        }
        else{
            // Cloud conversion and visualization
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *tmpCloud);
//            //int goodindex = 0;
////            for(int i = 0; i<tmpCloud->size(); i++){
////                if(tmpCloud->points[i].z > 1.9 && tmpCloud->points[i].z < 2.0 && tmpCloud->points[i].x > 0.32 && tmpCloud->points[i].x < 0.34
////                        && tmpCloud->points[i].y > -0.46 && tmpCloud->points[i].y < -0.43){
////                    goodindex = i;
////                    std::cout << "good index: " << i << std::endl;
////                    return;
////                }
////            }

//            //    cam1 <<  0.0200591,  -0.999758 , 0.0089781 ,  0.326005,
//            //             -0.627877, -0.0195849 , -0.778066 , -0.440615,
//            //              0.778054 ,0.00997016 , -0.628118 ,   1.98857,
//            //                     0   ,       0  ,        0  ,        1;

//            // Test area
//            //double x = tmpCloud->points[12000].x;
//            //double y = tmpCloud->points[12000].y;
//            //double z = tmpCloud->points[12000].z;
//            pcl::PointXYZ point;
//            point = tmpCloud->points[127241];
////            std::cout << "Point x: " << x << std::endl;
////            std::cout << "Point y: " << y << std::endl;
//            //std::cout << "Point z: " << z << std::endl;
//            std::cout << rofl->size() << std::endl;
//            rofl->push_back(point);
//            Q_EMIT setPointCloud(rofl);




//            if(rofl->size() == 10000){
//                pcl::io::savePCDFileASCII("/home/minions/zaxisfail.pcd", *rofl);
//            }



            Q_EMIT setPointCloud(tmpCloud);

        }
        picture_flag = true;

}

} //namespace qt_package
