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
	init_argv(argv)
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

	ros::init(init_argc,init_argv,"qt_package");
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
    pointCloud2Sub = n.subscribe<sensor_msgs::PointCloud2, QNode>(tmp, 1000, &QNode::cloudCallback,this);
}

void QNode::setAngle(double angle)
{
    std_msgs::Int32 msg;
    //convert from 0-360 to 0-12800 here
    msg.data = angle*35.5555555555556;
    anglepub.publish(msg);
}

void QNode::takePicture(int nrPicture, QString url)
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
                pcl::io::savePCDFileASCII(tmpUrl.toUtf8().constData(), cloudRGB);
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
            std::cout << "Done!" << std::endl;
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(tmpUrl.toUtf8().constData(), *displayCloud) == -1){
        std::cout << "Could not load file" << std::endl;
        return;
    }
    Q_EMIT displayImage(displayCloud);
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
        }

        // Cloud conversion and visualization
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *tmpCloud);
        picture_flag = true;
        Q_EMIT setPointCloud(tmpCloud);

}

} //namespace qt_package
