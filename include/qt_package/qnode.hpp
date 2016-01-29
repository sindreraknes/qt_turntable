/**
 * @file /include/qt_package/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_package_QNODE_HPP_
#define qt_package_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_package {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
    void angleCallback(const std_msgs::Int32::ConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);



Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void sendCurrentAngle(double d);
    void sendTopics(QStringList list);
    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c);
    void displayImage(pcl::PointCloud<pcl::PointXYZ>::Ptr c);

public Q_SLOTS:
    void findTopics();
    void subscribeToPointCloud2(QString topic, bool rgb);
    void setAngle(double angle);
    void takePicture(int nrPicture, QString url);

private:
	int init_argc;
	char** init_argv;
    pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    ros::Publisher anglepub;
    ros::Subscriber anglesub;
    ros::Subscriber pointCloud2Sub;
    bool picture_flag;
    bool rgb_enabled;
};

}  // namespace qt_package

#endif /* qt_package_QNODE_HPP_ */
