/**
 * @file /include/qt_package/main_window.hpp
 *
 * @brief Qt based gui for qt_package.
 *
 * @date November 2010
 **/
#ifndef qt_package_MAIN_WINDOW_H
#define qt_package_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_package {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();


	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
    //Auto connections
    void on_button_set_pos_clicked(bool check);
    void on_button_refresh_topic_clicked(bool check);
    void on_button_subscribe_topic_clicked(bool check);
    void on_button_folder_select_clicked(bool check);
    void on_button_take_pic_clicked(bool check);
    void on_button_load_picture_clicked(bool check);
    //Manual connections
    void readCurrentAngle(double d);
    void updateTopics(QStringList list);
    void getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c);
    void getPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c);
    void displayImage(QString url);

Q_SIGNALS:
    void getTopics();
    void subscribeToPointCloud2(QString topic, bool rgb);
    void setAngle(double angle);
    void takePictures(int nrPictures, QString url, bool display);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud;
    QVTKWidget *w;
    int left;
    int right;
};

}  // namespace qt_package

#endif // qt_package_MAIN_WINDOW_H
