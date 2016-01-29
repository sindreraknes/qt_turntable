/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/qt_package/main_window.hpp"

#include <pcl/common/common.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_package {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr >("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    // Setup ui and ros
    ui.setupUi(this);
    setWindowIcon(QIcon(":/images/icon2.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    // Signal for angles
    QObject::connect(&qnode, SIGNAL(sendCurrentAngle(double)), this, SLOT(readCurrentAngle(double)));
    QObject::connect(this, SIGNAL(setAngle(double)), &qnode, SLOT(setAngle(double)));
    // Signals for topic list in drop-down box
    QObject::connect(this, SIGNAL(getTopics()), &qnode, SLOT(findTopics()));
    QObject::connect(&qnode, SIGNAL(sendTopics(QStringList)), this, SLOT(updateTopics(QStringList)));
    // Signals for subscribing to pointcloud2 topic
    QObject::connect(this, SIGNAL(subscribeToPointCloud2(QString,bool)), &qnode, SLOT(subscribeToPointCloud2(QString,bool)));
    // Signal for taking picture
    QObject::connect(this, SIGNAL(takePictures(int, QString, bool)), &qnode, SLOT(takePicture(int, QString, bool)));
    // Signal for PointCloud visualizer
    QObject::connect(&qnode, SIGNAL(setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)), this, SLOT(getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)));
    QObject::connect(&qnode, SIGNAL(displayImage(QString)), this, SLOT(displayImage(QString)));
    // Init ROS
    qnode.init();

    this->showMaximized();

    // Initials
    ui.textbox_path->setText("/home/minions");
    ui.button_take_pic->setEnabled(false);
    ui.rgb_checkbox->setEnabled(false);

    //TEST AREA 1001
    left = 0;
    right = 1;
    w = new QVTKWidget();
    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer", false));

    w->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(w->GetInteractor(), w->GetRenderWindow());

    viewer->createViewPort(0, 0, 0.5, 1, left);
    viewer->createViewPort(0.5, 0, 1, 1, right);
    w->update();
    viewer->setCameraPosition( 0.146598, 0.0941454, -4.95334, -0.0857047, -0.0396425, 0.600109, -0.00146821, -0.999707, -0.0241453, 0);
    viewer->updateCamera();

    ui.verticalLayout->addWidget(w);


}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

 // ********* SLOTS ********* //
// ************************* //
void MainWindow::readCurrentAngle(double d){
    ui.current_position->display(d);
}

void MainWindow::updateTopics(QStringList list)
{
    ui.comboBox->clear();
    ui.comboBox->addItems(list);
}


 // ********* BUTTON AUTO ********* //
// ******************************** //

void MainWindow::on_button_set_pos_clicked(bool check)
{
    Q_EMIT setAngle(ui.set_position_input->value());
}

void MainWindow::on_button_refresh_topic_clicked(bool check)
{
    Q_EMIT getTopics();
}

void MainWindow::on_button_subscribe_topic_clicked(bool check)
{
    if(ui.comboBox->currentText().length() != 0){
        ui.button_take_pic->setEnabled(true);
        Q_EMIT subscribeToPointCloud2(ui.comboBox->currentText(), ui.rgb_checkbox->isChecked());
    }
}

void MainWindow::on_button_folder_select_clicked(bool check)
{
    QString fileName;
    fileName = QFileDialog::getExistingDirectory(this, tr("Choose Or Create Directory"),"/home/minions",QFileDialog::DontResolveSymlinks);
    ui.textbox_path->setText(fileName);
}

void MainWindow::on_button_take_pic_clicked(bool check)
{
    QString url = ui.textbox_path->toPlainText();
    url.append("/");
    url.append(ui.fileName->text());
    Q_EMIT takePictures(ui.set_nrof_pic->value(), url, ui.checkLoad->isChecked());
}

void MainWindow::on_button_load_picture_clicked(bool check)
{
    QString fileName;
    QString tmp = ui.textbox_path->toPlainText();
    QString filters = "PointClouds (*.pcd);; PointClouds (*.PCD)";
    if(tmp.length() == 0){
        fileName = QFileDialog::getExistingDirectory(this, tr("Choose Or Create Directory"),"/home/minions",QFileDialog::DontResolveSymlinks);
    }
    else{
        fileName = QFileDialog::getOpenFileName(this,tr("Choose a .pcd file to open"),tmp,tr("PointClouds (*.pcd *.PCD)"));
    }

    displayImage(fileName);
}

void MainWindow::getPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr c)
{

    if(!viewer->updatePointCloud(c, "cloud")){
        viewer->addPointCloud(c, "cloud", left);
        w->update();
    }
    w->update();
}

void MainWindow::displayImage(QString url)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(url.toUtf8().constData(), *displayCloud) == -1){
        std::cout << "Could not load file" << std::endl;
        return;
    }

    if(!viewer->updatePointCloud(displayCloud, "displayCloud")){
        viewer->addPointCloud(displayCloud, "displayCloud", right);
        w->update();
    }
    w->update();
}


}  // namespace qt_package

