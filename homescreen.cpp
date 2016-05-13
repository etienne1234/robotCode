#include "homescreen.h"
#include "ui_homescreen.h"
#include <iostream>

HomeScreen::HomeScreen(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::HomeScreen)
{
    ui->setupUi(this);
    this->setWindowTitle("NaMuRo Robot Management");
    ui->gridLayout->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    this->setLayout(ui->gridLayout);

    //Buttons/actions connection
    connect(ui->pushButton,SIGNAL(pressed()),this,SLOT(launchDumpProcess()));
    connect(ui->pushButton_4,SIGNAL(pressed()),this,SLOT(terminate()));
    connect(ui->pushButton_2,SIGNAL(pressed()),this,SLOT(launchLocalisationProcess()));
    connect(ui->pushButton_3,SIGNAL(pressed()),this,SLOT(launchNavigationProcess()));
    connect(ui->connexionButton,SIGNAL(pressed()),this,SLOT(connectToRobot()));
    connect(ui->disconnexionButton,SIGNAL(pressed()),this,SLOT(disconnectToRobot()));

    changeButtonColor();


    QRadialGradient gradient(this->rect().center(),this->width()/2);
    gradient.setColorAt(0, QColor::fromRgbF(0, 1, 0, 1));
    gradient.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
    QBrush brush(gradient);
    QPalette palette;
    palette.setBrush(this->backgroundRole(), brush);
    this->setPalette(palette);

//     QTimer *myTimer = new QTimer(this);
//     connect(myTimer,SIGNAL(timeout()),this,SLOT(changeButtonColor()));
//     myTimer->start(1000);

}

//void HomeScreen::robotStateCallback(const kobuki_msgs::RobotStateEvent::ConstPtr& msg)
//{

//    if(msg.get()->state == 0)
//    {
//        ui->label->setPixmap(QPixmap(":/images/off.png"));;
//    }
//    else if(msg.get()->state == 1)
//    {
//        ui->label->setPixmap(QPixmap(":/images/on.png"));
//    }

//}

/**
 * @brief
 *
 */
void HomeScreen::terminate()
{

    this->~HomeScreen();

}

/**
 * @brief
 *
 */
HomeScreen::~HomeScreen()
{
    if(dumpView != NULL)
    {
        delete dumpView;
    }
    delete ui;
//    exit(0);
}

/**
 * @brief
 *
 */
void HomeScreen::launchDumpProcess()
{

    //The process which will return the list of ros topics available
    QProcess topicListProcess;
    //Same as : rostopic list in a terminal
    topicListProcess.start("bash",QStringList() << "-i" << "-c" << " rostopic list");
    topicListProcess.waitForFinished();
    //We get the standard output of rostopic list when the process is done
    //which contains all the topics available line by line
    QString tmp = topicListProcess.readAllStandardOutput();
    if(tmp != NULL)
    {
        //we split this list with the end of line character to get
        //a topic in each string
        QStringList topicList = tmp.split('\n');
        //we delete the last one as it is a blank space (always)
        topicList.pop_back();
        qDebug() << topicList;
        //we instantiate our dumpview with the topic list
        dumpView = new DumpView(topicList,this);
        connect(dumpView,SIGNAL(destroyed()),this,SLOT(setVisible()));
        //        emit topicListReady(topicList);
        this->setInvisible();
    }
    else
    {
        QMessageBox::warning(this,"Warning!", \
                             "No topic list returned, check that a ROS instance is launched");
    }

    topicListProcess.close();

}

void HomeScreen::setVisible()
{

    ui->connexionButton->show();
    ui->disconnexionButton->show();
    ui->label->show();
    ui->pushButton->show();;
    ui->pushButton_2->show();
    ui->pushButton_3->show();
    ui->pushButton_4->show();

}

/**
 * @brief
 *
 */
void HomeScreen::launchNavigationProcess()
{


//    kinectProcess.waitForFinished();

    QProcess depthProcess;
    depthProcess.start("bash",QStringList() << "-i" << "-c" << \
                        "rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/camera/depth/image_raw");
//    depthProcess.waitForFinished();

    QProcess navigationProcess;
    navigationProcess.startDetached("bash",QStringList() << "-i" << "-c" << "./exec");

//    QMessageBox tmp(this);
//    tmp.setStandardButtons(0);
//    tmp.setWindowTitle("Navigating !!");
//    tmp.setText("Navigating, please wait for the robot to end its cartography");

//    connect(&navigationProcess,SIGNAL(finished(int,QProcess::ExitStatus)),&tmp,SLOT(close()));
//    connect(&navigationProcess,SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(killNavigationProcesses()));

//    tmp.show();

    depthProcess.close();

}

void HomeScreen::killNavigationProcesses()
{

    QProcess stopNavigationProcesses1;
    stopNavigationProcesses1.start("bash",QStringList() << "-c" << "-i" << "pgrep depthimage_to_laserscan");
    stopNavigationProcesses1.waitForFinished();
    QString stopNavigationProcesses1ID = stopNavigationProcesses1.readAll();
    QStringList stopNavigationProcesses1IDS = stopNavigationProcesses1ID.split("\n");
    stopNavigationProcesses1IDS.pop_back();
    qDebug() << stopNavigationProcesses1IDS;
    QProcess::execute("kill",QStringList() << "-INT" << stopNavigationProcesses1IDS);

    QProcess stopNavigationProcesses;
    stopNavigationProcesses.start("bash",QStringList() << "-c" << "-i" << "pgrep exec");
    stopNavigationProcesses.waitForFinished();
    QString stopNavigationProcessID = stopNavigationProcesses.readAll();
    QStringList stopNavigationProcessIDS = stopNavigationProcessID.split("\n");
    stopNavigationProcessIDS.pop_back();
    qDebug() << stopNavigationProcessIDS;
    QProcess::execute("kill",QStringList() << "-INT" << stopNavigationProcessIDS);
}

/**
 * @brief
 *
 */
void HomeScreen::launchLocalisationProcess()
{

    QProcess localisationProcess;
    localisationProcess.start("bash",QStringList() << "-i" << "-c" << "rqt -p localisation");
    localisationProcess.waitForFinished();
    localisationProcess.close();
}

void HomeScreen::connectToRobot()
{
    QMessageBox::warning(this,"Warning!!","Check the robot is plugged before connecting");
    QProcess kinectProcess;
    kinectProcess.start("bash",QStringList() << "-i" << "-c" << "roslaunch freenect_launch freenect.launch");

    QDialog tmp;
    tmp.setWindowTitle("Connecting to the robot please wait ...");
    tmp.setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowStaysOnTopHint);
    tmp.setFixedSize(QSize(300,0));
    tmp.setModal(true);
    tmp.show();

    kinectProcess.waitForFinished();
    kinectProcess.close();

    QProcess connexionProcess;
    connexionProcess.start("bash",QStringList() << "-i" << "-c" << "roslaunch turtlebot_bringup minimal.launch");

    QObject::connect(&kinectProcess,SIGNAL(finished(int,QProcess::ExitStatus)),&tmp,SLOT(close()));

    connexionProcess.waitForFinished();
    connexionProcess.close();



    changeButtonColor();

}

void HomeScreen::disconnectToRobot()
{

    QProcess stopRoslaunch;
    stopRoslaunch.start("bash",QStringList() << "-c" << "-i" << "pgrep roslaunch");
    stopRoslaunch.waitForFinished();
    QString roslaunchProcessID = stopRoslaunch.readAll();
    QStringList roslaunchProcessIDS = roslaunchProcessID.split("\n");
    roslaunchProcessIDS.pop_back();
    QProcess::execute("kill",QStringList() << "-INT" << roslaunchProcessIDS);
    stopRoslaunch.close();

    QProcess stopNodelet;
    stopNodelet.start("bash",QStringList() << "-c" << "-i" << "pgrep nodelet");
    stopNodelet.waitForFinished();
    QString nodeletProcessID = stopNodelet.readAll();
    QStringList nodeletProcessIDS = nodeletProcessID.split("\n");
    nodeletProcessIDS.pop_back();
    QProcess::execute("kill",QStringList() << "-INT" << nodeletProcessIDS);
    stopNodelet.close();

    QProcess stopPython;
    stopPython.start("bash",QStringList() << "-c" << "-i" << "pgrep python");
    stopPython.waitForFinished();
    QString pythonProcessID = stopPython.readAll();
    QStringList pythonProcessIDS = pythonProcessID.split("\n");
    pythonProcessIDS.pop_back();
    QProcess::execute("kill",QStringList() << "-INT" << pythonProcessIDS);
    stopPython.close();

    QProcess stopAggregator;
    stopAggregator.start("bash",QStringList() << "-c" << "-i" << "pgrep aggregator_node");
    stopAggregator.waitForFinished();
    QString aggregatorProcessID = stopAggregator.readAll();
    QStringList aggregatorProcessIDS = aggregatorProcessID.split("\n");
    aggregatorProcessIDS.pop_back();
    QProcess::execute("kill",QStringList() << "-INT" << aggregatorProcessIDS);
    stopAggregator.close();

    QProcess stopRobotState;
    stopRobotState.start("bash",QStringList() << "-c" << "-i" << "pgrep robot_s");
    stopRobotState.waitForFinished();
    QString robotStateProcessID = stopRobotState.readLine();
    QStringList robotStateProcessIDS = robotStateProcessID.split("\n");
    robotStateProcessIDS.pop_back();
    qDebug() << robotStateProcessID;
    QProcess::execute("kill",QStringList() << robotStateProcessIDS);
    stopRobotState.close();

    QProcess stopRosout;
    stopRosout.start("bash",QStringList() << "-c" << "-i" << "pgrep rosout");
    stopRosout.waitForFinished();
    QString rosoutProcessID = stopRosout.readAll();
    QStringList rosoutProcessIDS = rosoutProcessID.split("\n");
    rosoutProcessIDS.pop_back();
    QProcess::execute("kill",QStringList() << "-INT" << rosoutProcessIDS);
    stopRosout.close();

    QProcess stopRosmaster;
    stopRosmaster.start("bash",QStringList() << "-c" << "-i" << "pgrep rosmaster");
    stopRosmaster.waitForFinished();
    QString rosmasterProcessID = stopRosmaster.readAll();
    QStringList rosmasterProcessIDS = rosmasterProcessID.split("\n");
    rosmasterProcessIDS.pop_back();
    QProcess::execute("kill",QStringList() << "-INT" << rosmasterProcessIDS);
    stopRosmaster.close();

    QProcess stopKinectmaster;
    stopKinectmaster.start("bash",QStringList() << "-c" << "-i" << "pgrep static_transform_publisher");
    stopKinectmaster.waitForFinished();
    QString KinectmasterProcessID = stopKinectmaster.readAll();
    QStringList KinectmasterProcessIDS = KinectmasterProcessID.split("\n");
    KinectmasterProcessIDS.pop_back();
    QProcess::execute("kill",QStringList() << "-INT" << KinectmasterProcessIDS);
    stopKinectmaster.close();

    changeButtonColor();

}

void HomeScreen::changeButtonColor()
{

    QProcess testStateProcess;
    testStateProcess.start("bash",QStringList() << "-i" << "-c" << "rostopic echo /mobile_base/version_info");
    testStateProcess.waitForFinished(2000);
    QString testString = testStateProcess.readAll();
    if(testString.isEmpty() || testString == "ERROR: Unable to communicate with master!")
    {
        ui->label->setPixmap(QPixmap(":/images/off.png"));;
    }
    else
    {
        ui->label->setPixmap(QPixmap(":/images/on.png"));
    }
    testStateProcess.close();
}

void HomeScreen::setInvisible()
{

    ui->connexionButton->hide();
    ui->disconnexionButton->hide();
    ui->label->hide();
    ui->pushButton->hide();;
    ui->pushButton_2->hide();
    ui->pushButton_3->hide();
    ui->pushButton_4->hide();

}

void HomeScreen::resizeEvent(QResizeEvent *)
{


    QRadialGradient gradient(this->rect().center(),this->width()/2);
    gradient.setColorAt(0, QColor::fromRgbF(0, 1, 0, 1));
    gradient.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
    QBrush brush(gradient);
    QPalette palette;
    palette.setBrush(this->backgroundRole(), brush);
    this->setPalette(palette);

}
