#include <QApplication>
#include "homescreen.h"
#include "ros/ros.h"



int main(int argc, char* argv[])
{

    QApplication a(argc,argv);

//    ros::init(argc, argv, "listener");

    HomeScreen h;
    h.show();

//    ros::spin();

    return a.exec();

}
