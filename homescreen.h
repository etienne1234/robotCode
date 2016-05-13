#ifndef HOMESCREEN_H
#define HOMESCREEN_H

#include <QMainWindow>
#include <QProcess>
#include <QProcessEnvironment>
#include <QList>
#include <QByteArray>
#include <QDebug>
#include <QMessageBox>
#include <QPalette>
#include <QPixmap>
#include <QLabel>
#include <QTextBlock>
#include <QTimer>
#include <kobuki_msgs/RobotStateEvent.h>
#include "dumpview.h"
#include <QRadialGradient>
#include <QPalette>
#include <QBrush>
//#include "ros/ros.h"
//#include "ros/console.h"
//#include "XmlRpc.h"

namespace Ui {
class HomeScreen;
}

/**
 * @brief Main window f  ui->label->setPixmap(QPixmap(":/images/on.png")or our application, proposing the 3 functionnality
 * - Dumping datasets mode
 * - Navigation display mode
 * - Localisation display mode
 */
class HomeScreen : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief HomeScreen constructor
     *
     * @param parent As this is the main window it has no parent
     */
    explicit HomeScreen(QWidget *parent = 0);
    /**
     * @brief HomeScreen destructor
     * Deletes the views if the are not NULL, deletes ui, and exit(0)
     */
    ~HomeScreen();
//    void robotStateCallback(const kobuki_msgs::RobotStateEvent::ConstPtr& msg);
    void setInvisible();
    void changeButtonColor();
    void resizeEvent(QResizeEvent *);

public slots:
    /**
     * @brief Slot connected to the "Dump Datasets" button, which displays the Dumping view
     *
     */
    void launchDumpProcess();
    /**
     * @brief Slot connected to the "Navigation Display" button, which displays the Navigation view
     *
     */
    void launchNavigationProcess();
    /**
     * @brief Slot connected to the "Localisation Display" button, which displays the Localisation view
     *
     */
    void launchLocalisationProcess();
    /**
     * @brief Slot connected to the "Exit" button, which launches the destructor of HomeScreen
     *
     */
    void terminate();
    void connectToRobot();
    void disconnectToRobot();
//    void onConnectProcessFinished(int,QProcess::ExitStatus);
    void setVisible();
    void killNavigationProcesses();

signals:
//    void topicListReady(const QStringList &topicList);

private:
    Ui::HomeScreen *ui; /**< Our HomeScreen user interface made with QtDesigner */
    DumpView *dumpView; /**< The datasets dumping view */
//    ros::NodeHandle robotStateNodeHandler;
//    ros::Subscriber robotStateSubscriber;
//    QWidget *navigationView;
//    QWidget *localisationView;
};

#endif // HOMESCREEN_H
