#ifndef EXPORTDIALOG_H
#define EXPORTDIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <QMessageBox>
#include <QProcess>
#include <QListWidgetItem>
#include <QDir>
#include <QDebug>
#include <QTextStream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <QProgressBar>
#include <QFileSystemWatcher>
#include "exportoptionsdialog.h"

namespace Ui {
class exportDialog;
}

class exportDialog : public QDialog
{
    Q_OBJECT

public:
    explicit exportDialog(QWidget *parent = 0);
    ~exportDialog();

public slots:
    void displayTopicLists(QString);
    void displayMoreOptions();
    void exportData();
    void convertOdom();
    void convertImages(QString);
    void stopImagesProcess(int,QProcess::ExitStatus);
    void setPlayRate(double);
    void setOutputImageName(QString);

private:
    Ui::exportDialog *ui;
    QFileDialog fileDialog;
    QString currentBagName;
    QString bagFullPath;
    QString outputImageName;
    QStringList currentTopics;
    QStringList checkedTopicsList;
    rosbag::Bag bag;
    double imageReadingSpeed;

};

#endif // EXPORTDIALOG_H
