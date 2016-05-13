#include "exportdialog.h"
#include "ui_exportdialog.h"

exportDialog::exportDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::exportDialog)
{
    ui->setupUi(this);
    fileDialog.setDefaultSuffix("bag");
    connect(ui->fileDialogButton,SIGNAL(pressed()),&fileDialog,SLOT(exec()));
    connect(&fileDialog,SIGNAL(fileSelected(QString)),this,SLOT(displayTopicLists(QString)));
    connect(ui->optionsButton,SIGNAL(pressed()),this,SLOT(displayMoreOptions()));
    connect(ui->exportButton,SIGNAL(pressed()),this,SLOT(exportData()));

    this->setFixedSize(400,382);

//    bag = new rosbag::Bag();

    imageReadingSpeed = 0.5;
    outputImageName = "image_%05d.jpg";

}

exportDialog::~exportDialog()
{
    delete ui;
}

void exportDialog::displayTopicLists(QString fileName)
{
    bagFullPath = fileName;
    currentBagName = fileName;
    QProcess topicListProcess;
    //Same as : rostopic list in a terminal
    topicListProcess.start("bash",QStringList() << "-i" << "-c" << " rosbag info "+currentBagName);
    topicListProcess.waitForFinished();
    QString topicsString = topicListProcess.readAllStandardOutput();

    int tmp = currentBagName.lastIndexOf("/");
    currentBagName.remove(1,tmp);
    currentBagName.remove(".bag");
    qDebug() << currentBagName;
    ui->topicsListWidget->clear();

    //    view.addQuery(bag);
    //    std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();

    //We get the standard output of rostopic list when the process is done
    //which contains all the topics available line by line
    //    QString topicsString = topicListProcess.readAllStandardOutput();
    //    if(topicsString != NULL)
    //    {
    //we split this list with the end of line character to get
    //a topic in each string
    //        currentTopics = topicsString.split('\n');
    //we delete the last one as it is a blank space (always)
    //        currentTopics.pop_back();
    //    }

    if(topicsString.contains("/odom"))
    {
        QListWidgetItem *item = new QListWidgetItem("/odom",ui->topicsListWidget);
        //Flags to set the items enabled and make them checkable by the user
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);//unchecked initial status for every topic
        ui->topicsListWidget->addItem(item);
    }
    if(topicsString.contains("/camera/image_raw"))
    {
        QListWidgetItem *item = new QListWidgetItem("/camera/image_raw",ui->topicsListWidget);
        //Flags to set the items enabled and make them checkable by the user
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);//unchecked initial status for every topic
        ui->topicsListWidget->addItem(item);
    }
    if(topicsString.contains("/usb_cam/image_raw"))
    {
        QListWidgetItem *item = new QListWidgetItem("/usb_cam/image_raw",ui->topicsListWidget);
        //Flags to set the items enabled and make them checkable by the user
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);//unchecked initial status for every topic
        ui->topicsListWidget->addItem(item);
    }

    bag.open(fileName.toStdString(),rosbag::bagmode::Read);

    QMessageBox::warning(this,"Warning!","The images export can be very long, you can chose the speed at which the rosbag will be played, it is 50% slower by default, but for huge files it might not be enough. Consider reducing it to get the correct number of frames, you might lose some if you don't, depending on your computer performances aswell, press ""More Options"" to set this parameter (we play the rosbag file and extract images from it while playing it).");


    //Get the list of topics recorded in the bag
    //    foreach(const rosbag::ConnectionInfo *info, connection_infos) {
    //        currentTopics.append(QString::fromStdString(info->topic));
    //        QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(info->topic),ui->topicsListWidget);
    //        //Flags to set the items enabled and make them checkable by the user
    //        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
    //        item->setCheckState(Qt::Unchecked);//unchecked initial status for every topic
    //        ui->topicsListWidget->addItem(item);
    //    }

}

void exportDialog::exportData()
{

    QDir myDir;
    myDir.mkdir("./"+currentBagName);

    for(int i = 0 ; i < ui->topicsListWidget->count() ; i++)
    {
        if(ui->topicsListWidget->item(i)->checkState() == Qt::Checked)
        {
            checkedTopicsList << ui->topicsListWidget->item(i)->text();
        }
    }

    if(checkedTopicsList.contains("/odom"))
    {
        convertOdom();
    }
    if(checkedTopicsList.contains("/camera/image_raw"))
    {
        convertImages("/camera/image_raw");
    }
    if(checkedTopicsList.contains("/usb_cam/image_raw"))
    {
        convertImages("/usb_cam/image_raw");
    }

}

void exportDialog::convertOdom()
{


    QFile odomFile("./"+currentBagName+"/odom.txt");
    if(!odomFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Error, odom txt file could not be open";
    }
    QTextStream outStream(&odomFile);

    rosbag::View odomView(bag,rosbag::TopicQuery("/odom"));

    double x1,y1 = 0;
    double x2,y2 = 0;
    double distance = 0;
    bool firstIter = true;
    QString timestamp;

    BOOST_FOREACH(rosbag::MessageInstance const m, odomView)
    {
        nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
        if (odom_msg != NULL){
            const nav_msgs::Odometry *mps = odom_msg.get();
            if(firstIter)
            {
                x1 = mps->pose.pose.position.x; y1 = mps->pose.pose.position.y;
                //                distance += sqrt(pow(x1,2)+pow(y1,2));
                timestamp = QString::number(mps->header.stamp.sec)+QString::number(mps->header.stamp.nsec);
                timestamp.remove(16,3);
                outStream << timestamp << " " << distance << endl;
                firstIter = false;
            }
            else
            {
                x2 = mps->pose.pose.position.x; y2 = mps->pose.pose.position.y;
                distance += sqrt(pow(x2-x1,2)+pow(y2-y1,2));
                timestamp = QString::number(mps->header.stamp.sec)+QString::number(mps->header.stamp.nsec);
                timestamp.remove(16,3);
                outStream << timestamp << " " << distance << endl;
                x1 = x2;
                y1 = y2;
            }
        }
    }
    odomFile.close();
}

void exportDialog::convertImages(QString topic)
{

    rosbag::View imageView(bag,rosbag::TopicQuery(topic.toStdString()));

    QString nameTmp = topic;
    nameTmp.replace("/","_");

    QFile imageFile("./"+currentBagName+"/"+nameTmp+".txt");
    qDebug() << "./"+currentBagName+"/"+nameTmp+".txt";
    if(!imageFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Error, image txt file could not be open";
    }
    QTextStream outStream(&imageFile);

    double timeOfBag = imageView.getEndTime().toSec()-imageView.getBeginTime().toSec();
    int cpt = 0;
    QString timestamp;

    BOOST_FOREACH(rosbag::MessageInstance const m, imageView)
    {
        cpt++;
        sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
        if (image_msg != NULL){
            const sensor_msgs::Image *mps = image_msg.get();
            timestamp = QString::number(mps->header.stamp.sec)+QString::number(mps->header.stamp.nsec);
            timestamp.remove(16,3);
            outStream << timestamp << endl;
        }
    }

    qDebug() << "on est censés avoir fini le foreach";

    imageFile.close();

    double secsPerFrame = 1/(cpt/timeOfBag);

    QProcess *rosbagPlay = new QProcess();
    rosbagPlay->setProgram("bash");
    rosbagPlay->setArguments(QStringList() << "-i" << "-c" << " rosbag play -r "+QString::number(imageReadingSpeed)+" "+bagFullPath);
    QProcess *imageExtraction = new QProcess();
    imageExtraction->setProgram("bash");
    imageExtraction->setArguments(QStringList() << "-i" << "-c" << " rosrun image_view extract_images _sec_per_frame:=" \
                                  +QString::number(secsPerFrame)+" _filename_format:="+outputImageName+" image:=" \
                                  +topic);
    qDebug() << imageExtraction->arguments();

    QDir image_dir;
    image_dir.mkdir("./"+currentBagName+"/images"+nameTmp);
    //    QFileSystemWatcher myWatcher("./images"+nameTmp);
    //    connect(&myWatcher,SIGNAL(directoryChanged(QString)),this,SLOT(stopImagesProcess(QString)));
    imageExtraction->setWorkingDirectory("./"+currentBagName+"/images"+nameTmp);
    qDebug() << imageExtraction->workingDirectory();

    imageExtraction->start();
    imageExtraction->waitForStarted();
    rosbagPlay->start();

    QMessageBox tmp(this);
    tmp.setStandardButtons(0);
    tmp.setWindowTitle("Extracting images");
    tmp.setText("Extracting images, this might take a long time please wait..");

    connect(rosbagPlay,SIGNAL(finished(int,QProcess::ExitStatus)),this,SLOT(stopImagesProcess(int,QProcess::ExitStatus)));
    connect(rosbagPlay,SIGNAL(finished(int,QProcess::ExitStatus)),&tmp,SLOT(close()));

    tmp.show();

//    QDialog tmp;
//    tmp.setWindowTitle("Extracting images, this might take a long time ..");
//    tmp.setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowStaysOnTopHint);
//    tmp.setFixedSize(QSize(300,0));
//    tmp.setModal(true);
//    tmp.exec();
//    QProgressBar progressBar(tmp);
//    progressBar.setMinimum(0);
//    progressBar.setMaximum(cpt);


}

void exportDialog::stopImagesProcess(int,QProcess::ExitStatus)
{

    QProcess stopExtractImagesProcess;
    stopExtractImagesProcess.start("bash",QStringList() << "-c" << "-i" << "pgrep extract_images");
    stopExtractImagesProcess.waitForFinished();
    QString ExtractImagesProcessID = stopExtractImagesProcess.readAll();
    QStringList ExtractImagesProcessIDS = ExtractImagesProcessID.split("\n");
    ExtractImagesProcessIDS.pop_back();
    qDebug() << ExtractImagesProcessIDS;
    QProcess::execute("kill",QStringList() << "-INT" << ExtractImagesProcessIDS);

}

void exportDialog::displayMoreOptions()
{

    ExportOptionsDialog optionsDialog;
    connect(&optionsDialog,SIGNAL(sendPlayRate(double)),this,SLOT(setPlayRate(double)));
    connect(&optionsDialog,SIGNAL(sendOutputImageName(QString)),this,SLOT(setOutputImageName(QString)));
    optionsDialog.exec();

}

void exportDialog::setPlayRate(double playRate)
{

    imageReadingSpeed = playRate;
    qDebug() << "Play Rate of rosbag changed to : "+QString::number(imageReadingSpeed);

}

void exportDialog::setOutputImageName(QString outputName)
{

    outputImageName = outputName;
    qDebug() << "Output image name : "+outputImageName;

}
