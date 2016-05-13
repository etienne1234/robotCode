#include "dumpview.h"
#include "ui_dumpview.h"

DumpView::DumpView(QStringList topicList, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DumpView)
{
    this->setParent(parent);
    ui->setupUi(this);
    ui->gridLayout->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    this->setLayout(ui->gridLayout);
    parent->layout()->addWidget(this);
    //We set the geometry of this exactly like its parent's
//    setGeometry(parent->x(),parent->y(),parent->width(),parent->height());
    //Flag to tell Qt to let this view on Top of the others until the user exits it
//    this->setWindowFlags(Qt::WindowStaysOnTopHint);
//    ui->gridLayout->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
//    this->setLayout(ui->gridLayout);
//    this->setAutoFillBackground(true);

    //We fill our checkable topic list with the topicList constructor input object
    if(!topicList.isEmpty())
    {
        for(int i = 0 ; i < topicList.size() ; i++)
        {
            QListWidgetItem *item = new QListWidgetItem(topicList.at(i),ui->topicListView);
            //Flags to set the items enabled and make them checkable by the user
            item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
            item->setCheckState(Qt::Unchecked);//unchecked initial status for every topic
            ui->topicListView->addItem(item);
        }
    }
    else
    {
        qDebug("La liste de topics est vide! On ne peut pas remplir la liste");
    }

    //Buttons/actions connection
    connect(ui->CancelButton,SIGNAL(pressed()),this,SLOT(terminate()));
    connect(ui->DumpButton,SIGNAL(pressed()),this,SLOT(dumpLaunching()));
    connect(ui->StopDumpButton,SIGNAL(pressed()),this,SLOT(stopDumping()));
    connect(ui->DumpAllButton,SIGNAL(pressed()),this,SLOT(dumpAllLaunching()));
    connect(ui->exportButton,SIGNAL(pressed()),this,SLOT(exportToSlam()));

    dumpProcess = NULL;

    //We fill the QLineEdit containing the path with the current working path, and make it only readable
    ui->outputDirLineEdit->setReadOnly(true);
    ui->outputDirLineEdit->setText(QDir::currentPath());

    fileDialog.setFileMode(QFileDialog::Directory);//The QFileDialog in which the user can select its output directory,
                                                   //this function is used to tell the QFileDialog that it's a directory
                                                   //we want and not a file
    connect(ui->changeOutputDirButton,SIGNAL(pressed()),&fileDialog,SLOT(exec()));
    connect(&fileDialog,SIGNAL(fileSelected(QString)),this,SLOT(changeOutputDir(QString)));

    show();

}

DumpView::~DumpView()
{
    delete dumpProcess;
    delete ui;
}

void DumpView::terminate()
{
    emit dumpDeleted();
    this->~DumpView();
}

void DumpView::changeOutputDir(QString urlDir)
{

    outputDir = urlDir;
    outputDir.push_back('/');
    ui->outputDirLineEdit->setText(outputDir);

}

void DumpView::dumpLaunching()
{

    if(dumpProcess == NULL)
    {
        dumpProcess = new QProcess(this);
    }
    else
    {
        dumpProcess->terminate();
        dumpProcess->~QProcess();
        dumpProcess = new QProcess(this);
    }

    QStringList checkedTopicsList;
    QString processString = "rosbag record --output-prefix="+outputDir;
    for(int i = 0 ; i < ui->topicListView->count() ; i++)
    {    qDebug() << this->parentWidget()->rect().width() << this->parentWidget()->rect().height();
        this->setGeometry(this->parentWidget()->rect());
        if(ui->topicListView->item(i)->checkState() == Qt::Checked)
        {
//            qDebug() << "haha";
            processString = processString+" "+ui->topicListView->item(i)->text();
            checkedTopicsList << ui->topicListView->item(i)->text();
        }
    }
    qDebug() << processString;
    qDebug() << checkedTopicsList;

    dumpProcess->start("bash",QStringList() << "-c" << "-i" << processString);

}

void DumpView::dumpAllLaunching()
{

    if(dumpProcess == NULL)
    {
        dumpProcess = new QProcess(this);
    }
    else
    {
        dumpProcess->terminate();
        dumpProcess->~QProcess();
        dumpProcess = new QProcess(this);
    }

    QString processString = "rosbag record -a --output-prefix="+outputDir;
    dumpProcess->start("bash",QStringList() << "-c" << "-i" << processString);


}

void DumpView::stopDumping()
{    qDebug() << this->parentWidget()->rect().width() << this->parentWidget()->rect().height();
     this->setGeometry(this->parentWidget()->rect());

    QProcess stopDumpProcess;
    stopDumpProcess.start("bash",QStringList() << "-c" << "-i" << "pgrep record");
    stopDumpProcess.waitForFinished();
    QString dumpProcessID = stopDumpProcess.readAll();
    QStringList dumpProcessIDS = dumpProcessID.split("\n");
    dumpProcessIDS.pop_back();
    qDebug() << dumpProcessIDS;
//    QProcess::execute("bash", QStringList() << "-c" << "-i" << dumpProcessID);
    QProcess::execute("kill",QStringList() << "-INT" << dumpProcessIDS);
}

void DumpView::exportToSlam()
{

    exportDialog exprtDialog;
    exprtDialog.exec();

}

void DumpView::resizeEvent(QResizeEvent *event)
{

    qDebug() << this->parentWidget()->rect().width() << this->parentWidget()->rect().height();
    this->setGeometry(this->parentWidget()->rect());

}

