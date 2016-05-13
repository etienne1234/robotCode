#ifndef DUMPVIEW_H
#define DUMPVIEW_H

#include <QWidget>
#include <QListWidgetItem>
#include <QCheckBox>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QProcess>
#include <QObject>
#include <QDebug>
#include <QFileDialog>
#include <QDir>
#include "exportdialog.h"

namespace Ui {
class DumpView;
}

/**
 * @brief The datasets dumping view
 * Displaying all topics available, and allowing user to check them or
 * not if he wants to dump one, or more topics at the same time. He also
 * can dump all topics at once, and stop dumping when he wants to.
 */
class DumpView : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief
     *
     * @param topicList The list of ros topics available
     * @param parent The HomeScreen main window
     */
    explicit DumpView(QStringList topicList, QWidget *parent = 0);
    /**
     * @brief DumpView destructor
     *
     */
    ~DumpView();
    void resizeEvent(QResizeEvent *event);

public slots:
    /**
     * @brief Slot connected to the "Dump All" button, which dumps all the
     * topics available in ros<property name="minimumSize">
   <size>
    <width>1024</width>
    <height>768</height>
   </size>
  </property>
  <property name="maximumSize">
   <size>
     *
     */
    void dumpAllLaunching();
    /**
     * @brief Slot connected to the "Dump" button, which dumps the checked
     * topics in our checkable list
     */
    void dumpLaunching();
    /**
     * @brief Slot connected to the "Stop" button, which stops EVERY dumping
     * process
     *
     */
    void stopDumping();
    /**
     * @brief Slot connected to the "Exit" button, which calls the destructor
     * of dumpView, and exits this view to get back to the HomeScreen view
     *
     */
    void terminate();
    /**
     * @brief Slot connected to the fileDialog signal, which returns the path
     * we selected in the QFileDialog to change the path in which we dump.
     * The "change" button below the QLineEdit containing the path is connected
     * to a QFileDialog displaying a dialog allowing the user to chose the path
     * where he wants to dump. This QFileDialog has a "fileSelected" signal, and
     * it's the one we connect this slot to.
     *
     * @param QString The path selected by the user in the QFileDialog, in which
     * we will dump.
     */
    void changeOutputDir(QString);
    /**
     * @brief exportToSlam Slots connected to the "Export libSlam" button, which
     * exports the previoulsy dumped rosbag to the input format of the SLAM library
     *
     */
    void exportToSlam();

signals:
    void dumpDeleted();

private:
    Ui::DumpView *ui; /**< Our DumpView user interface made with QtDesigner */
    QProcess *dumpProcess; /**< The dump proces */
    QString outputDir; /**< The output directory of the dumping process */
    QFileDialog fileDialog; /**< The QFileDialog allowing the user to chose the path where he wants to dump */
};

#endif // DUMPVIEW_H
