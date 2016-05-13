#ifndef EXPORTOPTIONSDIALOG_H
#define EXPORTOPTIONSDIALOG_H

#include <QDialog>

namespace Ui {
class ExportOptionsDialog;
}

class ExportOptionsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ExportOptionsDialog(QWidget *parent = 0);
    ~ExportOptionsDialog();

public slots:
    void sendParameters();

signals:
    void sendPlayRate(double playRate);
    void sendOutputImageName(QString imageName);


private:
    Ui::ExportOptionsDialog *ui;
};

#endif // EXPORTOPTIONSDIALOG_H
