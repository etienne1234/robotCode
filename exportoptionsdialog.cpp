#include "exportoptionsdialog.h"
#include "ui_exportoptionsdialog.h"

ExportOptionsDialog::ExportOptionsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ExportOptionsDialog)
{
    ui->setupUi(this);
    connect(ui->OKButton,SIGNAL(pressed()),this,SLOT(sendParameters()));
    connect(ui->OKButton,SIGNAL(pressed()),this,SLOT(close()));
    connect(ui->cancelButton,SIGNAL(pressed()),this,SLOT(close()));

    this->setFixedSize(399,195);
}

ExportOptionsDialog::~ExportOptionsDialog()
{
    delete ui;
}

void ExportOptionsDialog::sendParameters()
{

    emit sendPlayRate(ui->playRateValue->value());
    emit sendOutputImageName(ui->outputImageValue->text());

}
