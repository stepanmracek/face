#include "dlgreferenceproperties.h"
#include "ui_dlgreferenceproperties.h"

DlgReferenceProperties::DlgReferenceProperties(const QString &name,
                                               const FrmKinectMain::Database &database,
                                               QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DlgReferenceProperties)
{
    ui->setupUi(this);

    ui->leName->setText(name);
    int id = database.mapNameToId.at(name);
    ui->leId->setText(QString::number(id));
    ui->leReferenceCount->setText(QString::number(database.scans.at(id).size()));
}

DlgReferenceProperties::~DlgReferenceProperties()
{
    delete ui;
}
