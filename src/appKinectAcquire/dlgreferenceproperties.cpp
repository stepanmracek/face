#include "dlgreferenceproperties.h"
#include "ui_dlgreferenceproperties.h"

DlgReferenceProperties::DlgReferenceProperties(const QString &name, const QHash<int, QString> &hashIdToName,
                                               const QHash<QString, int> &hashNameToId, const QHash<int, FaceTemplate> &database,
                                               QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DlgReferenceProperties)
{
    ui->setupUi(this);

    ui->leName->setText(name);
    int id = hashNameToId[name];
    ui->leId->setText(QString::number(id));
    ui->leReferenceCount->setText(QString::number(database.values(id).count()));
}

DlgReferenceProperties::~DlgReferenceProperties()
{
    delete ui;
}
