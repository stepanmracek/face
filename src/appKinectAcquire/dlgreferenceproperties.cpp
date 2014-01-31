#include "dlgreferenceproperties.h"
#include "ui_dlgreferenceproperties.h"

DlgReferenceProperties::DlgReferenceProperties(const QString &name, const QMap<int, QString> &mapIdToName,
                                               const QMap<QString, int> &mapNameToId,
                                               const QHash<int, Face::Biometrics::FaceTemplate *> &database,
                                               QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DlgReferenceProperties)
{
    ui->setupUi(this);

    ui->leName->setText(name);
    int id = mapNameToId[name];
    ui->leId->setText(QString::number(id));
    ui->leReferenceCount->setText(QString::number(database.values(id).count()));
}

DlgReferenceProperties::~DlgReferenceProperties()
{
    delete ui;
}
