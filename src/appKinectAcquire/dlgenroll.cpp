#include "dlgenroll.h"
#include "ui_dlgenroll.h"

DlgEnroll::DlgEnroll(QMap<int, QString> &mapIdToName, QMap<QString, int> mapNameToId,
                     QHash<int, FaceTemplate *> database, const FaceClassifier &classifier, QWidget *parent) :
    mapIdToName(mapIdToName), mapNameToId(mapNameToId), database(database), classifier(classifier),
    QDialog(parent), ui(new Ui::DlgEnroll)
{
    ui->setupUi(this);
}

DlgEnroll::~DlgEnroll()
{
    delete ui;
}

void DlgEnroll::on_btnAdd_clicked()
{
    // TODO: replace with proper code l8
    int count = ui->listScans->count();
    scans << Mesh::fromBIN("../../test/kinect/01-0" + QString::number(count+1) + ".bin", false);
    // end of TODO

    ui->listScans->addItem(QString::number(ui->listScans->count()+1));
    ui->listScans->item(ui->listScans->count()-1)->setSelected(true);
}

void DlgEnroll::on_btnRemove_clicked()
{
    QModelIndexList selection = ui->listScans->selectionModel()->selectedIndexes();
    if (selection.count() < 1) return;
    int index = selection[0].row();

    qDeleteAll(ui->listScans->selectedItems());
    scans.removeAt(index);
}

void DlgEnroll::on_listScans_itemSelectionChanged()
{
    QList<QListWidgetItem*> items = ui->listScans->selectedItems();
    ui->btnRemove->setEnabled(items.count() > 0);

    ui->glWidget->clearAll();
    if (items.count() > 0)
    {
        int index = ui->listScans->selectionModel()->selectedIndexes()[0].row();
        ui->glWidget->addFace(&scans[index]);
    }
    ui->glWidget->updateGL();
}

void DlgEnroll::on_buttonBox_accepted()
{
    // check name
    QString name = ui->leName->text();
    if (name.isEmpty() || name.isNull())
    {
        QMessageBox msg(QMessageBox::Critical, "Error", "Please enter username", QMessageBox::Ok, this);
        msg.exec();
        return;
    }
    if (mapNameToId.contains(name))
    {
        QMessageBox msg(QMessageBox::Critical, "Error", "Username " + name + " already taken", QMessageBox::Ok, this);
        msg.exec();
        return;
    }

    // check templates
    if (scans.count() < 2)
    {
        QMessageBox msg(QMessageBox::Critical, "Error", "Please scan more reference data", QMessageBox::Ok, this);
        msg.exec();
        return;
    }

    // check quality
    // TODO

    // insert to database
    int id = 1;
    int count = mapIdToName.keys().count();
    if (count > 0)
    {
        id = mapIdToName.keys()[count - 1] + 1;
    }
    mapIdToName[id] = name;
    mapNameToId[name] = id;

    foreach (const Mesh &m, scans)
    {
        database.insertMulti(id, new FaceTemplate(id, m, classifier));
    }

    qDebug() << "Added" << id << name << "scans: " << database.values(id).count();

    accept();
}
