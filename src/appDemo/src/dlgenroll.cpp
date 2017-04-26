#include "dlgenroll.h"
#include "ui_dlgenroll.h"

#include <QDebug>
#include <QMessageBox>
#include <QProgressDialog>
#include <QFileDialog>

//#include "kinect.h"
//#include "dlgscanface.h"

DlgEnroll::DlgEnroll(Face::Sensors::ISensor::Ptr sensor, Face::FaceData::FaceProcessor::Ptr processor,
                     Face::Biometrics::MultiExtractor::Ptr &extractor, FrmKinectMain::Database &database,
                     QWidget *parent) :
    sensor(sensor),
    processor(processor),
    extractor(extractor),
    database(database),
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
    sensor->scan();
	auto sensorData = sensor->sensorData();
    auto mesh = sensorData.processedScan.mesh;
    processor->process(mesh, sensorData.processedScan.landmarks);
    scans << new Face::FaceData::Mesh(mesh);

    ui->listScans->addItem(QString::number(ui->listScans->count()+1));
    ui->listScans->item(ui->listScans->count()-1)->setSelected(true);
}

void DlgEnroll::on_btnRemove_clicked()
{
    QModelIndexList selection = ui->listScans->selectionModel()->selectedIndexes();
    if (selection.count() < 1) return;
    int index = selection[0].row();

    qDeleteAll(ui->listScans->selectedItems());
    delete scans[index];
    scans.removeAt(index);
}

void DlgEnroll::on_listScans_itemSelectionChanged()
{
    QList<QListWidgetItem*> items = ui->listScans->selectedItems();
    bool enabled = items.count() > 0;
    ui->btnRemove->setEnabled(enabled);
    ui->btnExport->setEnabled(enabled);

    ui->glWidget->clearAll();
    if (items.count() > 0)
    {
        int index = ui->listScans->selectionModel()->selectedIndexes()[0].row();
        ui->glWidget->addFace(scans[index]);
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
    if (database.mapNameToId.count(name) > 0)
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

    // extract templates TODO
    int id = 1;
    int count = database.mapIdToName.size();
    if (count > 0)
    {
        id = database.mapIdToName.rbegin()->first + 1;
    }

    QProgressDialog progDlg("Extracting templates", QString(), 0, scans.count(), this);
    progDlg.setWindowModality(Qt::WindowModal);
    progDlg.setMinimumDuration(100);
    QList<Face::Biometrics::MultiTemplate> templates;
    int index = 0;
    foreach (const Face::FaceData::Mesh *m, scans)
    {
        progDlg.setValue(index);
        templates << Face::Biometrics::MultiTemplate(extractor->extract(*m, 0, id));
        index++;
    }
    progDlg.setValue(scans.count());

    /*// check quality
    int templateNum = 1;
    foreach (const Face::Biometrics::FaceTemplate *t, templates)
    {
        double d = classifier.compare(templates, t, Face::Biometrics::FaceClassifier::CompareMeanDistance);
        qDebug() << "template:" << templateNum << "distance:" << d;

        if (d >= -1.0)
        {
            QMessageBox msg(QMessageBox::Question, "Question",
                            "Template " + QString::number(templateNum) + " does not fulfil quality requirements. Proceed anyway?",
                            QMessageBox::Yes | QMessageBox::No, this);
            if (msg.exec() != QMessageBox::Yes)
            {
                qDeleteAll(templates);
                return;
            }
        }

        templateNum++;
    }*/

    // insert to database
    database.mapIdToName[id] = name;
    database.mapNameToId[name] = id;

    foreach (const Face::Biometrics::MultiTemplate &t, templates)
    {
        database.scans[id].push_back(t);
    }

    qDebug() << "Added" << id << name << "scans: " << database.scans[id].size();

    accept();
}

void DlgEnroll::on_btnExport_clicked()
{
    QModelIndexList selection = ui->listScans->selectionModel()->selectedIndexes();
    if (selection.count() < 1) return;
    int index = selection[0].row();

    QString path = QFileDialog::getSaveFileName(this, QString(), QString(), "*.binz");
    if (path.isEmpty() || path.isNull()) return;

    scans[index]->writeBINZ(path.toStdString());
}
