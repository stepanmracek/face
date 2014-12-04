#include "frmkinectmain.h"
#include "ui_frmkinectmain.h"

#include <QDebug>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>
#include <qwt/qwt_slider.h>

#include "dlgreferenceproperties.h"
#include "dlgidentifyresult.h"
#include "dlgenroll.h"
#include "dlglaunchproperties.h"

#include "faceCommon/linalg/loader.h"

FrmKinectMain::FrmKinectMain(QWidget *parent) :
    ui(new Ui::FrmKinectMain),
    QMainWindow(parent)
{
    ui->setupUi(this);
    //importDirectory("/mnt/data/face/softkinetic/epar/templates");


    DlgLaunchProperties launchProps;
    if (launchProps.exec() == QDialog::Accepted)
    {
        sensor = launchProps.getSensor();
        extractor = new Face::Biometrics::MultiExtractor(launchProps.getMultiExtractorPath().toStdString());
        aligner = new Face::FaceData::FaceAligner(Face::FaceData::Mesh::fromFile(launchProps.getAlignerPath().toStdString()),
                                                  launchProps.getPreAlignTemplatePath().toStdString());
        processor = new Face::FaceData::FaceProcessor(aligner, 0, 0);
    }
    else
    {
        exit(0);
    }

    ui->sliderRaw->setValue(0);
}

FrmKinectMain::~FrmKinectMain()
{
    delete ui;
}

void FrmKinectMain::importDirectory(const QString &dirPath)
{
    QDir dir(dirPath);
    QFileInfoList filenames = dir.entryInfoList(QStringList(), QDir::NoDotAndDotDot | QDir::Files, QDir::Name);
    foreach (const QFileInfo &fInfo, filenames)
    {
        qDebug() << fInfo.absoluteFilePath();

        QString name = fInfo.baseName().split('-').at(0);
        int id = name.toInt();
        database.mapIdToName[id] = name;
        database.mapNameToId[name] = id;

        database.scans[id].push_back(Face::Biometrics::MultiTemplate(fInfo.absoluteFilePath().toStdString()));
    }
    refreshList();
}

void FrmKinectMain::refreshList()
{
    QStringList labels;
    for (const auto &pair : database.mapNameToId)
        labels << pair.first;

    ui->listDatabase->clear();
    ui->listDatabase->addItems(labels);
    setMainButtonsState();
}

void FrmKinectMain::on_btnProperties_clicked()
{
    if (ui->listDatabase->selectedItems().count() < 1) return;
    QString name = ui->listDatabase->selectedItems()[0]->text();

    DlgReferenceProperties props(name, database, this);
    props.exec();
}

void FrmKinectMain::on_btnDelete_clicked()
{
    if (ui->listDatabase->selectedItems().count() < 1) return;
    QListWidgetItem *item = ui->listDatabase->selectedItems()[0];
    QString name = item->text();
    int id = database.mapNameToId[name];

    QMessageBox msg(QMessageBox::Question, "Question", "Delete reference template: " + name + "?", QMessageBox::Yes | QMessageBox::No, this);
    int button = msg.exec();

    if (button == QMessageBox::Yes)
    {
        database.mapIdToName.erase(id);
        database.mapNameToId.erase(name);
        database.scans.erase(id);
        qDeleteAll(ui->listDatabase->selectedItems());
        setMainButtonsState();
    }
}

void FrmKinectMain::on_btnIdentify_clicked()
{
    sensor->scan();
    Face::FaceData::Mesh m = sensor->mesh();
    processor->process(m);
    Face::Biometrics::MultiTemplate probe = extractor->extract(m, 0, 0);

    QMap<int, Face::Biometrics::MultiExtractor::ComparisonResult> result;
    for (const auto &dbEntry : database.scans)
    {
        int id = dbEntry.first;
        const auto &reference = dbEntry.second;
        result[id] = extractor->compare(reference, probe, 1);
    }

    DlgIdentifyResult dlg(result, database, ui->sbRaw->value(), this);
    dlg.exec();
}

void FrmKinectMain::on_btnVerify_clicked()
{
    QStringList items;
    for (const auto &pair : database.mapNameToId)
        items << pair.first;

    bool ok;
    QString name = QInputDialog::getItem(this, "Query", "Claimed identity:", items, 0, false, &ok);
    if (!ok) return;

    sensor->scan();
    Face::FaceData::Mesh m = sensor->mesh();
    processor->process(m);
    Face::Biometrics::MultiTemplate probe = extractor->extract(m, 0, 0);

    int id = database.mapNameToId[name];

    double score = extractor->compare(database.scans[id], probe, 1).distance;
    bool accepted = (score < ui->sliderRaw->value());
    QString result =  accepted ? "User " + name + " accepted" : "User " + name + " rejected";
    result.append(". Comparison score: " + QString::number(score));
    QMessageBox resultMsg(accepted ? QMessageBox::Information : QMessageBox::Critical, "Result", result, QMessageBox::Ok, this);
    resultMsg.exec();
}

void FrmKinectMain::on_btnEnroll_clicked()
{
    DlgEnroll dlgEnroll(sensor, processor, extractor, database, this);
    if (dlgEnroll.exec() == QDialog::Accepted)
    {
        refreshList();
    }
}

void FrmKinectMain::on_listDatabase_itemSelectionChanged()
{
    bool enabled = ui->listDatabase->selectedItems().count() > 0;
    ui->btnDelete->setEnabled(enabled);
    ui->btnProperties->setEnabled(enabled);
    ui->btnExport->setEnabled(enabled);
}

void FrmKinectMain::on_btnExport_clicked()
{
    if (ui->listDatabase->selectedItems().count() < 1) return;
    QString name = ui->listDatabase->selectedItems()[0]->text();
    int id = database.mapNameToId[name];

    QString path = QFileDialog::getExistingDirectory(this);
    if (path.isNull() || path.isEmpty()) return;

    const std::vector<Face::Biometrics::MultiTemplate> &templates = database.scans[id];
    int index = 1;
    foreach (const Face::Biometrics::MultiTemplate &t, templates)
    {
        QString p = path + QDir::separator() + QString().sprintf("%02d", id) + "-" + QString().sprintf("%02d", index) + ".yml";
        qDebug() << p;
        cv::FileStorage storage(p.toStdString(), cv::FileStorage::WRITE);
        t.serialize(storage);
        index++;
    }
}

void FrmKinectMain::setMainButtonsState()
{
    bool enabled = database.mapIdToName.size() > 0;
    ui->btnIdentify->setEnabled(enabled);
    ui->btnVerify->setEnabled(enabled);
}


void FrmKinectMain::on_sliderRaw_valueChanged(double value)
{
    ui->sbRaw->setValue(value);
    double converted = -value*1000.0 + 10000.0;
    ui->sliderConverted->setValue(converted);
}

void FrmKinectMain::on_sliderConverted_valueChanged(double value)
{
    ui->sbConverted->setValue(value);
    double raw = -(value - 10000.0)/1000.0;
    ui->sliderRaw->setValue(raw);
}

void FrmKinectMain::on_toolButton_clicked()
{
    QString dirPath = QFileDialog::getExistingDirectory(this, "Import directory with templates");
    if (dirPath.isNull()) return;
    importDirectory(dirPath);
}
