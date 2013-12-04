#include "frmkinectmain.h"
#include "ui_frmkinectmain.h"

#include <QListWidgetItem>
#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>

#include "dlgreferenceproperties.h"
#include "dlgidentifyresult.h"
#include "dlgenroll.h"
#include "dlgscanface.h"
#include "dlgrealtimecompare.h"

#include "kinect.h"
#include "linalg/loader.h"
#include "biometrics/realtimeclassifier.h"

FrmKinectMain::FrmKinectMain(KinectSensorPlugin &sensor, const FaceClassifier &classifier, const QString &databasePath,
                             QWidget *parent) :
    ui(new Ui::FrmKinectMain),
    sensor(sensor),
    classifier(classifier),
    QMainWindow(parent)
{
    ui->setupUi(this);
    initDatabase(databasePath);
}

FrmKinectMain::~FrmKinectMain()
{
    delete ui;
}

void FrmKinectMain::initDatabase(const QString &dirPath)
{
    QFile fileIdentities(dirPath + QDir::separator() + "identities");
    if (!fileIdentities.open(QFile::ReadOnly | QFile::Text)) return;
    QTextStream streamIdentities(&fileIdentities);
    while (!streamIdentities.atEnd())
    {
        QString line = streamIdentities.readLine();
        int spaceIndex = line.indexOf(" ");
        int id = line.left(spaceIndex).toInt();
        QString name = line.right(line.length() - spaceIndex - 1);
        mapIdToName[id] = name;
        mapNameToId[name] = id;
    }

    QVector<QString> templateFiles = Loader::listFiles(dirPath, "*.yml", AbsoluteFull);
    foreach(const QString &path, templateFiles)
    {
        int id = QFileInfo(path).baseName().split("-")[0].toInt();

        QString name = mapIdToName[id];
        Face3DTemplate *t = new Face3DTemplate(id, path, classifier);
        database.insertMulti(id, t);
    }

    refreshList();
}

void FrmKinectMain::refreshList()
{
    ui->listDatabase->clear();
    ui->listDatabase->addItems(mapNameToId.keys());
    setMainButtonsState();
}

void FrmKinectMain::on_btnProperties_clicked()
{
    if (ui->listDatabase->selectedItems().count() < 1) return;
    QString name = ui->listDatabase->selectedItems()[0]->text();

    DlgReferenceProperties props(name, mapIdToName, mapNameToId, database, this);
    props.exec();
}

void FrmKinectMain::on_btnDelete_clicked()
{
    if (ui->listDatabase->selectedItems().count() < 1) return;
    QListWidgetItem *item = ui->listDatabase->selectedItems()[0];
    QString name = item->text();
    int id = mapNameToId[name];

    QMessageBox msg(QMessageBox::Question, "Question", "Delete reference template: " + name + "?", QMessageBox::Yes | QMessageBox::No, this);
    int button = msg.exec();

    if (button == QMessageBox::Yes)
    {
        mapIdToName.remove(id);
        mapNameToId.remove(name);
        qDeleteAll(database.values(id));
        database.remove(id);
        qDeleteAll(ui->listDatabase->selectedItems());
        setMainButtonsState();
    }
}

void FrmKinectMain::on_btnIdentify_clicked()
{
    /*RealTimeClassifier rtc(classifier, ui->sliderThreshold->value(), database, "../../test/meanForAlign.obj");
    DlgRealTimeCompare dlg(&rtc, mapIdToName, "../../test/haar-face.xml");
    dlg.exec();*/

    sensor.scanFace();
    if (!sensor.mesh) return;
    sensor.align();

    Face3DTemplate *probe = new Face3DTemplate(0, *sensor.mesh, classifier);
    sensor.deleteMesh();

    QMap<int, double> result = classifier.identify(database, probe, FaceClassifier::CompareMeanDistance);
    delete probe;
    DlgIdentifyResult dlg(result, mapIdToName, ui->sliderThreshold->value(), this);
    dlg.exec();
}

void FrmKinectMain::on_btnVerify_clicked()
{
    bool ok;
    QString name = QInputDialog::getItem(this, "Query", "Claimed identity:", mapNameToId.keys(), 0, false, &ok);
    if (!ok) return;

    sensor.scanFace();
    if (!sensor.mesh) return;
    sensor.align();

    Face3DTemplate *probe = new Face3DTemplate(0, *sensor.mesh, classifier);
    sensor.deleteMesh();

    if (!mapNameToId.contains(name))
    {
        QMessageBox msg(QMessageBox::Critical, "Error", "User " + name + " is not enrolled", QMessageBox::Ok, this);
        msg.exec();
        return;
    }

    int id = mapNameToId[name];
    double score = classifier.compare(database.values(id), probe, FaceClassifier::CompareMeanDistance, true);
    delete probe;

    bool accepted = (score < ui->sliderThreshold->value());
    QString result =  accepted ? "User " + name + " accepted" : "User " + name + " rejected";
    result.append(". Comparison score: " + QString::number(score));
    QMessageBox resultMsg(accepted ? QMessageBox::Information : QMessageBox::Critical, "Result", result, QMessageBox::Ok, this);
    resultMsg.exec();
}

void FrmKinectMain::on_btnEnroll_clicked()
{
    DlgEnroll dlgEnroll(mapIdToName, mapNameToId, database, classifier,
                        sensor, this);
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
    int id = mapNameToId[name];

    QString path = QFileDialog::getExistingDirectory(this);
    if (path.isNull() || path.isEmpty()) return;

    QList<Face3DTemplate*> templates = database.values(id);
    int index = 1;
    foreach (const Face3DTemplate *t, templates)
    {
        QString p = path + QDir::separator() + QString().sprintf("%02d", id) + "-" + QString().sprintf("%02d", index) + ".yml";
        qDebug() << p;
        t->serialize(p, classifier);
        index++;
    }
}

void FrmKinectMain::setMainButtonsState()
{
    bool enabled = mapIdToName.count();
    ui->btnIdentify->setEnabled(enabled);
    ui->btnVerify->setEnabled(enabled);
}
