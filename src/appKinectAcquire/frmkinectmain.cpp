#include "frmkinectmain.h"
#include "ui_frmkinectmain.h"

#include <QListWidgetItem>
#include <QMessageBox>
#include <QInputDialog>

#include "dlgreferenceproperties.h"
#include "dlgidentifyresult.h"
#include "dlgenroll.h"
#include "dlgscanface.h"

#include "kinect.h"
#include "linalg/loader.h"

FrmKinectMain::FrmKinectMain(const QString &databasePath, const FaceClassifier &classifier, const QString &pathToAlignReference, QWidget *parent) :
    classifier(classifier),
    pathToAlignReference(pathToAlignReference),
    QMainWindow(parent),
    ui(new Ui::FrmKinectMain)
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
        FaceTemplate *t = new FaceTemplate(id, path, classifier);
        database.insertMulti(id, t);
    }

    refreshList();
}

void FrmKinectMain::refreshList()
{
    ui->listDatabase->clear();
    ui->listDatabase->addItems(mapIdToName.values());
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
    }
}

void FrmKinectMain::on_btnIdentify_clicked()
{
    DlgScanFace dlgScan(pathToAlignReference, this);
    if (dlgScan.exec() != QDialog::Accepted) return;
    Mesh *face = dlgScan.result;
    FaceTemplate *probe = new FaceTemplate(0, *face, classifier);
    delete face;

    QMap<int, double> result = classifier.identify(database, probe);
    delete probe;
    DlgIdentifyResult dlg(result, mapIdToName, ui->sliderThreshold->value(), this);
    dlg.exec();
}

void FrmKinectMain::on_btnVerify_clicked()
{
    bool ok;
    QString name = QInputDialog::getItem(this, "Query", "Claimed identity:", mapIdToName.values(), 0, false, &ok);
    if (!ok) return;

    DlgScanFace dlgScan(pathToAlignReference, this);
    if (dlgScan.exec() != QDialog::Accepted) return;
    Mesh *face = dlgScan.result;
    FaceTemplate *probe = new FaceTemplate(0, *face, classifier);
    delete face;

    if (!mapNameToId.contains(name))
    {
        QMessageBox msg(QMessageBox::Critical, "Error", "User " + name + " is not enrolled", QMessageBox::Ok, this);
        msg.exec();
        return;
    }

    int id = mapNameToId[name];
    double score = classifier.compare(database.values(id), probe);
    delete probe;

    bool accepted = (score < ui->sliderThreshold->value());
    QString result =  accepted ? "User " + name + " accepted" : "User " + name + " rejected";
    result.append(". Comparison score: " + QString::number(score));
    QMessageBox resultMsg(accepted ? QMessageBox::Information : QMessageBox::Critical, "Result", result, QMessageBox::Ok, this);
    resultMsg.exec();
}

void FrmKinectMain::on_btnEnroll_clicked()
{
    DlgEnroll dlgEnroll(mapIdToName, mapNameToId, database, classifier, pathToAlignReference, this);
    if (dlgEnroll.exec() == QDialog::Accepted)
    {
        refreshList();
    }
}

void FrmKinectMain::on_listDatabase_itemSelectionChanged()
{
    int count = ui->listDatabase->selectedItems().count();
    ui->btnDelete->setEnabled(count > 0);
    ui->btnProperties->setEnabled(count > 0);
}
