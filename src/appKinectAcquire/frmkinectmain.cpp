#include "frmkinectmain.h"
#include "ui_frmkinectmain.h"

#include <QListWidgetItem>
#include <QMessageBox>
#include <QInputDialog>

#include "dlgreferenceproperties.h"
#include "dlgidentifyresult.h"

#include "linalg/loader.h"

FrmKinectMain::FrmKinectMain(const QString &databasePath, const FaceClassifier &classifier, QWidget *parent) :
    classifier(classifier),
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

        //qDebug() << id << name;

        mapIdToName[id] = name;
    }

    QVector<QString> templateFiles = Loader::listFiles(dirPath, "*.yml", AbsoluteFull);
    foreach(const QString &path, templateFiles)
    {
        int id = QFileInfo(path).baseName().split("-")[0].toInt();

        //if (database.contains(id) && database.values(id).count() >= 4) continue;

        QString name = mapIdToName[id];
        //qDebug() << path << id << name;
        FaceTemplate *t = new FaceTemplate(id, path, classifier);
        database.insertMulti(id, t);

        if (!mapNameToId.contains(name))
        {
            mapNameToId[name] = id;
            ui->listDatabase->addItem(name);
            //qDebug() << "  " << id << name;
        }
    }
}

void FrmKinectMain::on_btnProperties_clicked()
{
    if (ui->listDatabase->selectedItems().count() < 1) return;
    QString name = ui->listDatabase->selectedItems()[0]->text();

    DlgReferenceProperties props(name, mapIdToName, mapNameToId, database);
    props.exec();
}

void FrmKinectMain::on_btnDelete_clicked()
{
    if (ui->listDatabase->selectedItems().count() < 1) return;
    QListWidgetItem *item = ui->listDatabase->selectedItems()[0];
    QString name = item->text();
    int id = mapNameToId[name];

    QMessageBox msg(QMessageBox::Question, "Question", "Delete reference template: " + name + "?", QMessageBox::Yes | QMessageBox::No);
    int button = msg.exec();

    if (button == QMessageBox::Yes)
    {
        mapIdToName.remove(id);
        mapNameToId.remove(name);
        database.remove(id);
        qDeleteAll(ui->listDatabase->selectedItems());
    }
}

void FrmKinectMain::on_btnIdentify_clicked()
{
    // TODO relaplace with proper code l8
    const FaceTemplate *probe;
    {
        if (ui->listDatabase->selectedItems().count() < 1) return;
        QListWidgetItem *item = ui->listDatabase->selectedItems()[0];
        QString name = item->text();
        int id = mapNameToId[name];
        probe = database.values(id)[0];
    }
    // end of TODO

    QMap<int, double> result = classifier.identify(database, probe);
    DlgIdentifyResult dlg(result, mapIdToName, ui->sliderThreshold->value(), this);
    dlg.exec();
}

void FrmKinectMain::on_btnVerify_clicked()
{
    QString name = QInputDialog::getText(this, "Query", "Enter username");
    if (name.isNull() || name.isEmpty()) return;

    // TODO relaplace with proper code l8
    const FaceTemplate *probe;
    {
        if (ui->listDatabase->selectedItems().count() < 1) return;
        QListWidgetItem *item = ui->listDatabase->selectedItems()[0];
        QString name = item->text();
        int id = mapNameToId[name];
        probe = database.values(id)[0];
    }
    // end of TODO

    if (!mapNameToId.contains(name))
    {
        QMessageBox msg(QMessageBox::Critical, "Error", "User " + name + " is not enrolled", QMessageBox::Ok, this);
        msg.exec();
        return;
    }

    int id = mapNameToId[name];
    double score = classifier.compare(database.values(id), probe);

    bool accepted = (score < ui->sliderThreshold->value());
    QString result =  accepted ? "User " + name + " accepted" : "User " + name + " rejected";
    QMessageBox resultMsg(accepted ? QMessageBox::Information : QMessageBox::Critical, "Result", result, QMessageBox::Ok, this);
    resultMsg.exec();
}
