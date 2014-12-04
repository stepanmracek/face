#include "winscans.h"
#include "ui_winscans.h"

#include <QFileInfo>
#include <QInputDialog>
#include <QFileDialog>
#include <QDateTime>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/facedata/surfaceprocessor.h"

using namespace Face;
using namespace Face::Sensors;

WinScans::WinScans(Face::Sensors::ISensor::Ptr sensor, FaceData::FaceAligner::Ptr aligner, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::WinScans),
    sensor(sensor),
    aligner(aligner),
    lastId(1)
{
    ui->setupUi(this);
}

WinScans::~WinScans()
{
    delete ui;
}

void WinScans::addFace(const QString &scanName, const Face::FaceData::Mesh &mesh)
{
    meshes[scanName] = mesh;
    ui->listWidget->addItem(scanName);
}

void WinScans::on_btnAdd_clicked()
{
    ui->btnAdd->setEnabled(false);
    sensor->scan();

    bool ok;
    int id = QInputDialog::getInt(this, "ID of subject", "ID", lastId, 1, 10000, 1, &ok);
    if (ok)
    {
        lastId = id;
        QDateTime now = QDateTime::currentDateTime();
        QString scanNumber = now.toString("yyyyMMddHHmmss");
        QString scanName = QString::number(id) + "-" + scanNumber;

        addFace(scanName, sensor->mesh());
    }

    ui->btnAdd->setEnabled(true);
}

void WinScans::on_listWidget_itemSelectionChanged()
{
    ui->widget->clearAll();
    if (ui->listWidget->selectedItems().count() > 0)
    {
        QString selection = ui->listWidget->selectedItems()[0]->text();

        if (landmarks.contains(selection))
        {
            ui->widget->addLandmarks(&landmarks[selection]);
        }
        ui->widget->addFace(&meshes[selection]);
    }
    else
    {
        ui->widget->updateGL();
    }
}

void WinScans::on_btnRemove_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    foreach(QListWidgetItem *i, items)
    {
        meshes.remove(i->text());
    }

    qDeleteAll(ui->listWidget->selectedItems());
}

void WinScans::on_btnSaveAll_clicked()
{
    QStringList types; types << "obj" << "binz";
    bool ok;
    QString selection = QInputDialog::getItem(this, "Select type", "Select type", types, 0, false, &ok);
    if (!ok) return;

    QString directory = QFileDialog::getExistingDirectory(this);
    if (directory.isEmpty() || directory.isNull()) return;
    if (!directory.endsWith(QDir::separator())) directory.append(QDir::separator());

    foreach (const QString &scan, meshes.keys())
    {
        if (selection.compare("binz") == 0)
        {
            meshes[scan].writeBINZ((directory + scan + ".binz").toStdString());
        }
        else
        {
            meshes[scan].writeOBJ((directory + scan + ".obj").toStdString());
        }
    }
}

void Face::Sensors::WinScans::on_btnMDenoise_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    foreach(QListWidgetItem *i, items)
    {
        Face::FaceData::SurfaceProcessor::mdenoising(meshes[i->text()], 0.01f, 10, 10);
        ui->widget->updateGL();
    }
}

void Face::Sensors::WinScans::on_btnZSmooth_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    foreach(QListWidgetItem *i, items)
    {
        Face::FaceData::SurfaceProcessor::zsmooth(meshes[i->text()], 0.2, 5);
        ui->widget->updateGL();
    }
}

void WinScans::on_btnLoad_clicked()
{
    QString directory = QFileDialog::getExistingDirectory(this);
    if (directory.isEmpty() || directory.isNull()) return;

    std::vector<std::string> filters;
    filters.push_back("*.binz");
    filters.push_back("*.bin");
    filters.push_back("*.obj");
    std::vector<std::string> files = Face::LinAlg::Loader::listFiles(directory.toStdString(), filters, Face::LinAlg::Loader::AbsoluteFull);
    foreach (const std::string &path, files)
    {
        QFileInfo info(QString::fromStdString(path));
        Face::FaceData::Mesh m = Face::FaceData::Mesh::fromFile(path, false);
        addFace(info.baseName(), m);

        QString lmFilePath = directory + QDir::separator() + info.baseName() + ".yml";
        if (QFile(lmFilePath).exists())
        {
            landmarks[info.baseName()] = Face::FaceData::Landmarks(lmFilePath.toStdString());
        }
    }
}


void Face::Sensors::WinScans::on_btnRemoveTexture_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    foreach(QListWidgetItem *i, items)
    {
        meshes[i->text()].colors.clear();
        ui->widget->updateGL();
    }
}

void Face::Sensors::WinScans::on_btnAlign_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    foreach(QListWidgetItem *i, items)
    {
        aligner->icpAlign(meshes[i->text()], 100, Face::FaceData::FaceAligner::TemplateMatching);
        ui->widget->updateGL();
    }
}

void Face::Sensors::WinScans::on_btnCrop_clicked()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    foreach(QListWidgetItem *i, items)
    {
        meshes[i->text()] = meshes[i->text()].radiusSelect(80);
        ui->widget->updateGL();
    }
}
