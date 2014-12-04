#include "faceExtras/gui/widgetmeshselect.h"
#include "ui_widgetmeshselect.h"

#include <QDir>
#include <QFileInfoList>
#include <QFileInfo>
#include <QDebug>
#include <QVBoxLayout>
#include <QDialog>

#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/landmarkdetector.h"
#include "faceCommon/facedata/landmarks.h"
#include "faceExtras/gui/glwidget.h"

using namespace Face::GUI;

WidgetMeshSelect::WidgetMeshSelect(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetMeshSelect)
{
    ui->setupUi(this);
}

WidgetMeshSelect::~WidgetMeshSelect()
{
    delete ui;
}

void WidgetMeshSelect::setPath(QString &path, QStringList &filters)
{
    qDebug() << "Setting path to" << path;
    QDir dir(path);
    //qDebug() << dir.exists() << dir.entryList().count();
    QFileInfoList entries = dir.entryInfoList(filters, QDir::Files, QDir::Name);
    foreach (const QFileInfo &info, entries)
    {
        QListWidgetItem *item = new QListWidgetItem(info.fileName(), ui->listWidget);
        item->setData(1, info.absoluteFilePath());
        ui->listWidget->addItem(item);
    }
}

void WidgetMeshSelect::on_listWidget_itemDoubleClicked(QListWidgetItem *item)
{
    std::string fullPath = item->data(1).toString().toStdString();

    Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromOBJ(fullPath);
    Face::FaceData::LandmarkDetector detector(mesh);
    Face::FaceData::Landmarks landmarks = detector.detect();

    GLWidget *w = new GLWidget();
    w->addFace(&mesh);
    w->addLandmarks(&landmarks);

    QDialog dlg;
    dlg.setFixedSize(240, 320);
    dlg.setLayout(new QVBoxLayout());
    dlg.layout()->addWidget(w);
    dlg.exec();
}
