#include "dlgrealtimecompare.h"
#include "ui_dlgrealtimecompare.h"

#include <QtConcurrentRun>

#include "kinect.h"

DlgRealTimeCompare::DlgRealTimeCompare(RealTimeClassifier *classifier, const QMap<int, QString> &mapIdToName,
                                       const QString &faceHaarPath, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DlgRealTimeCompare),
    classifier(classifier),
    mapIdToName(mapIdToName),
    tracker(faceHaarPath)
{
    ui->setupUi(this);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(showFace()));
    timer->start(50);
}

DlgRealTimeCompare::~DlgRealTimeCompare()
{
    delete ui;
}

void DlgRealTimeCompare::showFace()
{
    if (classifier->minDistanceId != -1)
    {
        ui->label->setText(mapIdToName[classifier->minDistanceId] + ": " + QString::number(classifier->minDistance));
    }

    Kinect::getRGB(rgbBuffer);
    frame = Kinect::RGBToGrayscale(rgbBuffer);
    std::vector<cv::Rect> faces = tracker.detect(frame);

    int faceCount = faces.size();
    if (faceCount > 0)
    {
        int maskIndex = 0;
        for (int r = 0; r < 480; r++)
        {
            for (int c = 0; c < 640; c++)
            {
                mask[maskIndex] = faces[0].contains(cv::Point(c, r));
                maskIndex++;
            }
        }
    }
    else
    {
        // set all bits of mask to zero if no face was detected
        memset(mask, 0, 307200); //640*480
    }

    Kinect::getDepth(depthBuffer, mask, 200, 1000);
    Mesh *m = Kinect::createMesh(depthBuffer, rgbBuffer);
    m->centralize();

    ui->widget->deleteAll();
    ui->widget->addFace(m);
    ui->widget->updateGL();

    if (classifier->comparing || faceCount == 0) return;
    QtConcurrent::run(classifier, &RealTimeClassifier::compare, ui->widget->getFace());
}
