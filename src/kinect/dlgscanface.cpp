#include "dlgscanface.h"
#include "ui_dlgscanface.h"

#include "kinect.h"
#include "facelib/facealigner.h"

DlgScanFace::DlgScanFace(const QString &pathToAlignReference, const QString &faceHaarPath, QWidget *parent) :
    QDialog(parent), ui(new Ui::DlgScanFace),
    pathToAlignReference(pathToAlignReference),
    tracker(faceHaarPath)
{
    ui->setupUi(this);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(showFace()));
    timer->start(50);

    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
}

DlgScanFace::~DlgScanFace()
{
    delete ui;
}

void DlgScanFace::showFace()
{
    Kinect::getRGB(rgbBuffer);
    frame = Kinect::RGBToGrayscale(rgbBuffer);
    std::vector<cv::Rect> faces = tracker.detect(frame);

    int faceCount = faces.size();
    if (faceCount > 0)
    {
        for (int i = 0; i < faceCount; i++)
        {
            int maskIndex = 0;
            for (int r = 0; r < 480; r++)
            {
                for (int c = 0; c < 640; c++)
                {
                    mask[maskIndex] = faces[i].contains(cv::Point(c, r));
                    maskIndex++;
                }
            }

            //cv::rectangle(frame, faces[i], 255);
        }
    }
    else
    {
        memset(mask, 0, 307200); //640*480
    }

    //cv::imshow("face", frame);
    //cv::waitKey(1);

    Kinect::getDepth(depthBuffer, mask, 200, 1000);
    Mesh *m = Kinect::createMesh(depthBuffer, rgbBuffer);

    bool enabled = false;
    ui->widget->deleteAll();
    if (m->points.count() > 0)
    {
        m->centralize();
        ui->widget->addFace(m);

        enabled = true;
    }
    ui->widget->updateGL();

    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(enabled);
}

void DlgScanFace::scan()
{
    Mesh *curFace = ui->widget->getFace();
    if (curFace == 0) return;
    timer->stop();

    Kinect::getDepth(depthBuffer, 10, mask, 200, 1000);
    result = Kinect::createMesh(depthBuffer, rgbBuffer);
    result->centralize();
    result->calculateTriangles();

    Mesh mean = Mesh::fromOBJ(pathToAlignReference);
    FaceAligner aligner(mean);
    aligner.icpAlign(*result, 10);

    accept();
}
