#include "morphable3dfacemodelwidget.h"
#include "ui_morphable3dfacemodelwidget.h"

#include <QFileDialog>
#include <QSlider>
#include <QVBoxLayout>

Morphable3DFaceModelWidget::Morphable3DFaceModelWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Morphable3DFaceModelWidget)
{
    ui->setupUi(this);
    updateModel = false;
}

Morphable3DFaceModelWidget::~Morphable3DFaceModelWidget()
{
    delete ui;
}

void Morphable3DFaceModelWidget::recalculateModel()
{
    if (!updateModel)
    {
        return;
    }
    if (!model)
    {
        qDebug() << "Model not set";
        return;
    }

    Face::LinAlg::Vector params(model->pca.getModes());
    for (int i = 0; i < model->pca.getModes(); i++)
    {
        QSlider *slider = sliders[i];
        double newValue = (slider->value() - 50.0)/100.0 * 3 * sqrt(model->pca.getVariation(i));
        params(i) = newValue;
    }

    model->setModelParams(params);

    /*int zcoordModesCount = model->pcaForZcoord.getModes();
    Vector zcoordParams(zcoordModesCount);
    for (int i = 0; i < zcoordModesCount; i++)
    {
        QSlider *slider = slidersForZcoord[i];
        double newValue = (slider->value() - 50.0)/100.0 * 3 * sqrt(model->pcaForZcoord.getVariation(i));
        zcoordParams(i) = newValue;
    }

    int textureModesCount = model->pcaForTexture.getModes();
    Vector textureParams(textureModesCount);
    for (int i = 0; i < textureModesCount; i++)
    {
        QSlider *slider = slidersForTexture[i];
        double newValue = (slider->value() - 50.0)/100.0 * 3 * sqrt(model->pcaForTexture.getVariation(i));
        textureParams(i) = newValue;
    }

    model->setModelParams(zcoordParams, textureParams);*/
    ui->glWidget->repaint();
}

void Morphable3DFaceModelWidget::setModel(Morphable3DFaceModel *model)
{
    sliders.clear();
    int modesCount = model->pca.getModes();

    QVBoxLayout *slidersLayout = new QVBoxLayout();
    for (int i = 0; i < modesCount; i++)
    {
        QSlider *slider = new QSlider(Qt::Horizontal);
        slider->setMinimum(0);
        slider->setMaximum(100);
        slider->setPageStep(10);
        slider->setValue(50);
        slidersLayout->addWidget(slider);
        sliders << slider;

        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(recalculateModel()));
    }
    ui->scrollAreaContent->setLayout(slidersLayout);

    /*slidersForZcoord.clear();
    int zCoordModesCount = model->pcaForZcoord.getModes();

    QVBoxLayout *zcoordSlidersLayout = new QVBoxLayout();
    for (int i = 0; i < zCoordModesCount; i++)
    {
        QSlider *slider = new QSlider(Qt::Horizontal);
        slider->setMinimum(0);
        slider->setMaximum(100);
        slider->setPageStep(10);
        slider->setValue(50);
        zcoordSlidersLayout->addWidget(slider);
        slidersForZcoord << slider;

        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(recalculateModel()));
    }
    ui->sacZcoord->setLayout(zcoordSlidersLayout);

    slidersForTexture.clear();
    int textureModesCount = model->pcaForTexture.getModes();
    QVBoxLayout *textureSlidersLayout = new QVBoxLayout();
    for (int i = 0; i < textureModesCount; i++)
    {
        QSlider *slider = new QSlider(Qt::Horizontal);
        slider->setMinimum(0);
        slider->setMaximum(100);
        slider->setPageStep(10);
        slider->setValue(50);
        textureSlidersLayout->addWidget(slider);
        slidersForTexture << slider;

        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(recalculateModel()));
    }
    ui->sacTexture->setLayout(textureSlidersLayout);*/

    this->model = model;
    ui->glWidget->addFace(&(model->mesh));
    ui->glWidget->addLandmarks(&(model->landmarks));
    updateModel = true;
    recalculateModel();
}

void Morphable3DFaceModelWidget::on_btnRandomize_clicked()
{
    if (!model)
    {
        qDebug() << "Model not set";
        return;
    }

    updateModel = false;
    int n = model->pca.getModes();
    for (int i = 0; i < n; i++)
    {
        double u = ((qrand() % 1000) + 1)/1000.0;
        double v = ((qrand() % 1000) + 1)/1000.0;
        double x = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
        sliders[i]->setValue( x * 20 + 50 );
    }
    /*int n = model->pcaForZcoord.getModes();
    for (int i = 0; i < n; i++)
    {
        double u = ((qrand() % 1000) + 1)/1000.0;
        double v = ((qrand() % 1000) + 1)/1000.0;
        double x = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
        slidersForZcoord[i]->setValue( x * 20 + 50 );
    }
    n = model->pcaForTexture.getModes();
    for (int i = 0; i < n; i++)
    {
        double u = ((qrand() % 1000) + 1)/1000.0;
        double v = ((qrand() % 1000) + 1)/1000.0;
        double x = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
        slidersForTexture[i]->setValue( x * 20 + 50 );
    }*/
    updateModel = true;
    recalculateModel();
}

void Morphable3DFaceModelWidget::on_btnInvert_clicked()
{
    if (!model)
    {
        qDebug() << "Model not set";
        return;
    }

    updateModel = false;
    int n = model->pca.getModes();
    for (int i = 0; i < n; i++)
    {
        sliders[i]->setValue( 100 - sliders[i]->value() );
    }
    /*int n = model->pcaForZcoord.getModes();
    for (int i = 0; i < n; i++)
    {
        slidersForZcoord[i]->setValue( 100 - slidersForZcoord[i]->value() );
    }
    n = model->pcaForTexture.getModes();
    for (int i = 0; i < n; i++)
    {
        slidersForTexture[i]->setValue( 100 - slidersForTexture[i]->value() );
    }*/
    updateModel = true;
    recalculateModel();
}

void Morphable3DFaceModelWidget::on_btnExport_clicked()
{
    if (!model)
    {
        qDebug() << "Model not set";
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this, "Export to OBJ", ".", "OBJ files (*.obj)");
    if (fileName.isEmpty() || fileName.isNull())
    {
        return;
    }
    model->mesh.writeOBJ(fileName, '.');
}

void Morphable3DFaceModelWidget::on_btnReset_clicked()
{
    if (!model)
    {
        qDebug() << "Model not set";
        return;
    }

    updateModel = false;

    int n = model->pca.getModes();
    for (int i = 0; i < n; i++)
    {
        sliders[i]->setValue(50);
    }

    /*int n = model->pcaForZcoord.getModes();
    for (int i = 0; i < n; i++)
    {
        slidersForZcoord[i]->setValue(50);
    }

    n = model->pcaForTexture.getModes();
    for (int i = 0; i < n; i++)
    {
        slidersForTexture[i]->setValue(50);
    }*/

    updateModel = true;
    recalculateModel();
}

