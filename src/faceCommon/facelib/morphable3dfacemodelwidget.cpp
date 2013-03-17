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

    int n = model->pca.getModes();
    Vector params(n);
    for (int i = 0; i < n; i++)
    {
        QSlider *slider = sliders[i];
        double newValue = (slider->value() - 50.0)/100.0 * 3 * sqrt(model->pca.getVariation(i));
        params(i) = newValue;
    }

    model->setModelParams(params);
    ui->glWidget->repaint();
}

void Morphable3DFaceModelWidget::setModel(Morphable3DFaceModel *model)
{
    int count = model->pca.getModes();
    sliders.clear();

    QVBoxLayout *layout = new QVBoxLayout();
    for (int i = 0; i < count; i++)
    {
        QSlider *slider = new QSlider(Qt::Horizontal);
        slider->setMinimum(0);
        slider->setMaximum(100);
        slider->setPageStep(10);
        slider->setValue(50);
        layout->addWidget(slider);
        sliders << slider;

        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(recalculateModel()));
    }
    ui->scrollAreaContents->setLayout(layout);

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
        //params(i) = ((qrand() % 100) - 50.0)/100.0 * 3 * sqrt(model->pca.getVariation(i));
    }
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

    updateModel = true;
    recalculateModel();
}
