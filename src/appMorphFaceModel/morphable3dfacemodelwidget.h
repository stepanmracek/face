#ifndef MORPHABLE3DFACEMODELWIDGET_H
#define MORPHABLE3DFACEMODELWIDGET_H

#include <QWidget>
#include <QList>
#include <QSlider>

#include "morphable3dfacemodel.h"

namespace Ui {
class Morphable3DFaceModelWidget;
}

class Morphable3DFaceModelWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit Morphable3DFaceModelWidget(QWidget *parent = 0);

    ~Morphable3DFaceModelWidget();

    void setModel(Morphable3DFaceModel *model);
    
private:
    Ui::Morphable3DFaceModelWidget *ui;
    Morphable3DFaceModel *model;
    QList<QSlider*> sliders;
    //QList<QSlider*> slidersForZcoord;
    //QList<QSlider*> slidersForTexture;
    bool updateModel;

public slots:
    void recalculateModel();
private slots:
    void on_btnRandomize_clicked();
    void on_btnExport_clicked();
    void on_btnReset_clicked();
    void on_btnInvert_clicked();
};

#endif // MORPHABLE3DFACEMODELWIDGET_H
