#ifndef WIDGETEVALUATION_H
#define WIDGETEVALUATION_H

#include <QWidget>
#include <QMap>
#include <qwt_plot_marker.h>

#include "faceCommon/biometrics/evaluation.h"
#include "faceExtras/faceExtras.h"

namespace Ui {
class WidgetEvaluation;
}

namespace Face {
namespace GUI {

class FACEEXTRAS_EXPORTS WidgetEvaluation : public QWidget
{
    Q_OBJECT

public:
    explicit WidgetEvaluation(QWidget *parent = 0);
    ~WidgetEvaluation();

    void addEvaluation(const QString &key, const Biometrics::Evaluation &newEvaluation);

private slots:
    void on_btnAdd_clicked();

    void on_pbtnAddDir_clicked();

    void on_sliderFMR_valueChanged(double value);

    void on_pbtnRemove_clicked();

private:
    Ui::WidgetEvaluation *ui;
    QMap<QString, Face::Biometrics::Evaluation> evaluations;
    QwtPlotMarker *detMarker;
    QList<QColor> colors;

    void setupSliderFMRScale();
    void calculateFRTable();
    void prepareDETPlot();
    void createDETCurves();
    void prepareScoresPlot();
    void createScoresCurves();
    void paintDETMarker();
    void paintScoresMarker();
    void replotAll();
};

}
}

#endif // WIDGETEVALUATION_H
