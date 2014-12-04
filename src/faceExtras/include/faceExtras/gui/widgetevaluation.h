#ifndef WIDGETEVALUATION_H
#define WIDGETEVALUATION_H

#include <QWidget>
#include <QMap>
#include <qwt/qwt_plot_marker.h>

#include "faceCommon/biometrics/evaluation.h"

namespace Ui {
class WidgetEvaluation;
}

namespace Face {
namespace GUI {

class WidgetEvaluation : public QWidget
{
    Q_OBJECT

public:
    explicit WidgetEvaluation(QWidget *parent = 0);
    ~WidgetEvaluation();

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
    void addEvaluation(const QString &key, const Biometrics::Evaluation &newEvaluation);
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
