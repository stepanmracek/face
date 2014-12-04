#include "faceExtras/gui/widgetevaluation.h"
#include "ui_widgetevaluation.h"

#include <QDir>
#include <QFileDialog>
#include <qwt_plot_curve.h>
#include <qwt_scale_engine.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_marker.h>
#include <qwt_symbol.h>
#include <qwt_legend.h>
#include <qwt_plot_renderer.h>

#include "faceExtras/gui/dialogevaluationload.h"
#include "faceCommon/linalg/histogram.h"

using namespace Face::GUI;

WidgetEvaluation::WidgetEvaluation(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetEvaluation),
    detMarker(new QwtPlotMarker("DET marker"))
    //scoresMarker(new QwtPlotMarker("scores marker"))
{
    ui->setupUi(this);
    setWindowTitle("Evaluation");

    setupSliderFMRScale();

    detMarker->attach(ui->plotDET);
    detMarker->setLineStyle(QwtPlotMarker::VLine);

    //scoresMarker->attach(ui->plotScores);
    //scoresMarker->setLineStyle(QwtPlotMarker::VLine);

    prepareDETPlot();
    prepareScoresPlot();

    ui->table->setColumnCount(4);
    QStringList labels; labels << "Name" << "FMR" << "FNMR" << "Distance";
    ui->table->setHorizontalHeaderLabels(labels);

    colors << Qt::red << Qt::green << Qt::blue << Qt::cyan << Qt::magenta << Qt::black << Qt::gray << Qt::darkYellow;
}

void WidgetEvaluation::setupSliderFMRScale()
{
    ui->sliderFMR->setScaleEngine(new QwtLogScaleEngine());
    ui->sliderFMR->setScale(1e-5, 1);
    ui->sliderFMR->setScaleMaxMajor(5);
    ui->sliderFMR->setValue(1e-3);
}

void WidgetEvaluation::prepareDETPlot()
{
    ui->plotDET->setAxisScaleEngine(QwtPlot::xBottom, new QwtLogScaleEngine());
    ui->plotDET->setAxisScale(QwtPlot::xBottom, 1e-5, 1);

    //ui->plotDET->setAxisScaleEngine(QwtPlot::yLeft, new QwtLogScaleEngine());
    //ui->plotDET->setAxisScale(QwtPlot::yLeft, 1e-5, 1);

    ui->plotDET->setTitle("DET");
    ui->plotDET->setAxisTitle(QwtPlot::xBottom, "FMR");
    ui->plotDET->setAxisTitle(QwtPlot::yLeft, "FNMR");

    auto grid = new QwtPlotGrid();
    grid->enableX(true);
    grid->enableY(true);
    grid->setPen(Qt::gray, 0.0, Qt::DashLine);
    grid->attach(ui->plotDET);

    ui->plotDET->insertLegend(new QwtLegend());
}

void WidgetEvaluation::createDETCurves()
{
    ui->plotDET->detachItems(QwtPlotItem::Rtti_PlotCurve);

    int colorIndex = 0;
    foreach (const QString &key, evaluations.keys())
    {
        const Face::Biometrics::Evaluation &e = evaluations[key];
        QVector<QPointF> detData;
        int n = e.fmr.size();
        for (int i = 0; i < n; i++)
            detData << QPointF(e.fmr[i], e.fnmr[i]);

        auto detCurve = new QwtPlotCurve(key);
        detCurve->setPen(colors[colorIndex++ % colors.count()]);
        detCurve->setData(new QwtPointSeriesData(detData));
        detCurve->setRenderHint(QwtPlotItem::RenderAntialiased);
        detCurve->attach(ui->plotDET);
    }

    ui->plotDET->replot();
}

void WidgetEvaluation::paintDETMarker()
{
    detMarker->setValue(ui->sliderFMR->value(), 0);
    ui->plotDET->replot();
}

void WidgetEvaluation::prepareScoresPlot()
{
    ui->plotScores->setTitle("Genuine (dashed), impostor (solid) distribution");
    ui->plotScores->setAxisTitle(QwtPlot::xBottom, "distance/score");
    ui->plotScores->setAxisTitle(QwtPlot::yLeft, "probability");
    //ui->plotScores->setAxisTitle(QwtPlot::yRight, "probability (FMR)");

    //ui->plotScores->setAxisScale(QwtPlot::yLeft, 0, 1);
    ui->plotScores->setAxisScale(QwtPlot::xBottom, 0, 1);

    auto grid = new QwtPlotGrid();
    grid->enableX(true);
    grid->enableY(true);
    grid->setPen(Qt::gray, 0.0, Qt::DashLine);
    grid->attach(ui->plotScores);

    ui->plotScores->insertLegend(new QwtLegend());
}

void WidgetEvaluation::createScoresCurves()
{
    ui->plotScores->detachItems(QwtPlotItem::Rtti_PlotCurve);
    //ui->plotScores->detachItems(QwtPlotItem::Rtti_PlotMarker);
    //scoresMarkers.clear();

    double min = 1e300;
    double max = -1e300;
    foreach (const QString &key, evaluations.keys())
    {
        const Face::Biometrics::Evaluation &e = evaluations[key];
        if (e.maxScore > max) max = e.maxScore;
        if (e.minScore < min) min = e.minScore;
    }
    if (max > min)
    {
        ui->plotScores->setAxisScale(QwtPlot::xBottom, min, max);
    }

    int colorIndex = 0;
    foreach (const QString &key, evaluations.keys())
    {
        const Face::Biometrics::Evaluation &e = evaluations[key];
        QVector<QPointF> impData;
        QVector<QPointF> genData;

        int bins = 50;
        Face::LinAlg::Histogram impHist(e.impostorScores, bins, true, e.minScore, e.maxScore);
        Face::LinAlg::Histogram genHist(e.genuineScores, bins, true, e.minScore, e.maxScore);
        for (int i = 0; i < 50; i++)
        {
            impData << QPointF(impHist.histogramValues[i], impHist.histogramCounter[i]);
            genData << QPointF(genHist.histogramValues[i], genHist.histogramCounter[i]);
            //qDebug() << impHist.histogramCounter.size();
            //qDebug() << impHist.histogramValues;
            //fmrData << QPointF((e.distances[i] - e.minDistance)/(e.maxDistance - e.minDistance), e.fmr[i]);
            //fnmrData << QPointF((e.distances[i] - e.minDistance)/(e.maxDistance - e.minDistance), e.fnmr[i]);
        }
        /*auto marker = new QwtPlotMarker(key);
        marker->setLineStyle(QwtPlotMarker::Cross);
        marker->attach(ui->plotScores);
        marker->setLinePen(colors[colorIndex % colors.count()], 0.0, Qt::DotLine);
        scoresMarkers << marker;*/

        auto impCurve = new QwtPlotCurve(key);
        impCurve->setData(new QwtPointSeriesData(impData));
        impCurve->setRenderHint(QwtPlotItem::RenderAntialiased);
        impCurve->attach(ui->plotScores);
        impCurve->setPen(colors[colorIndex % colors.count()]);

        auto genCurve = new QwtPlotCurve(key);
        genCurve->setItemAttribute(QwtPlotItem::Legend, false);
        genCurve->setData(new QwtPointSeriesData(genData));
        genCurve->setRenderHint(QwtPlotItem::RenderAntialiased);
        genCurve->attach(ui->plotScores);
        genCurve->setPen(colors[colorIndex % colors.count()], 0.0, Qt::DashLine);

        colorIndex++;
    }

    ui->plotScores->replot();
}

void WidgetEvaluation::paintScoresMarker()
{
    /*auto evalNames = evaluations.keys();
    for (int i = 0; i < evalNames.count(); i++)
    {
        const Face::Biometrics::Evaluation &e = evaluations[evalNames[i]];
        double score = ui->table->item(i, 3)->text().toDouble();
        double fmr = ui->sliderFMR->value();
        //scoresMarkers[i]->setValue((score - e.minDistance)/(e.maxDistance - e.minDistance), fmr);
    }
    ui->plotScores->replot();*/
}

WidgetEvaluation::~WidgetEvaluation()
{
    delete ui;
}

void WidgetEvaluation::calculateFRTable()
{
    double fnmr, distance;
    int row = 0;
    double fmr = ui->sliderFMR->value();
    foreach (const QString &key, evaluations.keys())
    {
        const Face::Biometrics::Evaluation &e = evaluations[key];
        e.fnmrAtFmr(fmr, fnmr, distance);
        ui->table->setItem(row, 0, new QTableWidgetItem(key));
        ui->table->setItem(row, 1, new QTableWidgetItem(QString::number(fmr)));
        ui->table->setItem(row, 2, new QTableWidgetItem(QString::number(fnmr)));
        ui->table->setItem(row, 3, new QTableWidgetItem(QString::number(distance)));
        row++;
    }
}

void WidgetEvaluation::on_btnAdd_clicked()
{
    DialogEvaluationLoad dlg(this);
    if (dlg.exec() != QDialog::Accepted) return;
    auto eval = dlg.getResult();
    addEvaluation(eval.first, eval.second);
}

void WidgetEvaluation::addEvaluation(const QString &key, const Biometrics::Evaluation &newEvaluation)
{
    evaluations.insert(key, newEvaluation);
    replotAll();
}

void WidgetEvaluation::replotAll()
{
    createDETCurves();
    createScoresCurves();
    ui->table->setRowCount(evaluations.count());
    calculateFRTable();
}

void WidgetEvaluation::on_pbtnAddDir_clicked()
{
    QString dirPath = QFileDialog::getExistingDirectory(this, "Select directory");
    if (dirPath.isEmpty() || dirPath.isNull()) return;

    QDir dir(dirPath);
    QStringList genFilter; genFilter << "*-gen-scores";
    QStringList impFilter; impFilter << "*-imp-scores";
    auto genInfos = dir.entryInfoList(genFilter, QDir::Files, QDir::Name);
    auto impInfos = dir.entryInfoList(impFilter, QDir::Files, QDir::Name);

    int n = genInfos.count();
    if (n != impInfos.count()) throw FACELIB_EXCEPTION("genInfos.count() != impInfos.count()");
    for (int i = 0; i < n; i++)
    {
        auto gen = Face::LinAlg::Vector::fromFile(genInfos[i].absoluteFilePath().toStdString()).toStdVector();
        auto imp = Face::LinAlg::Vector::fromFile(impInfos[i].absoluteFilePath().toStdString()).toStdVector();
        Face::Biometrics::Evaluation eval(gen, imp);
        int len = genInfos[i].baseName().length();
        QString key = genInfos[i].baseName().left(len - QString("-gen-scores").length());

        addEvaluation(key, eval);
    }

    //QwtPlotRenderer renderer;
    //renderer.renderDocument(ui->plotDET, "test.pdf", "pdf", QSizeF(80, 40));
}

void WidgetEvaluation::on_sliderFMR_valueChanged(double value)
{
    ui->leFMR->setText(QString::number(value));

    calculateFRTable();
    paintDETMarker();
    //paintScoresMarker();
}

void WidgetEvaluation::on_pbtnRemove_clicked()
{
    auto selection = ui->table->selectedItems();
    if (selection.count() == 0) return;

    QString key = selection[0]->text();
    evaluations.remove(key);
    replotAll();
}
