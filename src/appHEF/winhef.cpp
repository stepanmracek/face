#include "winhef.h"
#include "ui_winhef.h"

#include "biometrics/evaluation.h"
#include "linalg/histogram.h"

WinHef::WinHef(const QString &path, QWidget *parent) : QMainWindow(parent), ui(new Ui::WinHef), fileData(path)
{
    ui->setupUi(this);
    loadNewPair();
}

WinHef::~WinHef()
{
    delete ui;
}

void WinHef::loadFace(GLWidget *widget, const QString &path)
{
    widget->deleteAll();
    Face::FaceData::Mesh m = Face::FaceData::Mesh::fromBINZ(path);
    Face::FaceData::Mesh *m2 = new Face::FaceData::Mesh(m.radiusSelect(80, cv::Point3d(0, 10, 0)));
    m2->centralize();
    widget->addFace(m2);
    widget->repaint();
}

void WinHef::loadNewPair()
{
    QPair<QString, QString> pair = fileData.getNextPair(isCurrentPairSame);
    loadFace(ui->wdgFace1, pair.first);
    loadFace(ui->wdgFace2, pair.second);

    ui->label->setText("compared " + QString::number(sameScores.count() + diffScores.count()) + " pair(s)");

    handleEvalButton();
}

void WinHef::handleEvalButton()
{
    ui->btnEval->setEnabled(sameScores.count() >= 5 && diffScores.count() >= 5);
}

void WinHef::score(double s)
{
    if (isCurrentPairSame)
        sameScores << s;
    else
        diffScores << s;
}

void WinHef::on_btnDistance0_clicked()
{
    score(0);
    loadNewPair();
}

void WinHef::on_btnDistance25_clicked()
{
    score(1);
    loadNewPair();
}

void WinHef::on_btnDistance50_clicked()
{
    score(2);
    loadNewPair();
}

void WinHef::on_btnDistance75_clicked()
{
    score(3);
    loadNewPair();
}

void WinHef::on_btnDistance100_clicked()
{
    score(4);
    loadNewPair();
}

void WinHef::calculateDet(double s, double &fmr, double &fnmr)
{
    // fmr - pomer z diffScores, ktery ma skore mensi nebo rovno s
    fmr = 0;
    foreach (double d, diffScores)
    {
        if (d <= s) fmr++;
    }
    fmr = fmr/diffScores.count();

    // fnmr - pomer ze sameScores, ktery ma skore vetsi nez s
    fnmr = 0;
    foreach (double d, sameScores)
    {
        if (d > s) fnmr++;
    }
    fnmr = fnmr/sameScores.count();
}

void WinHef::on_btnEval_clicked()
{
    if (sameScores.count() == 0 || diffScores.count() == 0) return;

    Face::LinAlg::Histogram genuineDistribution(sameScores, 5, true, 0, 5);
    genuineDistribution.savePlot("genuine");

    Face::LinAlg::Histogram impostorDistribution(diffScores, 5, true, 0, 5);
    impostorDistribution.savePlot("impostor");

    QVector<double> fmr(4);
    QVector<double> fnmr(4);
    for (int i = 0; i < 4; i++)
    {
        calculateDet(i, fmr[i], fnmr[i]);
    }
    Face::LinAlg::Common::savePlot(fmr, fnmr, "det");
}
