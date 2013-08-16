#ifndef WINHEF_H
#define WINHEF_H

#include <QMainWindow>

#include "filedata.h"
#include "facelib/glwidget.h"

namespace Ui {
class WinHef;
}

class WinHef : public QMainWindow
{
    Q_OBJECT

    FileData fileData;
    QVector<double> sameScores;
    QVector<double> diffScores;
    
public:
    explicit WinHef(const QString &path, QWidget *parent = 0);
    ~WinHef();
    
private slots:
    void on_btnDistance0_clicked();
    void on_btnDistance25_clicked();
    void on_btnDistance50_clicked();
    void on_btnDistance75_clicked();
    void on_btnDistance100_clicked();
    void on_btnEval_clicked();

private:
    bool isCurrentPairSame;
    void loadNewPair();
    void handleEvalButton();
    void loadFace(GLWidget *widget, const QString &path);
    void score(double s);
    void calculateDet(double s, double &fmr, double &fnmr);

    Ui::WinHef *ui;
};

#endif // WINHEF_H
