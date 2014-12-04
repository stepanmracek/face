#ifndef DIALOGEVALUATIONLOAD_H
#define DIALOGEVALUATIONLOAD_H

#include <QDialog>

#include "faceCommon/biometrics/evaluation.h"

namespace Ui {
class DialogEvaluationLoad;
}

namespace Face {
namespace GUI {

class DialogEvaluationLoad : public QDialog
{
    Q_OBJECT

public:
    explicit DialogEvaluationLoad(QWidget *parent = 0);
    ~DialogEvaluationLoad();
    QPair<QString, Face::Biometrics::Evaluation> getResult();


private slots:
    void on_pbtnLoadGenuines_clicked();

    void on_pbtnLoadImpostors_clicked();

private:
    Ui::DialogEvaluationLoad *ui;
};

}
}

#endif // DIALOGEVALUATIONLOAD_H
