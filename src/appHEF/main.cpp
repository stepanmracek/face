#include <QApplication>
#include <QString>
#include <QTime>

#include "winhef.h"

int main(int argc, char *argv[])
{
    QTime time = QTime::currentTime();
    qsrand(time.msec());

    QApplication app(argc, argv);
    WinHef window("/home/stepo/data/frgc/spring2004/zbin-aligned/");
    window.show();
    return app.exec();
}
