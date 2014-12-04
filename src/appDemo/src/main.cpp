#include <QApplication>

#include "frmkinectmain.h"
#include "dlglaunchproperties.h"


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    FrmKinectMain mainWindow;
    mainWindow.show();
    return app.exec();
}
