#ifndef DLGREFERENCEPROPERTIES_H
#define DLGREFERENCEPROPERTIES_H

#include <QDialog>
#include <QHash>

#include "biometrics/facetemplate.h"

namespace Ui {
class DlgReferenceProperties;
}

class DlgReferenceProperties : public QDialog
{
    Q_OBJECT

public:
    explicit DlgReferenceProperties(const QString &name, const QHash<int, QString> &hashIdToName, const QHash<QString, int> &hashNameToId,
                                    const QHash<int, FaceTemplate> &database, QWidget *parent = 0);
    ~DlgReferenceProperties();

private:
    Ui::DlgReferenceProperties *ui;
};

#endif // DLGREFERENCEPROPERTIES_H
