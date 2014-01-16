#ifndef DLGREFERENCEPROPERTIES_H
#define DLGREFERENCEPROPERTIES_H

#include <QDialog>
#include <QMap>

#include "biometrics/facetemplate.h"

namespace Ui {
class DlgReferenceProperties;
}

class DlgReferenceProperties : public QDialog
{
    Q_OBJECT

public:
    explicit DlgReferenceProperties(const QString &name, const QMap<int, QString> &mapIdToName, const QMap<QString, int> &mapNameToId,
                                    const QHash<int, Face::Biometrics::Face3DTemplate*> &database, QWidget *parent = 0);
    ~DlgReferenceProperties();

private:
    Ui::DlgReferenceProperties *ui;
};

#endif // DLGREFERENCEPROPERTIES_H
