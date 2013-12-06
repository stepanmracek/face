#ifndef SYSTEM_H
#define SYSTEM_H

#include <QVector>
#include <QList>

#include "template.h"

template <typename InT, typename OutT>
class Module
{
public:
    virtual void learn(const QVector<InT> &data) = 0;
    virtual OutT process(const InT &input) = 0;

    Module<OutT, int> next;
};

template <class Matrix, class Vector>
class ModuleMat2Vec : public Module<Matrix, Vector>
{
public:
    void learn(const QVector<Matrix> &data) {}
    Vector process(const Matrix &input) { return Vector(1); }
};

template <typename SystemInputT>
class System
{
public:
    Template process(const SystemInputT &input)
    {

    }

    QVector<Template> batchProcess(const QVector<SystemInputT> &data);
};

#endif // SYSTEM_H
