/*
 * projectionbase.h
 *
 *  Created on: 9.5.2012
 *      Author: stepo
 */

#ifndef PROJECTIONBASE_H_
#define PROJECTIONBASE_H_

#include <QVector>

#include "common.h"
#include "vector.h"

class ProjectionBase
{
public:
    virtual Vector project(const Vector &vector) = 0;

    virtual QVector<Vector> batchProject(const QVector<Vector> &vectors)
    {
        QVector<Vector> result;
        foreach (const Vector &v, vectors)
        {
            result << project(v);
        }
        return result;
    }

    virtual Vector normalizeParams(const Vector &params) = 0;

	virtual ~ProjectionBase() {}
};

#endif /* PROJECTIONBASE_H_ */
