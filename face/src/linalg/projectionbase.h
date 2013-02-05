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

class ProjectionBase
{
public:
	virtual Matrix project(const Matrix &vector) = 0;
	virtual QVector<Matrix> project(const QVector<Matrix> &vectors) = 0;
    virtual Matrix normalizeParams(const Matrix &params) = 0;

	virtual ~ProjectionBase() {}
};

#endif /* PROJECTIONBASE_H_ */
