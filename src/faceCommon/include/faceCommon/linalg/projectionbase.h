#pragma once

#include "common.h"
#include "vector.h"

namespace Face {
namespace LinAlg {

class ProjectionBase
{
public:
    virtual Vector project(const Vector &vector) const = 0;

    virtual std::vector<Vector> batchProject(const std::vector<Vector> &vectors) const
    {
        std::vector<Vector> result;
        for (const Vector &v : vectors)
        {
            result.push_back(project(v));
        }
        return std::move(result);
    }

    virtual Vector normalizeParams(const Vector &params) = 0;

	virtual ~ProjectionBase() {}
};

}
}
