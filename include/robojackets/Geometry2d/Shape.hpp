#pragma once

#include "robojackets/Geometry2d/Point.hpp"
#include <stdexcept>

namespace Geometry2d {

class Segment;

/**
 * The shape class provides the interface to all shapes that are subclasses.
 *
 * We expose this class and its subclasses to python through boost python, which
 * unfortunately handles abstract base classes very strangely.  The only way I
 * could get inheritance to work properly with boost was to provide
 * implementations for the methods in this class even though we'd prefer them to
 * be pure virtual (not implemented in this class).
 */
class Shape {
public:
    Shape() {}
    virtual ~Shape() {}

    virtual Shape* clone() const {
        throw std::runtime_error("Unimplemented method");
        return nullptr;
    }

    virtual bool containsPoint(Point pt) const {
        throw std::runtime_error("Unimplemented method");
        return false;
    }

    /// Returns true if the given point is within one robot radius of the shape
    virtual bool hit(Point pt) const {
        throw std::runtime_error("Unimplemented method");
        return false;
    }

    virtual bool hit(const Segment& seg) const {
        throw std::runtime_error("Unimplemented method");
        return false;
    }

    virtual std::string toString() { return "Shape"; }

    friend std::ostream& operator<<(std::ostream& stream, Shape& shape) {
        stream << shape.toString();
        return stream;
    }
};

}  // namespace Geometry2d
