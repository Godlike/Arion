/*
* Copyright (C) 2017-2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_CSO_HPP
#define ARION_CSO_HPP

#include <Arion/Shape.hpp>

namespace arion
{
namespace intersection
{
namespace cso
{
/**
*  @brief  Calculates farthest vertex on the surface of the sphere in given direction
*
*  @param  sphere      shape object
*  @param  direction   normalized world space search vector
*
*  @return point on the surface
*/
inline glm::vec3 LazySupport(Sphere const& sphere, glm::vec3 direction)
{
    return sphere.centerOfMass + direction * sphere.radius;
}

/**
*  @brief  Calculates farthest vertex on the surface of the sphere in given direction
*
*  @param  sphere      shape object
*  @param  direction   normalized world space search vector
*  @param  inverseOrientation inverse orientation quaternion
*
*  @note @param inverseOrientation is not used and only present for polymorphic behaviour features
*
*  @return point on the surface
*/
inline glm::vec3 LazySupport(Sphere const& sphere, glm::vec3 direction, glm::quat inverseOrientation)
{
    return LazySupport(sphere, direction);
}

/**
*  @brief  Calculates farthest vertex on the surface of the sphere in given direction
*
*  @note   This is an alias for the corresponding LazySupport function
*
*  @param  sphere      shape object
*  @param  direction   normalized world space search vector
*  @param  inverseOrientation inverse orientation quaternion
*
*  @return point on the surface
*/
inline glm::vec3 Support(Sphere const& sphere, glm::vec3 direction)
{
    return LazySupport(sphere, direction);
}

/**
*  @brief  Calculates farthest vertex on the surface of the box in given direction
*
*  @param  box         shape object
*  @param  direction   normalized model space search vector
*
*  @return point on the surface
*/
inline glm::vec3 LazySupport(Box const& box, glm::vec3 direction)
{
    float dot = 0;
    float maxDot = glm::dot(box.vertices[0], direction);
    uint8_t maxIndex = 0;
    for (uint8_t i = 1; i < 8; ++i)
    {
        dot = glm::dot(box.vertices[i], direction);
        if (dot > maxDot)
        {
            maxDot = dot;
            maxIndex = i;
        }
    }

    return box.orientation * box.vertices[maxIndex] + box.centerOfMass;
}

/**
*  @brief  Calculates farthest vertex on the surface of the box in given direction
*
*  @param  box         shape object
*  @param  direction   normalized world space search vector
*  @param  inverseOrientation inverse box world orientation quaternion
*
*  @return point on the surface
*/
inline glm::vec3 LazySupport(Box const& box, glm::vec3 direction, glm::quat inverseOrientation)
{
    direction = inverseOrientation * direction;

    return LazySupport(box, direction);
}

/**
*  @brief  Calculates farthest vertex on the surface of the box in given direction
*
*  @param  box         shape object
*  @param  direction   normalized world space search vector
*
*  @return point on the surface
*/
inline glm::vec3 Support(Box const& box, glm::vec3 direction)
{
    return LazySupport(box, glm::inverse(box.orientation) * direction);
}

/**
*  @brief  Calculates farthest vertex on the surface of the Configuration Space
*          Object in given direction
*
*  Configuration Space Object (aka Minkowski Difference and Minkowski
*  Configuration Object) is a Cartesian product of two sets of points, where
*  each element in one of the sets is multiplied by -1.
*
*  @tparam ShapeA      any shape type for which gjk::Support is overloaded
*  @tparam ShapeB      any shape type for which gjk::Support is overloaded
*
*  @param  aShape      shape object
*  @param  bShape      shape object
*  @param  direction   normalized world space search vector
*
*  @return point on the surface
*/
template < typename ShapeA, typename ShapeB >
inline glm::vec3 Support(ShapeA const& aShape, ShapeB const& bShape, glm::vec3 direction)
{
    return Support(aShape, direction) - Support(bShape, -direction);
}
} // namespace cso
} // namespace intersection
} // namespace arion
#endif // ARION_CSO_HPP
