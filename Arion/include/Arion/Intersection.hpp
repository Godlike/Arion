/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_INTERSECTION_HPP
#define ARION_INTERSECTION_HPP

#include <Epona/FloatingPoint.hpp>
#include <glm/glm.hpp>

namespace arion
{
namespace intersection
{

/** Stores contact information */
struct ContactManifold
{
    struct ContactPoints
    {
        glm::vec3 aWorldSpace;
        glm::vec3 bWorldSpace;
    };

    ContactPoints points;
    glm::vec3 normal;
    float penetration;
};

using ContactPoints = ContactManifold::ContactPoints;

/** Stores ray factors for Ray collisions */
struct RayIntersectionFactors
{
    //! Factor to closest to ray origin intersection point
    float tMin;

    //! Factor to furthest from ray origin intersection point
    float tMax;
};

/**
 *  @brief  Checks if Ray and Sphere are intersecting
 *
 *  @param  raySphere      vector from the ray origin to the sphere center
 *  @param  sphereRadius   radius of the sphere
 *  @param  rayDirection   normalized direction vector of the ray
 *
 *  @return @c true if there is intersection, @c false otherwise
 */
bool CheckRaySphereIntersection(
    glm::vec3 const& raySphere, float sphereRadius, glm::vec3 const& rayDirection
);

/**
 *  @brief  Calculates ray factors for the Ray-Sphere intersection points
 *
 *  @attention  Must be called only if given ray and sphere are intersecting
 *
 *  @param  raySphere       vector from the ray to the sphere center
 *  @param  sphereRadius    radius of the sphere
 *  @param  rayDirection    normalized direction vector of the ray
 *
 *  @return ray factors for intersection
 */
RayIntersectionFactors CalculateRaySphereIntersectionFactors(
    glm::vec3 const& raySphere, float sphereRadius, glm::vec3 const& rayDirection
);

/** Stores AABB using minimum and maximum points */
struct AabbExtremalVertices
{
    glm::vec3 minVertex;
    glm::vec3 maxVertex;
};

/**
 *  @brief Calculates ray intersection factors for AABB-Ray collision
 *
 *  @param  boxMinPoint     min point of AABB
 *  @param  boxMaxPoint     max point of AABB
 *  @param  rayDirection    normalized direction vector
 *  @param  rayOrigin       ray origin
 *
 *  @return ray factors for intersection
 */
RayIntersectionFactors CalculateRayAabbIntersectionFactors(
    glm::vec3 const& boxMinPoint, glm::vec3 const& boxMaxPoint,
    glm::vec3 const& rayDirection, glm::vec3 const& rayOrigin
);

/**
  * @brief  Calculates AABB min and max points from the given OBB basis
  *
  * @attention  given vectors must be different
  *
  * @param  i   vector from an orthogonal basis
  * @param  j   vector from an orthogonal basis
  * @param  k   vector from an orthogonal basis
  *
  * @return AABB min and max points
  */
AabbExtremalVertices MakeExtremalVerticesAabb(
    glm::vec3 const& i, glm::vec3 const& j, glm::vec3 const& k
);

/**
 *  @brief  Checks if Ray intersection factors indicate a valid intersection
 *
 *  @param  factors ray factors for intersection
 *
 *  @return @c true if intersection factors are valid, @c false otherwise
 */
bool CheckRayIntersectionFactors(RayIntersectionFactors factors);

/**
 *  @brief  Checks if two vectors are at acute angle
 *
 *  @param  aVector input vector
 *  @param  bVector input vector
 *
 *  @return @c true if vectors are at acute angle, @c false otherwise
 */
inline bool IsAngleAcute(glm::vec3 const& aVector, glm::vec3 const& bVector)
{
    return epona::fp::IsGreater(glm::dot(aVector, bVector), 0);
}

/**
 *  @brief  Checks if two points are on the same side of the halfspace
 *
 *  Halfspace is defined by a line and a point
 *
 *  @param  lineStart   line start point
 *  @param  lineEnd     line end point
 *  @param  aPoint      point of interest
 *  @param  bPoint      point of interest
 *
 *  @return @c true if points are on the same side of the halfspace, @c false otherwise
 */
inline bool IsSameSide(
    glm::vec3 const& lineStart, glm::vec3 const& lineEnd,
    glm::vec3 const& aPoint, glm::vec3 const& bPoint
)
{
    glm::vec3 const cp1 = glm::cross(lineEnd - lineStart, aPoint - lineStart);
    glm::vec3 const cp2 = glm::cross(lineEnd - lineStart, bPoint - lineStart);
    return epona::fp::IsGreaterOrEqual(glm::dot(cp1, cp2), 0);
}

/**
 *  @brief  Checks if point is inside triangle
 *
 *  @param  triangleVertex1 triangle vertex
 *  @param  triangleVertex2 triangle vertex
 *  @param  triangleVertex3 triangle vertex
 *  @param  point           point of interest
 *
 *  @return @c true if point is inside triangle, @c false otherwise
 */
bool IsPointInsideTriangle(
    glm::vec3 const& triangleVertex1, glm::vec3 const& triangleVertex2,
    glm::vec3 const& triangleVertex3, glm::vec3 const& point
);
} // namespace intersection
} // namespace arion

#endif // ARION_INTERSECTION_HPP
