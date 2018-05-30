/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_RAY_CASTING_HPP
#define ARION_RAY_CASTING_HPP

#include <Arion/Shape.hpp>
#include <Arion/Intersection.hpp>

namespace arion
{
namespace intersection
{

struct CacheRayRay
{
    double denominator;
    glm::dvec3 aClosestApproach;
    glm::dvec3 bClosestApproach;
};

struct CacheRayPlane
{
    glm::dvec3 contact;
};

struct CacheRaySphere
{
    glm::dvec3 inPoint;
    glm::dvec3 outPoint;
    bool intersection;
};

struct CacheRayBox
{
    glm::dvec3 rayDirectionBoxSpace;
    glm::dvec3 rayOriginBoxSpace;
    glm::dmat3 boxModelMatrix;
    glm::dvec3 aabbMaxPoint;
    glm::dvec3 aabbMinPoint;
    glm::dvec3 inPoint;
    glm::dvec3 outPoint;
    glm::dvec3 boxContactNormal;
};

/** Ray, Ray CalculateIntersection specialization */
inline bool CalculateIntersection(Ray const& aRay, Ray const& bRay, CacheRayRay& cache);

/** Ray, Ray CalculateContactPoints specialization */
inline ContactPoints CalculateContactPoints(Ray const& aRay, Ray const& bRay, CacheRayRay& cache);

/** Ray, Plane CalculateIntersection specialization */
inline bool CalculateIntersection(Ray const& ray, Plane const& plane, CacheRayPlane& cache);

/** Ray, Plane CalculateContactNormal specialization */
inline glm::dvec3 CalculateContactNormal(Ray const& ray, Plane const& plane, CacheRayPlane& cache);

/** Ray, Plane CalculateContactPoints specialization */
inline ContactPoints CalculateContactPoints(Ray const& ray, Plane const& plane, CacheRayPlane& cache);

/** Ray, Plane CalculatePenetration specialization */
inline double CalculatePenetration(Ray const& ray, Plane const& plane, CacheRayPlane& cacheBase);

/** Ray, Sphere CalculateIntersection specialization */
inline bool CalculateIntersection(Ray const& ray, Sphere const& sphere, CacheRaySphere& cache);

/** Ray, Sphere CalculateContactNormal specialization */
inline glm::dvec3 CalculateContactNormal(Ray const& ray, Sphere const& sphere, CacheRaySphere& cache);

/** Ray, Sphere CalculateContactPoints specialization */
inline ContactPoints CalculateContactPoints(Ray const& ray, Sphere const& sphere, CacheRaySphere& cache);

/** Ray, Sphere CalculatePenetration specialization */
inline double CalculatePenetration(Ray const& ray, Sphere const& sphere, CacheRaySphere& cache);

/** Ray, Box CalculateIntersection specialization */
inline bool CalculateIntersection(Ray const& ray, Box const& box, CacheRayBox& cache);

/** Ray, Box CalculateContactNormal specialization */
inline glm::dvec3 CalculateContactNormal(Ray const& ray, Box const& box, CacheRayBox& cache);

/** Ray, Box CalculateContactPoints specialization */
inline ContactPoints CalculateContactPoints(Ray const& ray, Box const& box, CacheRayBox& cache);

/** Ray, Box CalculatePenetration specialization */
inline double CalculatePenetration(Ray const& ray, Box const& box, CacheRayBox& cache);

/** Plane, Ray CalculateIntersection specialization */
inline bool CalculateIntersection(Plane const& plane, Ray const& ray, CacheRayPlane& cache);

/** Plane, Ray CalculateContactNormal specialization */
inline glm::dvec3 CalculateContactNormal(Plane const& plane, Ray const& ray, CacheRayPlane& cache);

/** Plane, Ray CalculateContactPoints specialization */
inline ContactPoints CalculateContactPoints(Plane const& plane, Ray const& ray, CacheRayPlane& cache);

/** Plane, Ray CalculatePenetration specialization */
inline double CalculatePenetration(Plane const& plane, Ray const& ray, CacheRayPlane& cache);

/** Sphere, Ray CalculateIntersection specialization */
inline bool CalculateIntersection(Sphere const& sphere, Ray const& ray, CacheRaySphere& cache);

/** Sphere, Ray CalculateContactNormal specialization */
inline glm::dvec3 CalculateContactNormal(Sphere const& sphere, Ray const& ray, CacheRaySphere& cache);

/** Sphere, Ray CalculateContactPoints specialization */
inline ContactPoints CalculateContactPoints(Sphere const& sphere, Ray const& ray, CacheRaySphere& cache);

/** Sphere, Ray CalculatePenetration specialization */
inline double CalculatePenetration(Sphere const& sphere, Ray const& ray, CacheRaySphere& cache);

/** Box, Ray CalculateIntersection specialization */
inline bool CalculateIntersection(Box const& box, Ray const& ray, CacheRayBox& cache);

/** Box, Ray CalculateContactNormal specialization */
inline glm::dvec3 CalculateContactNormal(Box const& box, Ray const& ray, CacheRayBox& cache);

/** Box, Ray CalculateContactPoints specialization */
inline ContactPoints CalculateContactPoints(Box const& box, Ray const& ray, CacheRayBox& cache);

/** Box, Ray CalculatePenetration specialization */
inline double CalculatePenetration(Box const& box, Ray const& ray, CacheRayBox& cache);

} // namespace intersection
} // namepsace arion

#endif // ARION_RAY_CASTING_HPP
