/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#include <Arion/RayCasting.hpp>
#include <Epona/HyperPlane.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#include <array>
#include <algorithm>

namespace arion
{
namespace intersection
{

inline bool CalculateIntersection(Ray const& aRay, Ray const& bRay, CacheRayRay& cache)
{
    glm::dmat3 const aNumerator{
        bRay.centerOfMass - aRay.centerOfMass,
        bRay.direction,
        glm::cross(aRay.centerOfMass, bRay.centerOfMass)
    };
    glm::dmat3 const bNumerator{
        bRay.centerOfMass - aRay.centerOfMass,
        aRay.direction,
        glm::cross(aRay.centerOfMass, bRay.centerOfMass)
    };
    cache.denominator = glm::length2(aNumerator[2]);

    if (cache.denominator == 0.0)
    {
        return false;
    }

    cache.aClosestApproach =
        aRay.centerOfMass + aNumerator / cache.denominator * aRay.direction;
    cache.bClosestApproach =
        bRay.centerOfMass + bNumerator / cache.denominator * bRay.direction;

    return epona::fp::IsEqual(glm::length2(cache.aClosestApproach), glm::length2(cache.bClosestApproach));
}

inline ContactPoints CalculateContactPoints(Ray const& aRay, Ray const& bRay, CacheRayRay& cache)
{
    return {
        cache.aClosestApproach,
        cache.bClosestApproach,
    };
}

inline bool CalculateIntersection(Ray const& ray, Plane const& plane, CacheRayPlane& cache)
{
    epona::HyperPlane hyperPlane{ plane.normal, plane.centerOfMass };
    return hyperPlane.RayIntersection(ray.direction, ray.centerOfMass, cache.contact);
}

inline glm::dvec3 CalculateContactNormal(Ray const& ray, Plane const& plane, CacheRayPlane& cache)
{
    return ray.direction;
}

inline ContactPoints CalculateContactPoints(Ray const& ray, Plane const& plane, CacheRayPlane& cache)
{
    return {
        cache.contact,
        cache.contact,
    };
}

inline double CalculatePenetration(Ray const& ray, Plane const& plane, CacheRayPlane& cacheBase)
{
    return std::numeric_limits<double>::max();
}

inline bool CalculateIntersection(Ray const& ray, Sphere const& sphere, CacheRaySphere& cache)
{
    cache.intersection = CheckRaySphereIntersection(
        sphere.centerOfMass - ray.centerOfMass, sphere.radius, ray.direction);
    return cache.intersection;
}

inline glm::dvec3 CalculateContactNormal(Ray const& ray, Sphere const& sphere, CacheRaySphere& cache)
{
    RayIntersectionFactors const intersectionFactors = CalculateRaySphereIntersectionFactors(
        sphere.centerOfMass - ray.centerOfMass, sphere.radius, ray.direction
    );

    cache.inPoint = ray.centerOfMass + ray.direction * intersectionFactors.tMin;
    cache.outPoint = ray.centerOfMass + ray.direction * intersectionFactors.tMax;

    return ray.direction;
}

inline ContactPoints CalculateContactPoints(Ray const& ray, Sphere const& sphere, CacheRaySphere& cache)
{
    return {
        cache.inPoint,
        cache.inPoint
    };
}

inline double CalculatePenetration(Ray const& ray, Sphere const& sphere, CacheRaySphere& cache)
{
    return glm::length(cache.inPoint - cache.outPoint);
}

inline bool CalculateIntersection(Ray const& ray, Box const& box, CacheRayBox& cache)
{
    //Transforming OBB into AABB, and moving ray into AABB space
    cache.boxModelMatrix = glm::dmat3{
        glm::normalize(box.iAxis), glm::normalize(box.jAxis), glm::normalize(box.kAxis)
    };
    glm::dmat3 const boxModelMatrixInverse = glm::inverse(cache.boxModelMatrix);
    cache.rayDirectionBoxSpace = boxModelMatrixInverse * ray.direction;
    cache.rayOriginBoxSpace = boxModelMatrixInverse * (ray.centerOfMass - box.centerOfMass);

    AabbExtremalVertices const aabb = MakeExtremalVerticesAabb(box.iAxis, box.jAxis, box.kAxis);
    cache.aabbMinPoint = aabb.minVertex;
    cache.aabbMaxPoint = aabb.maxVertex;

    RayIntersectionFactors const rayFactors = CalculateRayAabbIntersectionFactors(
        cache.aabbMinPoint, cache.aabbMaxPoint, cache.rayDirectionBoxSpace, cache.rayOriginBoxSpace
    );

    //Calculating intersection points in the obb model space
    cache.inPoint = cache.rayOriginBoxSpace + cache.rayDirectionBoxSpace * rayFactors.tMin;
    cache.outPoint = cache.rayOriginBoxSpace + cache.rayDirectionBoxSpace * rayFactors.tMax;

    return CheckRayIntersectionFactors(rayFactors);
}

inline glm::dvec3 CalculateContactNormal(Ray const& ray, Box const& box, CacheRayBox& cache)
{
    return ray.direction;
}

inline ContactPoints CalculateContactPoints(Ray const& ray, Box const& box, CacheRayBox& cache)
{
    return {
        cache.inPoint,
        cache.inPoint
    };
}

inline double CalculatePenetration(Ray const& ray, Box const& box, CacheRayBox& cache)
{
    return glm::length(cache.outPoint - cache.inPoint);
}

inline bool CalculateIntersection(Plane const& plane, Ray const& ray, CacheRayPlane& cache)
{
    return CalculateIntersection(ray, plane, cache);
}

inline glm::dvec3 CalculateContactNormal(Plane const& plane, Ray const& ray, CacheRayPlane& cache)
{
    return plane.normal;
}

inline ContactPoints CalculateContactPoints(Plane const& plane, Ray const& ray, CacheRayPlane& cache)
{
    auto const contactPoints = CalculateContactPoints(ray, plane, cache);

    return {
        contactPoints.bWorldSpace,
        contactPoints.aWorldSpace,
    };
}

inline double CalculatePenetration(Plane const& plane, Ray const& ray, CacheRayPlane& cache)
{
    return CalculatePenetration(ray, plane, cache);
}

inline bool CalculateIntersection(Sphere const& sphere, Ray const& ray, CacheRaySphere& cache)
{
    return CalculateIntersection(ray, sphere, cache);
}

inline glm::dvec3 CalculateContactNormal(Sphere const& sphere, Ray const& ray, CacheRaySphere& cache)
{
    return glm::normalize(cache.inPoint - sphere.centerOfMass);
}

inline ContactPoints CalculateContactPoints(Sphere const& sphere, Ray const& ray, CacheRaySphere& cache)
{
    auto const rsContactPoints = CalculateContactPoints(ray, sphere, cache);
    return {
        rsContactPoints.bWorldSpace,
        rsContactPoints.aWorldSpace,
    };
}

inline double CalculatePenetration(Sphere const& sphere, Ray const& ray, CacheRaySphere& cache)
{
    return CalculatePenetration(ray, sphere, cache);
}

inline bool CalculateIntersection(Box const& box, Ray const& ray, CacheRayBox& cache)
{
    return CalculateIntersection(ray, box, cache);
}

inline glm::dvec3 CalculateContactNormal(Box const& box, Ray const& ray, CacheRayBox& cache)
{
    std::array<double, 6> const faces = {{
            cache.aabbMaxPoint[0], cache.aabbMaxPoint[1], cache.aabbMaxPoint[2],
            cache.aabbMinPoint[0], cache.aabbMinPoint[1], cache.aabbMinPoint[2]
    }};

    std::array<double, 6> const deltas = {{
            faces[0] - cache.inPoint[0], faces[1] - cache.inPoint[1], faces[2] - cache.inPoint[2],
            faces[3] - cache.inPoint[0], faces[4] - cache.inPoint[1], faces[5] - cache.inPoint[2],
    }};

    size_t const contactFaceIndex = std::distance(deltas.begin(),
        std::min_element(deltas.begin(), deltas.end(),
            [](double a, double b) -> bool
    {
        return glm::abs(a) < glm::abs(b);
    }
    ));

    //Transforming intersection points in the world space
    cache.inPoint = cache.boxModelMatrix * cache.inPoint + box.centerOfMass;
    cache.outPoint = cache.boxModelMatrix * cache.outPoint + box.centerOfMass;

    //Calculating box contact normal
    cache.boxContactNormal = {};
    cache.boxContactNormal[contactFaceIndex % 3] = faces[contactFaceIndex];
    cache.boxContactNormal = cache.boxModelMatrix * cache.boxContactNormal;

    return cache.boxContactNormal;
}

inline ContactPoints CalculateContactPoints(Box const& box, Ray const& ray, CacheRayBox& cache)
{
    auto const contactPoints = CalculateContactPoints(ray, box, cache);
    return {
        contactPoints.bWorldSpace,
        contactPoints.aWorldSpace,
    };
}

inline double CalculatePenetration(Box const& box, Ray const& ray, CacheRayBox& cache)
{
    return CalculatePenetration(ray, box, cache);
}

} // namespace intersection
} // namepsace arion
