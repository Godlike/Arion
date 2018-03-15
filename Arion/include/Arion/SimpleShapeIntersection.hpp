/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_SSI_HPP
#define ARION_SSI_HPP

#include <Arion/Intersection.hpp>
#include <Arion/GilbertJohnsonKeerthi.hpp>
#include <Arion/ExpandingPolytopeAlgorithm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/optimum_pow.hpp>

#include <utility>
#include <array>

namespace arion
{
namespace intersection
{

/** Base cache data structure for shapes intersection queries */
struct CacheBase
{
};

/** Specialized cache data structure for shapes intersection queries */
template <typename ShapeA, typename ShapeB>
struct Cache : CacheBase
{
};

/**
 * @brief Performs intersection test using shapes and cache data and returns true if shapes are intersecting
 *
 * @tparam ShapeA SimpleShape or SimpleShape derived object
 * @tparam ShapeB SimpleShape or SimpleShape derived object
 *
 * @param[in]       a pointer to the ShapeA type object
 * @param[in]       b pointer to the ShapeB type object
 * @param[in, out]  cache current computation cache
 *
 * @return @c true if there is intersection, @c false otherwise
 */
template <typename ShapeA, typename ShapeB>
bool CalculateIntersection(SimpleShape const* a, SimpleShape const* b, CacheBase* cache);

/**
 * @brief Calculates surface contact normal of b shape using shapes and cache data and returns it
 *
 * @attention Must be called strictly after the corresponding CalculateIntersection function call, otherwise result is undefined
 *
 * @tparam ShapeA SimpleShape or SimpleShape derived object
 * @tparam ShapeB SimpleShape or SimpleShape derived object
 *
 * @param[in]       a pointer to the ShapeA type object
 * @param[in]       b pointer to the ShapeB type object
 * @param[in, out]  cache current computation cache
 *
 * @return surface contact normal of the b object
 */
template <typename ShapeA, typename ShapeB>
glm::dvec3 CalculateContactNormal(SimpleShape const* a, SimpleShape const* b, CacheBase* cache);

/**
 * @brief Calculates world space contact points on the surfaces of given shapes
 *
 * @attention Must be called strictly after the corresponding CalculateContactNormal function call, otherwise result is undefined
 *
 * @tparam ShapeA SimpleShape or SimpleShape derived object
 * @tparam ShapeB SimpleShape or SimpleShape derived object
 *
 * @param[in]       a pointer to the ShapeA type object
 * @param[in]       b pointer to the ShapeB type object
 * @param[in, out]  cache current computation cache
 *
 * @return pair of the contact points where first is aShape contact point and second is bShape contact point in the world space
 */
template <typename ShapeA, typename ShapeB>
std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints(SimpleShape const* a, SimpleShape const* b, CacheBase* cache);

/**
 * @brief Calculates penetration depth using shapes and cache data and returns it
 *
 * @attention Must be called strictly after the corresponding CalculateContactNormal function call, otherwise result is undefined
 *
 * @tparam ShapeA SimpleShape or SimpleShape derived object
 * @tparam ShapeB SimpleShape or SimpleShape derived object
 *
 * @param[in]       a pointer to the ShapeA type object
 * @param[in]       b pointer to the ShapeB type object
 * @param[in, out]  cache current computation cache
 *
 * @return penetration depth
 */
template <typename ShapeA, typename ShapeB>
double CalculatePenetration(SimpleShape const* a, SimpleShape const* b, CacheBase* cache);

template <>
struct Cache<Ray, Ray> : CacheBase
{
    double denominator;
    glm::dvec3 aClosestApproach;
    glm::dvec3 bClosestApproach;
};

template <>
struct Cache<Ray, Plane> : CacheBase
{
    glm::dvec3 contact;
};

template <>
struct Cache<Ray, Sphere> : CacheBase
{
    glm::dvec3 sphereContactNormal;
    glm::dvec3 inPoint;
    glm::dvec3 outPoint;
    bool intersection;
};

template <>
struct Cache<Ray, Box> : CacheBase
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

template <>
struct Cache<Plane, Ray> : CacheBase
{
    Cache<Ray, Plane> rpCache;
};

template <>
struct Cache<Plane, Plane> : CacheBase
{
};

template <>
struct Cache<Plane, Sphere> : CacheBase
{
    double penetration;
};

template <>
struct Cache<Plane, Box> : CacheBase
{
    std::array<glm::dvec3, 8> boxVertices;
    std::array<glm::dvec3, 6> boxFaces;
    glm::dvec3 boxWorldContactPoint;
    std::array<double, 6> boxFaceDistances;
    std::array<double, 8> boxPenetrations;
    uint8_t closestVertexIndex;
};

template <>
struct Cache<Sphere, Ray> : CacheBase
{
    Cache<Ray, Sphere> rsCache;
};

template <>
struct Cache<Sphere, Plane> : CacheBase
{
    Cache<Plane, Sphere> psCache;
};

template <>
struct Cache<Sphere, Sphere> : CacheBase
{
    glm::dvec3 baVector;
    glm::dvec3 contactNormal;
    double radiusSum;
};

template <>
struct Cache<Sphere, Box> : CacheBase
{
    gjk::Simplex simplex;
    epa::ContactManifold manifold;
    bool intersection;
};

template <>
struct Cache<Box, Ray> : CacheBase
{
    Cache<Ray, Box> rbCache;
};

template <>
struct Cache<Box, Plane> : CacheBase
{
    Cache<Plane, Box> pbCache;
    glm::dvec3 planeMassCenter;
    glm::dvec3 boxContactNormal;
};

template <>
struct Cache<Box, Sphere> : CacheBase
{
    gjk::Simplex simplex;
    epa::ContactManifold manifold;
    bool intersection;
};

template <>
struct Cache<Box, Box> : CacheBase
{
    gjk::Simplex simplex;
    epa::ContactManifold manifold;
    bool intersection;
};

/** Ray, Ray CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Ray>*>(cacheBase);
    auto aRay = static_cast<Ray const*>(a);
    auto bRay = static_cast<Ray const*>(b);

    glm::dmat3 const aNumerator{
        bRay->centerOfMass - aRay->centerOfMass,
        bRay->direction,
        glm::cross(aRay->centerOfMass, bRay->centerOfMass)
    };
    glm::dmat3 const bNumerator{
        bRay->centerOfMass - aRay->centerOfMass,
        aRay->direction,
        glm::cross(aRay->centerOfMass, bRay->centerOfMass)
    };
    cache->denominator = glm::length2(aNumerator[2]);

    if (cache->denominator == 0.0)
    {
        return false;
    }

    cache->aClosestApproach =
        aRay->centerOfMass + aNumerator / cache->denominator * aRay->direction;
    cache->bClosestApproach =
        bRay->centerOfMass + bNumerator / cache->denominator * bRay->direction;

    return epona::fp::IsEqual(glm::length2(cache->aClosestApproach), glm::length2(cache->bClosestApproach));
}

/** Ray, Ray CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto aRay = static_cast<Ray const*>(a);
    auto bRay = static_cast<Ray const*>(b);

    return glm::normalize(glm::cross(glm::cross(bRay->direction, aRay->direction), bRay->direction));
}

/** Ray, Ray CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Ray>*>(cacheBase);
    return std::make_pair(cache->aClosestApproach, cache->bClosestApproach);
}

/** Ray, Ray CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    return 0;
}

/** Ray, Plane CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Plane>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto plane = static_cast<Plane const*>(b);

    epona::HyperPlane hyperPlane{ plane->normal, plane->centerOfMass };

    return hyperPlane.RayIntersection(ray->direction, ray->centerOfMass, cache->contact);
}

/** Ray, Plane CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto ray = static_cast<Ray const*>(a);
    return ray->direction;
}

/** Ray, Plane CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Plane>*>(cacheBase);
    return std::make_pair(cache->contact, cache->contact);
}

/** Ray, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    return std::numeric_limits<double>::max();
}

/** Ray, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Sphere>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto sphere = static_cast<Sphere const*>(b);

    cache->intersection = CheckRaySphereIntersection(
        sphere->centerOfMass - ray->centerOfMass, sphere->radius, ray->direction
    );

    return cache->intersection;
}

/** Ray, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Sphere>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto sphere = static_cast<Sphere const*>(b);

    if (cache->intersection)
    {
        RayIntersectionFactors intersectionFactors = CalculateRaySphereIntersectionFactors(
            sphere->centerOfMass - ray->centerOfMass, sphere->radius, ray->direction
        );

        cache->inPoint = ray->centerOfMass + ray->direction * intersectionFactors.tMin;
        cache->outPoint = ray->centerOfMass + ray->direction * intersectionFactors.tMax;
    }

    return glm::normalize(cache->inPoint - sphere->centerOfMass);
}

/** Ray, Sphere CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Sphere>*>(cacheBase);
    return std::make_pair(cache->inPoint, cache->inPoint);
}

/** Ray, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Sphere>*>(cacheBase);
    return glm::length(cache->inPoint - cache->outPoint);
}

/** Ray, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Ray, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Box>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto box = static_cast<Box const*>(b);

    //Transforming OBB into AABB, and moving ray into AABB space
    cache->boxModelMatrix = glm::dmat3{
        glm::normalize(box->iAxis), glm::normalize(box->jAxis), glm::normalize(box->kAxis)
    };
    glm::dmat3 const boxModelMatrixInverse = glm::inverse(cache->boxModelMatrix);
    cache->rayDirectionBoxSpace = boxModelMatrixInverse * ray->direction;
    cache->rayOriginBoxSpace = boxModelMatrixInverse * (ray->centerOfMass - box->centerOfMass);

    AabbExtremalVertices aabb = MakeExtremalVerticesAabb(box->iAxis, box->jAxis, box->kAxis);
    cache->aabbMinPoint = aabb.minVertex;
    cache->aabbMaxPoint = aabb.maxVertex;

    RayIntersectionFactors rayFactors = CalculateRayAabbIntersectionFactors(
        cache->aabbMinPoint, cache->aabbMaxPoint, cache->rayDirectionBoxSpace, cache->rayOriginBoxSpace
    );

    //Calculating intersection points in the obb model space
    cache->inPoint = cache->rayOriginBoxSpace + cache->rayDirectionBoxSpace * rayFactors.tMin;
    cache->outPoint = cache->rayOriginBoxSpace + cache->rayDirectionBoxSpace * rayFactors.tMax;

    return CheckRayIntersectionFactors(rayFactors);
}

/** Ray, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Ray, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Box>*>(cacheBase);
    auto box = static_cast<Box const*>(b);

    std::array<double, 6> const faces = { {
            cache->aabbMaxPoint[0], cache->aabbMaxPoint[1], cache->aabbMaxPoint[2],
            cache->aabbMinPoint[0], cache->aabbMinPoint[1], cache->aabbMinPoint[2]
        } };

    std::array<double, 6> const deltas = { {
            faces[0] - cache->inPoint[0], faces[1] - cache->inPoint[1], faces[2] - cache->inPoint[2],
            faces[3] - cache->inPoint[0], faces[4] - cache->inPoint[1], faces[5] - cache->inPoint[2],
        } };

    size_t const contactFaceIndex = std::distance(deltas.begin(),
        std::min_element(deltas.begin(), deltas.end(),
            [](double a, double b) -> bool
    {
        return glm::abs(a) < glm::abs(b);
    }
    ));

    //Transforming intersection points in the world space
    cache->inPoint = cache->boxModelMatrix * cache->inPoint + box->centerOfMass;
    cache->outPoint = cache->boxModelMatrix * cache->outPoint + box->centerOfMass;

    //Calculating box contact normal
    cache->boxContactNormal = {};
    cache->boxContactNormal[contactFaceIndex % 3] = faces[contactFaceIndex];
    cache->boxContactNormal = cache->boxModelMatrix * cache->boxContactNormal;

    return cache->boxContactNormal;
}

/** Ray, Box CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Ray, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Box>*>(cacheBase);
    return std::make_pair(cache->inPoint, cache->inPoint);
}

/** Ray, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Ray, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Box>*>(cacheBase);
    return glm::length(cache->outPoint - cache->inPoint);
}

/** Plane, Plane CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto aPlane = static_cast<Plane const*>(a);
    auto bPlane = static_cast<Plane const*>(b);

    return glm::length2(glm::cross(aPlane->normal, bPlane->normal)) != 0;
}

/** Plane, Plane CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    const auto bPlane = static_cast<Plane const*>(b);
    return bPlane->normal;
}

/** Plane, Plane CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const aPlane = static_cast<Plane const*>(a);
    auto const bPlane = static_cast<Plane const*>(b);
    epona::HyperPlane const bHyperPlane{ bPlane->normal, bPlane->centerOfMass };

    glm::dvec3 const contact = aPlane->normal
        * glm::dot(aPlane->normal, bHyperPlane.GetDistance() * bHyperPlane.GetNormal());

    return std::make_pair(contact, contact);
}

/** Plane, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    return std::numeric_limits<double>::max();
}

/** Plane, Ray CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Ray>*>(cacheBase);
    return CalculateIntersection<Ray, Plane>(b, a, &cache->rpCache);
}

/** Plane, Ray CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto ray = static_cast<Ray const*>(b);
    return -ray->direction;
}

/** Plane, Ray CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Ray>*>(cacheBase);
    return std::make_pair(cache->rpCache.contact, cache->rpCache.contact);
}

/** Plane, Ray CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Ray>*>(cacheBase);
    return CalculatePenetration<Ray, Plane>(b, a, &cache->rpCache);
}


/** Plane, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto plane = static_cast<Plane const*>(a);
    auto sphere = static_cast<Sphere const*>(b);
    auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);

    cache->penetration = sphere->radius -
        (glm::dot(sphere->centerOfMass, plane->normal) - glm::dot(plane->centerOfMass, plane->normal));;

    return cache->penetration >= 0.0;
}

/** Plane, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto plane = static_cast<Plane const*>(a);
    return -plane->normal;
}

/** Plane, Sphere CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const plane = static_cast<Plane const*>(a);
    auto const sphere = static_cast<Sphere const*>(b);
    epona::HyperPlane const hyperPlane{ plane->normal, plane->centerOfMass };

    return std::make_pair(
        hyperPlane.ClosestPoint(sphere->centerOfMass),
        (-plane->normal * sphere->radius + sphere->centerOfMass)
    );
}

/** Plane, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);
    return cache->penetration;
}

/** Plane, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto plane = static_cast<Plane const*>(a);
    auto box = static_cast<Box const*>(b);
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);

    cache->boxFaces = { { box->iAxis, box->jAxis, box->kAxis, -box->iAxis, -box->jAxis, -box->kAxis } };

    //Calculate world space box vertices
    epona::CalculateBoxVerticesModel(box->iAxis, box->jAxis, box->kAxis, cache->boxVertices.begin());
    for (glm::dvec3& vertex : cache->boxVertices)
    {
        glm::dvec3 const resultVertex = glm::dmat4(glm::toMat4(box->orientation))
            * glm::dvec4{ vertex, 1 } + glm::dvec4{ box->centerOfMass, 1 };
        vertex = glm::dvec3{ resultVertex.x, resultVertex.y, resultVertex.z };
    }

    //Calculate box vertices's distances to the plane
    double const planeDistance = glm::dot(plane->centerOfMass, plane->normal);
    std::transform(cache->boxVertices.begin(), cache->boxVertices.end(), cache->boxPenetrations.begin(),
        [planeDistance, &plane](glm::dvec3 const& p) -> double
    {
        return planeDistance - glm::dot(p, plane->normal);
    });

    //Find vertex with most penetration depth
    cache->closestVertexIndex = static_cast<uint8_t>(
        std::distance(cache->boxPenetrations.begin(),
            std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end())
    ));

    return cache->boxPenetrations[cache->closestVertexIndex] >= 0.0;
}

/** Plane, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);
    auto plane = static_cast<Plane const*>(a);

    std::transform(cache->boxFaces.begin(), cache->boxFaces.end(), cache->boxFaceDistances.begin(),
        [&plane](glm::dvec3 const& v) -> double
    {
        return glm::dot(v, plane->normal);
    });
    size_t const minIndex = std::distance(cache->boxFaceDistances.begin(),
        std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end())
    );

    return glm::normalize(cache->boxFaces[minIndex]);
}

/** Plane, Box CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);
    auto plane = static_cast<Plane const*>(a);
    epona::HyperPlane hyperPlane(plane->normal, plane->centerOfMass);

    uint8_t contactPointCounter = 0;
    cache->boxWorldContactPoint = glm::dvec3{ 0 };
    for (uint8_t i = 0; i < cache->boxPenetrations.size(); ++i)
    {
        if (epona::fp::IsGreaterOrEqual(cache->boxPenetrations[i], 0))
        {
            contactPointCounter++;
            cache->boxWorldContactPoint += cache->boxVertices[i];
        }
    }
    cache->boxWorldContactPoint = (contactPointCounter == 0)
        ? cache->boxVertices[cache->closestVertexIndex]
        : cache->boxWorldContactPoint / static_cast<double>(contactPointCounter);

    return std::make_pair(
        hyperPlane.ClosestPoint(cache->boxWorldContactPoint),
        cache->boxWorldContactPoint
    );
}

/** Plane, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);
    return *std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end());
}

/** Sphere, Plane CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    return CalculateIntersection<Plane, Sphere>(b, a, &cache->psCache);
}

/** Sphere, Plane CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    return -CalculateContactNormal<Plane, Sphere>(b, a, &cache->psCache);
}

/** Sphere, Plane CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    auto const psContactPointsPair = CalculateContactPoints<Plane, Sphere>(b, a, &cache->psCache);
    return std::make_pair(psContactPointsPair.second, psContactPointsPair.first);
}

/** Sphere, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    return CalculatePenetration<Plane, Sphere>(b, a, &cache->psCache);
}

/** Sphere, Ray CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Ray>*>(cacheBase);
    return CalculateIntersection<Ray, Sphere>(b, a, &cache->rsCache);
}

/** Sphere, Ray CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto ray = static_cast<Ray const*>(b);
    return -ray->direction;
}

/** Sphere, Ray CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Ray>*>(cacheBase);
    return std::make_pair(cache->rsCache.inPoint, cache->rsCache.inPoint);
}

/** Sphere, Ray CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Ray>*>(cacheBase);
    return CalculatePenetration<Ray, Sphere>(b, a, &cache->rsCache);
}

/** Sphere, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    auto const aSphere = static_cast<Sphere const*>(a);
    auto const bSphere = static_cast<Sphere const*>(b);

    cache->baVector = aSphere->centerOfMass - bSphere->centerOfMass;
    cache->radiusSum = aSphere->radius + bSphere->radius;

    return glm::pow2(cache->radiusSum) > glm::length2(cache->baVector);
}

/** Sphere, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    cache->contactNormal = glm::normalize(cache->baVector);
    return cache->contactNormal;
}

/** Sphere, Sphere CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Sphere> const*>(cacheBase);
    auto const aSphere = static_cast<Sphere const*>(a);
    auto const bSphere = static_cast<Sphere const*>(b);

    return std::make_pair(
        -cache->contactNormal * aSphere->radius + aSphere->centerOfMass,
         cache->contactNormal * bSphere->radius + bSphere->centerOfMass
    );
}

/** Sphere, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Sphere> const*>(cacheBase);
    return cache->radiusSum - glm::length(cache->baVector);
}

/** Sphere, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Box>*>(cacheBase);
    auto const sphere = static_cast<Sphere const*>(a);
    auto const box = static_cast<Box const*>(b);

    cache->intersection = gjk::CalculateIntersection(cache->simplex, *box, *sphere);
    return cache->intersection;
}

/** Sphere, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Box>*>(cacheBase);
    auto const sphere = static_cast<Sphere const*>(a);
    auto const box = static_cast<Box const*>(b);

    cache->manifold = epa::CalculateContactManifold(*box, *sphere, cache->simplex);

    return cache->manifold.contactNormal;
}

/** Sphere, Box CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Box> const*>(cacheBase);
    return std::make_pair(cache->manifold.aContactPointWorldSpace, cache->manifold.bContactPointWorldSpace);
}

/** Sphere, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Box> const*>(cacheBase);
    return cache->manifold.penetration;
}

/** Box, Ray CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Box, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Ray>*>(cacheBase);
    return CalculateIntersection<Ray, Box>(b, a, &cache->rbCache);
}

/** Box, Ray CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Ray>*>(cacheBase);
    auto const ray = static_cast<Ray const*>(b);
    CalculateContactNormal<Ray, Box>(b, a, &cache->rbCache);
    return ray->direction;
}

/** Box, Ray CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Box, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Ray>*>(cacheBase);
    auto const contactPointsPair = CalculateContactPoints<Ray, Box>(b, a, &cache->rbCache);
    return std::make_pair(contactPointsPair.second, contactPointsPair.first);
}

/** Box, Ray CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Box, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Ray>*>(cacheBase);
    return CalculatePenetration<Ray, Box>(b, a, &cache->rbCache);
}

/** Box, Plane CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
    return CalculateIntersection<Plane, Box>(b, a, &cache->pbCache);
}

/** Box, Plane CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const plane = static_cast<Plane const*>(b);
    return plane->normal;
}

/** Box, Plane CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3>  CalculateContactPoints<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
    auto const contactPointsPair = CalculateContactPoints<Plane, Box>(b, a, &cache->pbCache);
    return std::make_pair(contactPointsPair.second, contactPointsPair.first);
}

/** Box, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
    return CalculatePenetration<Plane, Box>(b, a, &cache->pbCache);
}

/** Box, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
    auto const box = static_cast<Box const*>(a);
    auto const sphere = static_cast<Sphere const*>(b);

    cache->intersection = gjk::CalculateIntersection(cache->simplex, *sphere, *box);

    return cache->intersection;
}

/** Box, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
    auto const box = static_cast<Box const*>(a);
    auto const sphere = static_cast<Sphere const*>(b);

    cache->manifold = epa::CalculateContactManifold(*sphere, *box, cache->simplex);

    return cache->manifold.contactNormal;
}

/** Box, Sphere CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Box, Sphere>(SimpleShape const* a, SimpleShape const *b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Sphere> const*>(cacheBase);

    return std::make_pair(cache->manifold.aContactPointWorldSpace, cache->manifold.bContactPointWorldSpace);
}

/** Box, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Sphere> const*>(cacheBase);

    return cache->manifold.penetration;
}

/** Box, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const aBox = static_cast<Box const*>(a);
    auto const bBox = static_cast<Box const*>(b);
    auto const cache = static_cast<Cache<Box, Box>*>(cacheBase);

    cache->intersection = gjk::CalculateIntersection(cache->simplex, *bBox, *aBox);
    return cache->intersection;
}

/** Box, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Box>*>(cacheBase);
    auto const aBox = static_cast<Box const*>(a);
    auto const bBox = static_cast<Box const*>(b);

    cache->manifold = epa::CalculateContactManifold(*bBox, *aBox, cache->simplex);

    return cache->manifold.contactNormal;
}

/** Box, Box CalculateContactPoints specialization */
template <>
inline std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Box> const*>(cacheBase);

    return std::make_pair(cache->manifold.aContactPointWorldSpace, cache->manifold.bContactPointWorldSpace);
}

/** Box, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Box> const*>(cacheBase);

    return cache->manifold.penetration;
}

} // namespace intersection
} // namespace arion

#endif // ARION_SSI_HPP
