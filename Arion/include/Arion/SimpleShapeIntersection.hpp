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
 * @brief Calculates surface contact normal of shape @p a using shapes and cache data and returns it
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
 * @return two pairs of contact points in the model and in the world space
 */
template <typename ShapeA, typename ShapeB>
ContactPoints CalculateContactPoints(SimpleShape const* a, SimpleShape const* b, CacheBase* cache);

/**
 * @brief Calculates penetration depth using shapes and cache data and returns it
 *
 * @attention Must be called strictly after the corresponding CalculateContactPoints function call, otherwise result is undefined
 * @note Result of calling this function for non-colliding shapes is undefined
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

/**
 * @brief Calculates contact manifold for given shapes
 *
 * This function performs calculations of the contact normal,
 * contact points and penetration depth. The calculations
 * do not overlap with those of separate corresponding functions.
 * It is safe to use any of the other Calculate functions after
 * calling this one.
 *
 * @attention Must be called strictly after the corresponding CalculateIntersection function call, otherwise result is undefined
 * @note Result of calling this function for non-colliding shapes is undefined
 *
 * @tparam ShapeA SimpleShape or SimpleShape derived object
 * @tparam ShapeB SimpleShape or SimpleShape derived object
 *
 * @param[in]       a pointer to the ShapeA type object
 * @param[in]       b pointer to the ShapeB type object
 * @param[in, out]  cache current computation cache
 *
 * @return contact manifold
 */
template <typename ShapeA, typename ShapeB>
ContactManifold CalculateContactManifold(SimpleShape const* a, SimpleShape const* b, CacheBase* cache)
{
    ContactManifold manifold;

    manifold.normal = CalculateContactNormal<ShapeA, ShapeB>(a, b, cache);
    manifold.points = CalculateContactPoints<ShapeA, ShapeB>(a, b, cache);
    manifold.penetration = CalculatePenetration<ShapeA, ShapeB>(a, b, cache);

    return manifold;
}

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
struct Cache<Sphere, Plane> : CacheBase
{
    Cache<Plane, Sphere> psCache;
};

template <>
struct Cache<Sphere, Sphere> : CacheBase
{
    glm::dvec3 abVector;
    glm::dvec3 contactNormal;
    double radiusSum;
};

template <>
struct Cache<Sphere, Box> : CacheBase
{
    gjk::Simplex simplex;
    ContactManifold manifold;
    bool intersection;
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
    ContactManifold manifold;
    bool intersection;
};

template <>
struct Cache<Box, Box> : CacheBase
{
    gjk::Simplex simplex;
    ContactManifold manifold;
    bool intersection;
};

/** Plane, Plane CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const aPlane = static_cast<Plane const*>(a);
    auto const bPlane = static_cast<Plane const*>(b);

    return !epona::fp::IsZero(glm::length2(glm::cross(aPlane->normal, bPlane->normal)));
}

/** Plane, Plane CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const aPlane = static_cast<Plane const*>(a);
    return aPlane->normal;
}

/** Plane, Plane CalculateContactPoints specialization */
template <>
inline ContactPoints CalculateContactPoints<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const aPlane = static_cast<Plane const*>(a);
    auto const bPlane = static_cast<Plane const*>(b);
    epona::HyperPlane const bHyperPlane{ bPlane->normal, bPlane->centerOfMass };

    glm::dvec3 const contact = aPlane->normal
        * glm::dot(aPlane->normal, bHyperPlane.GetDistance() * bHyperPlane.GetNormal());

    return {
        contact,
        contact,
    };
}

/** Plane, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    return std::numeric_limits<double>::max();
}

/** Plane, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const plane = static_cast<Plane const*>(a);
    auto const sphere = static_cast<Sphere const*>(b);
    auto const cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);

    cache->penetration = sphere->radius -
        (glm::dot(sphere->centerOfMass, plane->normal) - glm::dot(plane->centerOfMass, plane->normal));;

    return epona::fp::IsGreaterOrEqual(cache->penetration, 0.0);
}

/** Plane, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const plane = static_cast<Plane const*>(a);
    return plane->normal;
}

/** Plane, Sphere CalculateContactPoints specialization */
template <>
inline ContactPoints CalculateContactPoints<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const plane = static_cast<Plane const*>(a);
    auto const sphere = static_cast<Sphere const*>(b);
    epona::HyperPlane const hyperPlane{ plane->normal, plane->centerOfMass };

    ContactPoints contactPoints;
    contactPoints.aWorldSpace = hyperPlane.ClosestPoint(sphere->centerOfMass);
    contactPoints.bWorldSpace = (-plane->normal * sphere->radius + sphere->centerOfMass);

    return contactPoints;
}

/** Plane, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);
    return cache->penetration;
}

/** Plane, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const plane = static_cast<Plane const*>(a);
    auto const box = static_cast<Box const*>(b);
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);

    //Calculate world space box vertices
    epona::CalculateBoxVerticesWorld(
        box->iAxis, box->jAxis, box->kAxis, box->centerOfMass, box->orientation, cache->boxVertices.begin()
    );

    //Calculate box vertices's distances to the plane
    epona::HyperPlane const hyperPlane{ plane->normal, plane->centerOfMass };
    std::transform(cache->boxVertices.begin(), cache->boxVertices.end(), cache->boxPenetrations.begin(),
        [&hyperPlane](glm::dvec3 const& p) -> double
    {
        return -hyperPlane.SignedDistance(p);
    });

    //Find vertex with most penetration depth
    cache->closestVertexIndex = static_cast<uint8_t>(
        std::distance(cache->boxPenetrations.begin(),
            std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end())
    ));

    return epona::fp::IsGreaterOrEqual(cache->boxPenetrations[cache->closestVertexIndex], 0.0);
}

/** Plane, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const plane = static_cast<Plane const*>(a);
    return plane->normal;
}

/** Plane, Box CalculateContactPoints specialization */
template <>
inline ContactPoints CalculateContactPoints<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);
    auto const plane = static_cast<Plane const*>(a);
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

    ContactPoints contactPoints;
    contactPoints.aWorldSpace = hyperPlane.ClosestPoint(cache->boxWorldContactPoint);
    contactPoints.bWorldSpace = cache->boxWorldContactPoint;

    return contactPoints;
}

/** Plane, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Plane, Box>*>(cacheBase);
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
inline ContactPoints CalculateContactPoints<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    auto const psContactPoints = CalculateContactPoints<Plane, Sphere>(b, a, &cache->psCache);
    return {
        psContactPoints.bWorldSpace,
        psContactPoints.aWorldSpace,
    };
}

/** Sphere, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    return CalculatePenetration<Plane, Sphere>(b, a, &cache->psCache);
}

/** Sphere, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    auto const aSphere = static_cast<Sphere const*>(a);
    auto const bSphere = static_cast<Sphere const*>(b);

    cache->abVector = bSphere->centerOfMass - aSphere->centerOfMass;
    cache->radiusSum = aSphere->radius + bSphere->radius;

    return epona::fp::IsGreaterOrEqual(glm::pow2(cache->radiusSum), glm::length2(cache->abVector));
}

/** Sphere, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    cache->contactNormal = glm::normalize(cache->abVector);
    return cache->contactNormal;
}

/** Sphere, Sphere CalculateContactPoints specialization */
template <>
inline ContactPoints CalculateContactPoints<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Sphere> const*>(cacheBase);
    auto const aSphere = static_cast<Sphere const*>(a);
    auto const bSphere = static_cast<Sphere const*>(b);

    ContactPoints contactPoints;
    contactPoints.aWorldSpace = cache->contactNormal * aSphere->radius + aSphere->centerOfMass;
    contactPoints.bWorldSpace = -cache->contactNormal * bSphere->radius + bSphere->centerOfMass;

    return contactPoints;
}

/** Sphere, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Sphere> const*>(cacheBase);
    return cache->radiusSum - glm::length(cache->abVector);
}

/** Sphere, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Box>*>(cacheBase);
    auto const sphere = static_cast<Sphere const*>(a);
    auto const box = static_cast<Box const*>(b);

    cache->intersection = gjk::CalculateIntersection(cache->simplex, *sphere, *box);
    return cache->intersection;
}

/** Sphere, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Box>*>(cacheBase);
    auto const sphere = static_cast<Sphere const*>(a);
    auto const box = static_cast<Box const*>(b);

    cache->manifold = epa::CalculateContactManifold(*sphere, *box, cache->simplex);

    return cache->manifold.normal;
}

/** Sphere, Box CalculateContactPoints specialization */
template <>
inline ContactPoints CalculateContactPoints<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Box> const*>(cacheBase);
    return cache->manifold.points;
}

/** Sphere, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Sphere, Box> const*>(cacheBase);
    return cache->manifold.penetration;
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
    auto const cache = &static_cast<Cache<Box, Plane>*>(cacheBase)->pbCache;
    auto const box = static_cast<Box const*>(a);
    auto const plane = static_cast<Plane const*>(b);

    cache->boxFaces = {{ box->iAxis, box->jAxis, box->kAxis, -box->iAxis, -box->jAxis, -box->kAxis }};
    for (auto& face : cache->boxFaces)
    {
        face = epona::ModelToWorldSpace(face, glm::dvec3{ 0, 0, 0 }, glm::toMat3(box->orientation));
    }

    epona::HyperPlane const hyperPlane{ plane->normal, plane->centerOfMass };
    std::transform(cache->boxFaces.begin(), cache->boxFaces.end(), cache->boxFaceDistances.begin(),
        [&hyperPlane, box](glm::dvec3 const& face) -> double
    {
        return hyperPlane.SignedDistance(face + box->centerOfMass);
    });
    size_t const minIndex = std::distance(cache->boxFaceDistances.begin(),
        std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end())
    );

    return glm::normalize(cache->boxFaces[minIndex]);
}

/** Box, Plane CalculateContactPoints specialization */
template <>
inline ContactPoints CalculateContactPoints<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
    auto const contactPoints = CalculateContactPoints<Plane, Box>(b, a, &cache->pbCache);
    return {
        contactPoints.bWorldSpace,
        contactPoints.aWorldSpace,
    };
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

    cache->intersection = gjk::CalculateIntersection(cache->simplex, *box, *sphere);
    return cache->intersection;
}

/** Box, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
    auto const box = static_cast<Box const*>(a);
    auto const sphere = static_cast<Sphere const*>(b);

    cache->manifold = epa::CalculateContactManifold(*box, *sphere, cache->simplex);
    return cache->manifold.normal;
}

/** Box, Sphere CalculateContactPoints specialization */
template <>
inline ContactPoints CalculateContactPoints<Box, Sphere>(SimpleShape const* a, SimpleShape const *b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Sphere> const*>(cacheBase);
    return cache->manifold.points;
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

    cache->intersection = gjk::CalculateIntersection(cache->simplex, *aBox, *bBox);
    return cache->intersection;
}

/** Box, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Box>*>(cacheBase);
    auto const aBox = static_cast<Box const*>(a);
    auto const bBox = static_cast<Box const*>(b);

    cache->manifold = epa::CalculateContactManifold(*aBox, *bBox, cache->simplex);
    return cache->manifold.normal;
}

/** Box, Box CalculateContactPoints specialization */
template <>
inline ContactPoints CalculateContactPoints<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto const cache = static_cast<Cache<Box, Box> const*>(cacheBase);
    return cache->manifold.points;
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
