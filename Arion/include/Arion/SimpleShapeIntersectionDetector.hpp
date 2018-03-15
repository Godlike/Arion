/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_SSID_HPP
#define ARION_SSID_HPP

#include <Arion/Shape.hpp>
#include <Arion/SimpleShapeIntersection.hpp>
#include <glm/glm.hpp>
#include <memory>

namespace arion
{
namespace intersection
{

/**
 * @brief Provides generic interface for the runtime simple shape intersection detections
 */
class SimpleShapeIntersectionDetector
{
public:
    SimpleShapeIntersectionDetector();
    SimpleShapeIntersectionDetector(SimpleShapeIntersectionDetector const&) = delete;
    SimpleShapeIntersectionDetector& operator=(SimpleShapeIntersectionDetector const&) = delete;

    /**
     * @brief Performs intersection test of two shapes and returns true if shapes are intersecting
     * 
     * @param[in] a input shape
     * @param[in] b input shape
     * 
     * @return @c true if there is intersection, @c false otherwise
     */
    bool CalculateIntersection(SimpleShape const* a, SimpleShape const* b);

    /**
     * @brief Calculates surface contact normal of b shape
     *
     * @attention Must be called strictly after CalculateIntersection function call, otherwise result is undefined
     * 
     * @param[in] a input shape
     * @param[in] b input shape
     * 
     * @return contact normal
     */
    glm::dvec3 CalculateContactNormal(SimpleShape const* a, SimpleShape const* b);

    /**
     * @brief Calculates surface contact points of the given shapes in the world space
     *
     * @attention Must be called strictly after CalculateContactNormal function call, otherwise result is undefined
     * 
     * @param[in] a input shape
     * @param[in] b input shape
     * 
     * @return contact points
     */
    std::pair<glm::dvec3, glm::dvec3> CalculateContactPoints(SimpleShape const* a, SimpleShape const* b);

    /**
     * @brief Calculates penetration depth of two shapes
     *
     * @attention Must be called strictly after CalculateContactNormal function call, otherwise result is undefined
     * 
     * @param[in] a input shape
     * @param[in] b input shape
     * 
     * @return penetration depth
     */
    double CalculatePenetration(SimpleShape const* a, SimpleShape const* b);

private:
    using ShapeTypePair = std::pair<SimpleShape::Type, SimpleShape::Type>;

    /** Hasher for ShapeTypePair objects */
    struct ShapeTypePairHasher
    {
        size_t operator()(ShapeTypePair const& p) const;
    };

    static constexpr uint32_t s_unorderedMapInitialPrimeSize = 11;

    std::unordered_map<ShapeTypePair,
        std::unique_ptr<intersection::CacheBase>,
        ShapeTypePairHasher>
        m_intersectionCaches;

    std::unordered_map<ShapeTypePair,
        bool(*)(SimpleShape const*, SimpleShape const*, intersection::CacheBase*),
        ShapeTypePairHasher>
        m_calculateIntersectionFunctors;

    std::unordered_map<ShapeTypePair,
        glm::dvec3(*)(SimpleShape const*, SimpleShape const*, intersection::CacheBase*),
        ShapeTypePairHasher>
        m_calculateContactNormalFunctors;

    std::unordered_map<ShapeTypePair,
        std::pair<glm::dvec3, glm::dvec3>(*)(SimpleShape const*, SimpleShape const*, intersection::CacheBase*),
        ShapeTypePairHasher>
        m_calculateContactPointsFunctors;

    std::unordered_map<ShapeTypePair,
        double(*)(SimpleShape const*, SimpleShape const*, intersection::CacheBase*),
        ShapeTypePairHasher>
        m_calculatePenetrationFunctors;
};

} // namespace intersection
} // namespace arion

#endif // ARION_SSID_HPP
