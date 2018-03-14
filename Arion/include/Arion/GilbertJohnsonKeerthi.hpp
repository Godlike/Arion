/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_GJK_HPP
#define ARION_GJK_HPP

#include <Arion/ConfigurationSpaceObject.hpp>
#include <Arion/Debug.hpp>
#include <Epona/FloatingPoint.hpp>

#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <vector>

namespace arion
{
namespace intersection
{
namespace gjk
{
/** Simplex data container */
struct Simplex
{
    //! Holds a pair of support vertices
    struct SupportVertices
    {
        glm::dvec3 aSupportVertex;
        glm::dvec3 bSupportVertex;
    };

    //! Simplex support vertices. Note that some vertices may be unused
    std::array<SupportVertices, 4> supportVertices;

    //! Simplex vertices. Note that some vertices may be unused
    std::array<glm::dvec3, 4> vertices;

    //! Indicated number of used vertices
    uint8_t size;
};

/**
 * @brief  Checks if simplex contains origin
 *
 * @attention  Must only be called on a simplex of size in range [2; 4]
 *
 * @param  simplex simplex data
 *
 * @return @c true if simplex contains origin @c false otherwise
 */
bool SimplexContainsOrigin(Simplex const& simplex);

/**
 * @brief  Calculates nearest simplex to the origin
 *
 * Presumes that simplex vertices are stored in a way such that the latest
 * added vertex has index @c simplexSize - 1
 *
 * Given simplex may be reduced down to size 1 as a result of this method.
 *
 * @param[in,out]  simplex simplex data
 *
 * @return new search direction
 */
glm::dvec3 NearestSimplex(Simplex& simplex);

/**
 * @brief Checks if simplex contains origin
 *
 * If simplex does not contain origin it is replaced by a new sub simplex
 * that is closest to the origin
 *
 * @param[in,out]  simplex     current simplex
 * @param[in,out]  direction   current search direction
 *
 * @return @c true if simplex contains origin, @c false otherwise
 */
bool DoSimplex(gjk::Simplex& simplex, glm::dvec3& direction);

/**
 * @brief  Calculates a tetrahedron from the CSO such that it contains the origin
 *
 * If simplex contains origin then there is intersection between given shapes
 *
 * @tparam ShapeA  any shape type for which gjk::Support is overloaded
 * @tparam ShapeB  any shape type for which gjk::Support is overloaded
 *
 * @param[in,out]  simplex          initial simplex
 * @param[in]      aShape           reference to the shape object
 * @param[in]      bShape           reference to the shape object
 * @param[in]      direction        initial search direction vector of unit length
 * @param[in]      maxIterations    maximum allowed iterations
 *
 * @return @c true if simplex contains origin, @c false otherwise
 */
template <typename ShapeA, typename ShapeB>
bool CalculateSimplex(
        Simplex& simplex, ShapeA const& aShape, ShapeB const& bShape, glm::dvec3 direction, uint8_t maxIterations = 100
    )
{
    do
    {
        //Add new vertex to the simplex
        simplex.supportVertices[simplex.size++] = {
            cso::Support(aShape,  direction),
            cso::Support(bShape, -direction),
        };
        simplex.vertices[simplex.size - 1] = cso::Support(aShape, bShape, direction);

        //Debug call
        debug::Debug::GjkCall(simplex, false);

        //Calculate if the new vertex is past the origin
        double const scalarDirectionProjection = glm::dot(simplex.vertices[simplex.size - 1], direction);
        if (epona::fp::IsLess(scalarDirectionProjection, 0.0))
        {
            return false;
        }
    } while (!DoSimplex(simplex, direction) && --maxIterations);

    //Debug call
    debug::Debug::GjkCall(simplex, true);

    return true;
}

/**
 * @brief  Checks if two shapes are intersecting using GJK algorithm
 *
 * @tparam ShapeA  any shape type for which gjk::Support is overloaded
 * @tparam ShapeB  any shape type for which gjk::Support is overloaded
 *
 * @param  aShape           reference to the shape object
 * @param  bShape           reference to the shape object
 * @param  maxIterations    maximum allowed iterations
 *
 * @return @c true if there is intersection, @c false otherwise
 *
 * @sa CalculateSimplex, CalculateIntersection(Simplex& simplex, ShapeA const& aShape, ShapeB const& bShape)
 */
template <typename ShapeA, typename ShapeB>
bool CalculateIntersection(ShapeA const& aShape, ShapeB const& bShape, uint8_t maxIterations = 100)
{
    return CalculateIntersection(Simplex(), aShape, bShape, maxIterations);
}

/**
 * @brief  Checks if two shapes are intersecting using GJK algorithm
 *
 * @tparam ShapeA  any shape type for which gjk::Support is overloaded
 * @tparam ShapeB  any shape type for which gjk::Support is overloaded
 *
 * @param[out] simplex          tetrahedron from CSO points containing the origin if one exists
 * @param[in]  aShape           reference to the shape object
 * @param[in]  bShape           reference to the shape object
 * @param[in]  maxIterations    maximum allowed iterations
 *
 * @return @c true if there is intersection, @c false otherwise
 *
 * @sa CalculateSimplex, CalculateIntersection(ShapeA const& aShape, ShapeB const& bShape)
 */
template <typename ShapeA, typename ShapeB>
bool CalculateIntersection(Simplex& simplex, ShapeA const& aShape, ShapeB const& bShape, uint8_t maxIterations = 100)
{
    glm::dvec3 const direction = glm::normalize(glm::dvec3{ 1,1,1 });
    simplex = {
        {{{ cso::Support(aShape, direction), cso::Support(bShape, -direction) }}},
        {{ cso::Support(aShape, bShape, direction) }},
        1
    };

    return CalculateSimplex(simplex, aShape, bShape, glm::normalize(-simplex.vertices[0]), maxIterations);
}
} // namespace gjk
} // namespace intersection
} // namespace arion
#endif // ARION_GJK_HPP
