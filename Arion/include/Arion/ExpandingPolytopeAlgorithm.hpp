/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_EPA_HPP
#define ARION_EPA_HPP

#include <Arion/GilbertJohnsonKeerthi.hpp>
#include <Arion/Debug.hpp>
#include <Epona/Analysis.hpp>
#include <Epona/QuickhullConvexHull.hpp>
#include <glm/glm.hpp>

namespace arion
{
namespace intersection
{
namespace epa
{
/** Stores contact information */
struct ContactManifold
{
    glm::dvec3 aContactPointModelSpace;
    glm::dvec3 bContactPointModelSpace;
    glm::dvec3 aContactPointWorldSpace;
    glm::dvec3 bContactPointWorldSpace;
    glm::dvec3 contactNormal;
    double penetration;
};

/**
 *  @brief  Blows up simplex into tetrahedron
 *
 *  Works only for simplexes of size 2 or 3, otherwise does nothing
 *
 *  @tparam ShapeA  SimpleShape or SimpleShape derived object
 *  @tparam ShapeB  SimpleShape or SimpleShape derived object
 *
 *  @param[in,out]  simplex initial simplex
 *  @param[in]      aShape  input shape
 *  @param[in]      bShape  input shape
 */
template <typename ShapeA, typename ShapeB>
void BlowUpPolytope(gjk::Simplex& simplex, ShapeA const& aShape, ShapeB const& bShape)
{
    if (simplex.size == 2)
    {
        glm::dvec3 const A0 = -simplex.vertices[1];
        uint8_t const n = (epona::fp::IsNotEqual(A0[0], 0.0) ? 0 : (epona::fp::IsNotEqual(A0[1], 0.0) ? 1 : 2));
        uint8_t const m = (n == 0 ? 1 : (n == 1 ? 2 : 1));

        glm::dvec3 orthogonalDirection;
        orthogonalDirection[n] = A0[m];
        orthogonalDirection[m] = A0[n];
        orthogonalDirection = glm::normalize(orthogonalDirection);

        glm::dvec3 const a = cso::Support(aShape, bShape, orthogonalDirection);
        glm::dvec3 const b = cso::Support(aShape, bShape, -orthogonalDirection);
        double const adist = epona::LineSegmentPointDistance(simplex.vertices[0], simplex.vertices[1], a);
        double const bdist = epona::LineSegmentPointDistance(simplex.vertices[0], simplex.vertices[1], b);

        simplex.vertices[2] = epona::fp::IsGreater(adist, bdist) ? a : b;
        ++simplex.size;
    }

    if (simplex.size == 3)
    {
        epona::HyperPlane const hyperPlane{
            simplex.vertices[0], simplex.vertices[1], simplex.vertices[2]
        };

        glm::dvec3 const AB = simplex.vertices[1] - simplex.vertices[2];
        glm::dvec3 const AC = simplex.vertices[0] - simplex.vertices[2];
        glm::dvec3 const ABC = glm::cross(AB, AC);

        glm::dvec3 const a = cso::Support(aShape, bShape, glm::normalize(ABC));
        glm::dvec3 const b = cso::Support(aShape, bShape, glm::normalize(-ABC));

        simplex.vertices[3] = epona::fp::IsGreater(hyperPlane.Distance(a), hyperPlane.Distance(b)) ? a : b;
        ++simplex.size;
    }
}

/**
 *  @brief  Calculates contact manifold using Expanding Polytope Algorithm
 *
 *  @tparam ShapeA  SimpleShape or SimpleShape derived object
 *  @tparam ShapeB  SimpleShape or SimpleShape derived object
 *
 *  @param  aShape  input shape
 *  @param  bShape  input shape
 *  @param  simplex initial simplex
 *
 *  @return contact manifold
 */
template <typename ShapeA, typename ShapeB>
ContactManifold CalculateContactManifold(ShapeA const& aShape, ShapeB const& bShape, gjk::Simplex simplex)
{
    using ConvexHull = epona::QuickhullConvexHull<std::vector<glm::dvec3>>;

    //Blow up initial simplex if needed
    if (simplex.size < 4)
    {
        BlowUpPolytope(simplex, aShape, bShape);
    }

    //Initialize polytope and calculate initial convex hull
    std::vector<glm::dvec3> polytopeVertices{ simplex.vertices.begin(), simplex.vertices.end() };
    ConvexHull convexHull(polytopeVertices);
    convexHull.Calculate();

    //Support information
    glm::dvec3 direction;
    double supportVertexDistance;
    double distance;

    do
    {
        //Get polytope's faces and sort them by the distance to the origin
        ConvexHull::Faces chFaces = convexHull.GetFaces();
        chFaces.sort([](ConvexHull::Face& a, ConvexHull::Face& b) -> bool
        {
            return a.GetHyperPlane().GetDistance() < b.GetHyperPlane().GetDistance();
        });

        //Get distance and direction to the polytope's face that is nearest to the origin
        epona::HyperPlane const& hp = chFaces.front().GetHyperPlane();
        direction = hp.GetNormal();
        distance = glm::abs(hp.GetDistance());

        //Find CSO point using new search direction
        glm::dvec3 const supportVertex = cso::Support(aShape, bShape, direction);
        supportVertexDistance = glm::abs(glm::dot(supportVertex, direction));

        //If it's a face from the edge, end EPA
        if (epona::fp::IsGreater(supportVertexDistance, distance))
        {
            //Expand polytope if possible
            polytopeVertices.push_back(supportVertex);
            if (!convexHull.AddVertex(polytopeVertices.size() - 1))
            {
                polytopeVertices.pop_back();
            }
        }

        //Debug call
        debug::Debug::EpaCall(convexHull, polytopeVertices, simplex);
    } while (epona::fp::IsGreater(supportVertexDistance, distance));

    return {
        cso::Support(aShape, direction) - aShape.centerOfMass,
        cso::Support(bShape, -direction) - bShape.centerOfMass,
        cso::Support(aShape, direction),
        cso::Support(bShape, -direction),
        direction,
        distance
    };
}
} // namespace epa
} // namespace intersection
} // namespace arion
#endif // ARION_EPA_HPP
