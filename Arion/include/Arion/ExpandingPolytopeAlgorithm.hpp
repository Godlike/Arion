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

namespace
{
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
void BlowUpPolytope(arion::intersection::gjk::Simplex& simplex, ShapeA const& aShape, ShapeB const& bShape)
{
    using namespace arion;
    using namespace intersection;

    if (2 == simplex.size)
    {
        glm::dvec3 const A0 = -simplex.vertices[1];
        uint8_t const n = (epona::fp::IsNotEqual(A0[0], 0.0) ? 0 : (epona::fp::IsNotEqual(A0[1], 0.0) ? 1 : 2));
        uint8_t const m = (n == 0 ? 1 : (n == 1 ? 2 : 1));

        glm::dvec3 orthogonalDirection;
        orthogonalDirection[n] = A0[m];
        orthogonalDirection[m] = A0[n];
        orthogonalDirection = glm::normalize(orthogonalDirection);

        glm::dvec3 const a = cso::Support(aShape, bShape,  orthogonalDirection);
        glm::dvec3 const b = cso::Support(aShape, bShape, -orthogonalDirection);
        double const adist = epona::LineSegmentPointDistance(simplex.vertices[0], simplex.vertices[1], a);
        double const bdist = epona::LineSegmentPointDistance(simplex.vertices[0], simplex.vertices[1], b);

        simplex.supportVertices[3] = epona::fp::IsGreater(adist, bdist)
            ? gjk::Simplex::SupportVertices{
                cso::Support(aShape,  glm::normalize( orthogonalDirection)),
                cso::Support(bShape, -glm::normalize( orthogonalDirection)) }
            : gjk::Simplex::SupportVertices{
                cso::Support(aShape,  glm::normalize(-orthogonalDirection)),
                cso::Support(bShape, -glm::normalize(-orthogonalDirection)) };

        simplex.vertices[2] = epona::fp::IsGreater(adist, bdist) ? a : b;
        ++simplex.size;
    }

    if (3 == simplex.size)
    {
        epona::HyperPlane const hyperPlane{
            simplex.vertices[0], simplex.vertices[1], simplex.vertices[2]
        };

        glm::dvec3 const AB = simplex.vertices[1] - simplex.vertices[2];
        glm::dvec3 const AC = simplex.vertices[0] - simplex.vertices[2];
        glm::dvec3 const ABC = glm::cross(AB, AC);

        glm::dvec3 const a = cso::Support(aShape, bShape, glm::normalize( ABC));
        glm::dvec3 const b = cso::Support(aShape, bShape, glm::normalize(-ABC));

        simplex.supportVertices[3] = epona::fp::IsGreater(hyperPlane.Distance(a), hyperPlane.Distance(b))
            ? gjk::Simplex::SupportVertices{ cso::Support(aShape, glm::normalize( ABC)),
                cso::Support(bShape, -glm::normalize( ABC)) }
            : gjk::Simplex::SupportVertices{ cso::Support(aShape, glm::normalize(-ABC)),
                cso::Support(bShape, -glm::normalize(-ABC)) };

        simplex.vertices[3] = epona::fp::IsGreater(hyperPlane.Distance(a), hyperPlane.Distance(b)) ? a : b;
        ++simplex.size;
    }
}
} // namespace ::

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
 * @brief  Calculates contact manifold using Expanding Polytope Algorithm
 *
 * @tparam ShapeA  SimpleShape or SimpleShape derived object
 * @tparam ShapeB  SimpleShape or SimpleShape derived object
 *
 * @param  aShape           input shape
 * @param  bShape           input shape
 * @param  simplex          initial simplex
 * @param  maxIterations    maximum allowed iterations
 *
 * @return contact manifold
 */
template <typename ShapeA, typename ShapeB>
ContactManifold CalculateContactManifold(
        ShapeA const& aShape, ShapeB const& bShape, gjk::Simplex simplex, uint8_t maxIterations = 100
    )
{
    using ConvexHull = epona::QuickhullConvexHull<std::vector<glm::dvec3>>;

    //Blow up initial simplex if needed
    ::BlowUpPolytope(simplex, aShape, bShape);

    //Initialize polytope and calculate initial convex hull
    std::vector<glm::dvec3> polytopeVertices{ simplex.vertices.begin(), simplex.vertices.end() };
    ConvexHull convexHull(polytopeVertices);
    convexHull.Calculate();
    std::vector<gjk::Simplex::SupportVertices> supportVertices{
        simplex.supportVertices.begin(), simplex.supportVertices.end()
    };

    //Support information
    ConvexHull::Faces::const_iterator face;
    double closestFaceDistance;
    glm::dvec3 direction;
    bool endEpa;

    //Debug call
    debug::Debug::EpaCall(convexHull, polytopeVertices, simplex, aShape, bShape, {}, {});

    do
    {
        //Get distance and direction to the polytope's face that is nearest to the origin
        ConvexHull::Faces const& chFaces = convexHull.GetFaces();
        face = std::min_element(chFaces.begin(), chFaces.end(),
            [](ConvexHull::Face const& a, ConvexHull::Face const& b) -> bool
        {
            return a.GetHyperPlane().GetDistance() < b.GetHyperPlane().GetDistance();
        });
        direction = face->GetHyperPlane().GetNormal();
        closestFaceDistance = face->GetHyperPlane().Distance({0, 0, 0});

        //Find next CSO point using new search direction
        glm::dvec3 const aSupportVertex = cso::Support(aShape,  direction);
        glm::dvec3 const bSupportVertex = cso::Support(bShape, -direction);
        glm::dvec3 const supportVertex  = cso::Support(aShape, bShape, direction);
        double const supportVertexDistance = glm::abs(glm::dot(supportVertex, direction));

        //Check if we can expand polytope in the new direction
        endEpa = !(epona::fp::IsGreater(supportVertexDistance, closestFaceDistance) && --maxIterations);
        if (!endEpa)
        {
            //Expand polytope if possible
            polytopeVertices.push_back( supportVertex );
            supportVertices.push_back(gjk::Simplex::SupportVertices{ aSupportVertex, bSupportVertex });
            if (!convexHull.AddVertex(polytopeVertices.size() - 1))
            {
                polytopeVertices.pop_back();
                supportVertices.pop_back();
            }
        }

        //Debug call
        debug::Debug::EpaCall(convexHull, polytopeVertices, simplex, aShape, bShape, supportVertex, direction);
    }
    //If no expansion is possible and or max iterations is reached end EPA
    while (!endEpa);

    //Find point on the CSO that is nearest to the origin, this is the contact point
    std::array<size_t, 3> const faceIndices = face->GetIndices();
    std::array<glm::dvec3, 3> const faceVertices{{
        polytopeVertices[faceIndices[0]],
        polytopeVertices[faceIndices[1]],
        polytopeVertices[faceIndices[2]],
    }};
    glm::dvec3 const polytopeContactPoint = face->GetHyperPlane().ClosestPoint(glm::dvec3{ 0, 0, 0 });

    //Calculate borycentric coordinates of the CSO contact point
    glm::dvec3 const xyz = epona::CalculateBarycentricCoordinates(
        polytopeContactPoint, faceVertices[0], faceVertices[1], faceVertices[2]
    );

    //Calculate contact points on the primitives
    std::array<gjk::Simplex::SupportVertices, 3> const supportFaceVertices{{
        supportVertices[faceIndices[0]], supportVertices[faceIndices[1]], supportVertices[faceIndices[2]] }};
    glm::dvec3 const aContactPointWorld = supportFaceVertices[0].aSupportVertex * xyz.x
        + supportFaceVertices[1].aSupportVertex * xyz.y
        + supportFaceVertices[2].aSupportVertex * xyz.z;
    glm::dvec3 const bContactPointWorld = supportFaceVertices[0].bSupportVertex * xyz.x
        + supportFaceVertices[1].bSupportVertex * xyz.y
        + supportFaceVertices[2].bSupportVertex * xyz.z;

    //Debug call
    debug::Debug::EpaCall(convexHull, polytopeVertices, simplex, aShape, bShape, polytopeContactPoint, direction);

    return {
        epona::WorldToModelSpace(aContactPointWorld, -aShape.centerOfMass, glm::inverse(glm::toMat3(aShape.orientation))),
        epona::WorldToModelSpace(bContactPointWorld, -bShape.centerOfMass, glm::inverse(glm::toMat3(bShape.orientation))),
        aContactPointWorld,
        bContactPointWorld,
        direction,
        closestFaceDistance
    };
}
} // namespace epa
} // namespace intersection
} // namespace arion
#endif // ARION_EPA_HPP
