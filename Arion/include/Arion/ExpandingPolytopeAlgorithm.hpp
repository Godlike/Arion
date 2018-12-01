/*
* Copyright (C) 2017-2018 by Godlike
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
#include <iterator>

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

    if (1 == simplex.size)
    {
        glm::vec3 const A0 = -glm::normalize(simplex.vertices[0]);
        simplex.supportVertices[1] = gjk::Simplex::SupportVertices{
            cso::Support(aShape,  A0), cso::Support(bShape, -A0),
        };
        simplex.vertices[1] = cso::Support(aShape, bShape, A0);
        ++simplex.size;
    }

    if (2 == simplex.size)
    {
        glm::vec3 const A0 = -simplex.vertices[1];
        uint8_t const n = (epona::fp::IsNotEqual(A0[0], 0.0f) ? 0 : (epona::fp::IsNotEqual(A0[1], 0.0f) ? 1 : 2));
        uint8_t const m = (n < 2 ? (n + 1) : 1);

        glm::vec3 orthogonalDirection;
        orthogonalDirection[n] = A0[m];
        orthogonalDirection[m] = A0[n];
        orthogonalDirection = glm::normalize(orthogonalDirection);

        glm::vec3 const a = cso::Support(aShape, bShape,  orthogonalDirection);
        glm::vec3 const b = cso::Support(aShape, bShape, -orthogonalDirection);
        float const adist = epona::LineSegmentPointDistance(simplex.vertices[0], simplex.vertices[1], a);
        float const bdist = epona::LineSegmentPointDistance(simplex.vertices[0], simplex.vertices[1], b);

        simplex.supportVertices[3] = adist > bdist
            ? gjk::Simplex::SupportVertices{
                cso::Support(aShape,  orthogonalDirection), cso::Support(bShape, -orthogonalDirection) }
            : gjk::Simplex::SupportVertices{
                cso::Support(aShape, -orthogonalDirection), cso::Support(bShape,  orthogonalDirection) };

        simplex.vertices[2] = adist > bdist ? a : b;
        ++simplex.size;
    }

    if (3 == simplex.size)
    {
        epona::HyperPlane const hyperPlane{
            simplex.vertices[0], simplex.vertices[1], simplex.vertices[2]
        };
        glm::vec3 const& normal = hyperPlane.GetNormal();

        glm::vec3 const a = cso::Support(aShape, bShape,  normal);
        glm::vec3 const b = cso::Support(aShape, bShape, -normal);

        bool const aFartherThanB = (hyperPlane.SignedDistance(a) > hyperPlane.SignedDistance(b)
            && std::find(simplex.vertices.begin(), simplex.vertices.begin() + 3, a) == simplex.vertices.begin() + 3);

        simplex.supportVertices[3] = aFartherThanB
            ? gjk::Simplex::SupportVertices{ cso::Support(aShape,  normal), cso::Support(bShape, -normal) }
            : gjk::Simplex::SupportVertices{ cso::Support(aShape, -normal), cso::Support(bShape,  normal) };

        simplex.vertices[3] = simplex.supportVertices[3].aSupportVertex - simplex.supportVertices[3].bSupportVertex;
        ++simplex.size;

#ifndef NDEBUG
        for (uint8_t i = 0; i < 4; ++i)
        {
            for (uint8_t j = 0; j < 4; ++j)
            {
                if (i != j)
                {
                    assert(simplex.vertices[i] != simplex.vertices[j]);
                }
            }
        }
#endif
    }

#ifndef NDEBUG
    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 4; ++j)
        {
            if (i != j)
            {
                assert(simplex.vertices[i] != simplex.vertices[j]);
            }
        }
    }
#endif
    }
} // namespace ::

namespace arion
{
namespace intersection
{
namespace epa
{
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
    //Blow up initial simplex if needed
    ::BlowUpPolytope(simplex, aShape, bShape);

    //Initialize polytope and calculate initial convex hull
    std::vector<glm::vec3> polytopeVertices{ simplex.vertices.begin(), simplex.vertices.end() };
    epona::ConvexHull convexHull = epona::CalculateConvexHull(polytopeVertices);
    std::vector<gjk::Simplex::SupportVertices> supportVertices{
        simplex.supportVertices.begin(), simplex.supportVertices.end()
    };

    //Support information
    size_t faceIndex = SIZE_MAX;
    std::vector<epona::HyperPlane>::const_iterator planeIt;
    float closestFaceDistance;
    glm::vec3 direction;
    bool endEpa;

    //Debug call
    debug::Debug::EpaCall(convexHull, polytopeVertices, simplex, aShape, bShape, {}, {});

    do
    {
        //Get distance and direction to the polytope's face that is nearest to the origin
        planeIt = std::min_element(convexHull.planes.begin(), convexHull.planes.end(),
                [](auto& a, auto& b) -> bool {
                    return a.GetDistance() < b.GetDistance();
        });
        faceIndex = std::distance(convexHull.planes.cbegin(), planeIt);
        direction = planeIt->GetNormal();
        assert(!glm::isnan(direction.x));
        //FixMe: Replace with glm::abs(hyperPlane.GetDistance())
        closestFaceDistance = planeIt->Distance({0, 0, 0});

        //Find next CSO point using new search direction
        glm::vec3 const aSupportVertex = cso::Support(aShape,  direction);
        glm::vec3 const bSupportVertex = cso::Support(bShape, -direction);
        glm::vec3 const supportVertex  = cso::Support(aShape, bShape, direction);
        float const supportVertexDistanceSigned = glm::dot(supportVertex, direction);

        //Check if we can expand polytope in the new direction
        endEpa = !(epona::fp::IsGreater(supportVertexDistanceSigned, closestFaceDistance) && --maxIterations);
        if (!endEpa)
        {
            //Expand polytope if possible
            polytopeVertices.push_back( supportVertex );
            supportVertices.push_back(gjk::Simplex::SupportVertices{ aSupportVertex, bSupportVertex });
            if (!epona::RecalculateConvexHull(convexHull, polytopeVertices))
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
    glm::vec3 const polytopeContactPoint = planeIt->ClosestPoint({ 0, 0, 0 });

    //Calculate borycentric coordinates of the CSO contact point
    glm::vec3 const xyz = epona::CalculateBarycentricCoordinates(
        polytopeContactPoint,
        polytopeVertices[convexHull.faces[faceIndex][0]],
        polytopeVertices[convexHull.faces[faceIndex][1]],
        polytopeVertices[convexHull.faces[faceIndex][2]]
    );

    //Calculate contact points on the primitives
    std::array<gjk::Simplex::SupportVertices, 3> const supportFaceVertices{{
        supportVertices[convexHull.faces[faceIndex][0]],
        supportVertices[convexHull.faces[faceIndex][1]],
        supportVertices[convexHull.faces[faceIndex][2]]
    }};
    glm::vec3 const aContactPointWorld = supportFaceVertices[0].aSupportVertex * xyz.x
        + supportFaceVertices[1].aSupportVertex * xyz.y
        + supportFaceVertices[2].aSupportVertex * xyz.z;
    glm::vec3 const bContactPointWorld = supportFaceVertices[0].bSupportVertex * xyz.x
        + supportFaceVertices[1].bSupportVertex * xyz.y
        + supportFaceVertices[2].bSupportVertex * xyz.z;

    //Debug call
    debug::Debug::EpaCall(convexHull, polytopeVertices, simplex, aShape, bShape, polytopeContactPoint, direction);

    ContactManifold const manifold
    {
        ContactManifold::ContactPoints
        {
            aContactPointWorld,
            bContactPointWorld,
        },
        direction,
        closestFaceDistance
    };

    assert(!glm::isnan(aContactPointWorld.x));
    assert(!glm::isnan(bContactPointWorld.x));

    return manifold;
}
} // namespace epa
} // namespace intersection
} // namespace arion
#endif // ARION_EPA_HPP
