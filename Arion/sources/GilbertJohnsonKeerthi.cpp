/*
* Copyright (C) 2017-2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Arion/GilbertJohnsonKeerthi.hpp>
#include <Arion/Intersection.hpp>
#include <Epona/Analysis.hpp>

using namespace arion;

namespace
{
/**
*  @brief  Checks if line passes through the origin
*
*  @param  lineStart   line segment start point
*  @param  lineEnd     line segment end point
*
*  @return @c true if line passes though the origin, @c false otherwise
*/
bool LineSegmentContainsOrigin(glm::vec3 const& lineStart, glm::vec3 const& lineEnd)
{
    float const distance = epona::LineSegmentPointDistance(lineStart, lineEnd, glm::vec3{});
    return epona::fp::IsZero(distance);
}

/**
*  @brief  Checks if triangle contains the origin
*
*  @attention  All input points must not lie on the same line at the same time
*
*  @param  a   triangle vertex
*  @param  b   triangle vertex
*  @param  c   triangle vertex
*
*  @return @c true if triangle contains the origin, @c false otherwise
*/
bool TriangleContainsOrigin(glm::vec3 const& a, glm::vec3 const& b, glm::vec3 const& c)
{
    return intersection::IsPointInsideTriangle(a, b, c, glm::vec3{});
}

/**
*  @brief  Checks if tetrahedron contains the origin
*
*  @param  vertices    tetrahedron vertices
*
*  @return @c true if tetrahedron contains the origin, @c false otherwise
*/
bool TetrahedronContainsOrigin(std::array<glm::vec3, 4> const& vertices)
{
    std::array<epona::HyperPlane, 4> const faces{ {
            epona::HyperPlane{ vertices[0], vertices[1], vertices[2], &vertices[3] },
            epona::HyperPlane{ vertices[1], vertices[2], vertices[3], &vertices[0] },
            epona::HyperPlane{ vertices[0], vertices[2], vertices[3], &vertices[1] },
            epona::HyperPlane{ vertices[0], vertices[1], vertices[3], &vertices[2] }
        } };

    return epona::fp::IsGreaterOrEqual(faces[0].GetDistance(), 0.0)
        && epona::fp::IsGreaterOrEqual(faces[1].GetDistance(), 0.0)
        && epona::fp::IsGreaterOrEqual(faces[2].GetDistance(), 0.0)
        && epona::fp::IsGreaterOrEqual(faces[3].GetDistance(), 0.0);
}

/**
*  @brief  Finds nearest simplex from the line segment simplex to the origin
*
*  Given simplex may be reduced down to size 1 as a result of this method.
*
*  @param[in,out]  simplex line segment simplex
*
*  @return new search direction
*/
glm::vec3 NearestSimplexLineSegment(intersection::gjk::Simplex& simplex)
{
    glm::vec3 const AB = simplex.vertices[0] - simplex.vertices[1];
    glm::vec3 const A0 = glm::vec3{ 0, 0, 0 } -simplex.vertices[1];

    if (intersection::IsAngleAcute(AB, A0))
    {
        glm::vec3 const direction = glm::cross(glm::cross(AB, A0), AB);
        simplex.size = 2;
        return direction;
    }

    simplex.supportVertices[0] = simplex.supportVertices[1];
    simplex.vertices[0] = simplex.vertices[1];
    simplex.size = 1;
    return A0;
}

/**
*  @brief  Finds nearest simplex from given triangle simplex to the origin
*
*  Given simplex may be reduced down to size 1 as a result of this method.
*
*  @param[in,out]  simplex triangle simplex
*
*  @return new search direction
*/
glm::vec3 NearestSimplexTriangle(intersection::gjk::Simplex& simplex)
{
    glm::vec3 const A = simplex.vertices[2];
    glm::vec3 const B = simplex.vertices[1];
    glm::vec3 const C = simplex.vertices[0];

    glm::vec3 const AB = B - A;
    glm::vec3 const AC = C - A;

    glm::vec3 const A0 = glm::vec3{ 0, 0, 0 } - A;
    glm::vec3 const ABC = glm::cross(AB, AC);

    glm::vec3 result;

    if (intersection::IsAngleAcute(glm::cross(ABC, AC), -A))
    {
        if (intersection::IsAngleAcute(AC, A0))
        {
            simplex.supportVertices = {{ simplex.supportVertices[0], simplex.supportVertices[2] }};
            simplex.vertices = {{ C, A }};
            simplex.size = 2;

            result = glm::cross(glm::cross(AC, -B), AC);
        }
        else if (intersection::IsAngleAcute(AB, A0))
        {
            simplex.supportVertices = {{ simplex.supportVertices[1], simplex.supportVertices[2] }};
            simplex.vertices = {{ B, A }};
            simplex.size = 2;

            result = glm::cross(glm::cross(AB, -B), AB);
        }
        else
        {
            simplex.supportVertices = {{ simplex.supportVertices[0] }};
            simplex.vertices = {{ A }};
            simplex.size = 1;

            result = -C;
        }
    }
    else if (intersection::IsAngleAcute(glm::cross(AB, ABC), -A))
    {
        if (intersection::IsAngleAcute(AB, A0))
        {
            simplex.supportVertices = {{ simplex.supportVertices[1], simplex.supportVertices[2] }};
            simplex.vertices = {{ B, A }};
            simplex.size = 2;

            result = glm::cross(glm::cross(AB, -B), AB);
        }
        else
        {
            simplex.supportVertices = {{ simplex.supportVertices[2] }};
            simplex.vertices = {{ A }};
            simplex.size = 1;

            result = -C;
        }
    }
    else
    {
        if (intersection::IsAngleAcute(ABC, A0))
        {
            result = ABC;
        }
        else
        {
            result = -ABC;
        }
    }

    return result;
}

/**
*  @brief  Finds nearest simplex from the tetrahedron simplex to the origin
*
*  Given simplex may be reduced down to size 1 as a result of this method.
*
*  @param[in,out]  simplex tetrahedron simplex
*
*  @return new search direction
*
*  @sa NearestSimplexTriangle
*/
glm::vec3 NearestSimplexTetrahedron(intersection::gjk::Simplex& simplex)
{
    uint8_t const simplices[3][3] = {
        {0, 1, 3},
        {1, 2, 3},
        {0, 2, 3},
    };

    float const planeOriginDistances[3] = {
        epona::HyperPlane{ simplex.vertices[0], simplex.vertices[1], simplex.vertices[3], &simplex.vertices[2] }.GetDistance(),
        epona::HyperPlane{ simplex.vertices[1], simplex.vertices[2], simplex.vertices[3], &simplex.vertices[0] }.GetDistance(),
        epona::HyperPlane{ simplex.vertices[0], simplex.vertices[2], simplex.vertices[3], &simplex.vertices[1] }.GetDistance()
    };

    size_t const closestPlaneIndex = std::distance(planeOriginDistances,
        std::min_element(planeOriginDistances, planeOriginDistances + 3));

    simplex.supportVertices = {{
        simplex.supportVertices[simplices[closestPlaneIndex][0]],
        simplex.supportVertices[simplices[closestPlaneIndex][1]],
        simplex.supportVertices[simplices[closestPlaneIndex][2]],
    }};
    simplex.vertices = {{
        simplex.vertices[simplices[closestPlaneIndex][0]],
        simplex.vertices[simplices[closestPlaneIndex][1]],
        simplex.vertices[simplices[closestPlaneIndex][2]],
    }};
    simplex.size = 3;

    return NearestSimplexTriangle(simplex);
}
} // anonymous namespace

bool intersection::gjk::SimplexContainsOrigin(Simplex const& simplex)
{
    if (simplex.size == 2)
    {
        return ::LineSegmentContainsOrigin(simplex.vertices[0], simplex.vertices[1]);
    }

    if (simplex.size == 3)
    {
        return ::TriangleContainsOrigin(simplex.vertices[0], simplex.vertices[1], simplex.vertices[2]);
    }

    return ::TetrahedronContainsOrigin(simplex.vertices);
}

glm::vec3 intersection::gjk::NearestSimplex(Simplex& simplex)
{
    if (2 == simplex.size)
    {
        return ::NearestSimplexLineSegment(simplex);
    }

    if (3 == simplex.size)
    {
        return ::NearestSimplexTriangle(simplex);
    }

    return ::NearestSimplexTetrahedron(simplex);
}

bool intersection::gjk::DoSimplex(gjk::Simplex& simplex, glm::vec3& direction)
{
    //Check if a current simplex contains the origin
    if (SimplexContainsOrigin(simplex))
    {
        //FixMe: Note this check prevents (not always) degenerate polytops from going to QHull, but degrates perfomance, is it worth it?
        bool result = true;
        switch (simplex.size)
        {
            case 3:
                result = !epona::OneLine(simplex.vertices[0], simplex.vertices[1], simplex.vertices[2]);
                break;
            case 4:
                result = !epona::OneLine(simplex.vertices[0], simplex.vertices[1], simplex.vertices[2])
                    && !epona::OneLine(simplex.vertices[0], simplex.vertices[1], simplex.vertices[3])
                    && !epona::OneLine(simplex.vertices[0], simplex.vertices[2], simplex.vertices[3])
                    && !epona::OneLine(simplex.vertices[1], simplex.vertices[2], simplex.vertices[3]);
                break;
            default:
                break;
        }

        if (result)
            return true;
    }

    //Calculate sub simplex nearest to the origin
    direction = glm::normalize(NearestSimplex(simplex));

    return false;
}
