/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Arion/Intersection.hpp>
#include <Epona/HyperPlane.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>

namespace arion
{

bool intersection::CheckRaySphereIntersection(
    glm::vec3 const& raySphere, float sphereRadius, glm::vec3 const& rayDirection
)
{
    float const tCenter = glm::dot(raySphere, rayDirection);
    float const distanceSquare = glm::dot(raySphere, raySphere) - tCenter * tCenter;

    return sphereRadius * sphereRadius - distanceSquare >= 0.0;
}

intersection::RayIntersectionFactors
intersection::CalculateRaySphereIntersectionFactors(
    glm::vec3 const& raySphere, float sphereRadius, glm::vec3 const& rayDirection
)
{
    float const tCenter = glm::dot(raySphere, rayDirection);
    float const distanceSquare = glm::dot(raySphere, raySphere) - tCenter * tCenter;
    float const tDelta = glm::sqrt(sphereRadius * sphereRadius - distanceSquare);

    return {tCenter - tDelta, tCenter + tDelta};
}

intersection::RayIntersectionFactors
intersection::CalculateRayAabbIntersectionFactors(
    glm::vec3 const& boxMinPoint, glm::vec3 const& boxMaxPoint,
    glm::vec3 const& rayDirection, glm::vec3 const& rayOrigin
)
{
    float const t1 = (boxMinPoint.x - rayOrigin.x) / rayDirection.x;
    float const t2 = (boxMaxPoint.x - rayOrigin.x) / rayDirection.x;
    float const t3 = (boxMinPoint.y - rayOrigin.y) / rayDirection.y;
    float const t4 = (boxMaxPoint.y - rayOrigin.y) / rayDirection.y;
    float const t5 = (boxMinPoint.z - rayOrigin.z) / rayDirection.z;
    float const t6 = (boxMaxPoint.z - rayOrigin.z) / rayDirection.z;

    float const tmin = glm::max(glm::max(glm::min(t1, t2), glm::min(t3, t4)), glm::min(t5, t6));
    float const tmax = glm::min(glm::min(glm::max(t1, t2), glm::max(t3, t4)), glm::max(t5, t6));

    return {tmin, tmax};
}

intersection::AabbExtremalVertices
intersection::MakeExtremalVerticesAabb(
    glm::vec3 const& i, glm::vec3 const& j, glm::vec3 const& k
)
{
    glm::mat3 const boxModelMatrixInverse = glm::inverse(
        glm::mat3{glm::normalize(i), glm::normalize(j), glm::normalize(k)}
    );

    glm::mat3 const boxAxesModelSpace{
        boxModelMatrixInverse * i, boxModelMatrixInverse * j, boxModelMatrixInverse * k
    };

    auto const findMaxAbs = [](float a, float b)
    {
        return std::abs(a) < std::abs(b);
    };

    glm::vec3 const maxVertex = glm::vec3{
        glm::abs(*std::max_element(
                glm::value_ptr(boxAxesModelSpace[0]), glm::value_ptr(boxAxesModelSpace[0]) + 3, findMaxAbs)
        ),
        glm::abs(*std::max_element(
                glm::value_ptr(boxAxesModelSpace[1]), glm::value_ptr(boxAxesModelSpace[1]) + 3, findMaxAbs)
        ),
        glm::abs(*std::max_element(
                glm::value_ptr(boxAxesModelSpace[2]), glm::value_ptr(boxAxesModelSpace[2]) + 3, findMaxAbs)
        )
    };

    return {-maxVertex, maxVertex};
}

bool intersection::CheckRayIntersectionFactors(RayIntersectionFactors factors)
{
    // tMax < 0, intersection is behind ray; tMin > tMax, no intesection
    return factors.tMax > 0 && factors.tMin < factors.tMax;
}

bool intersection::IsPointInsideTriangle(
    glm::vec3 const& triangleVertex1, glm::vec3 const& triangleVertex2, glm::vec3 const& triangleVertex3, glm::vec3 const& point
)
{
    float const distance = epona::HyperPlane{triangleVertex1, triangleVertex2, triangleVertex3}.Distance(point);
    if (!epona::fp::IsZero(distance))
    {
        return false;
    }

    return IsSameSide(triangleVertex1, triangleVertex2, triangleVertex3, point)
        && IsSameSide(triangleVertex1, triangleVertex3, triangleVertex2, point)
        && IsSameSide(triangleVertex2, triangleVertex3, triangleVertex1, point);
}

} // namespace arion
