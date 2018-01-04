/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Arion/ConfigurationSpaceObject.hpp>
#include <Epona/FloatingPoint.hpp>
#include <Epona/Analysis.hpp>
#include <array>


using namespace arion;

glm::dvec3 intersection::cso::Support(Sphere const& sphere, glm::dvec3 direction)
{
    glm::dvec3 const vertex = sphere.centerOfMass + direction * sphere.radius;
    return vertex;
}

glm::dvec3 intersection::cso::Support(Box const& box, glm::dvec3 direction)
{
    using namespace intersection;

    std::array<glm::dvec3, 8> boxVertices;
    epona::CalculateBoxVertices(box.iAxis, box.jAxis, box.kAxis, box.centerOfMass, boxVertices.begin());
    for (glm::dvec3 & vertex : boxVertices)
    {
        glm::dvec3 const resultVertex = glm::dmat4(glm::toMat4(box.orientation)) 
            * glm::dvec4{ vertex, 1 } + glm::dvec4{ box.centerOfMass, 1 };
        vertex = glm::dvec3{ resultVertex.x, resultVertex.y, resultVertex.z };
    }

    std::sort(boxVertices.begin(), boxVertices.end(), [direction](glm::dvec3 a, glm::dvec3 b)
    {
        return glm::dot(a, direction) > glm::dot(b, direction);
    });

    glm::dvec3 const maxPoint = *std::max_element(boxVertices.begin(), boxVertices.end(),
        [&direction](glm::dvec3 const& a, glm::dvec3 const& b) -> bool
    {
        return glm::dot(a, direction) < glm::dot(b, direction);
    });

    return maxPoint;
}
