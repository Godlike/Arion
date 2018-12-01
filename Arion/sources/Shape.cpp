/*
* Copyright (C) 2017-2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Arion/Shape.hpp>

using namespace arion;

Shape::Shape(glm::vec3 centerOfMass, glm::quat orientation)
    : centerOfMass(centerOfMass)
    , orientation(orientation)
{
}

SimpleShape::SimpleShape(glm::vec3 centerOfMass, glm::quat orientation, SimpleShape::Type type)
    : Shape(centerOfMass, orientation)
    , type(type)
{
}

Ray::Ray()
    : SimpleShape(SimpleShape::Type::RAY)
{
}

Ray::Ray(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 normal)
    : SimpleShape(centerOfMass, orientation, SimpleShape::Type::RAY)
    , direction(normal)
{
}

Plane::Plane()
    : SimpleShape(SimpleShape::Type::PLANE)
{
}

Plane::Plane(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 normal)
    : SimpleShape(centerOfMass, orientation, SimpleShape::Type::PLANE)
    , normal(normal)
{
}

Triangle::Triangle()
    : SimpleShape(SimpleShape::Type::TRIANGLE)
{
}

Triangle::Triangle(
    glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 a, glm::vec3 b, glm::vec3 c
)
    : SimpleShape(centerOfMass, orientation, SimpleShape::Type::TRIANGLE)
    , aVertex(a)
    , bVertex(b)
    , cVertex(c)
{
    CalculateNormal();
}

void Triangle::CalculateNormal()
{
    normal = glm::cross(bVertex - aVertex, cVertex - aVertex);
}

Sphere::Sphere()
    : SimpleShape(SimpleShape::Type::SPHERE)
    , radius()
{
}

Sphere::Sphere(glm::vec3 centerOfMass, glm::quat orientation, float r)
    : SimpleShape(centerOfMass, orientation, SimpleShape::Type::SPHERE)
    , radius(r)
{
}

Cone::Cone()
    : SimpleShape(SimpleShape::Type::CONE)
    , radius()
{
}

Cone::Cone(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 a, float r)
    : SimpleShape(centerOfMass, orientation, SimpleShape::Type::CONE)
    , apex(a)
    , radius(r)
{
}

Capsule::Capsule()
    : SimpleShape(SimpleShape::Type::CAPSULE)
    , radius()
{
}

Capsule::Capsule(
    glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 halfHeight, float r
)
    : SimpleShape(centerOfMass, orientation, SimpleShape::Type::CAPSULE)
    , halfHeight(halfHeight)
    , radius(r)
{
}

Cylinder::Cylinder()
    : SimpleShape(SimpleShape::Type::CYLINDER)
    , radius()
{
}

Cylinder::Cylinder(
    glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 halfHeight, float r
)
    : SimpleShape(centerOfMass, orientation, SimpleShape::Type::CYLINDER)
    , halfHeight(halfHeight)
    , radius(r)
{
}

Box::Box()
    : SimpleShape(SimpleShape::Type::BOX)
{
}

Box::Box(
    glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 i, glm::vec3 j, glm::vec3 k
)
    : SimpleShape(centerOfMass, orientation, SimpleShape::Type::BOX)
    , iAxis(i)
    , jAxis(j)
    , kAxis(k)
{
    glm::vec3 const jPlusK = jAxis + kAxis;
    glm::vec3 const jMinusK = jAxis - kAxis;

    vertices[0] = (iAxis + jPlusK);
    vertices[1] = (iAxis - jPlusK);
    vertices[2] = (jAxis - iAxis + kAxis);
    vertices[3] = (-iAxis - jPlusK);
    vertices[4] = (iAxis + jMinusK);
    vertices[5] = (iAxis - jMinusK);
    vertices[6] = (jAxis - iAxis - kAxis);
    vertices[7] = (-iAxis - jMinusK);
}
