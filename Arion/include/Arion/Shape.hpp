/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_SHAPE_HPP
#define ARION_SHAPE_HPP

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace arion
{
/**
 * @brief Base shape class
 */
class Shape
{
public:
    Shape() = default;
    explicit Shape(glm::dvec3 centerOfMass, glm::dquat orientation = glm::angleAxis(0.0, glm::dvec3(0)));

    glm::dvec3 centerOfMass;
    glm::dquat orientation;
};

/**
 * @brief Generic class for representing shapes that could be described as a simple parametric or quadric surface
 */
class SimpleShape : public Shape
{
public:
    enum class Type : uint8_t
    {
        RAY,
        PLANE,
        TRIANGLE,
        SPHERE,
        CONE,
        CYLINDER,
        CAPSULE,
        BOX,
        NONE
    };

    Type type = Type::NONE;

    SimpleShape() = default;

    explicit SimpleShape(Type type)
        : type(type)
    {
    }

    SimpleShape(glm::dvec3 centerOfMass, glm::dquat orientation, Type type);
};

/** Ray data storage class */
class Ray : public SimpleShape
{
public:
    glm::dvec3 direction;

    Ray();
    Ray(glm::dvec3 centerOfMass, glm::dquat orientation, glm::dvec3 normal);
};

/** Plane data storage class */
class Plane : public SimpleShape
{
public:
    Plane();
    Plane(glm::dvec3 centerOfMass, glm::dquat orientation, glm::dvec3 normal);

    glm::dvec3 normal;
};

/** Triangle data storage class */
class Triangle : public SimpleShape
{
public:
    Triangle();
    Triangle(glm::dvec3 centerOfMass, glm::dquat orientation, glm::dvec3 a, glm::dvec3 b, glm::dvec3 c);

    /** Calculates normal from member vertices and writes it to the normal member field */
    void CalculateNormal();

    glm::dvec3 aVertex;
    glm::dvec3 bVertex;
    glm::dvec3 cVertex;
    glm::dvec3 normal;
};

/** Sphere data storage class */
class Sphere : public SimpleShape
{
public:
    Sphere();
    Sphere(glm::dvec3 centerOfMass, glm::dquat orientation, double r);

    double radius;
};

/** Cone data storage class */
class Cone : public SimpleShape
{
public:
    Cone();
    Cone(glm::dvec3 centerOfMass, glm::dquat orientation, glm::dvec3 a, double r);

    glm::dvec3 apex;
    double radius;
};

/** Capsule data storage class */
class Capsule : public SimpleShape
{
public:
    Capsule();
    Capsule(glm::dvec3 centerOfMass, glm::dquat orientation, glm::dvec3 halfHeight, double r);

    glm::dvec3 halfHeight;
    double radius;
};

/** Cylinder data storage class */
class Cylinder : public SimpleShape
{
public:
    Cylinder();
    Cylinder(glm::dvec3 centerOfMass, glm::dquat orientation, glm::dvec3 halfHeight, double r);

    glm::dvec3 halfHeight;
    double radius;
};

/** Box data storage class */
class Box : public SimpleShape
{
public:
    Box();
    Box(glm::dvec3 centerOfMass, glm::dquat orientation, glm::dvec3 i, glm::dvec3 j, glm::dvec3 k);

    glm::dvec3 iAxis;
    glm::dvec3 jAxis;
    glm::dvec3 kAxis;

    glm::dvec3 vertices[8];
};
} // namespace arion
#endif //ARION_SHAPE_HPP
