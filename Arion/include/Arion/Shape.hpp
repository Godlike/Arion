/*
* Copyright (C) 2017-2018 by Godlike
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
    explicit Shape(glm::vec3 centerOfMass, glm::quat orientation = glm::angleAxis(0.0f, glm::vec3(0)));

    glm::vec3 centerOfMass;
    glm::quat orientation;
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

    SimpleShape(glm::vec3 centerOfMass, glm::quat orientation, Type type);
};

/** Ray data storage class */
class Ray : public SimpleShape
{
public:
    glm::vec3 direction;

    Ray();
    Ray(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 normal);
};

/** Plane data storage class */
class Plane : public SimpleShape
{
public:
    Plane();
    Plane(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 normal);

    glm::vec3 normal;
};

/** Triangle data storage class */
class Triangle : public SimpleShape
{
public:
    Triangle();
    Triangle(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 a, glm::vec3 b, glm::vec3 c);

    /** Calculates normal from member vertices and writes it to the normal member field */
    void CalculateNormal();

    glm::vec3 aVertex;
    glm::vec3 bVertex;
    glm::vec3 cVertex;
    glm::vec3 normal;
};

/** Sphere data storage class */
class Sphere : public SimpleShape
{
public:
    Sphere();
    Sphere(glm::vec3 centerOfMass, glm::quat orientation, float r);

    float radius;
};

/** Cone data storage class */
class Cone : public SimpleShape
{
public:
    Cone();
    Cone(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 a, float r);

    glm::vec3 apex;
    float radius;
};

/** Capsule data storage class */
class Capsule : public SimpleShape
{
public:
    Capsule();
    Capsule(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 halfHeight, float r);

    glm::vec3 halfHeight;
    float radius;
};

/** Cylinder data storage class */
class Cylinder : public SimpleShape
{
public:
    Cylinder();
    Cylinder(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 halfHeight, float r);

    glm::vec3 halfHeight;
    float radius;
};

/** Box data storage class */
class Box : public SimpleShape
{
public:
    Box();
    Box(glm::vec3 centerOfMass, glm::quat orientation, glm::vec3 i, glm::vec3 j, glm::vec3 k);

    glm::vec3 iAxis;
    glm::vec3 jAxis;
    glm::vec3 kAxis;

    glm::vec3 vertices[8];
};
} // namespace arion
#endif //ARION_SHAPE_HPP
