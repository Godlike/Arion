/*
 * Copyright (C) 2017 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <Arion/Shape.hpp>
#include <Arion/SimpleShapeIntersectionDetector.hpp>

TEST_CASE("Sphere-Sphere Collision", "[collision][sphere]")
{
    using namespace arion;
    using namespace intersection;

    SimpleShapeIntersectionDetector detector;
    Sphere aSphere(glm::dvec3(0, 0, 0), glm::dquat{0, 0, 0, 0}, 2.0);
    Sphere bSphere(glm::dvec3(0, 0, 1), glm::dquat{0, 0, 0, 0}, 2.0);

    REQUIRE(true == detector.CalculateIntersection(&aSphere, &bSphere));
}
