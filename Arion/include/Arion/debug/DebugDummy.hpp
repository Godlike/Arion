/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_DEBUG_DUMMY_HPP
#define ARION_DEBUG_DUMMY_HPP

#include <vector>
#include <functional>
#include <glm/glm.hpp>

namespace arion {
namespace intersection {
namespace gjk {
struct Simplex;
}}}

namespace arion {
class SimpleShape;
}

namespace epona {
struct ConvexHull;
}
namespace arion
{
namespace debug
{

class DebugDummy
{
public:
    using EpaCallback = std::function<void(
        epona::ConvexHull&,
        std::vector<glm::vec3>&,
        intersection::gjk::Simplex&,
        SimpleShape const&,
        SimpleShape const&,
        glm::vec3,
        glm::vec3)>;

    /**
     * @brief EPA debug call function
     *
     * @note This is a dummy method that should be optimized away
     *
     * @param[in] convexHull        current convex hull containing cso
     * @param[in] vertexBuffer      current vertex buffer for the convex hull
     * @param[in] simplex           gjk simplex
     * @param[in] aShape            first shape
     * @param[in] bShape            second shape
     * @param[in] supportVertex     current support vertex
     * @param[in] direction         current search direction
     */
    static void EpaCall(
        epona::ConvexHull& convexHull,
        std::vector<glm::vec3>& vertexBuffer,
        intersection::gjk::Simplex& simplex,
        SimpleShape const& aShape,
        SimpleShape const& bShape,
        glm::vec3 supportVertex,
        glm::vec3 direction
    )
    {
    }

    /**
     * @brief Sets new callback for the EPA debug
     *
     * @note This is a dummy method that should be optimized away
     *
     * @tparam ConvexHullBuffer convex hull buffer type
     *
     * @param callback new callback function
     */
    static void SetEpaCallback(EpaCallback callback)
    {
    }

    /**
     * @brief GJK debug function
     *
     * @note This is a dummy method that should be optimized away
     *
     * @param[in] simplex   current simplex
     * @param[in] end       true if GJK ended
     */
    static void GjkCall(arion::intersection::gjk::Simplex& simplex, bool end)
    {
    }

    /**
     * @brief Sets GJK debug callback
     *
     * @note This is a dummy method that should be optimized away
     *
     * @param callback new GJK debug callback function
     */
    static void SetGjkCallback(
        std::function<void(arion::intersection::gjk::Simplex&, bool)> callback
    )
    {
    }
};

} // debug
} // arion

#endif // ARION_DEBUG_DUMMY_HPP
