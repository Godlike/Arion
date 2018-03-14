/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_DEBUG_DUMMY_HPP
#define ARION_DEBUG_DUMMY_HPP

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
template < typename T>
class QuickhullConvexHull;
}
namespace arion
{
namespace debug
{

class DebugDummy
{
public:
    template < typename ConvexHullBuffer >
    using EpaCallback = std::function<void(
        epona::QuickhullConvexHull<ConvexHullBuffer>&,
        ConvexHullBuffer&,
        intersection::gjk::Simplex&,
        SimpleShape const&,
        SimpleShape const&,
        glm::dvec3,
        glm::dvec3)>;

    /**
     * @brief EPA debug call function
     *
     * @note This is a dummy method that should be optimized away
     *
     * @tparam ConvexHullBuffer convex hull vertex buffer type
     *
     * @param[in] convexHull        current convex hull containing cso
     * @param[in] vertexBuffer      current vertex buffer for the convex hull
     * @param[in] simplex           gjk simplex
     * @param[in] aShape            first shape
     * @param[in] bShape            second shape
     * @param[in] supportVertex     current support vertex
     * @param[in] direction         current search direction
     */
    template < typename ConvexHullBuffer >
    static void EpaCall(
        epona::QuickhullConvexHull<ConvexHullBuffer>& convexHull,
        ConvexHullBuffer& vertexBuffer,
        intersection::gjk::Simplex& simplex,
        SimpleShape const& aShape,
        SimpleShape const& bShape,
        glm::dvec3 supportVertex,
        glm::dvec3 direction
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
    template < typename ConvexHullBuffer >
    static void SetEpaCallback(EpaCallback<ConvexHullBuffer> callback)
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
