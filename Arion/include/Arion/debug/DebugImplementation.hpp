/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_DEBUG_IMPLEMENTATION_HPP
#define ARION_DEBUG_IMPLEMENTATION_HPP

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

/* Provides callback interface for debugging */
class DebugImplementation
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
     * This method is called from within the EPA function
     * during the calculation. The method is called on every iteration of the EPA.
     * It then proxies all the arguments to the currently
     * set callback functor.
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
        GetEpaCallback()(
            convexHull, vertexBuffer, simplex, aShape, bShape, supportVertex, direction
        );
    }

    /**
     * @brief Sets new callback for the EPA debug
     *
     * @param callback new callback function
     */
    static void SetEpaCallback(EpaCallback callback)
    {
        GetEpaCallback() = callback;
    }

    /**
     * @brief GJK debug function
     *
     * This method is called from within the GJK function
     * during the calculation. The method is called on every iteration of the GJK.
     * It then proxies all the arguments to the currently
     * set callback functor.
     *
     * @param[in] simplex   current simplex
     * @param[in] end       true if GJK ended
     */
    static void GjkCall(arion::intersection::gjk::Simplex& simplex, bool end)
    {
        s_gjkCallback(simplex, end);
    }

    /**
     * @brief Sets GJK debug callback
     *
     * @param callback new GJK debug callback function
     */
    static void SetGjkCallback(
            std::function<void(arion::intersection::gjk::Simplex&, bool)> callback
        )
    {
        s_gjkCallback = callback;
    }

private:
    /**
     * @brief Returns reference to the current EPA debug callback
     *
     * @return current callback
     */
    static EpaCallback& GetEpaCallback()
    {
        static EpaCallback epaCallback = DummyEpaCallback;
        return epaCallback;
    }

    //! Stores GJK callback function
    static std::function<void(arion::intersection::gjk::Simplex&, bool)> s_gjkCallback;

    /**
    * @brief EPA debug call function
    *
    * @note This is an empty function for the callback initialization
    */
    static void DummyEpaCallback(
        epona::ConvexHull&,
        std::vector<glm::vec3>&,
        arion::intersection::gjk::Simplex&,
        arion::SimpleShape const&,
        arion::SimpleShape const&,
        glm::vec3,
        glm::vec3)
    {
    }
};

} // debug
} // arion

#endif // ARION_DEBUG_IMPLEMENTATION_HPP
