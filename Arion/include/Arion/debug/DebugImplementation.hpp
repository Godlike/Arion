/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_DEBUG_IMPLEMENTATION_HPP
#define ARION_DEBUG_IMPLEMENTATION_HPP

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

namespace {
template < typename ConvexHullBuffer >
void EpaCallback(
    epona::QuickhullConvexHull<ConvexHullBuffer>&,
    ConvexHullBuffer&,
    arion::intersection::gjk::Simplex&,
    arion::SimpleShape const&,
    arion::SimpleShape const&,
    glm::dvec3,
    glm::dvec3) 
{
}
}

namespace arion
{
namespace debug
{

class DebugImplementation
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
        GetEpaCallback<ConvexHullBuffer>()(
            convexHull, vertexBuffer, simplex, aShape, bShape, supportVertex, direction
        );
    }

    template < typename ConvexHullBuffer >
    static void SetEpaCallback(EpaCallback<ConvexHullBuffer> callback)
    {
        GetEpaCallback<ConvexHullBuffer>() = callback;
    }

    static void GjkCall(arion::intersection::gjk::Simplex& simplex, bool end)
    {
        gjkCallback(simplex, end);
    }

    static void SetGjkCallback(
            std::function<void(arion::intersection::gjk::Simplex&, bool)> callback
        )
    {
        gjkCallback = callback;
    }

private:
    template < typename ConvexHullBuffer >
    static EpaCallback<ConvexHullBuffer>& GetEpaCallback()
    {
        static EpaCallback<ConvexHullBuffer> epaCallback = ::EpaCallback<ConvexHullBuffer>;
        return epaCallback;
    }

    static std::function<void(arion::intersection::gjk::Simplex&, bool)> gjkCallback;
};

} // debug
} // arion

#endif // ARION_DEBUG_IMPLEMENTATION_HPP
