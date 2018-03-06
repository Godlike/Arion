#ifndef ARION_DEBUG_HPP
#define ARION_DEBUG_HPP

#include <functional>
#include <vector>
#include <glm/glm.hpp>

#define ARION_DEBUG

namespace arion {
namespace intersection {
namespace gjk {
    struct Simplex;
} // namespace gjk
} // namespace intresection
} // namespace arion

namespace epona {
    template <typename T>
    class QuickhullConvexHull;
} // namespace epona

namespace arion
{
namespace debug
{

class Debug
{
public:
	Debug() = default;
	Debug& operator==(Debug const&) = delete;
	Debug(Debug&) = delete;
	Debug& operator==(Debug&&) = delete;
	Debug(Debug&&) = delete;

	static Debug& GetInstace()
	{
		static Debug debug;
		return debug;
	}

	static void EpaCall(
			epona::QuickhullConvexHull<std::vector<glm::dvec3>>& convexHull,
			std::vector<glm::dvec3>& polytopeVertices,
            arion::intersection::gjk::Simplex& simplex,
            glm::dvec3 supportVertex,
            glm::dvec3 direction
		)
#ifdef ARION_DEBUG
	{
		Debug& debug = GetInstace();

        if (debug.epaCallback)
		{
            debug.epaCallback(convexHull, polytopeVertices, simplex, supportVertex, direction);
		}
	}
#else
	{
	}
#endif

    static void GjkCall(arion::intersection::gjk::Simplex& simplex)
#ifdef ARION_DEBUG
    {
        Debug& debug = GetInstace();

        if (debug.gjkCallback)
        {
            debug.gjkCallback(simplex);
        }
    }
#else
    {
    }
#endif

	std::function<
		void(epona::QuickhullConvexHull<std::vector<glm::dvec3>>&,
		std::vector<glm::dvec3>&,
		intersection::gjk::Simplex&,
        glm::dvec3,
        glm::dvec3)
	> epaCallback;

    std::function<void(arion::intersection::gjk::Simplex&)> gjkCallback;
};

} // debug
} // arion

#endif // ARION_DEBUG_HPP
