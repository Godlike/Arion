#ifndef ARION_DEBUG_HPP
#define ARION_DEBUG_HPP

#include <Arion/GilbertJohnsonKeerthi.hpp>
#include <Epona/Analysis.hpp>
#include <Epona/QuickhullConvexHull.hpp>
#include <functional>

#define ARION_DEBUG

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

	static void EpaDebugCall(
			epona::QuickhullConvexHull<std::vector<glm::dvec3>>& convexHull,
			std::vector<glm::dvec3>& polytopeVertices,
			intersection::gjk::Simplex& simplex
		)
#ifdef ARION_DEBUG
	{
		Debug& debug = GetInstace();

        if (debug.epaDebugCallback)
		{
            debug.epaDebugCallback(convexHull, polytopeVertices, simplex);
		}
	}
#else
	{
	}
#endif

	std::function<
		void(epona::QuickhullConvexHull<std::vector<glm::dvec3>>&,
		std::vector<glm::dvec3>&,
		intersection::gjk::Simplex&)
	> epaDebugCallback;
};

} // debug
} // arion

#endif // ARION_DEBUG_HPP
