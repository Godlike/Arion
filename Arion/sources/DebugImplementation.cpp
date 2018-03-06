#include <Arion/debug/DebugImplementation.hpp>

namespace arion
{

std::function<void(intersection::gjk::Simplex&, bool)>
    debug::DebugImplementation::gjkCallback = [](intersection::gjk::Simplex&, bool) -> void {};

} // namespace arion
