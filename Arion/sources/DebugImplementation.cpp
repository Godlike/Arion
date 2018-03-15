/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Arion/debug/DebugImplementation.hpp>

namespace arion
{

std::function<void(intersection::gjk::Simplex&, bool)>
    debug::DebugImplementation::s_gjkCallback = [](intersection::gjk::Simplex&, bool) -> void {};

} // namespace arion
