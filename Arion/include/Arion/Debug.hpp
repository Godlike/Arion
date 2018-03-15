/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_DEBUG_HPP
#define ARION_DEBUG_HPP

#ifdef ARION_DEBUG
#include <Arion/debug/DebugImplementation.hpp>
#else
#include <Arion/debug/DebugDummy.hpp>
#endif

namespace arion
{
namespace debug
{

#ifdef ARION_DEBUG
    using Debug = DebugImplementation;
#else
    using Debug = DebugDummy;
#endif

} // namespace debug
} // namespace arion

#endif // ARION_DEBUG_HPP
