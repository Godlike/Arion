/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Arion/Shape.hpp>
#include <iostream>

int main()
{
    arion::Shape shape = {};
    std::cout << shape.centerOfMass.x;
    return 0;
}
