#pragma once

#include <cmath>

/**
 * @brief Converts a degree to a radian
 * 
 * @param deg The degree
 * @return The radian equivalent of the degree
 */
double degToRad(double deg);
/**
 * @brief Converts a radian to a degree
 * 
 * @param rad The radian
 * @return The degree equivalent of the radian
 */
double radToDeg(double rad);

/**
 * @brief Returns the sign of the given number
 * 
 * @param n The number
 * @return 1 for positive, -1 for negative, 0 if n is 0
 */
int sign(double n);
