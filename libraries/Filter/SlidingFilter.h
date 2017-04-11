#// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
/// @file	KalmanFilter.h
/// @brief	A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain
#pragma once

#include <AP_Math/AP_Math.h>
#include "FilterClass.h"

#define SAMPLE_MAX 10

// LPF base class
template <class T>
class SlidingFilter {
public:
    SlidingFilter();
    SlidingFilter(uint8_t sum);

    // change parameters
    void set_sliding_param(uint8_t sum);
    T apply(T sample);
    const T &get() const;
    void reset(T value);

protected:
    int _sum;
    T _array[SAMPLE_MAX];
    T _array_sort[SAMPLE_MAX];
    T _output;
    T _max;
    T _min;
    T _second_max;
    T _second_min;
    uint8_t _offset;
};

// typedefs for compatibility
typedef SlidingFilter<int>      SlidingFilterInt;
typedef SlidingFilter<long>     SlidingFilterLong;
typedef SlidingFilter<float>    SlidingFilterFloat;
