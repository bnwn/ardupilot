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

// LPF base class
template <class T>
class KalmanFilter {
public:
    KalmanFilter();
    KalmanFilter(float p, float r);

    // change parameters
    void set_kalman_param(float p, float r);
    T apply(T sample);
    const T &get() const;
    void reset(T value);

protected:
    float _p;
    float _r;
    T _output;
};

// typedefs for compatibility
typedef KalmanFilter<int>      KalmanFilterInt;
typedef KalmanFilter<long>     KalmanFilterLong;
typedef KalmanFilter<float>    KalmanFilterFloat;
