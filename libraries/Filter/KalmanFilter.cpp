#include "KalmanFilter.h"
/// @file KalmanFilter.cpp
/// @brief  A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain

////////////////////////////////////////////////////////////////////////////////////////////
// KalmanFilter
////////////////////////////////////////////////////////////////////////////////////////////
template <class T>
KalmanFilter<T>::KalmanFilter() : _p(0.0f), _r(0.0f) {
    _output = T();
}
// constructor
template <class T>
KalmanFilter<T>::KalmanFilter(float p, float r) : _p(p), _r(r) {
    _output = T();
}

// change parameters
template <class T>
void KalmanFilter<T>::set_kalman_param(float p, float r) {
    _p = p;
    _r = r;
}

template <class T>
T KalmanFilter<T>::apply(T sample) {
    float gain = 0;

    //Kalman filter function start*******************************
    gain = pow(_p, 2) / (pow(_p, 2) + pow(_r, 2));

    float correction =  (sample - _output) * gain;
    _output = _output + correction;

    _p = pow((double)(1.0f - gain) * pow(_p, 2), 0.5);

    return _output;
}

template <class T>
const T &KalmanFilter<T>::get() const {
    return _output;
}

template <class T>
void KalmanFilter<T>::reset(T value) {
    _output = value;
}

/*
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class KalmanFilter<int>;
template class KalmanFilter<long>;
template class KalmanFilter<float>;

