#include "SlidingFilter.h"
#include "algorithm"
#include "string.h"
using namespace std;
/// @file SlidingFilter.cpp
/// @brief  A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain

////////////////////////////////////////////////////////////////////////////////////////////
// SlidingFilter
////////////////////////////////////////////////////////////////////////////////////////////
template <class T>
SlidingFilter<T>::SlidingFilter() :
    _sum(0),
    _array{0},
    _array_sort{0},
    _max(0),
    _min(0),
    _second_max(0),
    _second_min(0),
    _offset(0)
{
    _output = T();
}
// constructor
template <class T>
SlidingFilter<T>::SlidingFilter(uint8_t sum) :
    _sum(sum),
    _array{0},
    _array_sort{0},
    _max(0),
    _min(0),
    _second_max(0),
    _second_min(0),
    _offset(0)
{
    _output = T();
}

// change parameters
template <class T>
void SlidingFilter<T>::set_sliding_param(uint8_t sum) {
    _sum = sum;
    _offset = 0;
}

template <class T>
T SlidingFilter<T>::apply(T sample) {
    if (_offset < _sum) {
        _array[_offset++] = sample;
        _output = sample;
    } else {
        for (int i=0; i<(_sum-1); i++) {
            _array_sort[i] = _array[i];
            _array[i] = _array[i + 1];
        }

        _array_sort[_sum - 1] = _array[_sum - 1];
        _array[_sum - 1] = sample;
        sort(_array_sort, _array_sort+_sum);

        if (sample > _array_sort[_sum-2]) {
            _output = _array_sort[_sum-2];
        } else if (sample < _array_sort[1]) {
            _output = _array_sort[1];
        } else {
            _output = sample;
        }

    }

    return _output;
}

template <class T>
const T &SlidingFilter<T>::get() const {
    return _output;
}

template <class T>
void SlidingFilter<T>::reset(T value) {
    _output = value;
    _offset = 0;
}

/*
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class SlidingFilter<int>;
template class SlidingFilter<long>;
template class SlidingFilter<float>;

