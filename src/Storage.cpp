#include <ros/ros.h>
#include <fri2/Storage.h>

template <typename T>
void Storage<T>::add(T t) {
    __data[__index] = t;
    __index = (__index + 1) % 10;
}

template <typename T>
T Storage<T>::get() {
    int8_t spot = __index;
    if(spot - 1 < 0) spot = 10;
    spot--;

    return __data[spot];
}
