#include <ros/ros.h>

template <typename T>
class Storage {
public:
    Storage() : __index(0) {}
    void add(T);
    T get();
private:
    T __data[10];
    uint8_t __index;
};
