#include <cmath>
#include <cstdint>

#ifndef FUNCTIONS_H
#define FUNCTIONS_H


class Functions {
public:

    // Simple hypotenuse computation function.
    static float hypot_f(float x, float y) { return sqrtf(x*x + y*y); }
    static bool  read_float(const char *line, uint8_t *char_counter, float *float_ptr);
};


#endif // FUNCTIONS_H
