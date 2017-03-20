#include "sine_lookup.h"

const float SineLookupFixedPoint::s_sine_table[1 << NBITS][2] = {
  #include "sine_table.h"
};

const float SineLookupFixedPoint::PI = 3.14159265358979323846;
const float SineLookupFixedPoint::TWO_TO_THE_31 = 2147483648.0;
