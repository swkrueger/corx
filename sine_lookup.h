// Copied from GnuRadio
// Source:
//  - gnuradio/gnuradio-runtime/include/gnuradio/fxpt.h
//  - gnuradio/gnuradio-runtime/lib/math/fxpt.cc
//  - gnuradio/gnuradio-runtime/lib/math/sine_table.h
//  - gnuradio/gnuradio-runtime/include/gnuradio/fxpt_nco.h

#ifndef SINE_LOOKUP_H
#define SINE_LOOKUP_H

#include <complex>
#include <stdint.h>


/*!
 * \brief fixed point sine and cosine and friends.
 *
 *   fixed pt	radians
 *  ---------	--------
 *   -2**31       -pi
 *        0         0
 *  2**31-1        pi - epsilon
 */
// TODO: CamelCase
class SineLookupFixedPoint {
    static const int WORDBITS = 32;
    static const int NBITS = 10;
    static const float s_sine_table[1 << NBITS][2];
    static const float PI;
    static const float TWO_TO_THE_31;

public:
    static int32_t float_to_fixed(float x) {
        // Fold x into -PI to PI.
        int d = (int)floor(x/2/PI+0.5);
        x -= d*2*PI;
        // And convert to an integer.
        return (int32_t) ((float) x * TWO_TO_THE_31 / PI);
    }

    static float fixed_to_float (int32_t x) {
      return x * (PI / TWO_TO_THE_31);
    }

    static float sin(int32_t x) {
        uint32_t ux = x;
        int index = ux >> (WORDBITS - NBITS);
        return s_sine_table[index][0] * (ux >> 1) + s_sine_table[index][1];
    }

    static float cos(int32_t x) {
        uint32_t ux = x + 0x40000000;
        int index = ux >> (WORDBITS - NBITS);
        return s_sine_table[index][0] * (ux >> 1) + s_sine_table[index][1];
    }

    static std::complex<float> expj(float rads) {
        return std::complex<float>(cos(rads), sin(rads));
    }
};

// TODO: CamelCase
class SineLookup {
public:
    static float sin(float rads) {
        int32_t x = SineLookupFixedPoint::float_to_fixed(rads);
        return SineLookupFixedPoint::sin(x);
    }

    static float cos(float rads) {
        int32_t x = SineLookupFixedPoint::float_to_fixed(rads);
        return SineLookupFixedPoint::cos(x);
    }

    static std::complex<float> expj(float rads) {
        return std::complex<float>(cos(rads), sin(rads));
    }
};

// generate a sin / cos / expj "series", i.e. a NCO
class SineLookupNCO {
public:
    SineLookupNCO() : phase_(0), phase_step_(0) {}

    SineLookupNCO(float phase, float angle_rate) {
        set_phase(phase);
        set_freq(angle_rate);
    }

    // angle is in rads
    void set_phase(float angle) {
        phase_ = SineLookupFixedPoint::float_to_fixed(angle);
    }

    // angle_rate is in radians / step
    void set_freq(float angle_rate) {
        phase_step_ = SineLookupFixedPoint::float_to_fixed(angle_rate);
    }
    
    void step() {
        phase_ += phase_step_;
    }

    // compute cos or sin or the complex exponential for current phase angle
    float cos() const { return SineLookupFixedPoint::cos(phase_); }
    float sin() const { return SineLookupFixedPoint::sin(phase_); }
    std::complex<float> expj() const {
        return std::complex<float>(cos(), sin());
    }

    // compute the complex exponential function for a block of phase angles
    // and multiply it with an input signal.
    // src and dest may be the same for inline transformation.
    void expj_multiply(
            std::complex<float> *dest,
            const std::complex<float> *src,
            size_t len) {

        for (size_t i = 0; i < len; ++i) {
            dest[i] = expj() * src[i];
            step();
        }
    }

private:
    uint32_t phase_;
    int32_t phase_step_;
};

#endif /* SINE_LOOKUP_H */
