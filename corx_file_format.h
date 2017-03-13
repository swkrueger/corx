#ifndef CORX_FILE_FORMAT_H
#define CORX_FILE_FORMAT_H

#include <stdint.h>
#include <stdbool.h>

struct CorxFileHeader {
    uint16_t slice_start_idx;
    uint16_t slice_size;  // a.k.a. corr block length
} __attribute__((packed));


struct CorxBeaconHeader {
    double soa;
    uint64_t timestamp_sec;
    uint16_t timestamp_msec;
    uint32_t beacon_amplitude;
    uint32_t beacon_noise;
    float clock_error;
    float carrier_pos;
    uint32_t carrier_amplitude;
    bool preamp_on;
} __attribute__((packed));

#endif /* CORX_FILE_FORMAT_H */
