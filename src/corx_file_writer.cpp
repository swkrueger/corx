/**
 * Corx File Writer
 *
 * Assumptions of implementation:
 *  - structs are byte-aligned
 *  - ints are little endian
 *  - IEEE floating points
 *
 * TODO: better portability (e.g. do not memcpy structs)
 */

#include <cassert>

#include "corx_file_writer.h"

namespace corx {

void CorxFileWriter::write_file_header(const CorxFileHeader &header) {
    if (is_void()) {
        return;
    }

    // output file signature
    fprintf(out_.file(), "CORX");

    // output file format version
    fputc(version, out_.file());

    // output file header
    fwrite(reinterpret_cast<const char*>(&header),
           sizeof(header),
           1,
           out_.file());

    slice_size_ = header.slice_size;
}

void CorxFileWriter::write_cycle_start(const CorxBeaconHeader &header) {
    // sein sterkte: carrier, beacon
    if (is_void()) {
        return;
    }

    fwrite(reinterpret_cast<const char*>(&header),
           sizeof(header),
           1,
           out_.file());
}

void CorxFileWriter::write_cycle_block(int8_t phase_error,
                       const std::complex<float> *data,
                       uint16_t len) {
    if (is_void()) {
        return;
    }

    assert(len == slice_size_);
    assert(phase_error != -128);
    write_cycle_block_internal(phase_error, data, len);
}

void CorxFileWriter::write_cycle_stop() {
    if (is_void()) {
        return;
    }

    // indicate end of cycle
    write_cycle_block_internal(-128, NULL, 0);
}

void CorxFileWriter::write_cycle_block_internal(int8_t phase_error,
                                const std::complex<float> *data,
                                uint16_t len) {
    fputc(phase_error, out_.file());

    fwrite(data,
           sizeof(std::complex<float>),
           len,
           out_.file());
}

} // namespace corx
