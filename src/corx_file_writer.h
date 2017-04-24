#ifndef CORX_FILE_WRITER_H
#define CORX_FILE_WRITER_H

#include <complex>
#include <fastdet/fastcard_wrappers.h>
#include "corx_file_format.h"

namespace corx {

class CorxFileWriter {
  public:
    CorxFileWriter(CFile&& out)
        : out_(std::move(out)), slice_size_(0) {};

    void write_file_header(const CorxFileHeader &header);
    void write_cycle_start(const CorxBeaconHeader &header);
    void write_cycle_block(int8_t phase_error,
                           const std::complex<float> *data,
                           uint16_t len);
    void write_cycle_stop();
    bool is_void() { return out_.file() == nullptr; }

  private:
    void write_cycle_block_internal(int8_t phase_error,
                                    const std::complex<float> *data,
                                    uint16_t len);

    CFile out_;
    int slice_size_;
    const static uint8_t version = 0x01;
};

} // namespace corx

#endif /* CORX_FILE_WRITER_H */
