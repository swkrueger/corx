from __future__ import print_function
from __future__ import division

from collections import namedtuple
import struct

import numpy as np
import matplotlib.pyplot as plt

FILE_HEADER_FMT = '<HH'
BEACON_HEADER_FMT = '<dQHIIffI'

FileHeader = namedtuple('FileHeader', 'slice_start, slice_size')
BeaconHeader = namedtuple('BeaconHeader', 'soa, timestamp_sec, timestamp_msec,'
                          'beacon_amplitude, beacon_noise, clock_error,'
                          'carrier_pos, carrier_amplitude')
Block = namedtuple('Block', 'phase_error, data')


def read(stream, size):
    data = stream.read(size)
    assert(len(data) == size)
    return data


def cycle_block_reader(stream, block_len):
    while True:
        header_bytes = read(stream, 1)
        error_fp = struct.unpack('b', header_bytes)[0]
        if error_fp == -128:
            break
        error_deg = error_fp / 127. / 2 * 360  # TODO: use rads instead?
        data = np.fromfile(stream, dtype='complex64', count=block_len, sep='')
        # data = read(stream, block_len * 8)
        yield error_deg, data


def cycle_reader(stream, block_len):
    while True:
        header_len = struct.calcsize(BEACON_HEADER_FMT)
        header_bytes = stream.read(header_len)
        if len(header_bytes) == 0:  # eof
            break
        assert(len(header_bytes) == header_len)
        header = BeaconHeader._make(struct.unpack(BEACON_HEADER_FMT, header_bytes))
        block_reader = cycle_block_reader(stream, block_len)

        yield header, block_reader

        # ensure that block_reader reads everything it should from the stream
        for _ in block_reader:
            pass


def corx_reader(stream):
    # validate signature and header
    signature = read(stream, 4)
    assert(signature == 'CORX')
    version = read(stream, 1)
    assert(version == '\x01')

    file_header_bytes = read(stream, struct.calcsize(FILE_HEADER_FMT))
    file_header = FileHeader._make(struct.unpack(FILE_HEADER_FMT,
                                                 file_header_bytes))
    return file_header, cycle_reader(stream, file_header.slice_size)
    


def _main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('input',
                        type=argparse.FileType('rb'), default='-',
                        help=".corx file ('-' streams from stdin)")
    args = parser.parse_args()
    file_header, cycles = corx_reader(args.input)

    print('Slice start:', file_header.slice_start)
    print('Slice size:', file_header.slice_size)

    for beacon_header, cycle_reader in cycles:
        print(beacon_header)
        for i, (error, block) in enumerate(cycle_reader):
            print("Error in corr block #%d: %.0f deg" % (i, error))


if __name__ == '__main__':
    _main()
