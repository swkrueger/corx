"""Calculate correlation coefficients using two .corx files."""

from __future__ import print_function

import itertools
import numpy as np
import matplotlib.pyplot as plt

from corx_reader import corx_reader


def _main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('corx1', type=argparse.FileType('rb'),
                        help=".corx file to correlate ('-' streams from stdin)")
    parser.add_argument('corx2', type=argparse.FileType('rb'),
                        help=".corx file to correlate ('-' streams from stdin)")
    parser.add_argument('--period', type=float, default=1.0,
                        help='Expected time delay between subsequent beacon pulses.')
    args = parser.parse_args()

    file_header1, reader1 = corx_reader(args.corx1)
    file_header2, reader2 = corx_reader(args.corx2)

    assert(file_header1.slice_start == file_header2.slice_start)
    assert(file_header1.slice_size == file_header2.slice_size)
    print('Slice start:', file_header1.slice_start)
    print('Slice size:', file_header1.slice_size)

    fft_len = file_header1.slice_size
    xcorr_sum = np.zeros(fft_len, dtype='complex64')
    autocorr1_sum = np.zeros(fft_len, dtype='complex64')
    autocorr2_sum = np.zeros(fft_len, dtype='complex64')
    cnt = 0

    timediff = 0
    timediff_thresh = args.period / 4.

    errors1 = []
    errors2 = []

    _, _ = reader1.next() #  FIXME: Temp

    while True:
        # advance all readers if synced
        # only advance the reader that are lagging behind if not synced
        try:
            if abs(timediff) < timediff_thresh or timediff < 0:
                header1, cycles1 = reader1.next()
            if abs(timediff) < timediff_thresh or timediff > 0:
                header2, cycles2 = reader2.next()
        except StopIteration:
            break
        print(header1, header2)

        timestamp1 = header1.timestamp_sec + header1.timestamp_msec / 1000.
        timestamp2 = header2.timestamp_sec + header2.timestamp_msec / 2000.
        timediff = timestamp1 - timestamp2

        if timediff >= timediff_thresh:
            print('Timestamp mismatch:', timediff)

        timediff = 0  # FIXME: Temp

        if abs(timediff) < timediff_thresh:
            for (error1, fft1), (error2, fft2) in itertools.izip(cycles1, cycles2):
                xcorr_sum += fft1 * fft2.conjugate()
                autocorr1_sum += fft1 * fft1.conjugate()
                autocorr2_sum += fft2 * fft2.conjugate()
                cnt += 1

                errors1.append(error1)
                errors2.append(error2)

    xcorr = xcorr_sum / cnt
    autocorr1 = autocorr1_sum / cnt
    autocorr2 = autocorr2_sum / cnt

    # corr
    xcorr_coeffs = xcorr / np.sqrt(np.abs(autocorr1) * np.abs(autocorr2))

    plt.semilogy(np.fft.fftshift(autocorr1.real))
    plt.semilogy(np.fft.fftshift(autocorr2.real))
    plt.figure()
    plt.plot(np.fft.fftshift(xcorr_coeffs.real), label='Real')
    plt.plot(np.fft.fftshift(xcorr_coeffs.imag), label='Imag')
    plt.plot(np.fft.fftshift(np.abs(xcorr_coeffs)), label='Mag')
    plt.legend()
    plt.figure()
    plt.plot(errors1)
    plt.plot(errors2)
    plt.show()


if __name__ == '__main__':
    _main()
