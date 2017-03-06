#!/usr/bin/env python

"""Calculate correlation coefficients from data in two .corx files."""

from __future__ import print_function

import itertools
import sys
import numpy as np
import matplotlib.pyplot as plt

from corx_reader import corx_reader


def correlate(corx1, corx2, period):
    file_header1, reader1 = corx_reader(corx1)
    file_header2, reader2 = corx_reader(corx2)

    assert(file_header1.slice_start == file_header2.slice_start)
    assert(file_header1.slice_size == file_header2.slice_size)
    print('Slice start:', file_header1.slice_start)
    print('Slice size:', file_header1.slice_size)

    fft_len = file_header1.slice_size
    xcorr_sum = np.zeros(fft_len, dtype='complex64')
    autocorr1_sum = np.zeros(fft_len, dtype='complex64')
    autocorr2_sum = np.zeros(fft_len, dtype='complex64')
    cnt = 0

    autocorr1_off_sum = np.zeros(fft_len, dtype='complex64')
    autocorr2_off_sum = np.zeros(fft_len, dtype='complex64')
    autocorr1_off_cnt = 0
    autocorr2_off_cnt = 0

    timediff = 0
    timediff_thresh = period / 4.

    errors1 = []
    errors2 = []

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

        print(header1)
        print(header2)
        print()

        timestamp1 = header1.timestamp_sec + header1.timestamp_msec / 1000.
        timestamp2 = header2.timestamp_sec + header2.timestamp_msec / 1000.
        timediff = timestamp1 - timestamp2

        if timediff >= timediff_thresh:
	    print('Skip pair due to timestamp mismatch of {:.3f} s'.format(timediff))

        if not header1.preamp_on or not header2.preamp_on:
            if not header1.preamp_on:
                for _, fft1 in cycles1:
                    autocorr1_off_sum += fft1 * fft1.conjugate()
                    autocorr1_off_cnt += 1
            if not header2.preamp_on:
                for _, fft2 in cycles2:
                    autocorr2_off_sum += fft2 * fft2.conjugate()
                    autocorr2_off_cnt += 2
            timediff = 0  # force both readers to be moved forward

        elif abs(timediff) < timediff_thresh:
            for (error1, fft1), (error2, fft2) in itertools.izip(cycles1, cycles2):
                xcorr_sum += fft1 * fft2.conjugate()
                autocorr1_sum += fft1 * fft1.conjugate()
                autocorr2_sum += fft2 * fft2.conjugate()
                cnt += 1

                errors1.append(error1)
                errors2.append(error2)

    if cnt == 0:
        print("No beacon matches :(")
        return None

    xcorr = xcorr_sum / cnt
    autocorr1 = autocorr1_sum / cnt
    autocorr2 = autocorr2_sum / cnt

    if autocorr1_off_cnt > 0:
        autocorr1_off = autocorr1_off_sum / autocorr1_off_cnt
    else:
        autocorr1_off = None
        print("Warning: No data with preamp off in first .corx file.")

    if autocorr2_off_cnt > 0:
        autocorr2_off = autocorr2_off_sum / autocorr2_off_cnt
    else:
        autocorr2_off = None
        print("Warning: No data with preamp off in second .corx file.")

    return (xcorr, autocorr1, autocorr2, cnt,
            autocorr1_off, autocorr1_off_cnt,
            autocorr2_off, autocorr2_off_cnt,
            errors1, errors2)


def plot_coeffs(xcorr_coeffs):
    plt.figure()
    plt.plot(np.fft.fftshift(xcorr_coeffs.real), label='Real')
    plt.plot(np.fft.fftshift(xcorr_coeffs.imag), label='Imag')
    plt.plot(np.fft.fftshift(np.abs(xcorr_coeffs)), label='Mag')
    plt.title('Cross-correlation coefficients')
    plt.legend()


def plot_autocorr(autocorr1, autocorr2, autocorr1_off, autocorr2_off):
    plt.figure()
    plt.semilogy(np.fft.fftshift(autocorr1.real), label='corx1')
    plt.semilogy(np.fft.fftshift(autocorr2.real), label='corx2')
    if autocorr1_off:
        plt.semilogy(np.fft.fftshift(autocorr1_off.real), label='corx1 (off)')
    if autocorr2_off:
        plt.semilogy(np.fft.fftshift(autocorr2_off.real), label='corx2 (off)')
    plt.title('Autocorrelation (real)')
    plt.legend()


def _main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('corx1', type=argparse.FileType('rb'), nargs='?',
                        help=".corx file to correlate ('-' streams from stdin)")
    parser.add_argument('corx2', type=argparse.FileType('rb'), nargs='?',
                        help=".corx file to correlate ('-' streams from stdin)")
    parser.add_argument('-o', '--output', type=argparse.FileType('wb'),
                        help=".npz output file")
    parser.add_argument('-p', '--plot', const=True, action='store', nargs='?',
                        default=None, type=argparse.FileType('rb'),
                        help="Plot autocorr and xcorr coefficients")
    parser.add_argument('--period', type=float, default=1.0,
                        help='Expected time delay between subsequent beacon pulses.')
    args = parser.parse_args()

    if args.plot and args.plot is not True:
        print('Plot xcorr from file...')
        npzfile = np.load(args.plot)
        print("Xcorr was calculated from {} blocks.".format(npzfile['cnt']))
        plot_coeffs(npzfile['coeffs'])
        plot_autocorr(npzfile['autocorr1'], npzfile['autocorr2'],
                      npzfile['autocorr1_off'], npzfile['autocorr2_off'])
        plt.show()
        sys.exit(0)

    elif not args.corx1 or not args.corx2:
        print("too few arguments: no .corx files specified.", file=sys.stderr)
        sys.exit(2)

    ret = correlate(args.corx1, args.corx2, args.period)
    if ret is not None:
        xcorr, autocorr1, autocorr2, cnt, \
                autocorr1_off, autocorr1_off_cnt, \
                autocorr2_off, autocorr2_off_cnt, \
                errors1, errors2 = ret
        xcorr_coeffs = xcorr / np.sqrt(np.abs(autocorr1) * np.abs(autocorr2))

        print("Calculated xcorr from {} blocks.".format(cnt))
        print("Number of autocorr off blocks: {}, {}".format(
            autocorr1_off_cnt, autocorr2_off_cnt))

        # save
        if args.output:
            np.savez(args.output,
                     coeffs=xcorr_coeffs,
                     autocorr1=autocorr1,
                     autocorr2=autocorr2,
                     autocorr1_off=autocorr1_off,
                     autocorr1_off_cnt=autocorr1_off_cnt,
                     autocorr2_off=autocorr2_off,
                     autocorr2_off_cnt=autocorr2_off_cnt,
                     cnt=cnt)

        # plot
        if args.plot:
            plot_coeffs(xcorr_coeffs)
            plot_autocorr(autocorr1,
                          autocorr2,
                          autocorr1_off,
                          autocorr2_off)

            plt.figure()
            plt.plot(errors1, label='corx1')
            plt.plot(errors2, label='corx2')
            plt.title('Phase errors')

            plt.show()


if __name__ == '__main__':
    _main()
