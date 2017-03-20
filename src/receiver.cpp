// A quick-n-dirty proof-of-concept fast C++ implementation.

// requires librtlsdr with software-switchable bias tee support
// (https://github.com/rtlsdrblog/rtl-sdr)
#define LIBRTLSDR_BIAS_TEE_SUPPORT

#include <algorithm>
#include <complex>
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <memory>

#include <cmath>

#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdio.h>

#include <gflags/gflags.h>

// fastcard and fastdet headers
#include <fastdet/corr_detector.h>
#include <fastdet/fastcard_wrappers.h>
#include <fastcard/parse.h>
#include <fastcard/rtlsdr_reader.h>

#include "corx_file_writer.h"
#include "sine_lookup.h"

using namespace std;

#define PI 3.14159265358979323846


//// Fastcard settings
// Input
DEFINE_string(input, "rtlsdr",
              "Input file with samples "
              "('-' for stdin, 'rtlsdr' for librtlsdr)");
DEFINE_string(wisdom, "",
              "Wisfom file to use for FFT calculation"
              "\n[default: don't use wisdom file]");

// Block settings
DEFINE_uint64(block_size, 16384,
              "Length of fixed-sized blocks, which should be a power of two");
DEFINE_uint64(history_size, 4920,
              "The number of samples at the beginning of a block that should "
              "be copied from the end of the previous block [default: 4920]");
DEFINE_uint64(skip, 1,
              "Number of blocks to skip while waiting for the SDR to "
              "stabilize");

// Tuner settings
DEFINE_string(frequency, "1.42G",
              "Frequency to tune to [default: 433.83M]");
DEFINE_string(sample_rate, "2.4M", "Sample rate");
DEFINE_uint64(gain, 0, "Tuner gain");
DEFINE_uint64(device_index, 0, "RTL-SDR device index");

// Carrier detection
DEFINE_string(carrier_window, "0--1",
              "Window of frequency bins used for carrier detection");
DEFINE_string(carrier_threshold, "100c2s",
              "Carrier detection theshold");

//// Fastdet settings
// Beacon detection
DEFINE_string(beacon_threshold, "15s",
              "Beacon correlation detection theshold");
DEFINE_string(template, "template.tpl",
              "Template to correlate with for beacon detection");

//// Corx settings
DEFINE_string(output, "",
              ".corx file to write output to");

DEFINE_double(carrier_ref, -277800,
              "The expected nominal frequency offset of the reference "
              "transmitter's carrier, in Hertz. "
              "Used for estimating the clock error.");
DEFINE_double(beacon_interval, 1.0,
              "Nominal time interval between subsequent beacon "
              "transmissions, in seconds");
DEFINE_uint64(segment_size, 1024,
              "size of small fixed-sized chunks of data used for "
              "cross-correlation");
DEFINE_uint64(beacon_padding, 6000,
              "Number of samples to skip before and after a beacon signal,"
              "i.e. between a beacon signal and the first correlation block");

DEFINE_double(max_tracking_phase_diff, 50,
             "Maximum phase difference between subsequent blocks of data "
             "to be tolerated by the phase lock loop (in degrees)");
DEFINE_double(tracking_diff_coeff, 0.2,
              "Coefficient for derivative term of the carrier tracking loop.");
DEFINE_double(avg_angle_weight, 0.1,
              "Weighting factor for the Exponential Moving Average of the "
              "carrier phase");
DEFINE_double(avg_mag_weight, 0.1,
              "Weighting factor for the Exponential Moving Average of the "
              "carrier magnitude");
DEFINE_double(beacon_carrier_trigger_factor, 0.8,
              "Only search for presence of a beacon signal when the carrier "
              "magnitude is less than the average carrier magnitude times "
              "this factor.");

DEFINE_double(timeout, -1,
              "Timeout, in seconds, after which the program will stop "
              "regardless of the detection state (-1 to disable timeout)");
DEFINE_double(carrier_search_timeout, 5,
              "Maximum time, in seconds, to search for a carrier before "
              "giving up");
DEFINE_double(capture_time, 10.1,
              "time in seconds to capture correlation data after the first "
              "beacon detection");
DEFINE_double(preamp_off_time, 2.0,
              "time in seconds to capture data with the preamp switched off "
              "after --capture_time");
DEFINE_double(preamp_off_transition_time, 0.2,
              "time in seconds to wait after the preamp has been switced off "
              "and before data is being captured with the preamp off");

DEFINE_string(slice, "0--1",
              "Only store the specified slice of the correlation segment FFTs");


bool parse_slice_str(const std::string &slice, int segment_size,
                     int& start, int& len) {
    int stop;
    int r = sscanf(slice.data(), "%d-%d", &start, &stop);

    if (r == 1) {
        stop = start;
    } else if (r != 2) {
        return false;
    }

    if (start < 0) {
        start = segment_size + start;
    }
    if (stop < 0) {
        stop = segment_size + stop;
    }
    if (start >= segment_size || stop >= segment_size) {
        return false;
    }

    len = stop - start + 1;
    return true;
}


// Angles are stored as a value between -0.5 and 0.5 to simplify normalisation
// TODO: convert to class
using DeciAngle = float;
DeciAngle normalize_deciangle(DeciAngle angle) {
    return angle - int(round(angle));
}


// Apply a frequency and phase shift to the given signal.
// src and dest may be the same for inline transformation.
void freq_shift(complex<float> *dest,
                const complex<float> *src,
                size_t len,
                float shift_freq,
                DeciAngle shift_phase) {
    SineLookupNCO nco(2 * (float)PI * shift_phase,
                      2 * (float)PI * shift_freq / (float)len);
    nco.expj_multiply(dest, src, len);
}


// Like freq_shift, but accounts for discontinuity at DC due to FFT
// representation (i.e. zero-frequency at index 0).
void fft_shift(complex<float> *dest,
                const complex<float> *src,
                size_t len,
                float shift_freq,
                DeciAngle shift_phase,
                size_t carrier_offset) {
    SineLookupNCO nco(2 * (float)PI * shift_phase,
                      2 * (float)PI * shift_freq / (float)len);
    size_t pos_len = (len+1)/2 + carrier_offset;  // number of positive frequency components
    nco.expj_multiply(dest, src, pos_len);
    nco.adjust_phase(-2 * (float)PI * shift_freq);
    nco.expj_multiply(dest+pos_len, src+pos_len, len-pos_len);
}


// Calculate the 0 Hz frequency component from a time-domain signal
complex<float> calculate_dc(complex<float> *signal, size_t len) {
    std::complex<float> sum = std::accumulate(signal,
                                              signal + len,
                                              std::complex<float>(0, 0));
    return sum;
}


inline complex<float>* to_complex_star(fcomplex* array) {
    return reinterpret_cast<complex<float>*>(array);
}



class CorxReceiver {
public:
    // Warning: args should outlive CorxReceiver
    //  (TODO: use unique_ptr and move ownership of args)
    CorxReceiver(fargs_t* args,
                  std::string template_file,
                  float corr_thresh_const,
                  float corr_thresh_snr,
                  int corr_size,
                  int slice_start,
                  int slice_len,
                  CFile&& out)
                : args_(args),
                  block_size_(args->block_len),
                  history_size_(args->history_len),
                  nonhistory_size_(args->block_len - args->history_len),
                  synced_fft_calc_(args->block_len, true),
                  corr_size_(corr_size),
                  corr_fft_calc_(corr_size, true),
                  corrected_corr_fft_(corr_size),
                  writer_(std::move(out)) {

        carrier_det_.reset(new CarrierDetector(args_));
        vector<float> template_samples = load_template(template_file);
        corr_det_.reset(new CorrDetector(template_samples,
                                         block_size_,
                                         history_size_,
                                         corr_thresh_const,
                                         corr_thresh_snr));

        synced_signal_ = to_complex_star(synced_fft_calc_.input());
        synced_fft_ = to_complex_star(synced_fft_calc_.output());

        corr_signal_ = to_complex_star(corr_fft_calc_.input());
        corr_fft_ = to_complex_star(corr_fft_calc_.output());

        blocks_skip_ = args_->skip;
        
        num_cycles_ = ((FLAGS_beacon_interval * args_->sdr_sample_rate
                        - 2*FLAGS_beacon_padding) / 
                       corr_size_);

        slice_start_ = max(0, slice_start);
        slice_len_ = (slice_len <= 0) ? corr_size_-slice_start_
                     : min(corr_size_-slice_start_, (size_t)slice_len);

    }

    bool set_bias_tee(bool on) {
        if (strcmp(args_->input_file, "rtlsdr") != 0) {
            return false;
        }

#ifdef LIBRTLSDR_BIAS_TEE_SUPPORT
        rtlsdr_reader_set_bias_tee(carrier_det_->get()->reader, on);
        printf(on ? "Enabled bias tee\n" : "Disabled bias tee\n");
        return true;
#else
        return false;
#endif
    }

    void start() {
        detected_carrier_ = false;
        carrier_det_->start();

        set_bias_tee(true);

        writer_.write_file_header({(uint16_t)slice_start_,
                                   (uint16_t)slice_len_});
    }

    bool next() {
        // TODO: set last_block_ to FIRST_BEACON_SEARCH_TIMEOUT...
        //       ... output "timeout: failed to find first beacon"
        if (last_block_ > 0 && block_idx_ == last_block_) {
            carrier_det_->cancel();
        }
        if (carrier_timeout_block_ > 0
                && block_idx_ == carrier_timeout_block_) {
            printf("Timeout: could not find carrier\n");
            carrier_det_->cancel();
        }
        if (FLAGS_timeout >= 0 &&
                block_idx_ >= FLAGS_timeout * args_->sdr_sample_rate
                                            / nonhistory_size_) {
            printf("Timeout: maximum time exceeded\n");
            carrier_det_->cancel();
        }

        // read the next block without performing carrier detection
        bool success = carrier_det_->next();
        if (!success) {
            if (cycle_ >= 0) {
                writer_.write_cycle_stop();
            }
            carrier_det_->print_stats(stdout);
            return false;
        }

        block_idx_++;

        if (blocks_skip_) {
            --blocks_skip_;
            return true;
        }

        if (preamp_off_block_ > 0 && block_idx_ == preamp_off_block_) {
            next_preamp_off();
        } else {
            next_preamp_on();
        }

        return true;
    }

    void cancel() {
        if (carrier_det_) {
            carrier_det_->cancel();
        }
    }

protected:
    // Capture raw data without performing carrier or beacon detection using
    // the last known carrier frequency for carrier recovery.
    void next_preamp_off() {
        // FIXME: Clean up preamp logic scattered around next() function
        if (block_idx_ == preamp_off_block_) {
            printf("block #%d: Switching off preamp...\n", block_idx_);

            if (cycle_ >= 0) {
                cycle_ = -1;
                writer_.write_cycle_stop();
            }

            set_bias_tee(false);

            blocks_skip_ = (FLAGS_preamp_off_transition_time
                            * args_->sdr_sample_rate / nonhistory_size_);
            printf("Skipping %d blocks...\n", blocks_skip_);
        }

        if (block_idx_ > preamp_off_block_) {
            // continue with last carrier frequency from when the preamp was on
            freq_shift(synced_signal_,
                       to_complex_star(carrier_det_->data().samples),
                       block_size_,
                       -carrier_pos_,
                       sample_phase_);

            if (cycle_ == -1) {
                // FIXME: copy-pasta
                printf("block #%d: Capture noise: next cycle\n", block_idx_);

                soa_ = (nonhistory_size_ * block_idx_);  // FIXME: no padding

                cycle_ = 0;
                num_phase_errors_ = 0;

                const struct timeval ts = carrier_det_->data().block->timestamp;
                CorxBeaconHeader header;
                header.soa = soa_;
                header.timestamp_sec = ts.tv_sec;
                header.timestamp_msec = ts.tv_usec / 1000;
                header.beacon_amplitude = 0;
                header.beacon_noise = 0;
                header.clock_error = clock_error_;  // N/A
                header.carrier_pos = carrier_pos_;
                header.carrier_amplitude = 0;
                header.preamp_on = false;
                writer_.write_cycle_start(header);
            }

            extract_corr_blocks();
        }
    }

    void next_preamp_on() {
        // calculate detected_carrier_, synced_signal_ and dc_angle_.
        recover_carrier();

        sample_phase_ -= (carrier_pos_ *
                          (1.f - (float)history_size_ / block_size_));
        sample_phase_ = normalize_deciangle(sample_phase_);

        avg_dc_angle_ = (dc_angle_ * FLAGS_avg_angle_weight +
                         avg_dc_angle_ * (1-FLAGS_avg_angle_weight));
        avg_dc_ampl_ = (dc_ampl_ * FLAGS_avg_mag_weight +
                         avg_dc_ampl_ * (1-FLAGS_avg_mag_weight));

        if (!detected_carrier_) {
            return;
        }

        if (cycle_ == -1 && dc_ampl_ < avg_dc_ampl_ * FLAGS_beacon_carrier_trigger_factor) {
            printf("DC: %.1f; avg: %.1f\n", dc_ampl_, avg_dc_ampl_);

            // TODO: use change in signal strength to determine whether it is
            // worth looking for a beacon signal

            CorrDetection corr = find_beacon();
            if (corr.detected) {
                clock_error_ = estimate_clock_error();

                printf("beacon #%d: ppm=%.3f\n",
                       beacon_,
                       clock_error_ * 1e6);

                cycle_ = 0;
                num_phase_errors_ = 0;

                if (beacon_ == 0) {
                    float last_block_secs = (FLAGS_capture_time +
                                             FLAGS_preamp_off_time +
                                             FLAGS_preamp_off_transition_time);
                    last_block_ = ((last_block_secs *
                                    args_->sdr_sample_rate) / nonhistory_size_
                                   + block_idx_);
                    printf("block %u: Found first beacon.\n"
                           "We'll stop after %.1f seconds "
                           "(at block block #%u).\n",
                           block_idx_,
                           last_block_secs,
                           last_block_);

                    float preamp_off_secs = (FLAGS_capture_time +
                                             FLAGS_preamp_off_transition_time);
                    preamp_off_block_ = ((preamp_off_secs
                                          * args_->sdr_sample_rate)
                                          / nonhistory_size_
                                         + block_idx_);
                }

                // TODO: move code below to extract_corr_blocks?
                const struct timeval ts = carrier_det_->data().block->timestamp;

                CorxBeaconHeader header;
                header.soa = soa_;
                header.timestamp_sec = ts.tv_sec;
                header.timestamp_msec = ts.tv_usec / 1000;
                header.beacon_amplitude = sqrt(corr.peak_power);
                header.beacon_noise = sqrt(corr.noise_power);  // FIXME
                header.clock_error = clock_error_;
                header.carrier_pos = carrier_pos_;
                header.carrier_amplitude = dc_ampl_;
                header.preamp_on = true;
                writer_.write_cycle_start(header);
            }

        }
        
        if (cycle_ >= 0) {
            extract_corr_blocks();  // pass parameters, e.g. clock_error_
        }
    }

    // Synchronise to / track the carrier.
    // Sets detected_carrier_, synced_signal_ and dc_angle_.
    // Returns false if carrier detection fails.
    bool recover_carrier() {
        if (detected_carrier_) {
            //// Carrier tracking and synchronization
            freq_shift(synced_signal_,
                       to_complex_star(carrier_det_->data().samples),
                       block_size_,
                       -carrier_pos_,
                       sample_phase_);

            prev_dc_angle_ = dc_angle_;

            complex<float> dc = calculate_dc(synced_signal_, block_size_);
            dc_ampl_ = abs(dc);
            dc_angle_ = normalize_deciangle(arg(dc) / (float)PI / 2);

            float angle_diff = normalize_deciangle(dc_angle_ - prev_dc_angle_);

            // printf("block #%u: Carrier phase error: %.0f deg\n",
            //        block_idx_,
            //        angle_diff * 360);

            if (angle_diff * 360 > FLAGS_max_tracking_phase_diff) {
                // tracking loop failed
                detected_carrier_ = false;
                printf("block #%u: Tracking loop failed\n", block_idx_);
            } else {
                // track
                carrier_pos_ += angle_diff * FLAGS_tracking_diff_coeff;
            }
        }

        if (!detected_carrier_) {
            //// Tracking loop failed
            //// Carrier detection and synchronization

            carrier_det_->process();
            const fastcard_data_t& carrier = carrier_det_->data();

            if (carrier.detected) {
                float carrier_offset = CorrDetector::interpolate_parabolic(
                        &carrier.fft_power[carrier.detection.argmax]);
                carrier_pos_ = carrier.detection.argmax + carrier_offset;

                // calculate signed index
                if (carrier_pos_ > block_size_ / 2) {
                    carrier_pos_ -= block_size_;
                }
                
                printf("block #%u: Detected carrier @ %.3f; SNR: %.1f / %.1f\n",
                       block_idx_,
                       carrier_pos_,
                       carrier.detection.max,
                       carrier.detection.noise);

                detected_carrier_ = true;

                // perform freq shift
                freq_shift(synced_signal_,
                           to_complex_star(carrier_det_->data().samples),
                           block_size_,
                           -carrier_pos_,
                           sample_phase_);

                complex<float> dc = calculate_dc(synced_signal_, block_size_);
                dc_ampl_ = abs(dc);
                dc_angle_ = normalize_deciangle(arg(dc) / (float)PI / 2);

            } else {
                printf("block #%u: No carrier detected\n", block_idx_);
            }

            if (!detected_carrier_ && carrier_timeout_block_ == 0) {
                carrier_timeout_block_ = (block_idx_ +
                                          FLAGS_carrier_search_timeout
                                          * nonhistory_size_);
            }
            if (detected_carrier_ && carrier_timeout_block_ > 0) {
                carrier_timeout_block_ = 0;
            }
        }

        return detected_carrier_;
    }

    CorrDetection find_beacon() {
        synced_fft_calc_.execute();
        float signal_energy = 0; // TODO: calculate signal_energy
        CorrDetection corr = corr_det_->detect(synced_fft_, signal_energy);

        if (corr.detected) {
            printf("block #%d: detected beacon (ampl: %.0f)\n",
                   block_idx_,
                   corr.peak_power);
            
            prev_soa_ = soa_;
            soa_ = (nonhistory_size_ * block_idx_
                    + corr.peak_idx + corr.peak_offset);
            float time_step = (soa_ - prev_soa_) / args_->sdr_sample_rate;

            if ((beacon_ > 0) && (time_step > 1.5 * FLAGS_beacon_interval)) {
                // We missed a pulse. Estimate beacon index from sample index.
                printf("Large time step!\n");
                beacon_ += (int)round(time_step);
            } else {
                beacon_++;
            }

            printf("beacon #%d: soa = %.3f; timestep = %.1f\n",
                   beacon_,
                   soa_,
                   time_step);
        }

        return corr;
    }

    void extract_corr_blocks() {
        for (; cycle_ < num_cycles_; ++cycle_) {
            // calculate index of first sample in correlation block
            double start = (soa_
                            + (FLAGS_beacon_padding + cycle_ * corr_size_)
                             * (1 - clock_error_)
                           - block_idx_ * nonhistory_size_);
            size_t start_idx = int(round(start));

            if (start_idx + corr_size_ > block_size_) {
                break;
            }

            memcpy(corr_signal_,
                   synced_signal_+start_idx,
                   corr_size_ * sizeof(complex<float>));

            // calculate FFT
            corr_fft_calc_.execute();

            // correct for complex phase offset and time offset
            fft_shift(corrected_corr_fft_.data(),
                      corr_fft_,
                      corr_size_,
                      start - start_idx,
                      -avg_dc_angle_,
                      -carrier_pos_ * corr_size_ / block_size_);

            DeciAngle error = arg(corrected_corr_fft_.data()[0]) / 2 / PI;
            if (abs(error) > 0.2) {
                num_phase_errors_++;
            }


            // printf("block #%d, beacon #%d, cycle #%d, start %lu: error %.1f deg\n",
            //        block_idx_,
            //        beacon_,
            //        cycle_,
            //        start_idx,
            //        error / 2 / PI * 360);

            // Dump to output file
            int8_t error_fp = error / 0.5 * 127;
            writer_.write_cycle_block(error_fp,
                                      corrected_corr_fft_.data()+slice_start_,
                                      slice_len_);
        }

        if (cycle_ >= num_cycles_) {
            cycle_ = -1;
            writer_.write_cycle_stop();
            if (num_phase_errors_ > 0) {
                printf("beacon %d: %d / %d corr blocks have large phase error\n",
                       beacon_, num_phase_errors_, num_cycles_);
            }
        }
    }

    // Estimate the receiver's clock offset from the position of the carrier
    // frequency. It is assumed that the downconverter and ADC have the same
    // local oscillator (i.e. that they are coherent).
    float estimate_clock_error() {
        return (carrier_pos_ * args_->sdr_sample_rate / block_size_
                - FLAGS_carrier_ref) / args_->sdr_freq;
    }

private:
    fargs_t* args_;

    size_t block_size_;
    size_t history_size_;
    size_t nonhistory_size_;

    // Read input and perform carrier detection using fastcard.
    std::unique_ptr<CarrierDetector> carrier_det_;

    // Perform correlation detection using fastdet.
    std::unique_ptr<CorrDetector> corr_det_;

    // Number of blocks read.
    unsigned block_idx_ = 0;

    // Number of blocks to skip.
    unsigned blocks_skip_;

    // Stop at the given block index (if > 0).
    unsigned last_block_ = 0;

    // Block index at which preamp should be switched off
    unsigned preamp_off_block_ = 0;

    // Carrier timeout block
    unsigned carrier_timeout_block_ = 0;

    // Phase of first sample in block.
    // Used to ensure a continuous phase between subsequent blocks.
    DeciAngle sample_phase_ = 0;

    // Position of carrier in FFT bins.
    float carrier_pos_ = 0;
    bool detected_carrier_ = false;

    // Complex argument at DC frequency
    DeciAngle dc_angle_ = 0;
    DeciAngle prev_dc_angle_;
    float dc_ampl_;

    // Estimated clock error.
    float clock_error_;

    // Running average of dc_angle_.
    float avg_dc_angle_ = 0;

    // Running average of dc_ampl_.
    float avg_dc_ampl_ = 0;

    // Number of beacon pulses received.
    // Beacon pulses are used to relate samples of different receivers to each other.
    // Called "pulse" in Python code.
    int32_t beacon_ = -1;

    // Timestamp of last beacon detection
    // struct timeval beacon_timestamp_;

    // Beacon Sample of Arrival
    double soa_ = 0;
    double prev_soa_ = 0;

    // Correlation block within block of data between subsequent beacons.
    // -1: waiting for first pulse
    int32_t cycle_ = -1;

    // Synced signal, i.e. signal after carrier recovery.
    FFT synced_fft_calc_;
    complex<float>* synced_signal_;
    complex<float>* synced_fft_;

    // Correlation block size.
    size_t corr_size_;

    // Number of correlatino blocks between beacon pulses.
    int num_cycles_;

    // Correlation block buffers.
    FFT corr_fft_calc_;
    complex<float>* corr_signal_;
    complex<float>* corr_fft_;
    AlignedArray<complex<float>> corrected_corr_fft_;

    // Number of correlation blocks with large phase offsets.
    int num_phase_errors_;

    // Output slice
    int slice_start_;
    int slice_len_;

    // Output stream.
    CorxFileWriter writer_;
};


static void parse_fargs(fargs_t* fargs) {
    fargs->input_file = FLAGS_input.c_str();
    fargs->wisdom_file = FLAGS_wisdom.c_str();
    if (!parse_carrier_str(FLAGS_carrier_window.c_str(),
                           &fargs->carrier_freq_min,
                           &fargs->carrier_freq_max)) {
        fprintf(stderr,
                "Invalid value for --carrier_window: %s\n",
                FLAGS_carrier_window.c_str());
        exit(1);
    }
    if (!parse_theshold_str(FLAGS_carrier_threshold.c_str(),
                            &fargs->threshold_const,
                            &fargs->threshold_snr)) {
        fprintf(stderr,
                "Invalid value for --carrier_threshold: %s\n",
                FLAGS_carrier_window.c_str());
        exit(1);
    }
    fargs->block_len = FLAGS_block_size;
    fargs->history_len = FLAGS_history_size;
    fargs->skip = FLAGS_skip;

    fargs->sdr_freq = (uint32_t)parse_si_float(&FLAGS_frequency[0]);
    fargs->sdr_gain = (int)FLAGS_gain * 10; // unit: tenths of a dB
    fargs->sdr_sample_rate = (uint32_t)parse_si_float(&FLAGS_sample_rate[0]);
    fargs->sdr_dev_index = FLAGS_device_index;
}

std::unique_ptr<CorxReceiver> detector;

void signal_handler(int signo) {
    (void)signo;  // unused
    if (detector) {
        detector->cancel();
    }
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    unique_ptr<fargs_t, decltype(free)*> args = {NULL, free};
    args.reset(fargs_new());
    parse_fargs(args.get());

    float arg_corr_thresh_const, arg_corr_thresh_snr ;
    if (!parse_theshold_str(FLAGS_beacon_threshold.c_str(),
                            &arg_corr_thresh_const,
                            &arg_corr_thresh_snr)) {
        fprintf(stderr, "Invalid value for --beacon_threshold: %s\n",
                FLAGS_carrier_window.c_str());
        exit(1);
    }

    int slice_start, slice_len;
    if (!parse_slice_str(FLAGS_slice,
                         FLAGS_segment_size,
                         slice_start,
                         slice_len)) {
        fprintf(stderr, "Invalid value for --slice: %s\n",
                FLAGS_carrier_window.c_str());
        exit(1);
    }

    if (argc > 1) {
        fprintf(stderr, "We do not take positional arguments\n");
        exit(1);
    }

    detector.reset(new CorxReceiver(args.get(),
                                    FLAGS_template,
                                    arg_corr_thresh_const,
                                    arg_corr_thresh_snr,
                                    FLAGS_segment_size,
                                    slice_start,
                                    slice_len,
                                    CFile(FLAGS_output)));

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGPIPE, signal_handler);

    try {
        detector->start();
        while (detector->next()) {
        }
    } catch (FastcardException& e) {
        cerr << e.what() << endl;
        return e.getCode();
    } catch (std::exception& e) {
        cerr << e.what() << endl;
        return -1;
    }

    return 0;
}
