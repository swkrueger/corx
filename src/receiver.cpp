// A quick-n-dirty proof-of-concept fast C++ implementation.

// requires librtlsdr with software-switchable bias tee support
// (https://github.com/rtlsdrblog/rtl-sdr)
#define LIBRTLSDR_BIAS_TEE_SUPPORT

#include <algorithm>
#include <complex>
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <stdexcept>
#include <memory>

#include <cmath>

#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdio.h>

#include <sys/poll.h>
#include <unistd.h>

#include <gflags/gflags.h>

// fastcard and fastdet headers
#include <fastdet/corr_detector.h>
#include <fastdet/fastcard_wrappers.h>
#include <fastcard/parse.h>
#include <fastcard/rtlsdr_reader.h>

#include "corx_file_writer.h"
#include "sine_lookup.h"
#include "receiver.h"

using namespace std;

#define PI 3.14159265358979323846

// printf with block ID prepended
#define BPRINTF(fmt, ...) printf("[#%u] " fmt, block_idx_, ##__VA_ARGS__)

namespace corx {
// TODO: Writer should also be in corx namespace

//// Fastcard settings
// Input
DEFINE_string(input, "rtlsdr",
              "Input file with samples "
              "('-' for stdin, 'rtlsdr' for librtlsdr)");
DEFINE_string(wisdom, "",
              "Wisfom file to use for FFT calculation"
              "\n[default: don't use wisdom file]");
DEFINE_string(debug, "",
              "File for temporary debugging output");

// Block settings
DEFINE_uint64(block_size, 16384,
              "Length of fixed-sized blocks, which should be a power of two");
DEFINE_uint64(history_size, 4920,
              "The number of samples at the beginning of a block that should "
              "be copied from the end of the previous block [default: 4920]");

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
DEFINE_double(beacon_carrier_trigger_factor, 1.1,
              "Only search for presence of a beacon signal when the carrier "
              "magnitude is less than the average carrier magnitude times "
              "this factor.");

DEFINE_double(timeout, 0,
              "Maximum amount of time the receiver is allowed to spend in "
              "the active state regardless of detection state (0 to disable).");
DEFINE_double(carrier_search_timeout, 5,
              "Maximum time, in seconds, to search for a carrier before "
              "giving up");
DEFINE_double(capture_time, 10.5,
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


bool isInactiveState(ReceiverState state) {
    return (state == ReceiverState::STOPPED ||
            state == ReceiverState::STANDBY);
}


// TODO: move to header (and user PIMPL?)
class Receiver {
public:
    // Warning: args should outlive Receiver
    //  (TODO: use unique_ptr and move ownership of args)
    Receiver(fargs_t* args,  // TODO: calculate within class from flags
             float corr_thresh_const,
             float corr_thresh_snr,
             int slice_start,
             int slice_len,
             CFile&& out)
           : args_(args),
             synced_fft_calc_(args->block_len, true),
             corr_fft_calc_(FLAGS_segment_size, true),
             corrected_corr_fft_(FLAGS_segment_size),
             block_size_(args->block_len),
             history_size_(args->history_len),
             nonhistory_size_(args->block_len - args->history_len),
             corr_size_(FLAGS_segment_size),
             debug_(FLAGS_debug) {

        state_ = ReceiverState::STOPPED;
        mode_ = ReceiverMode::STOP;
        last_inactive_mode_ = ReceiverMode::STOP;

        carrier_det_.reset(new CarrierDetector(args_));
        vector<float> template_samples = load_template(FLAGS_template);
        corr_det_.reset(new CorrDetector(template_samples,
                                         block_size_,
                                         history_size_,
                                         corr_thresh_const,
                                         corr_thresh_snr));

        synced_signal_ = to_complex_star(synced_fft_calc_.input());
        synced_fft_ = to_complex_star(synced_fft_calc_.output());

        corr_signal_ = to_complex_star(corr_fft_calc_.input());
        corr_fft_ = to_complex_star(corr_fft_calc_.output());

        slice_start_ = max(0, slice_start);
        slice_len_ = (slice_len <= 0) ? corr_size_-slice_start_
                     : min(corr_size_-slice_start_, (size_t)slice_len);

        setOutput(std::move(out));
    }

    void stop() {
        setMode(ReceiverMode::STOP);
    }

    void standby() {
        setMode(ReceiverMode::STANDBY);
    }

    void lock() {
        setMode(ReceiverMode::LOCK);
    }

    void capture() {
        setMode(ReceiverMode::CAPTURE);
    }

    // Get the desired state (mode) of the receiver
    ReceiverMode getMode() {
        return mode_;
    }

    // Get the actual state of the receiver
    ReceiverState getState() {
        return state_;
    }

    const char* stateToString(ReceiverState state) {
        switch (state) {
            case ReceiverState::STOPPED:
                return "STOPPED";
            case ReceiverState::STANDBY:
                return "STANDBY";
            case ReceiverState::TRACK:
                return "TRACK";
            case ReceiverState::NOISE_WAIT:
                return "NOISE_WAIT";
            case ReceiverState::NOISE_CAPTURE:
                return "NOISE_CAPTURE";
        }
        return "UNKNOWN";
    }

    const char* modeToString(ReceiverMode state) {
        switch (state) {
            case ReceiverMode::STOP:
                return "STOP";
            case ReceiverMode::STANDBY:
                return "STANDBY";
            case ReceiverMode::LOCK:
                return "LOCK";
            case ReceiverMode::CAPTURE:
                return "CAPTURE";
        }
        return "UNKNOWN";
    }

    const char* trackStateToString(TrackState state) {
        switch (state) {
            case TrackState::INACTIVE:
                return "INACTIVE";
            case TrackState::FIND_CARRIER:
                return "FIND_CARRIER";
            case TrackState::LOCKED:
                return "LOCKED";
            case TrackState::FIND_BEACON:
                return "FIND_BEACON";
            case TrackState::CAPTURE:
                return "CAPTURE";
        }
        return "UNKNOWN";
    }

    // Mode is always set by the user, but will also be set by the class if
    // the state transitions to STOP() or to STANDBY()
    void setMode(ReceiverMode new_mode);

    void setOutput(CFile&& out) {
        assert(isInactiveState(state_));

        // Open output file
        writer_.reset(new CorxFileWriter(std::move(out)));

        // Write header
        writer_->write_file_header({(uint16_t)slice_start_,
                                   (uint16_t)slice_len_});

        // TODO
        // FLAGS_frequency = filename
        // printf("Output file set to \"%s\"\n");
    }

    void setFrequency() {
        // TODO
        // May only be set in STANDBY state
        // (or STOPPED state, in which case we simply change fargs)
    }

    void setGain() {
        // TODO
        // May only be set in STANDBY state
        // (or STOPPED state, in which case we simply change fargs)
    }

    // next() should be called repeatedly until it returns false, in
    // which case the receiver will be in the STOPPED state.
    bool next();

protected:
    // State transitions should only happend from within the next() function
    void setState(ReceiverState new_state);

    // Capture raw data without performing carrier or beacon detection using
    // the last known carrier frequency for carrier recovery.
    void nextNoiseCapture();

    void nextActive();

    void setTrackState(TrackState new_state);

    // Synchronise to / track the carrier.
    // Sets synced_signal_ and dc_angle_.
    // Sets track state to FIND_CARRIER if the carrier is lost.
    // Sets track state to LOCKED when the carrier has been recovered.
    void recoverCarrier();
    void findBeacon();

    // Capture segment FFTs for cross-correlation.
    // Returns false when the last segment cycle has been reached.
    bool captureCorrSegments();

    // Estimate the receiver's clock offset from the position of the carrier
    // frequency. It is assumed that the downconverter and ADC have the same
    // local oscillator (i.e. that they are coherent).
    float estimateClockError() {
        return (carrier_pos_ * args_->sdr_sample_rate / block_size_
                - FLAGS_carrier_ref) / args_->sdr_freq;
    }

    // Set bias tee, i.e. enable or disable preamp
    bool setBiasTee(bool on) {
        if (strcmp(args_->input_file, "rtlsdr") != 0) {
            return false;
        }

#ifdef LIBRTLSDR_BIAS_TEE_SUPPORT
        rtlsdr_reader_set_bias_tee(carrier_det_->get()->reader, on);
        BPRINTF("%s", on ? "Enabled bias tee\n" : "Disabled bias tee\n");
        // TODO: clear buffers
        // printf("Cleared buffers");
        return true;
#else
        return false;
#endif
    }

    void setStandby(bool on) {
        if (strcmp(args_->input_file, "rtlsdr") == 0) {
            rtlsdr_reader_set_standby(carrier_det_->get()->reader, on);
            BPRINTF("%s",
                    on ? "Input discard enabled\n" : "Input discard disabled\n");
        }
    }

    // Timeout functions
    void setTimeout(size_t &timeout, float delta_secs) {
        if (delta_secs == 0) {
            timeout = 0;
        } else {
            timeout = block_idx_;
            timeout += delta_secs * args_->sdr_sample_rate / nonhistory_size_;
        }
    }

    bool isTimeoutExpired(const size_t &timeout) {
        return timeout > 0 && block_idx_ >= timeout;
    }

    void deactiveTimeout(size_t &timeout) {
        timeout = 0;
    }

private:
    ReceiverState state_;
    ReceiverMode mode_;

    // Last inactive state to return to after a capture session
    ReceiverMode last_inactive_mode_;

    // -- Submodules / buffers
    fargs_t* args_;  // Arguments passed to carrier_det_
    // Read input and perform carrier detection using fastcard.
    std::unique_ptr<CarrierDetector> carrier_det_;

    // Perform correlation detection using fastdet.
    std::unique_ptr<CorrDetector> corr_det_;

    // Synced signal, i.e. signal after carrier recovery.
    FFT synced_fft_calc_;
    complex<float>* synced_signal_;
    complex<float>* synced_fft_;

    // Correlation block buffers.
    FFT corr_fft_calc_;
    complex<float>* corr_signal_;
    complex<float>* corr_fft_;
    AlignedArray<complex<float>> corrected_corr_fft_;

    // -- Variables that may not change after construction
    const size_t block_size_;
    const size_t history_size_;
    const size_t nonhistory_size_;

    // Correlation block size.
    size_t corr_size_;

    // Output slice
    int slice_start_;
    int slice_len_;

    // -- Variables used by all states

    // Number of blocks read.
    unsigned block_idx_ = 0;

    // Stream for temporary debugging output
    CFile debug_;

    // -- Variables used by all non-STOPPED states

    // -- Variables used by TRACK and NOISE_CAPTURE state
    //    (reset when entering TRACK state)

    // Number of correlation blocks between beacon pulses.
    int num_cycles_;

    // Phase of first sample in block.
    // Used to ensure a continuous phase between subsequent blocks.
    DeciAngle sample_phase_;

    // Position of carrier in FFT bins.
    float carrier_pos_;

    // Running average of dc_angle_.
    float avg_dc_angle_;

    // Beacon Sample of Arrival
    double soa_;
    double prev_soa_;

    // Last beacon detection
    CorrDetection beacon_corr_;

    // Correlation block within block of data between subsequent beacons.
    // -1: waiting for first pulse
    int32_t cycle_ = -1;
    // TODO: assert cycle_ == -1 on destruction

    // Output stream.
    // Opened when entering non-inactive state.
    // Closed when exiting non-inactive state.
    unique_ptr<CorxFileWriter> writer_;

    // -- Variables specific to TRACK state
    TrackState track_state_;

    // Complex argument at DC frequency
    DeciAngle dc_angle_;
    DeciAngle prev_dc_angle_;
    float dc_ampl_;

    // Estimated clock error.
    float clock_error_;

    // Running average of dc_ampl_.
    float avg_dc_ampl_;

    // Number of beacon pulses received.
    // Beacon pulses are used to relate samples of different receivers to each other.
    // Called "pulse" in Python code.
    int32_t beacon_;

    // Number of correlation blocks with large phase offsets.
    int num_phase_errors_;

    // -- Timeouts
    size_t track_timeout_;
    size_t lock_timeout_;
    size_t capture_timeout_;
    size_t noise_wait_timeout_;
    size_t noise_capture_timeout_;
};


void Receiver::setMode(ReceiverMode new_mode) {
    if (new_mode == mode_) {
        return;
    }

    ReceiverMode old_mode = mode_;
    mode_ = new_mode;

    if (mode_ == ReceiverMode::STOP || mode_ == ReceiverMode::STANDBY) {
        last_inactive_mode_ = mode_;
    }

    BPRINTF("MODE changed from %s to %s\n",
            modeToString(old_mode),
            modeToString(new_mode));

    if (new_mode == ReceiverMode::STOP) {
        if (carrier_det_) {
            carrier_det_->cancel();
        }
        // the next next() call will transition to the STOP state
    }
}


void Receiver::setState(ReceiverState new_state) {
    if (new_state == state_) {
        return;
    }

    ReceiverState old_state = state_;
    state_ = new_state;

    // -- Actions for changing from old state
    switch (old_state) {
        case ReceiverState::STANDBY:
            setStandby(false);
            break;

        case ReceiverState::TRACK:
            // Stop write cycle if it is still open
            // This action should be taken before the file is closed
            // (also performed for NOISE_CAPTURE state)
            if (cycle_ >= 0) {
                cycle_ = -1;
                writer_->write_cycle_stop();
            }
            // TODO: flush output

            setTrackState(TrackState::INACTIVE);

            // Bias tee should be off in all states other than TRACK
            setBiasTee(false);
            break;

        case ReceiverState::NOISE_WAIT:
            deactiveTimeout(noise_wait_timeout_);
            break;

        case ReceiverState::NOISE_CAPTURE:
            // Stop write cycle if it is still open
            // This action should be taken before the file is closed
            // (also performed for NOISE_CAPTURE state)
            if (cycle_ >= 0) {
                cycle_ = -1;
                writer_->write_cycle_stop();
            }
            // TODO: flush output

            // May only reach INACTIVE state from NOISE_CAPTURE
            assert(isInactiveState(new_state));
            deactiveTimeout(noise_capture_timeout_);
            break;

        default:
            break;
    }

    BPRINTF("STATE changed from %s to %s\n",
            stateToString(old_state),
            stateToString(new_state));

    // -- Actions for changing to new state
    switch (new_state) {
        case ReceiverState::STOPPED:
            // Output stats
            carrier_det_->print_stats(stdout);
            break;

        case ReceiverState::STANDBY:
            setStandby(true);
            break;

        case ReceiverState::TRACK:
            // Bias tee should be on in the TRACK state
            setBiasTee(true);
            // Reset variables
            track_state_ = TrackState::INACTIVE;
            num_cycles_ = ((FLAGS_beacon_interval * args_->sdr_sample_rate
                            - 2*FLAGS_beacon_padding) / 
                           corr_size_);
            sample_phase_ = 0;
            carrier_pos_ = 0;
            avg_dc_angle_ = 0;
            soa_ = 0;
            prev_soa_ = 0;
            dc_angle_ = 0;
            prev_dc_angle_ = 0;
            dc_ampl_ = 0;
            clock_error_ = 0;
            avg_dc_ampl_ = 0;
            beacon_ = -1;
            num_phase_errors_ = 0;

            // set timers
            setTimeout(track_timeout_, FLAGS_timeout);
            deactiveTimeout(lock_timeout_);
            deactiveTimeout(capture_timeout_);

            setTrackState(TrackState::FIND_CARRIER);

            break;

        case ReceiverState::NOISE_WAIT:
            // May only reach NOISE_WAIT from TRACK state
            assert(old_state == ReceiverState::TRACK);
            setTimeout(noise_wait_timeout_,
                       FLAGS_preamp_off_transition_time);
            break;

        case ReceiverState::NOISE_CAPTURE:
            // May only reach NOISE_CAPTURE from NOISE_WAIT state
            assert(old_state == ReceiverState::NOISE_WAIT);
            setTimeout(noise_capture_timeout_,
                       FLAGS_preamp_off_time);
            break;
    }

    // -- Asserts
    // cycle_ should always be -1 after a state change
    // (i.e. write cycle should be stopped)
    assert(cycle_ == -1);

    // -- Actions that should be performed last
    // // Transition from inactive state
    // if (isInactiveState(old_state) && !isInactiveState(new_state)) {
    // }

    // // Transition to inactive state
    // if (!isInactiveState(old_state) && isInactiveState(new_state)) {
    // }

    // Transition from STOPPED
    if (old_state == ReceiverState::STOPPED) {
        // RTL should be on in all states other that STOPPED
        carrier_det_->start();
    }
}


bool Receiver::next() {
    if (state_ == ReceiverState::STOPPED &&
            mode_ == ReceiverMode::STOP) {
        return false;
    }
    if (state_ != ReceiverState::STANDBY &&
            mode_ == ReceiverMode::STANDBY) {
        // Transition to STANDBY mode
        setState(ReceiverState::STANDBY);
    }

    // Ensure the state we reference is the same
    // throughout the rest of this function
    ReceiverState state = state_;

    // Handle STOPPED and STANDBY state
    switch (state) {
        case ReceiverState::STOPPED:
        case ReceiverState::STANDBY:
            // Handle transition from inactive state.
            if (mode_ == ReceiverMode::LOCK ||
                    mode_ == ReceiverMode::CAPTURE) {
                setState(ReceiverState::TRACK);
            }
            break;
        default:
            break;
    }

    // read next block without performing carrier detection
    bool success = carrier_det_->next();
    if (!success) {
        // Transition to STOPPED state
        setState(ReceiverState::STOPPED);
        if (mode_ != ReceiverMode::STOP) {
            // ended prematurely
            setMode(ReceiverMode::STOP);
        }
        return false;
    }

    // Increase block ID in active states
    if (state_ != ReceiverState::STANDBY) {
        block_idx_++;
    }

    // Handle states
    switch (state) {
        case ReceiverState::STOPPED:
        case ReceiverState::STANDBY:
            // Skip one block to ensure that the data history is filled with
            // valid data.
            break;
        case ReceiverState::TRACK:
            nextActive();
            if (isTimeoutExpired(track_timeout_) ||
                    isTimeoutExpired(lock_timeout_) ||
                    isTimeoutExpired(capture_timeout_)) {

                if (isTimeoutExpired(track_timeout_)) {
                    BPRINTF("Track timeout: "
                            "maximum time in active state expired\n");
                }
                if (isTimeoutExpired(lock_timeout_)) {
                    BPRINTF("Lock timeout: could not find carrier\n");
                }
                if (isTimeoutExpired(capture_timeout_)) {
                    BPRINTF("Capture timeout\n");
                }

                if (beacon_ == -1) {
                    setMode(last_inactive_mode_);
                } else {
                    setState(ReceiverState::NOISE_WAIT);
                }
            }
            break;
        case ReceiverState::NOISE_WAIT:
            if (isTimeoutExpired(noise_wait_timeout_)) {
                setState(ReceiverState::NOISE_CAPTURE);
            }
            break;
        case ReceiverState::NOISE_CAPTURE:
            nextNoiseCapture();
            if (isTimeoutExpired(noise_capture_timeout_)) {
                setMode(last_inactive_mode_);
            }
            break;
    }

    return true;
}

void Receiver::nextNoiseCapture() {
    // continue with last carrier frequency from active state
    freq_shift(synced_signal_,
               to_complex_star(carrier_det_->data().samples),
               block_size_,
               -carrier_pos_,
               sample_phase_);

    if (cycle_ == -1) {
        // FIXME: copy-pasta
        BPRINTF("Capture noise: next cycle\n");

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
        header.clock_error = 0;
        header.carrier_pos = carrier_pos_;
        header.carrier_amplitude = 0;
        header.preamp_on = false;
        writer_->write_cycle_start(header);
    }

    bool has_more_segments = captureCorrSegments();
    if (!has_more_segments) {
        cycle_ = -1;
        writer_->write_cycle_stop();
    }
}

void Receiver::nextActive() {
    // Track carrier
    // Transition to FIND_CARRIER if lost; transition to LOCKED if found
    recoverCarrier();

    sample_phase_ -= (carrier_pos_ *
                      (1.f - (float)history_size_ / block_size_));
    sample_phase_ = normalize_deciangle(sample_phase_);

    if (track_state_ == TrackState::FIND_CARRIER) {
        return;
    }
    if (track_state_ == TrackState::LOCKED && mode_ == ReceiverMode::CAPTURE) {
        setTrackState(TrackState::FIND_BEACON);
    }

    // mitigate rapid discontinuities at +-0.5
    while (dc_angle_ > avg_dc_angle_ + 0.5) { dc_angle_ -= 1; }
    while (dc_angle_ < avg_dc_angle_ - 0.5) { dc_angle_ += 1; }

    avg_dc_angle_ = (dc_angle_ * (float)FLAGS_avg_angle_weight +
                     avg_dc_angle_ * (1-(float)FLAGS_avg_angle_weight));
    avg_dc_ampl_ = (dc_ampl_ * (float)FLAGS_avg_mag_weight +
                    avg_dc_ampl_ * (1-(float)FLAGS_avg_mag_weight));

    if (debug_.file() != nullptr) {
        fwrite(&avg_dc_angle_, sizeof(float), 1, debug_.file());
    }

    if (track_state_ == TrackState::FIND_BEACON) {
        // Look for beacon signal and transition to CAPTURE if found
        findBeacon();
    }

    if (track_state_ == TrackState::CAPTURE) {
        // Capture xcorr segments
        bool has_more_segments = captureCorrSegments();
        // Transition to FIND_BEACON when done with segment cycle
        if (!has_more_segments) {
            setTrackState(TrackState::FIND_BEACON);
        }
    }
}

void Receiver::setTrackState(TrackState new_state) {
    if (new_state == track_state_) {
        return;
    }
    TrackState old_state = track_state_;
    track_state_ = new_state;

    switch (old_state) {
        case TrackState::INACTIVE:
            break;

        case TrackState::FIND_CARRIER:
            deactiveTimeout(lock_timeout_);
            break;

        case TrackState::LOCKED:
            break;

        case TrackState::FIND_BEACON:
            break;

        case TrackState::CAPTURE:
            cycle_ = -1;
            writer_->write_cycle_stop();
            if (num_phase_errors_ > 0) {
                printf("beacon %d: %d / %d corr blocks have large phase error\n",
                       beacon_, num_phase_errors_, num_cycles_);
            }
            break;
    }

    BPRINTF("TRACK state changed from %s to %s\n",
            trackStateToString(old_state),
            trackStateToString(new_state));

    switch (new_state) {
        case TrackState::INACTIVE:
            break;

        case TrackState::FIND_CARRIER:
            setTimeout(lock_timeout_, FLAGS_carrier_search_timeout);
            break;

        case TrackState::LOCKED:
            break;

        case TrackState::FIND_BEACON:
            break;

        case TrackState::CAPTURE:
            cycle_ = 0;
            num_phase_errors_ = 0;

            if (beacon_ == 0) {
                BPRINTF("Found first beacon. "
                        "Capture mode will expire after %.1f seconds.\n",
                        FLAGS_capture_time);

                setTimeout(capture_timeout_, FLAGS_capture_time);
            }

            const struct timeval ts = carrier_det_->data().block->timestamp;

            CorxBeaconHeader header;
            header.soa = soa_;
            header.timestamp_sec = ts.tv_sec;
            header.timestamp_msec = ts.tv_usec / 1000;
            header.beacon_amplitude = sqrt(beacon_corr_.peak_power);
            header.beacon_noise = sqrt(beacon_corr_.noise_power);
            header.clock_error = clock_error_;
            header.carrier_pos = carrier_pos_;
            header.carrier_amplitude = dc_ampl_;
            header.preamp_on = true;
            writer_->write_cycle_start(header);
            break;
    }
}

void Receiver::recoverCarrier() {
    TrackState initial_track_state = track_state_;

    //// Carrier tracking and synchronization
    if (track_state_ != TrackState::FIND_CARRIER) {
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

        // BPRINTF("Carrier phase error: %.0f deg\n", angle_diff * 360);

        if (angle_diff * 360 > FLAGS_max_tracking_phase_diff) {
            // tracking loop failed
            BPRINTF("Tracking loop failed\n");
            setTrackState(TrackState::FIND_CARRIER);
        } else {
            // track
            carrier_pos_ += angle_diff * FLAGS_tracking_diff_coeff;

            if (debug_.file() != nullptr) {
                fwrite(&carrier_pos_, sizeof(float), 1, debug_.file());
                fwrite(&dc_angle_, sizeof(float), 1, debug_.file());
            }
        }
    }

    //// Carrier detection and synchronization
    if (track_state_ == TrackState::FIND_CARRIER) {
        //// Tracking loop failed
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

            // perform freq shift
            freq_shift(synced_signal_,
                       to_complex_star(carrier_det_->data().samples),
                       block_size_,
                       -carrier_pos_,
                       sample_phase_);

            complex<float> dc = calculate_dc(synced_signal_, block_size_);
            dc_ampl_ = abs(dc);
            dc_angle_ = normalize_deciangle(arg(dc) / (float)PI / 2);

            avg_dc_ampl_ = dc_ampl_ * 2;
            avg_dc_angle_ = dc_angle_;

            BPRINTF("Detected carrier @ %.3f; SNR: %.1f / %.1f; (DC: %.1f)\n",
                    carrier_pos_,
                    carrier.detection.max,
                    carrier.detection.noise,
                    dc_ampl_);

            setTrackState(TrackState::LOCKED);

        } else if (initial_track_state != TrackState::FIND_CARRIER) {
            BPRINTF("No carrier detected\n");
        }
    }
}

void Receiver::findBeacon() {
    assert(cycle_ == -1);
    assert(track_state_ == TrackState::FIND_BEACON);

    // use change in signal strength to determine whether it is worth
    // looking for a beacon signal

    if (FLAGS_beacon_carrier_trigger_factor < 1 &&
        dc_ampl_ > avg_dc_ampl_ * FLAGS_beacon_carrier_trigger_factor) {
        // falling edge trigger
        return;
    }
    if (FLAGS_beacon_carrier_trigger_factor > 1 &&
        dc_ampl_ < avg_dc_ampl_ * FLAGS_beacon_carrier_trigger_factor) {
        // rising edge trigger
        return;
    }

    BPRINTF("beacon check. avg: %.0f; DC: %.0f; thresh: %.0f\n",
            avg_dc_ampl_,
            dc_ampl_,
            avg_dc_ampl_ * FLAGS_beacon_carrier_trigger_factor);

    synced_fft_calc_.execute();
    float signal_energy = 0; // TODO: calculate signal_energy
    CorrDetection corr = corr_det_->detect(synced_fft_, signal_energy);
    if (corr.detected) {
        BPRINTF("detected beacon (ampl: %.0f)\n", corr.peak_power);
        
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

        clock_error_ = estimateClockError();
        printf("beacon #%d: soa = %.3f; timestep = %.1f; ppm=%.3f\n",
               beacon_,
               soa_,
               time_step,
               clock_error_ * 1e6);

        beacon_corr_ = corr;
        setTrackState(TrackState::CAPTURE);
    }
}

bool Receiver::captureCorrSegments() {
    assert(state_ == ReceiverState::TRACK ||
           state_ == ReceiverState::NOISE_CAPTURE);
    assert(state_ != ReceiverState::TRACK ||
           track_state_ == TrackState::CAPTURE);

    assert(cycle_ >= 0);

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
        writer_->write_cycle_block(error_fp,
                                  corrected_corr_fft_.data()+slice_start_,
                                  slice_len_);
    }

    return (cycle_ < num_cycles_);
}


///////////////////////////////////////////////////////////////////////////////
// TODO: move parse_fargs to within Receiver class (to process FLAGS)
// or to cli.cpp

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
    // fargs->skip = FLAGS_skip;

    fargs->sdr_freq = (uint32_t)parse_si_float(&FLAGS_frequency[0]);
    fargs->sdr_gain = (int)FLAGS_gain * 10; // unit: tenths of a dB
    fargs->sdr_sample_rate = (uint32_t)parse_si_float(&FLAGS_sample_rate[0]);
    fargs->sdr_dev_index = FLAGS_device_index;
}


// TODO: move LineReader and InteractiveReceiver to interactive_receiver.cpp

#define LINEREADER_BUFLEN 2048

// A simple line reader
// Use readline or editline instead?
//  (see http://www.delorie.com/gnu/docs/readline/rlman_41.html)
//  (and https://stackoverflow.com/questions/1706678/ncurses-and-stdin-blocking)
class LineReader {
public:
    LineReader() {
        fds.fd = STDIN_FILENO;
        fds.events = POLLIN;
        linebuf_len_ = 0;
        eof_ = false;
    }

    bool readInput(bool block) {
        int ret = poll(&fds, 1, block ? -1 : 0);
        if (ret > 0) {
            // clear buffer on overflow
            if (linebuf_len_ == LINEREADER_BUFLEN - 1) {
                fprintf(stderr, "Warning: Read buffer overflow\n");
                linebuf_len_ = 0;
            }

            ssize_t len = read(STDIN_FILENO,
                              linebuf_ + linebuf_len_,
                              LINEREADER_BUFLEN - linebuf_len_ - 1);

            if (len <= 0) {
                if (len == 0) {
                    // eof
                    eof_ = true;
                } else {
                    // error
                    fprintf(stderr, "Warning: Read error: %zd\n", len);
                }
                return false;
            }

            // printf("Read (size %zu): %s\n", len, linebuf_ + linebuf_len_);

            // add to buffer
            linebuf_len_ += len;
            linebuf_[linebuf_len_] = '\0';

            // add lines to queue
            const char* head = linebuf_;
            const char* tail = head;
            while ((head = strchr(head, '\n')) != NULL) {
                size_t len = head - tail;
                std::string line(tail, len);
                lines_.push_back(line);
                ++head;
                tail = head;
                // printf("Push line: <<%s>>\n", line.data());
                // fflush(stdout);
            }
            size_t newlen = linebuf_ + linebuf_len_ - tail;
            memmove(linebuf_, tail, newlen);
            linebuf_len_ = newlen;
        } else if (ret < 0) {
            fprintf(stderr, "Poll error: %d\n", ret);
            return false;
        }

        return true;
    }

    bool hasLines() {
        return !lines_.empty();
    }

    std::string popLine() {
        std::string s = lines_.front();
        lines_.pop_front();
        return s;
    }

private:
    std::deque<std::string> lines_;
    char linebuf_[LINEREADER_BUFLEN];
    size_t linebuf_len_;
    struct pollfd fds;
    bool eof_;
};


class InteractiveReceiver {
public:
    InteractiveReceiver(std::unique_ptr<Receiver>&& detector)
        : detector_(std::move(detector)) {};
    void run();
    void sigint() { sigint_ = true; }
    void exit() { sigint_ = true; eof_ = true; }

private:
    void executeCommand(std::string line);
    // TODO: own detector?

    bool eof_;
    bool sigint_ = false;
    std::unique_ptr<Receiver> detector_;
};


void InteractiveReceiver::run() {
    LineReader reader;
    eof_ = false;
    bool waiting = false;

    while (true) {
        ReceiverMode mode = detector_->getMode();
        ReceiverState state = detector_->getState();
        bool stopped = (state == ReceiverState::STOPPED &&
                        mode == ReceiverMode::STOP);

        if (sigint_) {
            //  (first Ctrl-C will stop the receiver; second one will exit)
            waiting = false;
            while (reader.hasLines()) {
                reader.popLine();
            }

            if (stopped) {
                eof_ = true;
            } else {
                detector_->setMode(ReceiverMode::STOP);
                printf("Receiver stopped. Press Ctrl-C again to exit.\n");
            }

            sigint_ = false;
        }

        if (waiting) {
            // waiting... check mode
        } else {
            if (!eof_) {
                if (stopped) {
                    printf("corx> ");
                    fflush(stdout);
                }
                eof_ = !reader.readInput(stopped);
            }
            if (eof_) {
                if (stopped) {
                    break;
                } else if (!reader.hasLines()) {
                    detector_->setMode(ReceiverMode::STOP);
                }
            }

            // process lines
            while (reader.hasLines()) {
                string line = reader.popLine();
                executeCommand(line);
            }
        }

        if (!stopped) {
            detector_->next();
            // TODO: check return value?
        }
    }
}

void InteractiveReceiver::executeCommand(std::string line) {
    // Split command
    string command, argument;
    string::size_type idx = line.find(' ');
    if (idx == string::npos) {
        command = line;
        argument.clear();
    } else {
        command = line.substr(0, idx);
        argument = line.substr(idx + 1);
    }

    // Handle command
    transform(command.begin(), command.end(), command.begin(), ::tolower);
    ReceiverState state = detector_->getState();
    if (command == "stop") {
        detector_->stop();
    } else if (command == "standby") {
        detector_->standby();
    } else if (command == "lock") {
        detector_->lock();
    } else if (command == "capture") {
        detector_->capture();
    } else if (command == "exit") {
        detector_->stop();
        eof_ = true;
    } else if (command == "output") {
        if (state != ReceiverState::STOPPED &&
                state != ReceiverState::STANDBY) {
            printf("Output file may be changed only when the receiver is "
                   "in the STOPPED or STANDBY state.\n");
        } else {
            FLAGS_output = argument;
            detector_->setOutput(CFile(argument));
        }
    } else {
        if (command != "help") {
            printf("Invalid command: %s\n", command.data());
        }
        printf("Valid commands: stop standby lock capture output exit help\n");
    }
    // output <new_filename>
    //   may be changed in STOPPED or STANDBY state only
    // freq <new_freq>
    //   may be changed in STOPPED or STANDBY state only
    // set <new_flags>
    //   reconstruct detector after set command
    //   may be changed in STOPPED state only
}

} // namespace corx

using namespace corx;


// TODO: move everything below this line to cli.cpp

std::unique_ptr<Receiver> receiver;
std::unique_ptr<InteractiveReceiver> interactive_receiver;

void signal_handler(int signo) {
    if (receiver) {
        receiver->stop();
    }
    if (interactive_receiver) {
        if (signo == SIGINT) {
            interactive_receiver->sigint();
        } else {
            interactive_receiver->exit();
        }
    }
}



DEFINE_bool(interactive, false, "Use stdin for controlling the receiver with "
                                "an interactive prompt.");

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // TODO: move arg parsing to detector class!
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

    receiver.reset(new Receiver(args.get(),
                                arg_corr_thresh_const,
                                arg_corr_thresh_snr,
                                slice_start,
                                slice_len,
                                CFile(FLAGS_output)));

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGPIPE, signal_handler);

    try {
        if (!FLAGS_interactive) {
            receiver->capture();
            while (receiver->next()) {}
        } else {
            interactive_receiver.reset(
                    new InteractiveReceiver(std::move(receiver)));
            interactive_receiver->run();
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
