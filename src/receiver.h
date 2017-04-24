#ifndef CORX_RECEIVER
#define CORX_RECEIVER

namespace corx {

enum class ReceiverState {
    STOPPED,        // RTL off (RTL on unless in STOPPED)
                    // \_ Mode == STANDBY -> STANDBY
                    // \_ Mode == LOCK || CAPTURE -> TRACK
    STANDBY,        // Discard input (consume unless in STANDBY)
                    // \_ SDR fail -> STOPPED
                    // \_ Mode == LOCK || CAPTURE -> TRACK
    TRACK,          // Preamp on (off unless in TRACK), output file open
                    // \_ Active or lock or capture timeout
                    //    ... no beacons detected -> STANDBY _mode_
                    //    ... at least one beacon -> NOISE_WAIT
                    // \_ SDR fail -> STOPPED
    NOISE_WAIT,     // Consume and ignore input, output file open
                    // \_ SDR fail -> STOPPED
                    // \_ Noise wait timeout -> NOISE_CAPTURE
    NOISE_CAPTURE,  // Preamp off, output file open
                    // Capture (blind) data and output FFTs
                    // \_ SDR fail -> STOPPED
                    // \_ Noise capture timeout -> STANDBY _mode_

    // State groups:
    //   INACTIVE: STOPPED, STANDBY
    //   TRACK: FIND_CARRIER, LOCKED, FIND_BEACON, CAPTURE
    //   ZOMBIE: NOISE
    //
    // State transitions:
    //   STOPPED -> *: start RTL-SDR (perform after "to STANDBY" actions)
    //   * -> STANDBY: discard input
    //   STANDBY -> *: unset discard input
    //   INACTIVE -> TRACK: read one dummy block (fill history)
    //   INACTIVE -> *: open output file
    //   * -> INACTIVE: close output file
    //   * -> TRACK: write_cycle_start, preamp on
    //   TRACK -> *: write_cycle_stop, preamp off
    //
    // State of resources:
    //   RTL: on in all states except STOPPED
    //   Preamp: on in all states except NOISE
    //   Output file: open in all non-INACTIVE states
    //   Discard mode: set when entering STANDBY, unset when exiting STANDBY
    //
    // Timeouts:
    //   Active timeout: set when entering TRACK state,
    //                   deactivated when exiting TRACK state
    //   Lock timeout: set when entering FIND_CARRIER state,
    //                 deactivated when exiting FIND_CARRIER active state
    //   Capture timeout: set when entering CAPTURE state for the first time,
    //                    deactivated when exiting TRACK state
    //   Noise wait timeout: set when entering NOISE_WAIT state,
    //                       deactived when exiting NOISE_WAIT state
    //   Noise capture timeout: set when entering NOISE_CAPTURE state,
    //                          deactived when exiting NOISE_CAPTURE state
};

enum class TrackState {
    FIND_CARRIER,   // searching for carrier
                    // \_ SDR fail -> STOPPED
                    // \_ lock timeout -> STANDBY
                    // \_ locked -> LOCKED
    LOCKED,         // tracking carrier
                    // \_ SDR fail -> STOPPED
                    // \_ Tracking loop fail -> FIND_CARRIER
                    // \_ Mode == CAPTURE -> FIND_BEACON
    FIND_BEACON,    // searching for beacon signal
                    // \_ SDR fail -> STOPPED
                    // \_ Tracking loop fail -> FIND_CARRIER
                    // \_ Capture timeout -> NOISE (or LOCKED)
                    // \_ Found -> CAPTURE
    CAPTURE,        // capture data and output FFTs
                    // \_ SDR fail -> STOPPED
                    // \_ Tracking loop fail -> FIND_CARRIER
                    // \_ Done with segment -> FIND_BEACON
                    // \_ Capture timeout -> NOISE_WAIT if preamp_off_time > 0
                    //                       else STANDBY
};

// The mode is the desired state of the receiver
enum class ReceiverMode {
    STOP,
    STANDBY,
    LOCK,
    CAPTURE
};

} // namespace corx

#endif /* CORX_RECEIVER */
