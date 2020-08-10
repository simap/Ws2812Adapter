
#include "stdint.h"
#include "Arduino.h"

/// Class to ensure that a minimum amount of time has kicked since the last time run - and delay if not enough time has passed yet
/// this should make sure that chipsets that have
template<int WAIT> class CMinWait {
    uint16_t mLastMicros;
public:
    CMinWait() { mLastMicros = 0; }

    void wait() {
        uint16_t diff;
        do {
            diff = (micros() & 0xFFFF) - mLastMicros;
        } while(diff < WAIT);
    }

    void mark() { mLastMicros = micros() & 0xFFFF; }
};


// Allow clock that clockless controller is based on to have different
// frequency than the CPU.
#if !defined(CLOCKLESS_FREQUENCY)
#define CLOCKLESS_FREQUENCY F_CPU
#endif

#define C_NS(_NS) (((_NS * ((CLOCKLESS_FREQUENCY / 1000000L)) + 999)) / 1000)



#include "clockless_rmt_esp32.h"