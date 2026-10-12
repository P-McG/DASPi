#include <array>
#include <cstdint>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace DASPi
{

// -----------------------------------------------------------------------------
// ApertureComputeModuleTimer
//
// Raspberry Pi Pico 2 timing controller for the DASPi Global Shutter cameras.
//
// IMX296 XTR behaviour:
//   HIGH = idle
//   LOW  = exposure
//
// All camera outputs are changed using a single GPIO mask operation so that
// every camera receives the trigger edge at effectively the same time.
// -----------------------------------------------------------------------------

constexpr std::array<uint, 10> CameraTriggerPins
{
    2,  // Camera 0
    3,  // Camera 1
    4,  // Camera 2
    5,  // Camera 3
    6,  // Camera 4
    7,  // Camera 5
    8,  // Camera 6
    9,  // Camera 7
    10, // Camera 8
    11  // Camera 9
};

// TEMPORARY BENCH TEST
//constexpr uint32_t FrameRate = 1;

// Target frame rate.
constexpr uint32_t FrameRate = 30;

// 30 fps = 33,333.333... us/frame.
//
// We'll deal with the fractional microsecond by alternating period lengths.
constexpr uint32_t FramePeriodBaseUs = 1'000'000 / FrameRate;  // 33333


// Desired camera exposure.
//
// Raspberry Pi documents:
//
//      exposure = XTR low time + 14.26 us
//
// Therefore, for approximately 20,000 us exposure:
//
//      XTR low = 20,000 - 14.26
//              ≈ 19,986 us
//
// Operational Pulse
constexpr uint32_t DesiredExposureUs = 20'000;
constexpr uint32_t TriggerCorrectionUs = 14;
// Long pulse so it can be observed with a voltmeter.
//constexpr uint32_t DesiredExposureUs = 500'000;   // 500 ms
//constexpr uint32_t TriggerCorrectionUs = 0;

constexpr uint32_t TriggerLowUs =
    DesiredExposureUs - TriggerCorrectionUs;


// -----------------------------------------------------------------------------
// Build a GPIO bitmask containing all ten camera trigger pins.
// -----------------------------------------------------------------------------

constexpr uint32_t MakeTriggerMask()
{
    uint32_t mask = 0;

    for (const uint pin : CameraTriggerPins)
        mask |= (1u << pin);

    return mask;
}

constexpr uint32_t TriggerMask = MakeTriggerMask();


// -----------------------------------------------------------------------------
// Initialise the camera trigger outputs.
// -----------------------------------------------------------------------------

void InitialiseCameraTriggers()
{
    for (const uint pin : CameraTriggerPins)
    {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);

        // XTR is active-low. Start inactive.
        gpio_put(pin, true);
    }
}


// -----------------------------------------------------------------------------
// Set every camera trigger HIGH simultaneously.
// -----------------------------------------------------------------------------

inline void TriggerHigh()
{
    gpio_set_mask(TriggerMask);
}


// -----------------------------------------------------------------------------
// Set every camera trigger LOW simultaneously.
// -----------------------------------------------------------------------------

inline void TriggerLow()
{
    gpio_clr_mask(TriggerMask);
}


// -----------------------------------------------------------------------------
// Generate one exposure.
// -----------------------------------------------------------------------------

inline void TriggerFrame()
{
    // Falling edge starts exposure.
    TriggerLow();

    busy_wait_us_32(TriggerLowUs);

    // Rising edge ends exposure.
    TriggerHigh();
}


// -----------------------------------------------------------------------------
// Main timing loop.
//
// 1,000,000 us is not evenly divisible by 30, so simply delaying 33,333 us
// every frame produces a tiny clock error.
//
// This accumulator inserts an extra microsecond periodically:
//
//      33333
//      33333
//      33334
//
// giving exactly 1,000,000 us for every 30 frames.
// -----------------------------------------------------------------------------

void RunCameraTimer()
{
    uint32_t remainderAccumulator = 0;

    absolute_time_t nextFrame = get_absolute_time();

    while (true)
    {
        TriggerFrame();

        uint32_t framePeriodUs = FramePeriodBaseUs;

        remainderAccumulator += 1'000'000 % FrameRate;

        if (remainderAccumulator >= FrameRate)
        {
            remainderAccumulator -= FrameRate;
            ++framePeriodUs;
        }

        nextFrame = delayed_by_us(nextFrame, framePeriodUs);

        busy_wait_until(nextFrame);
    }
}

} // namespace DASPi


int main()
{
    stdio_init_all();

    DASPi::InitialiseCameraTriggers();

    // Allow cameras and the timing distribution circuitry to settle.
    sleep_ms(1000);

    DASPi::RunCameraTimer();

    return 0;
}
