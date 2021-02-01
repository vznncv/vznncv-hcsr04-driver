#ifndef VZNNCV_HCSR04_DRIVER_H
#define VZNNCV_HCSR04_DRIVER_H

#include "mbed.h"
#include "mbed_chrono.h"

namespace vznncvhcsr04driver {

using microseconds_u32 = mbed::chrono::microseconds_u32;

/**
 * Measure result.
 */
struct measure_result_t {
    measure_result_t() = default;
    measure_result_t(int err, microseconds_u32 delay)
        : err(err)
        , delay(delay)
    {
    }

    /**
     * Error code.
     *
     * If it's value is non-zero then measure is failed.
     */
    int err;

    /**
     * Delay between signal and echo.
     *
     * Note: use SimpleHCSR04Driver::delay_to_distance_default to get distance in meters.
     */
    microseconds_u32 delay;
};

/**
 * Simple IRQ based HC-SR04 driver.
 *
 * Note: it's recommended to wait at least 50ms after driver initialization,
 *       as before trigger pin configuration, a noise may trigger measurement.
 */
class SimpleHCSR04Driver : private NonCopyable<SimpleHCSR04Driver> {
private:
    DigitalOut _trigger_out;
    InterruptIn _echo_in;

    Ticker t;
    class TimeoutExt : public TimerEvent {
    private:
        SimpleHCSR04Driver *_driver;
        bool _deepsleep_flag;

    public:
        TimeoutExt(SimpleHCSR04Driver *driver);
        virtual ~TimeoutExt();

        void trigger_event(microseconds_u32 rel_time);
        void cancel_event();
        TickerDataClock::time_point now();

    protected:
        void handler() override final;
    };

    enum ErrorCodes : int {
        _ERR_BUSY = -1,
        _ERR_TIMEOUT = -2,
        _ERR_NO_START = -3
    };

    friend TimeoutExt;
    TimeoutExt _timeout_ext;

    atomic_flag _measure_lock = ATOMIC_FLAG_INIT;
    TickerDataClock::time_point _measurement_start;
    microseconds_u32 _echo_start_rel;
    Callback<void(const measure_result_t *result)> _result_callback;

    static constexpr float _PULSE_K = 341.0f / (2 * 1'000'000);
    static constexpr microseconds_u32 _MEASURE_TIMEOUT = 50ms;
    static constexpr unsigned int _TRIGGER_PULSE_NS = 10'000;

    void _echo_irq_rise_handler();
    void _echo_irq_fall_handler();
    void _timeout_handler();

    EventFlags _event_flag;
    static constexpr uint32_t _EVENT_FLAG = 0x01;

    int _measure_distance_async_impl(Callback<void(const measure_result_t *result)> result_callback, bool check_lock);

public:
    /**
     * Constructor.
     *
     * @param trigger_pin trigger pin
     * @param echo_pin echo pin
     */
    SimpleHCSR04Driver(PinName trigger_pin, PinName echo_pin);
    ~SimpleHCSR04Driver() = default;

    /**
     * Measure delay between signal and echo asynchronously.
     *
     * This method starts measurement and returns control. Result will be passed by @c result_callback.
     *
     * If the method returns non-zero code, the callback won't be called.
     *
     * The method itself is IRQ safe, but it consumes about 10us, so it isn't recommended to run it from IRQ context.
     *
     * @param result_callback IRQ safe result callback
     * @return 0 on success, otherwise non-zero value
     */
    int measure_delay_async(Callback<void(const measure_result_t *result)> result_callback);

    /**
     * Measure delay between signal and echo.
     *
     * @param delay variable to store delay
     * @return 0 on success, otherwise non-zero value
     */
    int measure_delay(microseconds_u32 *delay);

    /**
     * Helper base coefficient to convert delay to distance under normal conditions.
     */
    static constexpr float DELAY_TO_DISTANCE_K = 341.0f / (2 * 1'000'000);

    /**
     * Convert delay between signal and echo to distance.
     *
     * This function assumes normal conditions.
     *
     * @param delay delay between signal and echo
     * @return distance in meters
     */
    static constexpr float delay_to_distance_default(microseconds_u32 delay)
    {
        return delay.count() * DELAY_TO_DISTANCE_K;
    }

    /**
     * Measure distance.
     *
     * @param distance variable to store distance
     * @return 0 on success, otherwise non-zero value
     */
    int measure_distance(float *distance);
};
}

#endif // VZNNCV_HCSR04_DRIVER_H
