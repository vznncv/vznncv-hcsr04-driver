/**
 * HC-SR04 distant sensor usage example.
 *
 * This example show asynchronous measurement.
 */
#include "mbed.h"

#include "vznncv_hcsr04_driver.h"

using vznncvhcsr04driver::measure_result_t;
using vznncvhcsr04driver::microseconds_u32;
using vznncvhcsr04driver::SimpleHCSR04Driver;

/**
 * Configuration
 */
constexpr PinName HCSR04_TRIG_PIN = PB_4;
constexpr PinName HCSR04_ECHO_PIN = PB_5;
constexpr PinName APP_USER_LED = LED1;
constexpr bool APP_USER_LED_INVERT = true;

/**
 * Main code
 */

class AsyncMeasureDemo {
private:
    DigitalOut &_user_led;
    bool _invert_led;
    Timer _timer;
    measure_result_t _result = { 0, 0ms };
    microseconds_u32 _call_delay = 0ms;
    Event<void()> _print_measure_result_event;

    void _print_measure_result()
    {
        float distance = SimpleHCSR04Driver::delay_to_distance_default(_result.delay);
        printf("measurement: err = %2i, call_delay = %5u distance = %f\n", _result.err, _call_delay.count(), distance);
    }

    void _measure_cb(const measure_result_t *result)
    {
        _timer.stop();
        _call_delay = _timer.elapsed_time();
        _user_led = _invert_led;
        _result = *result;
        _print_measure_result_event();
    }

public:
    AsyncMeasureDemo(DigitalOut &user_led, bool invert_led)
        : _user_led(user_led)
        , _invert_led(invert_led)
        , _print_measure_result_event(mbed_event_queue()->event(callback(this, &AsyncMeasureDemo::_print_measure_result)))
    {
        _user_led = _invert_led;
    }

    void run_measurement(SimpleHCSR04Driver *driver)
    {
        _timer.start();
        _timer.reset();
        _user_led = !_invert_led;
        driver->measure_delay_async(callback(this, &AsyncMeasureDemo::_measure_cb));
    }
};

int main()
{
    static DigitalOut user_led(APP_USER_LED, APP_USER_LED_INVERT);
    static SimpleHCSR04Driver hcsr04_driver(HCSR04_TRIG_PIN, HCSR04_ECHO_PIN);
    static AsyncMeasureDemo measure_demo(user_led, APP_USER_LED_INVERT);

    // Note: wait a little before first measurement
    ThisThread::sleep_for(50ms);

    printf("-- start --\n");
    while (true) {
        measure_demo.run_measurement(&hcsr04_driver);
        ThisThread::sleep_for(200ms);
    }
    return 0;
}
