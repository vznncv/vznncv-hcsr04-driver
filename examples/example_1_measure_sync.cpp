/**
 * HC-SR04 distant sensor usage example.
 *
 * This example show simple measurement.
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

int main()
{
    static SimpleHCSR04Driver hcsr04_driver(HCSR04_TRIG_PIN, HCSR04_ECHO_PIN);

    static DigitalOut user_led(APP_USER_LED, APP_USER_LED_INVERT);
    static Timer timer;
    int err;
    float distance = 0.0f;
    microseconds_u32 call_delay;

    // Note: wait a little before first measurement
    ThisThread::sleep_for(50ms);

    printf("-- start --\n");
    while (true) {
        user_led = !APP_USER_LED_INVERT;
        timer.reset();
        timer.start();
        err = hcsr04_driver.measure_distance(&distance);
        timer.stop();
        user_led = APP_USER_LED_INVERT;
        call_delay = timer.elapsed_time();
        printf("measurement: err = %2i, call_delay = %5u distance = %f\n", err, call_delay.count(), distance);

        ThisThread::sleep_for(200ms);
    }
    return 0;
}
