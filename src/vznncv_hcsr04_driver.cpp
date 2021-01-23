#include "vznncv_hcsr04_driver.h"
using namespace vznncvhcsr04driver;

SimpleHCSR04Driver::TimeoutExt::TimeoutExt(SimpleHCSR04Driver *driver)
    : TimerEvent(get_us_ticker_data())
    , _driver(driver)
    , _deepsleep_flag(false)
{
}

SimpleHCSR04Driver::TimeoutExt::~TimeoutExt()
{
    cancel_event();
}

void SimpleHCSR04Driver::TimeoutExt::trigger_event(vznncvhcsr04driver::microseconds_u32 rel_time)
{
    CriticalSectionLock lock;
    if (!_deepsleep_flag) {
        sleep_manager_lock_deep_sleep();
        _deepsleep_flag = true;
    }
    insert(rel_time);
}

void SimpleHCSR04Driver::TimeoutExt::cancel_event()
{
    CriticalSectionLock lock;
    remove();
    if (_deepsleep_flag) {
        sleep_manager_unlock_deep_sleep();
        _deepsleep_flag = false;
    }
}

TickerDataClock::time_point SimpleHCSR04Driver::TimeoutExt::now()
{
    return _ticker_data.now();
}

void SimpleHCSR04Driver::TimeoutExt::handler()
{
    _driver->_timeout_handler();
}

constexpr unsigned int SimpleHCSR04Driver::_TRIGGER_PULSE_NS;
constexpr microseconds_u32 SimpleHCSR04Driver::_MEASURE_TIMEOUT;
constexpr uint32_t SimpleHCSR04Driver::_EVENT_FLAG;

void SimpleHCSR04Driver::_echo_irq_rise_handler()
{
    _echo_start_rel = _timeout_ext.now() - _measurement_start;
}

void SimpleHCSR04Driver::_echo_irq_fall_handler()
{
    measure_result_t result;
    microseconds_u32 _echo_end_rel = _timeout_ext.now() - _measurement_start;
    // stop echo processing
    _echo_in.disable_irq();
    // cancel timeout if it's set
    _timeout_ext.cancel_event();
    // calculate distance
    if (_result_callback) {
        if (_echo_start_rel == 0ms) {
            result.err = _ERR_NO_START;
            result.delay = 0ms;
        } else {
            result.err = 0;
            result.delay = _echo_end_rel - _echo_start_rel;
        }
        _result_callback(&result);
    }
    // unlock next measurement
    _measure_lock.clear();
}

void SimpleHCSR04Driver::_timeout_handler()
{
    measure_result_t result;
    // stop echo processing
    _echo_in.disable_irq();
    if (_result_callback) {
        result.err = _ERR_TIMEOUT;
        result.delay = 0ms;
        _result_callback(&result);
    }
    // unlock next measurement
    _measure_lock.clear();
}

int SimpleHCSR04Driver::_measure_distance_async_impl(Callback<void(const measure_result_t *)> result_callback, bool check_lock)
{
    if (check_lock && _measure_lock.test_and_set()) {
        // measurement in progress
        return _ERR_BUSY;
    }

    // clear state
    _result_callback = result_callback;
    _echo_start_rel = 0ms;
    // trigger measurement
    _trigger_out = 1;
    wait_ns(_TRIGGER_PULSE_NS);
    {
        CriticalSectionLock lock;
        _measurement_start = _timeout_ext.now();
        _trigger_out = 0;
        _timeout_ext.trigger_event(_MEASURE_TIMEOUT);
        _echo_in.enable_irq();

        _event_flag.clear();
    }

    return 0;
}

SimpleHCSR04Driver::SimpleHCSR04Driver(PinName trigger_pin, PinName echo_pin)
    : _trigger_out(trigger_pin, 0)
    , _echo_in(echo_pin)
    , _timeout_ext(this)
{
    _echo_in.disable_irq();
    _echo_in.rise(callback(this, &SimpleHCSR04Driver::_echo_irq_rise_handler));
    _echo_in.fall(callback(this, &SimpleHCSR04Driver::_echo_irq_fall_handler));
}

int SimpleHCSR04Driver::measure_delay_async(Callback<void(const measure_result_t *)> result_callback)
{
    return _measure_distance_async_impl(result_callback, true);
}

int SimpleHCSR04Driver::measure_delay(microseconds_u32 *delay)
{
    int err;
    measure_result_t cb_result = { 0, 0ms };

    if (_measure_lock.test_and_set()) {
        // measurement in progress
        *delay = 0ms;
        return _ERR_BUSY;
    }
    _event_flag.clear();

    auto result_callback = [&cb_result, this](const measure_result_t *result) {
        cb_result = *result;
        this->_event_flag.set(_EVENT_FLAG);
    };
    err = _measure_distance_async_impl(result_callback, false);
    if (err) {
        return err;
    }
    _event_flag.wait_all(_EVENT_FLAG);

    *delay = cb_result.delay;
    return cb_result.err;
}

int SimpleHCSR04Driver::measure_distance(float *distance)
{
    microseconds_u32 delay;
    int err = measure_delay(&delay);
    if (err) {
        *distance = 0;
        return err;
    }
    *distance = delay_to_distance_default(delay);
    return 0;
};
