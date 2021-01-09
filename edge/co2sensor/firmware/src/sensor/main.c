#include "bme280/fg_bme280_actor.h"
#include "lp8/fg_lp8_actor.h"
#include "mqttsn/fg_mqttsn_actor.h"
#include "rtc/fg_rtc_actor.h"
#include "saadc/fg_saadc_actor.h"
#include <app_scheduler.h>
#include <nrf_assert.h>
#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <nrf_log_default_backends.h>
#include <stdlib.h>
#include <thread_utils.h>


FG_ACTOR_RESULT_HANDLERS_DEC(init_or_wakeup_result_handler, pressure_measurement_result_handler,
    co2_measurement_result_handler, mqttsn_sleep_handler);

/** BME280 child actor resources */
static const fg_actor_t * m_p_bme280_actor;

/** LP8 child actor resources */
static const fg_actor_t * m_p_lp8_actor;

/** RTC child actor resources */
static const fg_actor_t * m_p_rtc_actor;
static uint32_t m_main_mqttsn_sleep_ms;
static uint32_t m_main_idle_time_ms;

/** SAADC child actor resources */
static const fg_actor_t * m_p_saadc_actor;
#define FG_SAADC_MEASUREMENT_CYCLES 15;
static uint8_t m_main_saadc_remaining_measurements = 1;

/** MQTTSN child actor resources */
static const fg_actor_t * m_p_mqttsn_actor;
#define FG_MQTT_MESSAGE_MAX_LEN 20
static char m_mqttsn_message_buffer[FG_MQTT_TOPIC_NUM][FG_MQTT_MESSAGE_MAX_LEN];

static fg_mqttsn_message_t m_mqttsn_message[FG_MQTT_TOPIC_NUM] = {
  {.p_data = m_mqttsn_message_buffer[FG_MQTT_TOPIC_PRESSURE]},
  {.p_data = m_mqttsn_message_buffer[FG_MQTT_TOPIC_TEMPERATURE]},
  {.p_data = m_mqttsn_message_buffer[FG_MQTT_TOPIC_HUMIDITY]},
  {.p_data = m_mqttsn_message_buffer[FG_MQTT_TOPIC_CO2]},
  {.p_data = m_mqttsn_message_buffer[FG_MQTT_TOPIC_BAT]},
  {.p_data = m_mqttsn_message_buffer[FG_MQTT_TOPIC_STATUS]},
};


/** Main loop */

// TODO: connect gpio pin to sleep state and check whether the device is sleeping as intended.
// TODO: Make sure EN_BLK_REVERSE is Hi-Z during reset to make sure that VCAP will not
//       (reverse) aliment 3V3. Hide VCAP behind MOSFET.
// TODO: Add (optional) timeout to actor framework to discover failed actions.
// TODO: implement watchdog (capture ASSERTS and show it somehow e.g. via buzzer), stack guard, mpu,
//       NRFX_PRS
// TODO: install and use DC/DC regulator
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("FG sensor app started.");

    fg_actor_init();

    m_p_rtc_actor = fg_rtc_actor_init(); // Starts the RTC clock - so must be called before
                                         // any other actor requiring RTC service.
    m_p_mqttsn_actor = fg_mqttsn_actor_init();
    m_p_bme280_actor = fg_bme280_actor_init();
    m_p_lp8_actor = fg_lp8_actor_init();
    m_p_saadc_actor = fg_saadc_actor_init();

    fg_actor_transaction_t * const p_init_transaction =
        fg_actor_allocate_root_transaction(init_or_wakeup_result_handler);
    fg_mqttsn_actor_allocate_message(FG_MQTTSN_CONNECT, p_init_transaction);
    fg_bme280_actor_allocate_message(FG_BME280_RESET, p_init_transaction);
    fg_saadc_actor_allocate_message(FG_SAADC_CALIBRATE, p_init_transaction);

    while (true)
    {
        fg_actor_run();
        thread_process();
        app_sched_execute();

        if (NRF_LOG_PROCESS() == false)
        {
            thread_sleep();
        }
    }
}

static void check_publish_action(const fg_actor_action_t * const p_completed_action)
{
    ASSERT(p_completed_action != NULL);
    ASSERT(p_completed_action->p_actor == m_p_mqttsn_actor)
    ASSERT(p_completed_action->message.code == FG_MQTTSN_PUBLISH)
    CHECK_COMPLETED_ACTION(p_completed_action, "MQTT publishing error!");
}

/** Implementation */
FG_ACTOR_RESULT_HANDLER(init_or_wakeup_result_handler)
{
    static fg_bme280_measurement_t bme280_measurement;
    static fg_saadc_result_t saadc_bat_measurement;

    const fg_actor_action_t * p_completed_action = FG_ACTOR_GET_FIRST_COMPLETED_ACTION();
    if (p_completed_action->p_actor == m_p_mqttsn_actor)
    {
        // First iteration:
        ASSERT(p_completed_action->message.code == FG_MQTTSN_CONNECT)
        CHECK_COMPLETED_ACTION(p_completed_action, "MQTTSN gateway not found!");

        p_completed_action = p_completed_action->p_next_concurrent_action;
        ASSERT(p_completed_action != NULL);
        ASSERT(p_completed_action->p_actor == m_p_bme280_actor)
        ASSERT(p_completed_action->message.code == FG_BME280_RESET)
        CHECK_COMPLETED_ACTION(p_completed_action, "BME280 initialization error!");

        p_completed_action = p_completed_action->p_next_concurrent_action;
        ASSERT(p_completed_action != NULL);
        ASSERT(p_completed_action->p_actor == m_p_saadc_actor)
        ASSERT(p_completed_action->message.code == FG_SAADC_CALIBRATE)
        CHECK_COMPLETED_ACTION(p_completed_action, "SAADC calibration error!");
    }
    else
    {
        // Subsequent iterations:
        ASSERT(p_completed_action->p_actor == m_p_rtc_actor)
        ASSERT(p_completed_action->message.code == FG_RTC_START_TIMER);
        CHECK_COMPLETED_ACTION(p_completed_action, "RTC error!");

        p_completed_action = p_completed_action->p_next_concurrent_action;
        ASSERT(p_completed_action != NULL);
        ASSERT(p_completed_action->p_actor == m_p_mqttsn_actor)
        ASSERT(p_completed_action->message.code == FG_MQTTSN_SLEEP)
        CHECK_COMPLETED_ACTION(p_completed_action, "Error while trying to put MQTTSN gateway to sleep!");
    }

    // (Re-)Start measuring.
    fg_actor_action_t * p_next_action = FG_ACTOR_POST_MESSAGE(bme280, FG_BME280_MEASURE);
    FG_ACTOR_SET_P_RESULT(p_next_action, fg_bme280_measurement_t, &bme280_measurement);

    m_main_saadc_remaining_measurements--;
    if (m_main_saadc_remaining_measurements == 0) {
      m_main_saadc_remaining_measurements = FG_SAADC_MEASUREMENT_CYCLES;
      p_next_action = FG_ACTOR_POST_MESSAGE(saadc, FG_SAADC_MEASURE);
      FG_ACTOR_SET_P_RESULT(p_next_action, fg_saadc_result_t, &saadc_bat_measurement);
    }

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(pressure_measurement_result_handler);
}

static void publish_measurement(
    fg_actor_transaction_t * const p_next_transaction, int32_t measurement, uint16_t topic_id)
{
    snprintf(m_mqttsn_message_buffer[topic_id], sizeof(m_mqttsn_message_buffer[topic_id]), "%d", measurement);
    m_mqttsn_message[topic_id].size = strlen(m_mqttsn_message_buffer[topic_id]);
    ASSERT(m_mqttsn_message[topic_id].size <= sizeof(m_mqttsn_message_buffer[topic_id]));
    m_mqttsn_message[topic_id].topic_id = topic_id;

    fg_actor_action_t * p_next_action = FG_ACTOR_POST_MESSAGE(mqttsn, FG_MQTTSN_PUBLISH);
    FG_ACTOR_SET_ARGS(p_next_action, m_mqttsn_message[topic_id]);
}

FG_ACTOR_RESULT_HANDLER(pressure_measurement_result_handler)
{
    static uint16_t pressure;
    static fg_lp8_measurement_t lp8_measurement;

    const fg_actor_action_t * p_completed_action = FG_ACTOR_GET_FIRST_COMPLETED_ACTION();
    CHECK_COMPLETED_ACTION(p_completed_action, "BME280 measurement error!");

    ASSERT(p_completed_action->p_actor == m_p_bme280_actor)
    ASSERT(p_completed_action->message.code == FG_BME280_MEASURE)

    FG_ACTOR_GET_P_RESULT(fg_bme280_measurement_t, p_measurement_result, p_completed_action);
    NRF_LOG_INFO("Measurement: pressure %u, temperature %d, humidity %u",
        p_measurement_result->pressure, p_measurement_result->temperature,
        p_measurement_result->humidity);

    // BME280 returns pressure in units of 1/100Pa, LP8 expects units of 10Pa = 0.1hPa
    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(lp8, FG_LP8_MEASURE);
    pressure = p_measurement_result->pressure / 1000;
    FG_ACTOR_SET_ARGS(p_next_action, pressure);

    FG_ACTOR_SET_P_RESULT(p_next_action, fg_lp8_measurement_t, &lp8_measurement);

    publish_measurement(
        p_next_transaction, p_measurement_result->temperature, FG_MQTT_TOPIC_TEMPERATURE);
    publish_measurement(p_next_transaction, p_measurement_result->humidity, FG_MQTT_TOPIC_HUMIDITY);
    publish_measurement(p_next_transaction, p_measurement_result->pressure, FG_MQTT_TOPIC_PRESSURE);

    // Publish battery voltage if measured
    if (p_completed_action->p_next_concurrent_action != NULL) {
      p_completed_action = p_completed_action->p_next_concurrent_action;
      ASSERT(p_completed_action->p_actor == m_p_saadc_actor)
      ASSERT(p_completed_action->message.code == FG_SAADC_MEASURE)
      FG_ACTOR_GET_P_RESULT(fg_saadc_result_t, p_saadc_result, p_completed_action);
      NRF_LOG_INFO("Measurement: battery: %u", *p_saadc_result);
      publish_measurement(p_next_transaction, *p_saadc_result, FG_MQTT_TOPIC_BAT);
    }

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(co2_measurement_result_handler);
}

FG_ACTOR_RESULT_HANDLER(co2_measurement_result_handler)
{
    const fg_actor_action_t * const p_completed_action = FG_ACTOR_GET_FIRST_COMPLETED_ACTION();
    CHECK_COMPLETED_ACTION(p_completed_action, "LP8 measurement error!");

    ASSERT(p_completed_action->p_actor == m_p_lp8_actor)
    ASSERT(p_completed_action->message.code == FG_LP8_MEASURE)

    fg_actor_action_t * publish_action = p_completed_action->p_next_concurrent_action;
    while (publish_action)
    {
        check_publish_action(publish_action);
        publish_action = publish_action->p_next_concurrent_action;
    }

    FG_ACTOR_GET_P_RESULT(fg_lp8_measurement_t, p_measurement_result, p_completed_action);
    NRF_LOG_INFO("Measurement: filtered %u (raw %u) PPM CO2, %d C.",
        p_measurement_result->conc_filtered_pc, p_measurement_result->conc_pc,
        p_measurement_result->temperature);


    publish_measurement(
        p_next_transaction, p_measurement_result->conc_filtered_pc, FG_MQTT_TOPIC_CO2);

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(mqttsn_sleep_handler);
}

FG_ACTOR_RESULT_HANDLER(mqttsn_sleep_handler)
{
    fg_actor_action_t * p_next_action;

    const fg_actor_action_t * p_completed_action = FG_ACTOR_GET_FIRST_COMPLETED_ACTION();
    check_publish_action(p_completed_action);

    // Set timer for next measurement.
    p_next_action = FG_ACTOR_POST_MESSAGE(rtc, FG_RTC_START_TIMER);
    m_main_idle_time_ms = fg_lp8_get_idle_time() * 1000;
    FG_ACTOR_SET_ARGS(p_next_action, m_main_idle_time_ms);

    // Put MQTTSN client to sleep (i.e. thread reduce poll period and save energy).
    // Obs: This message must always be protected by a timer in parallel
    // as lost MQTTSN sleep/disconnect messages will not trigger a timeout.
    p_next_action = FG_ACTOR_POST_MESSAGE(mqttsn, FG_MQTTSN_SLEEP);
    m_main_mqttsn_sleep_ms = m_main_idle_time_ms + 1000;
    FG_ACTOR_SET_ARGS(p_next_action, m_main_mqttsn_sleep_ms);

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(init_or_wakeup_result_handler);
}
