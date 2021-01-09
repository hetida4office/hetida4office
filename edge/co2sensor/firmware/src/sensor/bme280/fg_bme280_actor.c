#include "bme280/fg_bme280_actor.h"
#include "i2c/fg_i2c_actor.h"
#include "rtc/fg_rtc_actor.h"
#include <nrf_queue.h>
#include <nrfx.h>
#include <stddef.h>
#include <utils.h>

/** Logging */
#define NRFX_FG_BME280_ACTOR_CONFIG_LOG_ENABLED 1
#define NRFX_FG_BME280_ACTOR_CONFIG_LOG_LEVEL 4
#define NRFX_FG_BME280_ACTOR_CONFIG_INFO_COLOR 5
#define NRFX_FG_BME280_ACTOR_CONFIG_DEBUG_COLOR 5
#define NRFX_LOG_MODULE FG_BME280_ACTOR
#include <nrfx_log.h>


/** Actor resources */
FG_ACTOR_SLOTS_DEC(fg_bme280_reset, fg_bme280_measure);

#define FG_BME280_ACTOR_SLOT_ASSIGNMENTS                                                           \
    {                                                                                              \
        [FG_BME280_RESET] = fg_bme280_reset, [FG_BME280_MEASURE] = fg_bme280_measure               \
    }
#define FG_BME280_ACTOR_STATES                                                                     \
    {                                                                                              \
        BME280_UNINITIALIZED, BME280_OFF, BME280_RESETTING, BME280_IDLE, BME280_MEASURING,         \
            BME280_RECOVERING                                                                      \
    }
FG_ACTOR_DEF(FG_BME280_ACTOR_SLOT_ASSIGNMENTS, FG_BME280_ACTOR_STATES, FG_ACTOR_NO_TASKS);

// Initialization result handlers
FG_ACTOR_RESULT_HANDLERS_DEC(fg_bme280_reset_read_chip_id, fg_bme280_reset_write_reset_cmd,
    fg_bme280_reset_wait, fg_bme280_reset_reenable_i2c, fg_bme280_reset_read_temp_press_calib_data,
    fg_bme280_reset_store_temp_press_calib_data, fg_bme280_reset_read_hum_calib_data,
    fg_bme280_reset_store_hum_calib_data, fg_bme280_reset_write_hum_osr_config,
    fg_bme280_reset_finalize, fg_bme280_reset_finished);

// Measure result handlers
FG_ACTOR_RESULT_HANDLERS_DEC(fg_bme280_measure_write_meas_cmd, fg_bme280_measure_wait,
    fg_bme280_measure_reenable_i2c, fg_bme280_measure_read_result, fg_bme280_measure_finalize,
    fg_bme280_measure_finished);

// Hard reset result handlers
FG_ACTOR_RESULT_HANDLERS_DEC(fg_bme280_hard_reset, fg_bme280_hard_reset_done);

FG_ACTOR_INTERFACE_LOCAL_DEC();

static bool m_hard_reset;
static bool m_i2c_enabled;


/** BME280 resources */
// Allocate buffers and data structures to collect pending transactions.
#define BME280_DEVICE_ADDRESS 0x77 // maybe 0x76 depending on hardware setup
#define BME280_ADDRESS_SIZE sizeof(uint8_t)
#define BME280_REGISTER_SIZE sizeof(uint8_t) // size of a single BME280 register

#define BME280_MAX_READ_OP_LEN BME280_TEMP_PRESS_CALIB_DATA_LEN
#define BME280_MAX_WRITE_OP_LEN (BME280_ADDRESS_SIZE + BME280_REGISTER_SIZE)

// tSTARTUP delay (max, in ms) == 2, see BME280 spec, 1.1, calculate using (3)
#define BME280_RESET_DELAY_MS 2

// tMEASURE delay (max, in ms) ==
//   1.25 + (2.3 * T-OSR) + (2.3 * P-OSR + 0.575) + (2.3 * H-OSR + 0.575),
// see BME280 spec, 9.1, , calculate using (4)
#define BME280_MEASUREMENT_DELAY_MS ((uint32_t)CEIL_DIV(1250 + (2300 * 1) + (2300 * 1 + 575) + (2300 * 1 + 575), 1000))

static struct bme280_calib_data m_calib_data;


/** I2C child actor resources */
#define BME280_I2C_TX_BUFFER_SIZE BME280_MAX_WRITE_OP_LEN
#define BME280_I2C_RX_BUFFER_SIZE BME280_MAX_READ_OP_LEN

static uint8_t m_i2c_tx_buffer[BME280_I2C_TX_BUFFER_SIZE];
static uint8_t m_i2c_rx_buffer[BME280_I2C_RX_BUFFER_SIZE];

static fg_i2c_actor_transaction_t m_current_i2c_transaction = {.address = BME280_DEVICE_ADDRESS};

__STATIC_INLINE void fg_bme280_i2c_reset_current_transaction(void)
{
    m_current_i2c_transaction.tx_size = 0;
    m_current_i2c_transaction.p_tx_data = NULL;
    m_current_i2c_transaction.rx_size = 0;
    m_current_i2c_transaction.p_rx_data = NULL;
}

static uint32_t m_current_timeout;


/** Public API */
FG_ACTOR_INIT_DEF(bme280, BME280_UNINITIALIZED, BME280_OFF, fg_bme280_init);


/** Internal implementation */
static void fg_bme280_init(void)
{
    // Initialize child actors.
    fg_i2c_actor_init();
    fg_rtc_actor_init();
}

static fg_i2c_actor_transaction_t * const fg_bme280_i2c_read_transaction(
    const uint8_t reg_addr, const uint16_t rx_size);

static fg_i2c_actor_transaction_t * const fg_bme280_i2c_write_transaction(
    const uint8_t reg_addr, const uint8_t data);

#define BME280_RESET_CODE 0xB6
FG_ACTOR_SLOT(fg_bme280_reset)
{
    FG_ACTOR_STATE_TRANSITION(BME280_OFF, BME280_RESETTING, "enable peripherals");

    FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_ENABLE);
    m_i2c_enabled = true;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_reset_read_chip_id);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_read_chip_id)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "resetting (enabling peripherals): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_RESETTING, BME280_RESETTING, "read chip id");

    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_TRANSACT);
    FG_ACTOR_SET_P_ARGS(p_next_action, fg_i2c_actor_transaction_t,
        fg_bme280_i2c_read_transaction(BME280_CHIP_ID_ADDR, BME280_REGISTER_SIZE));
    p_next_action->result_size = BME280_REGISTER_SIZE;
    p_next_action->p_result = &m_i2c_rx_buffer;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_reset_write_reset_cmd);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_write_reset_cmd)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "resetting (reading chip id): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_RESETTING, BME280_RESETTING, "write reset command");

    FG_ACTOR_GET_RESULT(uint8_t, chip_id, FG_ACTOR_GET_FIRST_COMPLETED_ACTION());
    if (chip_id != BME280_CHIP_ID)
        fg_bme280_hard_reset(FG_ACTOR_RESULT_HANDLER_ARGS);

    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_TRANSACT);
    FG_ACTOR_SET_P_ARGS(p_next_action, fg_i2c_actor_transaction_t,
        fg_bme280_i2c_write_transaction(BME280_RESET_ADDR, BME280_RESET_CODE));

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_reset_wait);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_wait)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "resetting (writing reset cmd): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_RESETTING, BME280_RESETTING, "wait for reset cmd to finish");

    FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_DISABLE);
    m_i2c_enabled = false;

    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(rtc, FG_RTC_START_TIMER);
    m_current_timeout = BME280_RESET_DELAY_MS;
    FG_ACTOR_SET_ARGS(p_next_action, m_current_timeout);

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_reset_reenable_i2c);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_reenable_i2c)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "resetting (waiting for reset cmd to finish): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_RESETTING, BME280_RESETTING, "re-enable I2C");

    FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_ENABLE);
    m_i2c_enabled = true;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_reset_read_temp_press_calib_data);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_read_temp_press_calib_data)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "resetting (re-enabling I2C): %#x!");

    FG_ACTOR_STATE_TRANSITION(
        BME280_RESETTING, BME280_RESETTING, "read temp/press calibration data");

    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_TRANSACT);
    FG_ACTOR_SET_P_ARGS(p_next_action, fg_i2c_actor_transaction_t,
        fg_bme280_i2c_read_transaction(
            BME280_TEMP_PRESS_CALIB_DATA_ADDR, BME280_TEMP_PRESS_CALIB_DATA_LEN));
    p_next_action->result_size = BME280_TEMP_PRESS_CALIB_DATA_LEN;
    p_next_action->p_result = &m_i2c_rx_buffer;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_reset_store_temp_press_calib_data);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_store_temp_press_calib_data)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "resetting (reading temp/press calibration data): %#x!");

    FG_ACTOR_GET_P_RESULT_ARR(
        uint8_t, BME280_TEMP_PRESS_CALIB_DATA_LEN, reg_data, FG_ACTOR_GET_FIRST_COMPLETED_ACTION());
    m_calib_data.dig_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    m_calib_data.dig_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    m_calib_data.dig_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    m_calib_data.dig_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    m_calib_data.dig_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    m_calib_data.dig_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    m_calib_data.dig_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    m_calib_data.dig_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    m_calib_data.dig_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    m_calib_data.dig_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    m_calib_data.dig_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    m_calib_data.dig_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    m_calib_data.dig_H1 = reg_data[25];

    fg_bme280_reset_read_hum_calib_data(FG_ACTOR_RESULT_HANDLER_ARGS);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_read_hum_calib_data)
{
    FG_ACTOR_STATE_TRANSITION(BME280_RESETTING, BME280_RESETTING, "read hum calibration data");

    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_TRANSACT);
    FG_ACTOR_SET_P_ARGS(p_next_action, fg_i2c_actor_transaction_t,
        fg_bme280_i2c_read_transaction(
            BME280_HUMIDITY_CALIB_DATA_ADDR, BME280_HUMIDITY_CALIB_DATA_LEN));
    p_next_action->result_size = BME280_HUMIDITY_CALIB_DATA_LEN;
    p_next_action->p_result = &m_i2c_rx_buffer;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_reset_store_hum_calib_data);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_store_hum_calib_data)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "resetting (reading hum calibration data): %#x!");

    FG_ACTOR_GET_P_RESULT_ARR(
        uint8_t, BME280_HUMIDITY_CALIB_DATA_LEN, reg_data, FG_ACTOR_GET_FIRST_COMPLETED_ACTION());
    m_calib_data.dig_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    m_calib_data.dig_H3 = reg_data[2];

    int16_t dig_H4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    int16_t dig_H4_lsb = (int16_t)(reg_data[4] & 0x0F);
    m_calib_data.dig_H4 = dig_H4_msb | dig_H4_lsb;

    int16_t dig_H5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    int16_t dig_H5_lsb = (int16_t)(reg_data[4] >> 4);
    m_calib_data.dig_H5 = dig_H5_msb | dig_H5_lsb;
    m_calib_data.dig_H6 = (int8_t)reg_data[6];

    fg_bme280_reset_write_hum_osr_config(FG_ACTOR_RESULT_HANDLER_ARGS);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_write_hum_osr_config)
{
    FG_ACTOR_STATE_TRANSITION(BME280_RESETTING, BME280_RESETTING, "write hum OSR config");

    // Humidity settings can be set on initialization, as they only need to be written once
    // but BEFORE the measurement is started. Pressure and temperature settings will be set
    // when starting the sensor (as starting the sensor in force mode needs a write to the
    // same register as the settings anyway).
    // We don't have to write standby or filter settings as both are set to the right
    // values on reset when using forced mode.
    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_TRANSACT);
    uint8_t hum_ctrl = BME280_OVERSAMPLING_1X << BME280_CTRL_HUM_POS;
    FG_ACTOR_SET_P_ARGS(p_next_action, fg_i2c_actor_transaction_t,
        fg_bme280_i2c_write_transaction(BME280_CTRL_HUM_ADDR, hum_ctrl));

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_reset_finalize);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_finalize)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "resetting (writing hum OSR config): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_RESETTING, BME280_RESETTING, "finish reset");

    FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_DISABLE);
    m_i2c_enabled = false;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_reset_finished);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_reset_finished)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "resetting (disabling peripherals): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_RESETTING, BME280_IDLE, "reset finished");

    // If this was a hard reset then now
    // we successfully reached a well-defined
    // state again.
    m_hard_reset = false;
}

FG_ACTOR_SLOT(fg_bme280_measure)
{
    FG_ACTOR_STATE_TRANSITION(BME280_IDLE, BME280_MEASURING, "enabling peripherals");

    FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_ENABLE);
    m_i2c_enabled = true;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_measure_write_meas_cmd);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_measure_write_meas_cmd)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "measuring (enabling peripherals): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_MEASURING, BME280_MEASURING, "write meas cmd");

    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_TRANSACT);
    uint8_t meas_ctrl = (BME280_OVERSAMPLING_1X << BME280_CTRL_TEMP_POS) |
                        (BME280_OVERSAMPLING_1X << BME280_CTRL_PRESS_POS) |
                        (BME280_FORCED_MODE << BME280_SENSOR_MODE_POS);
    FG_ACTOR_SET_P_ARGS(p_next_action, fg_i2c_actor_transaction_t,
        fg_bme280_i2c_write_transaction(BME280_CTRL_MEAS_ADDR, meas_ctrl));

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_measure_wait);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_measure_wait)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "measuring (writing meas cmd): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_MEASURING, BME280_MEASURING, "wait for meas cmd to finish");

    FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_DISABLE);
    m_i2c_enabled = false;

    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(rtc, FG_RTC_START_TIMER);
    m_current_timeout = BME280_MEASUREMENT_DELAY_MS;
    FG_ACTOR_SET_ARGS(p_next_action, m_current_timeout);

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_measure_reenable_i2c);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_measure_reenable_i2c)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "measuring (waiting for meas cmd to finish): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_MEASURING, BME280_MEASURING, "re-enable I2c");

    FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_ENABLE);
    m_i2c_enabled = true;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_measure_read_result);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_measure_read_result)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "measuring (re-enabling I2C): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_MEASURING, BME280_MEASURING, "read result data");

    fg_actor_action_t * const p_next_action = FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_TRANSACT);
    FG_ACTOR_SET_P_ARGS(p_next_action, fg_i2c_actor_transaction_t,
        fg_bme280_i2c_read_transaction(BME280_DATA_ADDR, BME280_P_T_H_DATA_LEN));
    p_next_action->result_size = BME280_P_T_H_DATA_LEN;
    p_next_action->p_result = &m_i2c_rx_buffer;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_measure_finalize);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_measure_finalize)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "measuring (reading result data): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_MEASURING, BME280_MEASURING, "finalize");

    FG_ACTOR_GET_P_RESULT_ARR(
        uint8_t, BME280_P_T_H_DATA_LEN, reg_data, FG_ACTOR_GET_FIRST_COMPLETED_ACTION());

    // Parse the read data from the sensor.
    struct bme280_uncomp_data uncomp_data = {0};
    bme280_parse_sensor_data(reg_data, &uncomp_data);

    // Compensate the pressure, temperature and humidity data
    // from the sensor and write it to the calling action's result.
    const fg_actor_action_t * const p_calling_action = p_completed_transaction->p_calling_action;
    ASSERT(p_calling_action)
    ASSERT(p_calling_action->result_size == sizeof(fg_bme280_measurement_t))
    ASSERT(p_calling_action->p_result)

    fg_bme280_measurement_t * const p_measurement = p_calling_action->p_result;
    int8_t rslt = bme280_compensate_data(BME280_ALL, &uncomp_data, p_measurement, &m_calib_data);
    p_measurement->humidity = (p_measurement->humidity * 1000) >> 10;

    FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_DISABLE);
    m_i2c_enabled = false;

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_measure_finished);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_measure_finished)
{
    RESET_ON_ERROR(fg_bme280_hard_reset, "measuring (disabling peripherals): %#x!");

    FG_ACTOR_STATE_TRANSITION(BME280_MEASURING, BME280_IDLE, "measurement finished");
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_hard_reset)
{
    if (m_fg_actor_state == BME280_RECOVERING)
        return;

    // If we get a recursive error on reset
    // then something is really wrong and
    // we give up trying.
    ASSERT(!m_hard_reset);
    m_hard_reset = true;

    // Make sure we've not yet started another transaction.
    ASSERT(!p_next_transaction->p_first_concurrent_action);
    ASSERT(!p_next_transaction->result_handler);

    m_fg_actor_state = BME280_RECOVERING;
    NRFX_LOG_WARNING("hard reset");

    // Try to force a well-defined actor state.
    fg_bme280_i2c_reset_current_transaction();
    if (m_i2c_enabled)
    {
        FG_ACTOR_POST_MESSAGE(i2c, FG_I2C_DISABLE);
        m_i2c_enabled = false;
    }

    FG_ACTOR_SET_TRANSACTION_RESULT_HANDLER(fg_bme280_hard_reset_done);
}

FG_ACTOR_RESULT_HANDLER(fg_bme280_hard_reset_done)
{
    ASSERT(!FG_ACTOR_GET_FIRST_COMPLETED_ACTION()->error_flags)

    FG_ACTOR_STATE_TRANSITION(BME280_RECOVERING, BME280_OFF, "re-initializing after hard reset");

    FG_ACTOR_POST_MESSAGE(bme280, FG_BME280_RESET);
}

static fg_i2c_actor_transaction_t * const fg_bme280_i2c_build_transaction(
    const uint8_t * p_tx_data, const size_t tx_size, const size_t rx_size);

static fg_i2c_actor_transaction_t * const fg_bme280_i2c_read_transaction(
    const uint8_t reg_addr, const uint16_t rx_size)
{
    return fg_bme280_i2c_build_transaction(&reg_addr, sizeof(reg_addr), rx_size);
}

static fg_i2c_actor_transaction_t * const fg_bme280_i2c_write_transaction(
    const uint8_t reg_addr, const uint8_t data)
{
    uint8_t tx_data[2];
    tx_data[0] = reg_addr;
    tx_data[1] = data;
    return fg_bme280_i2c_build_transaction(tx_data, sizeof(tx_data), 0);
}

static fg_i2c_actor_transaction_t * const fg_bme280_i2c_build_transaction(
    const uint8_t * p_tx_data, const size_t tx_size, const size_t rx_size)
{
    m_current_i2c_transaction.tx_size = tx_size;
    m_current_i2c_transaction.p_tx_data = m_i2c_tx_buffer;
    m_current_i2c_transaction.rx_size = rx_size;
    m_current_i2c_transaction.p_rx_data = rx_size == 0 ? NULL : m_i2c_rx_buffer;

    ASSERT(tx_size <= BME280_I2C_TX_BUFFER_SIZE)
    memcpy(m_i2c_tx_buffer, p_tx_data, tx_size);

    return &m_current_i2c_transaction;
}

FG_ACTOR_INTERFACE_DEF(bme280, fg_bme280_actor_message_code_t)