/**
* @file dht.c
* @author Alexander Grin
* @copyright (c) 2020 Grin development. All rights reserved.
*/

#include "dht_driver.h"

#define DHT_TRANSFER_BITES_COUNT            (40)

#define DHT_START_LOW_TIME_US               (18*1000)
#define DHT_START_HIGH_TIME_US              (40)

#define DHT_RESPONSE_LOW_TIME_US            (80)
#define DHT_RESPONSE_HIGH_TIME_US           (80)

#define DHT_DATA_BIT_START_TIME_US          (50)
#define DHT_DATA_BIT_LOW_TIME_US            (28)
#define DHT_DATA_BIT_HIGH_TIME_US           (70)

/**
 * DHT sensor transfer packet
 */
typedef struct {
    uint8_t humidity_integer;
    uint8_t humidity_decimal;
    uint8_t temperature_integer;
    uint8_t temperature_decimal;
    uint8_t crc;
} dht_packet_t;

/**
 * @brief Send start communication signal to DHT sensor
 * @param sensor - sensor handle structure
 */
static void dht_start(const dht_sensor_t *sensor)
{
    sensor->intrf.pin_init(DHT_GPIO_OUTPUT_MODE);

    sensor->intrf.pin_set(DHT_GPIO_LOW);
    sensor->intrf.delay_us(DHT_START_LOW_TIME_US);

    sensor->intrf.pin_set(DHT_GPIO_HIGH);
    sensor->intrf.delay_us(DHT_START_HIGH_TIME_US);
}

/**
 * @brief Wait change line state or timeout error
 *
 * @param sensor - pointer to sensor handle structure
 * @param timeout_us - timeout time in microseconds
 * @param state - expected line state
 * @return
 *          DHT_TIMEOUT - Line state has not changed
 *          other - microseconds before the expected state
 */
static int16_t dht_wait_or_timeout(const dht_sensor_t *sensor,
        int16_t timeout_us, dht_gpio_state_t state)
{
    uint16_t time_us = 0;

    sensor->intrf.pin_init(DHT_GPIO_INPUT_MODE);

    //! Wait for expected state
    while(sensor->intrf.pin_read() == state) {
        if(time_us++ > timeout_us)
            return DHT_TIMEOUT;

        sensor->intrf.delay_us(1);
    }

    return time_us;
}

/**
 * @brief Check response from DHT sensor
 *
 * @param sensor - pointer to sensor handle structure
 * @return
 *      DHT_OK - success
 *      DHT_TIMEOUT - timeout during reading
 */
static dht_err_t dht_check_response(const dht_sensor_t *sensor)
{
    dht_err_t rslt;

    rslt = dht_wait_or_timeout(sensor, DHT_RESPONSE_LOW_TIME_US,
            DHT_GPIO_LOW);
    if(rslt == DHT_TIMEOUT)
        return rslt;

    rslt = dht_wait_or_timeout(sensor, DHT_RESPONSE_HIGH_TIME_US,
            DHT_GPIO_HIGH);
    if(rslt == DHT_TIMEOUT)
        return rslt;

    return DHT_OK;
}

/**
 * @brief Check CRC of DHT sensor transfer packet
 *
 * @param packet - pointer to DHT sensor transfer packet
 * @return
 */
static dht_err_t dht_check_crc(dht_packet_t *packet)
{
    uint8_t check_sum = 0;

    check_sum = packet->humidity_integer+packet->humidity_decimal;
    check_sum += packet->temperature_integer+packet->temperature_decimal;

    if(check_sum == packet->crc)
        return DHT_OK;
    else
        return DHT_CRC_ERR;

}

/**
 * @brief Parse data according to sensor type
 *
 * @param sensor - pointer to sensor handle structure
 * @param packet - - pointer to DHT sensor transfer packet
 */
static void dht_data_parse(dht_sensor_t *sensor, dht_packet_t *packet)
{
    uint8_t temperature_integer = packet->temperature_integer;
    uint8_t temperature_decimal = packet->temperature_decimal;
    uint8_t humidity_integer = packet->humidity_integer;
    uint8_t humidity_decimal= packet->humidity_decimal;
    float temperature = 0;
    float humidity = 0;

    switch(sensor->type)
    {
        case DHT_11:
            temperature = temperature_integer;
            temperature += (temperature_decimal & 0x0F) * 0.1;
            if (temperature_decimal & 0x80)
                temperature = -1 - temperature;

            humidity = humidity_integer + humidity_decimal * 0.1;
            break;
        case DHT_12:
            temperature = temperature_integer;
            temperature += (temperature_decimal & 0x0F) * 0.1;
            if (temperature_integer & 0x80) {
                temperature *= -1;
            }

            humidity = humidity_integer + humidity_decimal * 0.1;
            break;
        case DHT_21:
        case DHT_22:
            temperature = temperature_integer & 0x7F;
            temperature = (uint16_t)temperature << 8;
            temperature = (uint16_t)temperature | temperature_decimal;
            temperature *= 0.1;
            if (temperature_integer & 0x80)
                temperature *= -1;

            humidity = ((uint16_t)humidity_integer) << 8 | humidity_decimal;
            humidity *= 0.1;
            break;
        default:
            break;
    }

    sensor->temperature = temperature;
    sensor->humidity = humidity;
}

dht_err_t dht_read(dht_sensor_t *sensor)
{
    dht_err_t rslt;
    int16_t wait_time_us;
    dht_packet_t packet = {0};
    dht_packet_t *ptr_packet = &packet;

    //! Null pointer check
    if(sensor->intrf.pin_init == NULL)
        return DHT_INVALID_ARG;
    if(sensor->intrf.pin_set == NULL)
        return DHT_INVALID_ARG;
    if(sensor->intrf.pin_read == NULL)
        return DHT_INVALID_ARG;
    if(sensor->intrf.delay_us == NULL)
        return DHT_INVALID_ARG;

    if(sensor->type >= DHT_MAX)
        return DHT_INVALID_ARG;

    //! Send start signal to sensor
    dht_start(sensor);

    //! Check response from the sensor
    rslt = dht_check_response(sensor);
    if(rslt != DHT_OK) {
        sensor->status = rslt;
        return rslt;
    }

    //! Read sensor data process
    for(int bit = 0; bit < DHT_TRANSFER_BITES_COUNT; bit++) {
        uint8_t byte_num = bit / 8;

        //! Each data bit starts with DHT_DATA_BIT_START_TIME_US
        wait_time_us = dht_wait_or_timeout(sensor,
                DHT_DATA_BIT_START_TIME_US, DHT_GPIO_LOW);
        if(wait_time_us == DHT_TIMEOUT) {
            rslt = DHT_TIMEOUT;
            sensor->status = rslt;
            return rslt;
        }

        wait_time_us = dht_wait_or_timeout(sensor,
                DHT_DATA_BIT_HIGH_TIME_US, DHT_GPIO_HIGH);
        if(wait_time_us == DHT_TIMEOUT) {
            rslt = DHT_TIMEOUT;
            sensor->status = rslt;
            return rslt;
        }
        else if(wait_time_us > DHT_DATA_BIT_LOW_TIME_US) {
            //! Receive high bit
            //! Sensor sends data in Most Significant Bit format
            ((uint8_t*)ptr_packet)[byte_num] |= (1 << (7 - (bit % 8)));
        }
    }

    //! Check CRC
    rslt = dht_check_crc(ptr_packet);
    if(rslt != DHT_OK)
        return rslt;
    else {
        //! Parse data according to sensor type
        dht_data_parse(sensor, ptr_packet);

        sensor->status = rslt;
        return rslt;
    }
}