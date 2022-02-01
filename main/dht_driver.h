/**
* @file DHT.h
* @author Alexander Grin
* @copyright (c) 2020 Grin development. All rights reserved.
*/

#ifndef DHT_H
#define DHT_H

#ifdef __cplusplus
extern "C" {
#endif

/*************************** C types headers *****************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/*************************** Common macros   *****************************/
/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif

/*************************** Type definition *****************************/

/**
 * @brief DHT errors enumeration
 */
typedef enum {
    DHT_OK = 0,         //!< DHT_OK - success
    DHT_TIMEOUT = -1,   //!< DHT_TIMEOUT - timeout during reading
    DHT_CRC_ERR = -2,   //!< DHT_CRC_ERR - invalid sensor data
    DHT_INVALID_ARG = -3//!< DHT_INVALID_ARG - dht_intrf_t structure invalid
} dht_err_t;

/**
 * Supports DHT sensors enumeration
 */
typedef enum {
   DHT_11 = 0,//!< DHT_11
   DHT_12,    //!< DHT_12
   DHT_21,    //!< DHT_21
   DHT_22,    //!< DHT_22
   DHT_MAX    //!< DHT_MAX
} dht_type_t;

typedef enum {
    DHT_GPIO_INPUT_MODE = 0,
    DHT_GPIO_OUTPUT_MODE
} dht_gpio_mode_t;

typedef enum {
   DHT_GPIO_LOW = 0,
   DHT_GPIO_HIGH
} dht_gpio_state_t;

typedef void (*dht_pin_init_fptr_t)(dht_gpio_mode_t mode);
typedef void (*dht_pin_set_fptr_t)(dht_gpio_state_t state);
typedef dht_gpio_state_t (*dht_pin_read_fptr_t)();
typedef void (*dht_delay_us_fptr_t)(uint32_t period);

/**
 * @brief DHT sensor platform functions
 */
typedef struct {
    dht_pin_init_fptr_t pin_init;
    dht_pin_set_fptr_t pin_set;
    dht_pin_read_fptr_t pin_read;
    dht_delay_us_fptr_t delay_us;
} dht_intrf_t;

/**
 * @brief DHT sensor handle
 */
typedef struct
{
    float temperature;      //!< Temperature in degrees Celsius
    float humidity;         //!< Percentage relative humidity
    dht_err_t status;       //!< Status of last reading
    const dht_type_t type;  //!< Type of DHT sensor
    const dht_intrf_t intrf;//!< Platform interface
} dht_sensor_t;

/*************************** Driver interface *****************************/

/**
 * @brief DHT sensor read data
 * @param sensor - pointer to sensor handle structure
 * @return
 *      DHT_OK - success
 *      DHT_TIMEOUT - timeout during reading
 *      DHT_CRC_ERR - invalid sensor data
 *      DHT_INVALID_ARG - dht_intrf_t structure invalid
 */
dht_err_t dht_read(dht_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* DHT_H */